/*
 * MCP2518FD SPI-CAN driver (bring-up version).
 *
 * Register addresses and values: DS20006027B (datasheet), DS20005678E (reference manual).
 * Why the bit timing matches ODrive (87.5% sample point): doc/FDCAN_NOTES.md, "Bit timing".
 */
#include "mcp2518fd.h"

#include <stdio.h>
#include <string.h>

#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* ---- SPI instructions (DS 4.0) ---- */
#define CMD_RESET 0x0
#define CMD_READ  0x3
#define CMD_WRITE 0x2

/* ---- Register addresses ---- */
#define REG_CON_B2      0x002 /* CiCON byte 2: OPMOD in bits 7:5 */
#define REG_CON_B3      0x003 /* CiCON byte 3: REQOP in bits 2:0 */
#define REG_NBTCFG      0x004
#define REG_FIFOCON1    0x05C
#define REG_FIFOCON1_B1 0x05D /* UINC bit 0, TXREQ bit 1 */
#define REG_FIFOSTA1    0x060 /* bit 0: TX FIFO not full */
#define REG_FIFOUA1     0x064
#define REG_FIFOCON2    0x068
#define REG_FIFOCON2_B1 0x069 /* UINC bit 0 */
#define REG_FIFOSTA2    0x06C /* bit 0: RX FIFO not empty */
#define REG_FIFOUA2     0x070
#define REG_FLTCON0     0x1D0 /* byte 0: FLTEN0 bit 7, F0BP bits 4:0 */
#define REG_FLTOBJ0     0x1F0
#define REG_MASK0       0x1F4
#define REG_OSC_B1      0xE01 /* OSCRDY bit 2 */
#define RAM_START       0x400

#define CON_B2_AFTER_RESET 0x98 /* OPMOD = Config, TXQEN = STEF = 1 */
#define OBJ_SIZE           16   /* 8 B header + 8 B payload, no timestamp */
#define WAIT_MS            10   /* oscillator start-up is <= 3 ms (DS p.79) */
#define SPI_TIMEOUT_MS     10

static uint8_t tx_buf[2 + OBJ_SIZE];
static uint8_t rx_buf[2 + OBJ_SIZE];

/* ---- Layer 1: SPI instructions ---- */

static void cs_low(void)  { HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_SET); }

/* First two bytes of every instruction: 4-bit command + 12-bit address */
static void put_header(uint8_t cmd, uint16_t addr)
{
    tx_buf[0] = (uint8_t)((cmd << 4) | ((addr >> 8) & 0x0F));
    tx_buf[1] = (uint8_t)(addr & 0xFF);
}

static mcp_status_t mcp_reset(void)
{
    put_header(CMD_RESET, 0x000);
    cs_low();
    HAL_StatusTypeDef hs = HAL_SPI_Transmit(&hspi1, tx_buf, 2, SPI_TIMEOUT_MS);
    cs_high(); /* reset takes effect on CS rising edge */
    return hs == HAL_OK ? MCP_OK : MCP_ERR_SPI;
}

static mcp_status_t mcp_write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    if (len > OBJ_SIZE) return MCP_ERR_ARG;
    put_header(CMD_WRITE, addr);
    memcpy(&tx_buf[2], data, len);
    cs_low();
    HAL_StatusTypeDef hs = HAL_SPI_Transmit(&hspi1, tx_buf, 2 + len, SPI_TIMEOUT_MS);
    cs_high();
    return hs == HAL_OK ? MCP_OK : MCP_ERR_SPI;
}

static mcp_status_t mcp_read(uint16_t addr, uint8_t *out, uint16_t len)
{
    if (len > OBJ_SIZE) return MCP_ERR_ARG;
    memset(tx_buf, 0, 2 + len); /* dummy bytes clock the data out */
    put_header(CMD_READ, addr);
    cs_low();
    HAL_StatusTypeDef hs = HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2 + len, SPI_TIMEOUT_MS);
    cs_high();
    if (hs != HAL_OK) return MCP_ERR_SPI;
    memcpy(out, &rx_buf[2], len); /* skip the 2 bytes clocked in during the header */
    return MCP_OK;
}

static mcp_status_t write_u8(uint16_t addr, uint8_t v) { return mcp_write(addr, &v, 1); }

static mcp_status_t write_u32(uint16_t addr, uint32_t v)
{
    uint8_t b[4] = { v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF }; /* little-endian */
    return mcp_write(addr, b, 4);
}

/* Poll one byte until (byte & mask) == value, or give up after timeout_ms. */
static mcp_status_t mcp_wait(uint16_t addr, uint8_t mask, uint8_t value, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t b;
    do {
        mcp_status_t st = mcp_read(addr, &b, 1);
        if (st != MCP_OK) return st;
        if ((b & mask) == value) return MCP_OK;
    } while (HAL_GetTick() - start < timeout_ms);
    return MCP_ERR_TIMEOUT;
}

/* FIFO user address -> absolute RAM address of the next message object */
static mcp_status_t read_slot_addr(uint16_t ua_reg, uint16_t *addr)
{
    uint8_t ua[4];
    mcp_status_t st = mcp_read(ua_reg, ua, 4);
    if (st != MCP_OK) return st;
    *addr = (uint16_t)(RAM_START + (ua[0] | (ua[1] << 8)));
    return MCP_OK;
}

#define TRY(expr) do { mcp_status_t st_ = (expr); if (st_ != MCP_OK) return st_; } while (0)

/* ---- Layer 2: driver interface ---- */

mcp_status_t mcp_set_mode(mcp_mode_t mode)
{
    /* Byte 3 also holds ABAT and TXBWS; we keep them 0 (their reset value). */
    TRY(write_u8(REG_CON_B3, (uint8_t)mode));
    return mcp_wait(REG_CON_B2, 0xE0, (uint8_t)(mode << 5), WAIT_MS);
}

mcp_status_t mcp_init(void)
{
    /* RESET is only accepted in Config mode. After an STM32 reflash the chip may still be
     * in another mode, so request Config first. A timeout here is not fatal: if SPI is
     * broken, the ID check below reports it more clearly. */
    mcp_status_t st = mcp_set_mode(MCP_MODE_CONFIG);
    if (st == MCP_ERR_SPI) return st;

    TRY(mcp_reset());
    TRY(mcp_wait(REG_OSC_B1, 0x04, 0x04, WAIT_MS)); /* OSCRDY */

    uint8_t con_b2;
    TRY(mcp_read(REG_CON_B2, &con_b2, 1));
    if (con_b2 != CON_B2_AFTER_RESET) return MCP_ERR_BAD_ID;

    /* 1 Mbps, sample point as close to ODrive's 87.5% as the clock allows.
     * BRP=1, TSEG1, TSEG2, SJW stored as value-1; SJW = TSEG2. */
#if MCP_XTAL_40MHZ
    TRY(write_u32(REG_NBTCFG, 0x00210404)); /* 40 TQ: 1 + 34 + 5 = 87.5% */
#else
    TRY(write_u32(REG_NBTCFG, 0x000F0202)); /* 20 TQ: 1 + 16 + 3 = 85% (87.5% not reachable) */
#endif

    /* PLSIZE = 000 (8 B), FSIZE = 7 (8 msgs), TXAT = 11 (reset value). Byte 0 bit 7 = TXEN. */
    TRY(write_u32(REG_FIFOCON1, 0x07600080)); /* FIFO 1: TX */
    TRY(write_u32(REG_FIFOCON2, 0x07600000)); /* FIFO 2: RX */

    /* Filter 0: mask 0 = compare nothing = accept all. Filter must be off while editing. */
    TRY(write_u8(REG_FLTCON0, 0x00));
    TRY(write_u32(REG_FLTOBJ0, 0));
    TRY(write_u32(REG_MASK0, 0));
    TRY(write_u8(REG_FLTCON0, 0x80 | 2)); /* FLTEN0, point to FIFO 2 */

    return MCP_OK;
}

mcp_status_t mcp_send(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (id > 0x7FF || len > 8) return MCP_ERR_ARG;

    uint8_t sta;
    TRY(mcp_read(REG_FIFOSTA1, &sta, 1));
    if (!(sta & 0x01)) return MCP_ERR_FULL;

    uint16_t addr;
    TRY(read_slot_addr(REG_FIFOUA1, &addr));

    /* T0: SID in bits 10:0. T1 byte 0: DLC, with FDF/BRS/RTR/IDE = 0 (classic, standard ID). */
    uint8_t obj[OBJ_SIZE] = {0};
    obj[0] = id & 0xFF;
    obj[1] = (id >> 8) & 0x07;
    obj[4] = len;
    memcpy(&obj[8], data, len);
    TRY(mcp_write(addr, obj, OBJ_SIZE)); /* RAM: 4-byte aligned, multiple of 4 */

    TRY(write_u8(REG_FIFOCON1_B1, 0x03));                  /* UINC + TXREQ together */
    return mcp_wait(REG_FIFOCON1_B1, 0x02, 0x00, WAIT_MS); /* TXREQ clears when sent */
}

mcp_status_t mcp_receive(uint16_t *id, uint8_t *data, uint8_t *len, uint32_t timeout_ms)
{
    TRY(mcp_wait(REG_FIFOSTA2, 0x01, 0x01, timeout_ms)); /* RX FIFO not empty */

    uint16_t addr;
    TRY(read_slot_addr(REG_FIFOUA2, &addr));

    uint8_t obj[OBJ_SIZE];
    TRY(mcp_read(addr, obj, OBJ_SIZE));
    TRY(write_u8(REG_FIFOCON2_B1, 0x01)); /* UINC: free the slot */

    /* R0 bits 31:30 and most of R1 read as 'x': only take the fields we need. */
    *id = (uint16_t)((obj[0] | (obj[1] << 8)) & 0x7FF);
    uint8_t dlc = obj[4] & 0x0F;
    *len = dlc > 8 ? 8 : dlc; /* classic CAN: DLC 9-15 also mean 8 bytes */
    memcpy(data, &obj[8], 8);
    return MCP_OK;
}

/* ---- Layer 3: bring-up test ---- */

static const char *status_str(mcp_status_t st)
{
    switch (st) {
    case MCP_OK:           return "OK";
    case MCP_ERR_SPI:      return "SPI error";
    case MCP_ERR_TIMEOUT:  return "timeout";
    case MCP_ERR_BAD_ID:   return "bad ID";
    case MCP_ERR_FULL:     return "TX FIFO full";
    case MCP_ERR_MISMATCH: return "mismatch";
    case MCP_ERR_ARG:      return "bad argument";
    }
    return "?";
}

mcp_status_t mcp_loopback_test(void)
{
    /* Goal 1: SPI works and the chip answers */
    mcp_status_t st = mcp_init();
    printf("MCP init: %s\r\n", status_str(st));
    if (st == MCP_ERR_BAD_ID) {
        uint8_t b = 0;
        mcp_read(REG_CON_B2, &b, 1);
        printf("  0x002 = 0x%02X, expected 0x%02X (0x00/0xFF: check wiring and power)\r\n",
               b, CON_B2_AFTER_RESET);
    }
    if (st != MCP_OK) return st;

    /* Goal 2: send one frame in internal loopback and read it back */
    st = mcp_set_mode(MCP_MODE_INT_LOOPBACK);
    printf("MCP loopback mode: %s\r\n", status_str(st));
    if (st != MCP_OK) return st;

    const uint16_t tx_id = 0x123;
    const uint8_t tx_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    st = mcp_send(tx_id, tx_data, 8);
    printf("MCP send: %s\r\n", status_str(st));
    if (st != MCP_OK) return st;

    uint16_t rx_id;
    uint8_t rx_data[8], rx_len;
    st = mcp_receive(&rx_id, rx_data, &rx_len, WAIT_MS);
    printf("MCP receive: %s\r\n", status_str(st));
    if (st != MCP_OK) return st;

    printf("  id 0x%03X len %u data %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           rx_id, rx_len, rx_data[0], rx_data[1], rx_data[2], rx_data[3],
           rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

    if (rx_id != tx_id || rx_len != 8 || memcmp(rx_data, tx_data, 8) != 0) st = MCP_ERR_MISMATCH;
    printf("MCP loopback test: %s\r\n", st == MCP_OK ? "PASS" : "FAIL");
    return st;
}
