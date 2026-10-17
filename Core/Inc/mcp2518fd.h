/*
 * MCP2518FD SPI-CAN driver (bring-up version).
 *
 * Classic CAN frames only, standard 11-bit IDs, up to 8 data bytes.
 * FIFO 1 = TX, FIFO 2 = RX, filter 0 accepts everything into FIFO 2.
 * Not thread-safe: call from a single task.
 */
#ifndef MCP2518FD_H
#define MCP2518FD_H

#include <stdint.h>

/* Module crystal: 1 = 40 MHz, 0 = 20 MHz. Selects the 1 Mbps bit timing. TBD: check the module. */
#define MCP_XTAL_40MHZ 1

typedef enum {
    MCP_OK = 0,
    MCP_ERR_SPI,      /* HAL SPI call failed */
    MCP_ERR_TIMEOUT,  /* a status bit never reached the expected value */
    MCP_ERR_BAD_ID,   /* chip did not read 0x98 after reset: wiring/SPI problem */
    MCP_ERR_FULL,     /* TX FIFO full */
    MCP_ERR_MISMATCH, /* loopback frame differs from what was sent */
    MCP_ERR_ARG,      /* bad argument (id > 0x7FF or len > 8) */
} mcp_status_t;

/* CAN controller modes (REQOP / OPMOD values) */
typedef enum {
    MCP_MODE_NORMAL_FD     = 0,
    MCP_MODE_SLEEP         = 1,
    MCP_MODE_INT_LOOPBACK  = 2,
    MCP_MODE_LISTEN_ONLY   = 3,
    MCP_MODE_CONFIG        = 4,
    MCP_MODE_EXT_LOOPBACK  = 5,
    MCP_MODE_NORMAL_CAN20  = 6,
    MCP_MODE_RESTRICTED    = 7,
} mcp_mode_t;

/* Reset the chip and configure bit timing, FIFOs and filter. Leaves the chip in Config mode. */
mcp_status_t mcp_init(void);

/* Request a mode and wait until the chip reports it. */
mcp_status_t mcp_set_mode(mcp_mode_t mode);

/* Queue one classic CAN frame and wait until it has been sent. len <= 8. */
mcp_status_t mcp_send(uint16_t id, const uint8_t *data, uint8_t len);

/* Wait up to timeout_ms for one frame and copy it out. data must hold 8 bytes. */
mcp_status_t mcp_receive(uint16_t *id, uint8_t *data, uint8_t *len, uint32_t timeout_ms);

/* Bring-up test: SPI check (goal 1) + internal loopback send/receive (goal 2). Prints each step. */
mcp_status_t mcp_loopback_test(void);

#endif /* MCP2518FD_H */
