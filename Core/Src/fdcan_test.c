/*
 * FDCAN1-3 bring-up test.
 *
 * Internal loopback: the controller receives its own frames inside the chip.
 * The TX pin stays recessive and the RX pin is ignored, so no transceiver or wiring is needed.
 */
#include "fdcan_test.h"

#include <stdio.h>
#include <string.h>

#include "fdcan.h"

#define WAIT_MS 10

/* Re-run HAL init with a different mode. Controller must be stopped. */
static HAL_StatusTypeDef set_mode(FDCAN_HandleTypeDef *h, uint32_t mode)
{
    h->Init.Mode = mode;
    return HAL_FDCAN_Init(h);
}

/* Filter 0: mask 0 = compare no ID bits, so every standard ID goes to RX FIFO0.
 * Global filter rejects non-matching frames, so a frame only arrives if filter 0 matched it
 * (the reset default would accept it into FIFO0 anyway and hide a broken filter). */
static HAL_StatusTypeDef accept_all(FDCAN_HandleTypeDef *h)
{
    FDCAN_FilterTypeDef f = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000, /* filter */
        .FilterID2    = 0x000, /* mask */
    };
    HAL_StatusTypeDef st = HAL_FDCAN_ConfigFilter(h, &f);
    if (st != HAL_OK) return st;
    return HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/* Put it back the way CubeMX left it: stopped, Normal mode, reset-default global filter.
 * Stop only if Start succeeded: HAL_FDCAN_Stop returns an error on a controller that never started.
 * Init does not touch GFC, so the global filter must be restored explicitly. */
static HAL_StatusTypeDef restore(FDCAN_HandleTypeDef *h, int started)
{
    HAL_StatusTypeDef st = started ? HAL_FDCAN_Stop(h) : HAL_OK;
    if (st == HAL_OK) st = set_mode(h, FDCAN_MODE_NORMAL);
    if (st == HAL_OK) {
        st = HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                          FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    }
    return st;
}

static HAL_StatusTypeDef send(FDCAN_HandleTypeDef *h, uint32_t id, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef hdr = {
        .Identifier          = id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    return HAL_FDCAN_AddMessageToTxFifoQ(h, &hdr, data);
}

/* Wait up to timeout_ms for a frame in RX FIFO0 and copy it out. data must hold 8 bytes. */
static HAL_StatusTypeDef receive(FDCAN_HandleTypeDef *h, FDCAN_RxHeaderTypeDef *hdr,
                                 uint8_t *data, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) == 0) {
        if (HAL_GetTick() - start > timeout_ms) return HAL_TIMEOUT;
    }
    return HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, hdr, data);
}

static const char *status_str(HAL_StatusTypeDef st)
{
    switch (st) {
    case HAL_OK:      return "OK";
    case HAL_ERROR:   return "error";
    case HAL_BUSY:    return "busy";
    case HAL_TIMEOUT: return "timeout";
    }
    return "?";
}

/* One instance: loopback mode -> filter -> start -> send -> receive -> compare. 0 = PASS. */
static int test_one(FDCAN_HandleTypeDef *h, const char *name)
{
    const uint32_t tx_id = 0x123;
    const uint8_t tx_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    HAL_StatusTypeDef st = set_mode(h, FDCAN_MODE_INTERNAL_LOOPBACK);
    printf("%s loopback mode: %s\r\n", name, status_str(st));
    if (st == HAL_OK) {
        st = accept_all(h);
        printf("%s filter: %s\r\n", name, status_str(st));
    }
    int started = 0;
    if (st == HAL_OK) {
        st = HAL_FDCAN_Start(h);
        printf("%s start: %s\r\n", name, status_str(st));
        started = st == HAL_OK;
    }
    if (st == HAL_OK) {
        st = send(h, tx_id, tx_data);
        printf("%s send: %s\r\n", name, status_str(st));
    }

    int fail = 1;
    if (st == HAL_OK) {
        FDCAN_RxHeaderTypeDef rx;
        uint8_t rx_data[8];
        st = receive(h, &rx, rx_data, WAIT_MS);
        printf("%s receive: %s\r\n", name, status_str(st));
        if (st == HAL_OK) {
            printf("  id 0x%03lX len %lu data %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned long)rx.Identifier, (unsigned long)rx.DataLength,
                   rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                   rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
            fail = rx.Identifier != tx_id || rx.DataLength != FDCAN_DLC_BYTES_8 ||
                   memcmp(rx_data, tx_data, 8) != 0;
        }
    }

    /* Always restore, even after a failure. A failed restore is a FAIL: later CAN code would
     * find the controller in the wrong state. */
    st = restore(h, started);
    printf("%s restore: %s\r\n", name, status_str(st));
    if (st != HAL_OK) fail = 1;

    printf("%s loopback test: %s\r\n", name, fail ? "FAIL" : "PASS");
    return fail;
}

int fdcan_loopback_test(void)
{
    int fails = 0;
    fails += test_one(&hfdcan1, "FDCAN1");
    fails += test_one(&hfdcan2, "FDCAN2");
    fails += test_one(&hfdcan3, "FDCAN3");
    printf("FDCAN loopback: %d/3 PASS\r\n", 3 - fails);
    return fails;
}
