/*
 * FDCAN1-3 bring-up test.
 *
 * Internal loopback: the controller receives its own frames inside the chip.
 * The TX pin stays recessive and the RX pin is ignored, so no transceiver or wiring is needed.
 */
#include "fdcan_interface.h"

#include <stdio.h>
#include <string.h>

#include "fdcan.h"
#include "joint_interface.h"
#include "odrive_can.h"

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

/* Filter 0: ID 0x001 mask 0x01F = compare only the 5 cmd_id bits, so the heartbeat (cmd 0x01)
 * of any node goes to RX FIFO0 and every other command is rejected by the global filter. */
static HAL_StatusTypeDef heartbeat_filter(FDCAN_HandleTypeDef *h)
{
    FDCAN_FilterTypeDef f = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = ODRIVE_HEARTBEAT_FILTER_ID,   /* filter */
        .FilterID2    = ODRIVE_HEARTBEAT_FILTER_MASK, /* mask */
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

/* Send one fake heartbeat in loopback and check it comes back and parses as expected.
 * expect_error catches byte-order bugs when non-zero. 0 = PASS. */
static int heartbeat_roundtrip(FDCAN_HandleTypeDef *h, const char *name, uint8_t node,
                               uint32_t expect_error, uint8_t expect_state)
{
    const uint32_t id = odrive_make_id(node, ODRIVE_CMD_HEARTBEAT);
    const uint8_t tx[8] = {
        (uint8_t)expect_error, (uint8_t)(expect_error >> 8),
        (uint8_t)(expect_error >> 16), (uint8_t)(expect_error >> 24),
        expect_state, 0, 0, 0,
    };

    HAL_StatusTypeDef st = send(h, id, tx);
    printf("%s send 0x%03lX: %s\r\n", name, (unsigned long)id, status_str(st));
    if (st != HAL_OK) return 1;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t rx_data[8];
    st = receive(h, &rx, rx_data, WAIT_MS);
    printf("%s receive: %s\r\n", name, status_str(st));
    if (st != HAL_OK) return 1;

    uint8_t rx_node;
    odrive_heartbeat_t hb;
    if (odrive_parse_heartbeat(rx.Identifier, rx_data, rx.DataLength, &rx_node, &hb) != 0) {
        printf("  id 0x%03lX len %lu: not a heartbeat\r\n",
               (unsigned long)rx.Identifier, (unsigned long)rx.DataLength);
        return 1;
    }
    printf("  node %u state %u (%s) error 0x%08lX\r\n", rx_node, hb.axis_state,
           odrive_axis_state_str(hb.axis_state), (unsigned long)hb.axis_error);
    return rx_node != node || hb.axis_state != expect_state || hb.axis_error != expect_error;
}

/* Send a non-heartbeat frame in loopback and check the filter drops it: a timeout is the PASS.
 * 0 = PASS. */
static int heartbeat_reject(FDCAN_HandleTypeDef *h, const char *name, uint32_t id)
{
    const uint8_t tx[8] = {0};

    HAL_StatusTypeDef st = send(h, id, tx);
    printf("%s send 0x%03lX: %s\r\n", name, (unsigned long)id, status_str(st));
    if (st != HAL_OK) return 1;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t rx_data[8];
    st = receive(h, &rx, rx_data, WAIT_MS);
    printf("%s receive: %s (expect timeout)\r\n", name, status_str(st));
    if (st == HAL_OK) {
        printf("  id 0x%03lX got through the filter\r\n", (unsigned long)rx.Identifier);
    }
    return st != HAL_TIMEOUT;
}

int fdcan_heartbeat_test(void)
{
    FDCAN_HandleTypeDef *h = &hfdcan1;
    const char *name = "FDCAN1 heartbeat";

    HAL_StatusTypeDef st = set_mode(h, FDCAN_MODE_INTERNAL_LOOPBACK);
    printf("%s loopback mode: %s\r\n", name, status_str(st));
    if (st == HAL_OK) {
        st = heartbeat_filter(h);
        printf("%s filter: %s\r\n", name, status_str(st));
    }
    int started = 0;
    if (st == HAL_OK) {
        st = HAL_FDCAN_Start(h);
        printf("%s start: %s\r\n", name, status_str(st));
        started = st == HAL_OK;
    }

    int fail = 1;
    if (st == HAL_OK) {
        fail = 0;
        /* Hip node 1, no error, closed loop. */
        fail |= heartbeat_roundtrip(h, name, 1, 0, ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
        /* Knee node 2 with a non-zero error: wrong byte order would show as 0x78563412. */
        fail |= heartbeat_roundtrip(h, name, 2, 0x12345678, ODRIVE_AXIS_STATE_IDLE);
        /* Encoder frame from node 1 must not pass the heartbeat filter. */
        fail |= heartbeat_reject(h, name, odrive_make_id(1, ODRIVE_CMD_GET_ENCODER));
    }

    st = restore(h, started);
    printf("%s restore: %s\r\n", name, status_str(st));
    if (st != HAL_OK) fail = 1;

    printf("%s test: %s\r\n", name, fail ? "FAIL" : "PASS");
    return fail;
}

static const char *lec_str(uint32_t lec)
{
    switch (lec) {
    case FDCAN_PROTOCOL_ERROR_NONE:      return "none";
    case FDCAN_PROTOCOL_ERROR_STUFF:     return "stuff";
    case FDCAN_PROTOCOL_ERROR_FORM:      return "form";
    case FDCAN_PROTOCOL_ERROR_ACK:       return "ack";
    case FDCAN_PROTOCOL_ERROR_BIT1:      return "bit1";
    case FDCAN_PROTOCOL_ERROR_BIT0:      return "bit0";
    case FDCAN_PROTOCOL_ERROR_CRC:       return "crc";
    case FDCAN_PROTOCOL_ERROR_NO_CHANGE: return "no change";
    }
    return "?";
}

/* Bus health as numbers, so a wiring / termination / bit-timing problem shows up even when
 * no frame arrives. Rough reading: stuff/form/crc errors with REC climbing = bit timing or
 * noise; REC stuck at 0 with no frames = nothing on the bus (wiring, no 12 V, autobaud node
 * waiting); ErrorPassive / BusOff cannot happen here since we never transmit. */
static void print_bus_status(FDCAN_HandleTypeDef *h, const char *name)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef ec;
    if (HAL_FDCAN_GetProtocolStatus(h, &ps) != HAL_OK || HAL_FDCAN_GetErrorCounters(h, &ec) != HAL_OK) {
        printf("%s status: read failed\r\n", name);
        return;
    }
    printf("%s status: last error %s, warning %lu, error passive %lu, bus off %lu\r\n", name,
           lec_str(ps.LastErrorCode), (unsigned long)ps.Warning,
           (unsigned long)ps.ErrorPassive, (unsigned long)ps.BusOff);
    printf("%s counters: TEC %lu REC %lu (passive %lu) logged %lu\r\n", name,
           (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
           (unsigned long)ec.RxErrorPassive, (unsigned long)ec.ErrorLogging);
}

int fdcan_listen_heartbeats(FDCAN_HandleTypeDef *h, uint32_t ms)
{
    const char *name = "listen";

    /* Normal mode, not bus monitoring: a listen-only node never ACKs, so the ODrive would see
     * ACK errors and retransmit forever. We ACK but never transmit a frame of our own. */
    HAL_StatusTypeDef st = set_mode(h, FDCAN_MODE_NORMAL);
    printf("%s normal mode: %s\r\n", name, status_str(st));
    if (st == HAL_OK) {
        st = heartbeat_filter(h);
        printf("%s filter: %s\r\n", name, status_str(st));
    }
    int started = 0;
    if (st == HAL_OK) {
        st = HAL_FDCAN_Start(h);
        printf("%s start: %s\r\n", name, status_str(st));
        started = st == HAL_OK;
    }

    int count = -1;
    if (st == HAL_OK) {
        count = 0;
        printf("%s %lu ms on 1 Mbps, heartbeats only (filter 0x001/0x01F)\r\n", name, (unsigned long)ms);
        uint32_t start = HAL_GetTick();
        while (HAL_GetTick() - start < ms) {
            FDCAN_RxHeaderTypeDef rx;
            uint8_t rx_data[8];
            /* Short timeout so the loop stays responsive to the overall deadline. */
            if (receive(h, &rx, rx_data, 1) != HAL_OK) continue;
            count++;
            uint8_t node;
            odrive_heartbeat_t hb;
            if (odrive_parse_heartbeat(rx.Identifier, rx_data, rx.DataLength, &node, &hb) != 0) {
                /* Filter passed it, so this is a heartbeat with DLC < 8: print raw. */
                printf("  id 0x%03lX len %lu: not a heartbeat\r\n",
                       (unsigned long)rx.Identifier, (unsigned long)rx.DataLength);
                continue;
            }
            printf("  node %u state %u (%s) error 0x%08lX proc %u traj %u\r\n", node,
                   hb.axis_state, odrive_axis_state_str(hb.axis_state),
                   (unsigned long)hb.axis_error, hb.procedure_result, hb.trajectory_done);
        }
        printf("%s done: %d heartbeat frames in %lu ms\r\n", name, count, (unsigned long)ms);
        print_bus_status(h, name);
    }

    st = restore(h, started);
    printf("%s restore: %s\r\n", name, status_str(st));
    return count;
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

int fdcan_echo_test(void)
{
  const char *name = "FDCAN1";
  FDCAN_HandleTypeDef *h = &hfdcan1;
  HAL_StatusTypeDef st;

  printf("STARTING THE FDCAN ECHO");

  /* Set to normal physical bus mode */
  st = set_mode(h, FDCAN_MODE_NORMAL);
  printf("%s mode normal: %s\r\n", name, status_str(st));
  if (st != HAL_OK) return 1;

  /* Configure filter to accept standard frames into FIFO0 */
  st = accept_all(h);
  printf("%s filter: %s\r\n", name, status_str(st));
  if (st != HAL_OK) return 1;

  /* Enable the RX FIFO0 New Message interrupt notification */
  st = HAL_FDCAN_ConfigInterruptLines(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
  if (st != HAL_OK) return 1;
  st = HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  printf("%s activate notification: %s\r\n", name, status_str(st));
  if (st != HAL_OK) return 1;

  

  /* Start the peripheral */
  st = HAL_FDCAN_Start(h);
  printf("%s start: %s\r\n", name, status_str(st));
  if (st != HAL_OK) return 1;

  printf("done with setup of %s\r\n", name);

  return 0;
}

// Callback 
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  printf("CAN msg received!\r\n");
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    // new message arrived
    FDCAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx_data[8];

    // getting the 
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK) {
      
      if((rx_hdr.Identifier & (~((1U << 11) - 1))) != 0){
        printf("CAN MSG ERR: UNKNOWN CAN ID: %d\r\n", rx_hdr.Identifier);
      }
      uint32_t node_id = rx_hdr.Identifier >> 5;
      uint32_t cmd_id = rx_hdr.Identifier & 0b11111;
      uint8_t joint_index;
      switch (node_id) {
        case KNEE_NODE_ID:
          joint_index = KNEE_INDEX;
          break;
        case HIP_NODE_ID:
          joint_index = HIP_INDEX;
          break;
        default:
          printf("CAN MSG ERR: UNKNOWN NODE ID: %d\r\n", node_id);
          break;
      }
      
      // THIS IS SET UP TO ECHO BACK TO THE BUS
      // Copying message header for transmitting
      // FDCAN_TxHeaderTypeDef tx_hdr = {
      //     .Identifier          = rx_hdr.Identifier + 1, // Echo with ID+1 or same ID
      //     .IdType              = rx_hdr.IdType,
      //     .TxFrameType         = FDCAN_DATA_FRAME,
      //     .DataLength          = rx_hdr.DataLength,
      //     .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      //     .BitRateSwitch       = FDCAN_BRS_OFF,
      //     .FDFormat            = rx_hdr.FDFormat,
      //     .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
      //     .MessageMarker       = 0,
      // };
      // // Add the message to the transmit fifo
      // HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_hdr, rx_data);
    }
    else {
      printf("Failed here idk why this would happen gl debugging this if you see this");
    }
  }
}


