/*
 * FDCAN1-3 bring-up test (on-chip CAN controllers, no transceiver needed).
 *
 * Bit timing, pins and Message RAM come from CubeMX (fdcan.c).
 * See doc/FDCAN_NOTES.md for how each value was chosen.
 */
#pragma once

#include <stdint.h>

#include "fdcan.h"

/* Internal loopback on FDCAN1, 2 and 3: send one frame on each and read it back.
 * Prints each step. Restores Normal mode afterwards (controllers left stopped).
 * Returns the number of instances that failed (0 = all PASS). */
int fdcan_loopback_test(void);
int fdcan_echo_test(void);

/* ODrive heartbeat path on FDCAN1 internal loopback: hardware filter (heartbeat of any node,
 * everything else rejected) + odrive_parse_heartbeat(). Sends fake hip/knee heartbeats and
 * one encoder frame that must be dropped. Prints each step. Restores like above.
 * Returns 0 on PASS, 1 on FAIL. */
int fdcan_heartbeat_test(void);

/* Real bus: Normal mode (we ACK, never transmit), heartbeat filter, poll RX FIFO0 for ms
 * milliseconds and print node / state / error for each heartbeat, then the protocol status
 * and error counters so a wiring or termination problem shows up as numbers.
 * Needs a transceiver and a live bus. Restores like above.
 * Returns the number of heartbeat frames received, or -1 if setup failed. */
int fdcan_listen_heartbeats(FDCAN_HandleTypeDef *h, uint32_t ms);
