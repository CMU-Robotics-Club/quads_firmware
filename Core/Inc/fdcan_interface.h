/*
 * FDCAN1-3 bring-up test (on-chip CAN controllers, no transceiver needed).
 *
 * Bit timing, pins and Message RAM come from CubeMX (fdcan.c).
 * See doc/FDCAN_NOTES.md for how each value was chosen.
 */
#pragma once

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
