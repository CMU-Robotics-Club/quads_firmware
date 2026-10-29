/*
 * FDCAN1-3 bring-up test (on-chip CAN controllers, no transceiver needed).
 *
 * Bit timing, pins and Message RAM come from CubeMX (fdcan.c).
 * See doc/FDCAN_NOTES.md for how each value was chosen.
 */
#ifndef FDCAN_TEST_H
#define FDCAN_TEST_H

/* Internal loopback on FDCAN1, 2 and 3: send one frame on each and read it back.
 * Prints each step. Restores Normal mode afterwards (controllers left stopped).
 * Returns the number of instances that failed (0 = all PASS). */
int fdcan_loopback_test(void);
int fdcan_echo_test(void);

#endif /* FDCAN_TEST_H */
