#include "odrive_can.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  uint8_t node;
  odrive_heartbeat_t hb;

  /* ID helpers: 6-bit node, 5-bit cmd. */
  assert(odrive_make_id(1, ODRIVE_CMD_HEARTBEAT) == 0x021);
  assert(odrive_make_id(2, ODRIVE_CMD_HEARTBEAT) == 0x041);
  assert(odrive_make_id(1, ODRIVE_CMD_GET_ENCODER) == 0x029);
  assert(odrive_node_id(0x7E1) == 63 && odrive_cmd_id(0x7E1) == 1);
  assert(odrive_make_id(1, 0xFF) == 0x03F); /* cmd truncated to 5 bits */

  /* Hip (node 1), no error, CLOSED_LOOP_CONTROL, trajectory_done set. */
  const uint8_t hip[8] = {0, 0, 0, 0, 8, 0, 1, 0};
  memset(&hb, 0xAA, sizeof(hb));
  assert(odrive_parse_heartbeat(0x021, hip, 8, &node, &hb) == 0);
  assert(node == 1);
  assert(hb.axis_error == 0);
  assert(hb.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL);
  assert(hb.procedure_result == 0);
  assert(hb.trajectory_done == 1);

  /* Knee (node 2): error 0x12345678 little-endian, IDLE, procedure_result 3,
   * byte 6 upper bits must be ignored (only bit 0 is trajectory_done). */
  const uint8_t knee[8] = {0x78, 0x56, 0x34, 0x12, 1, 3, 0xFE, 0xFF};
  assert(odrive_parse_heartbeat(0x041, knee, 8, &node, &hb) == 0);
  assert(node == 2);
  assert(hb.axis_error == 0x12345678);
  assert(hb.axis_state == ODRIVE_AXIS_STATE_IDLE);
  assert(hb.procedure_result == 3);
  assert(hb.trajectory_done == 0);

  /* Rejects: wrong cmd_id (encoder frame), short frame, NULL. Outputs untouched. */
  node = 0x55;
  memset(&hb, 0xAA, sizeof(hb));
  assert(odrive_parse_heartbeat(0x029, hip, 8, &node, &hb) == -1);
  assert(odrive_parse_heartbeat(0x021, hip, 7, &node, &hb) == -1);
  assert(odrive_parse_heartbeat(0x021, hip, 0, &node, &hb) == -1);
  assert(odrive_parse_heartbeat(0x021, NULL, 8, &node, &hb) == -1);
  assert(node == 0x55 && hb.axis_state == 0xAA);

  /* Longer than 8 is accepted (CAN FD could pad); only the first 8 bytes matter. */
  assert(odrive_parse_heartbeat(0x021, hip, 12, &node, &hb) == 0);

  /* State names. */
  assert(strcmp(odrive_axis_state_str(0), "UNDEFINED") == 0);
  assert(strcmp(odrive_axis_state_str(1), "IDLE") == 0);
  assert(strcmp(odrive_axis_state_str(8), "CLOSED_LOOP_CONTROL") == 0);
  assert(strcmp(odrive_axis_state_str(14), "ANTICOGGING_CAL") == 0);
  assert(strcmp(odrive_axis_state_str(16), "HARMONIC_CAL_COMM") == 0);
  assert(strcmp(odrive_axis_state_str(5), "?") == 0);
  assert(strcmp(odrive_axis_state_str(17), "?") == 0);
  assert(strcmp(odrive_axis_state_str(0xFF), "?") == 0);

  puts("test_odrive_can: PASS");
  return 0;
}
