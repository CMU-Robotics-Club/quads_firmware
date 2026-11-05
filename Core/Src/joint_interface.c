#include "joint_interface.h"
#include "fdcan_interface.h"
#include <stdio.h>

void odrive_process_rx_msg(uint8_t joint_index, uint32_t cmd_id, uint8_t *data){
  // data should be 8 long
  // WE SHOULD BE IN THE ISR CONTEXT HERE
  switch (cmd_id) {
    case 0x001:
      printf("Received heartbeat for joint %d\n\r", joint_index);
      return;
    default:
      break;
  }
}

void odrive_process_tx_msg(uint8_t joint_index, uint32_t cmd_id, uint8_t *data){
  // data should be 8 long
  uint32_t node_id;
  switch (joint_index) {
    case KNEE_INDEX: node_id = KNEE_NODE_ID;
    case HIP_INDEX: node_id = HIP_NODE_ID;
  }
  uint32_t can_id = node_id<<5 | cmd_id;
  send_msg_canfd1(can_id, data);
}
