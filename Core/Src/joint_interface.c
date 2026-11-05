#include "joint_interface.h"
#include "fdcan_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

void odrive_process_rx_msg(uint8_t joint_index, uint32_t cmd_id, uint8_t *data, uint32_t length){
  // data should be 8 long
  // WE SHOULD BE IN THE ISR CONTEXT HERE
  // Only used by fdcan_interface
  switch (cmd_id) {
    case 0x001:
      printf("Received heartbeat for joint %d\n\r", joint_index);
      g_raw_joints[joint_index].last_msg_tick = xTaskGetTickCountFromISR();
      return;
    default:
      break;
  }
}

void odrive_process_tx_msg(uint8_t joint_index, uint32_t cmd_id, uint8_t *data, uint32_t length){
  // data should be 8 long
  uint32_t node_id;
  switch (joint_index) {
    case KNEE_INDEX: 
      node_id = KNEE_NODE_ID;
      break;
    case HIP_INDEX: 
      node_id = HIP_NODE_ID;
      break;
    default:
      printf("UNKNOWN JOINT: %d\r\n", joint_index);
      return;
  }
  uint32_t can_id = node_id<<5 | cmd_id;
  send_msg_canfd1(can_id, data, length);
}

void initialize_all_buses(){
  initialize_CAN_buses();
  // In the future, we would initialize the SPI bus as well
}


void odrive_SetAxisState(uint8_t bus_num, uint8_t can_id, AxisState state){
  if(bus_num == 1){
    //TODO
  }
  else{
    printf("ERROR: UNSUPPORTED BUS NUMBER: %d\r\n", bus_num);
  }
}

void ODrive_ClearErrors(uint8_t bus_num, uint8_t can_id){
  
}

