#include "joint_interface.h"
#include "fdcan_interface.h"
#include "rtt_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

void odrive_process_rx_msg(uint8_t joint_index, uint32_t cmd_id, uint8_t *data, uint32_t length){
  // data should be 8 long
  // WE SHOULD BE IN THE ISR CONTEXT HERE
  // Only used by fdcan_interface
  if (data == NULL || length < 8) {
    fast_printf("ERROR: BAD DATA\r\n");
    return;
  }
  if (joint_index >= NUM_JOINTS) {
    fast_printf("ERROR: RECEIVED CAN MESSAGE WITH OOB INDEX: %d\r\n", joint_index);
    return;
  }

  UBaseType_t uxSavedInterruptStatus;
  switch (cmd_id) {
    case 0x001:
      fast_printf("Received heartbeat for joint %d\n\r", joint_index);
      uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
      g_raw_joints[joint_index].last_msg_tick = xTaskGetTickCountFromISR();
      g_raw_joints[joint_index].joint_initialized = true;
      memcpy((void*)&g_raw_joints[joint_index].odrive_error, data, sizeof(uint32_t));
      g_raw_joints[joint_index].axisState = data[4];
      taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
      break;
    case 0x09:
      uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
      memcpy((void*)&g_raw_joints[joint_index].motor_pos_turns, data, 4);
      memcpy((void*)&g_raw_joints[joint_index].motor_vel_turns, &data[4], 4);
      taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
      break;
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
      fast_printf("UNKNOWN JOINT: %d\r\n", joint_index);
      return;
  }
  uint32_t can_id = (node_id << 5) | (cmd_id & 0x1F);
  send_msg_canfd1(can_id, data, length);
}

void initialize_all_buses(){
  initialize_CAN_buses();
  // In the future, we would initialize the SPI bus as well
}

void odrive_SetAxisState(uint8_t joint_index, JointRawFeedback_t *joints, AxisState state){
  uint32_t cmd = 0x07;
  uint32_t state_val = (uint32_t)state;
  odrive_process_tx_msg(joint_index, cmd, (uint8_t*) &state_val, 4);
}

void odrive_ClearErrors(uint8_t joint_index, JointRawFeedback_t *joints){
  uint32_t cmd = 0x18;
  uint8_t data[1] = {1}; // 1 should flash the led on the odrive
  odrive_process_tx_msg(joint_index, cmd, data, 1);
}

