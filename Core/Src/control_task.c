#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "rtt_log.h"
#include <stdio.h>
#include "joint_interface.h"
#include <string.h>

// volatile since ISRs can overwrite these vars
// So we don't want the cpu to cache it locally
// we instead want it to fetch directly from ram
volatile JointRawFeedback_t g_raw_joints[NUM_JOINTS];
volatile JointCommand_t g_joint_command[NUM_JOINTS];
static SystemState_t sys_state = SYSTEM_STATE_INIT; // Current system state (see joint_interface.h)

extern void StartControlTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 ms period = 1 kHz

  fast_printf("Starting control task...\n\r");

  while(true){
    // continue;
    // next time this task is called is offsetted from the last wake time
    BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (xWasDelayed == pdFALSE) {
      fast_printf("OVERRUN DETECTED\r\n");
    }

    // variable to copy the current state of stuff
    JointRawFeedback_t raw_joints[NUM_JOINTS];
    JointCommand_t joint_command[NUM_JOINTS];
    taskENTER_CRITICAL();
    memcpy(raw_joints, (void*)g_raw_joints, sizeof(raw_joints));
    memcpy(joint_command, (void*)g_joint_command, sizeof(joint_command));
    taskEXIT_CRITICAL();

    switch (sys_state) {
      case SYSTEM_STATE_INIT:
        for(int i = 0; i<NUM_JOINTS; i++){
          JointRawFeedback_t *joint = &raw_joints[i];
          joint->joint_initialized = false;
          joint->motor_pos_rad = 0.0f;
          joint->invert_direction = false;
          joint->last_msg_tick = 0;
          if(i == HIP_INDEX){
            joint->node_id = HIP_NODE_ID;
          }
          else if(i == KNEE_INDEX){
            joint->node_id = KNEE_NODE_ID;
          }
          else{
            fast_printf("BAD UNKNOWN INDEX: %d\n\r", i);
            break;
          }
        }
        taskENTER_CRITICAL();
        memcpy((void*) g_raw_joints, raw_joints, sizeof(g_raw_joints));
        taskEXIT_CRITICAL();
        initialize_all_buses();
        sys_state = SYSTEM_STATE_WAIT_CAN;
        break;

      case SYSTEM_STATE_WAIT_CAN:
        bool done = true;
        for (int i = 0; i<NUM_JOINTS; i++){
          if(raw_joints[i].joint_initialized == false){
            done=false;
            break;
          }
        }
        sys_state = SYSTEM_STATE_WAIT_CAN;
        if(done){
          printf("All Joints initialized!");
          sys_state = SYSTEM_STATE_CLEAR_ERRORS;
        }
        break;
      
      case SYSTEM_STATE_CLEAR_ERRORS:
        odrive_ClearErrors(KNEE_INDEX, raw_joints);
        odrive_ClearErrors(HIP_INDEX, raw_joints);
        sys_state = SYSTEM_STATE_ARM_MOTORS;
        break;

      case SYSTEM_STATE_ARM_MOTORS:
        for (int i = 0; i<NUM_JOINTS; i++){
          odrive_SetAxisState(i, raw_joints, AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        sys_state = SYSTEM_STATE_CALIBRATE_OFFSETS;
        break;

      case SYSTEM_STATE_CALIBRATE_OFFSETS:
        // TODO: Ideally, we would wait until a usb command
        // is received that lets us know that the leg is ready for calibration
        
        bool all_calibrated = true;
        for (int i = 0; i<NUM_JOINTS; i++){
          // Check that they aren't uninitialized
          float pos = raw_joints[i].motor_pos_rad;
          if(pos!=0.0f){
            raw_joints[i].initial_pos_offset = pos;
          }
          else{
            all_calibrated = false;
          }
          taskENTER_CRITICAL();
          g_raw_joints[i].initial_pos_offset = raw_joints[i].initial_pos_offset;
          taskEXIT_CRITICAL();
        }
        sys_state = SYSTEM_STATE_CALIBRATE_OFFSETS;
        if(all_calibrated){
          sys_state = SYSTEM_STATE_RUNNING;
        }
        break;

      case SYSTEM_STATE_RUNNING: {
        for(int i = 0; i<NUM_JOINTS; i++){
          // check 10 ticks
          if((uint32_t)xLastWakeTime > raw_joints[i].last_msg_tick + 5){
            fast_printf("JOINT TIMEOUT: JOINT %d\r\n", i);
            sys_state = SYSTEM_STATE_FAULT;
            break;
          }
          if((uint32_t)xLastWakeTime > joint_command[i].last_msg_tick + 100){
            fast_printf("COMMAND TIMEOUT: JOINT %d\r\n", i);
            sys_state = SYSTEM_STATE_FAULT;
            break;
          }
        }

        float hip_pos = raw_joints[HIP_INDEX].motor_pos_rad / 14.0f;
        float hip_vel = raw_joints[HIP_INDEX].motor_vel_rad / 14.0f;
        float knee_pos = raw_joints[KNEE_INDEX].motor_pos_rad / 14.0f;
        float knee_vel = raw_joints[KNEE_INDEX].motor_vel_rad / 14.0f;
        fast_printf("hip_pos: %f\r\n", hip_pos);
        fast_printf("hip_vel: %f\r\n", hip_vel);
        fast_printf("knee_pos: %f\r\n", knee_pos);
        fast_printf("knee_vel: %f\r\n", knee_vel);
        break;
      }

      case SYSTEM_STATE_FAULT:
      default:
        for (int i = 0; i<NUM_JOINTS; i++){
          odrive_SetAxisState(i, raw_joints, AXIS_STATE_IDLE);
        }
        break;
    }
  }
}
