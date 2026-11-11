#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "rtt_log.h"
#include <stdio.h>
#include "joint_interface.h"
#include "joint_control.h"
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

  static TickType_t last_action_tick = 0; // Used in some states to make sure we don't spam the bus with messages
  int loop_iter = 0;
  
  fast_printf("Starting control task...\n\r");

  while(true){
    loop_iter++;
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
          joint->motor_pos_turns = 0.0f;
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
        // uint32_t pos = (uint32_t) raw_joints[1].motor_pos_turns;
        // fast_printf("pos: %lu.%03lu us\r\n",
        //             (unsigned long)(pos / 1000U),
        //             (unsigned long)(pos % 1000U));
        for (int i = 0; i<NUM_JOINTS; i++){
          if(raw_joints[i].joint_initialized == false){
            done=false;
            break;
          }
        }
        sys_state = SYSTEM_STATE_WAIT_CAN;
        if(done){
          fast_printf("All Joints initialized, clearing errors\r\n");
          sys_state = SYSTEM_STATE_CLEAR_ERRORS;
        }
        break;
      
      case SYSTEM_STATE_CLEAR_ERRORS:
        joint_control_feed_watchdog();
        if ((xTaskGetTickCount() - last_action_tick) >= pdMS_TO_TICKS(200)) {
          // Runs every 200 ms
          for (int i = 0; i<NUM_JOINTS; i++){
            odrive_ClearErrors(i, raw_joints);
          }
          last_action_tick = xTaskGetTickCount();
        }
        done = true;
        for (int i = 0; i<NUM_JOINTS; i++){
          if(raw_joints[i].odrive_error != 0){
            done=false;
            // fast_printf("Errors: %b\r\n", (unsigned int)raw_joints[i].odrive_error);
            break;
          }
        }
        sys_state = SYSTEM_STATE_CLEAR_ERRORS;
        if(done){
#if PD_DRY_RUN
          // Motors stay in IDLE: the PD torque is only computed and printed (zero torque is sent)
          fast_printf("Cleared errors, PD dry run: not arming motors\r\n");
          sys_state = SYSTEM_STATE_CALIBRATE_OFFSETS;
#else
          fast_printf("Cleared errors, arming motors\r\n");
          sys_state = SYSTEM_STATE_ARM_MOTORS;
#endif
          last_action_tick = 0; // reset for next state
        }
        break;

      case SYSTEM_STATE_ARM_MOTORS:
        joint_control_feed_watchdog();
        if ((xTaskGetTickCount() - last_action_tick) >= pdMS_TO_TICKS(1000)) {
          // Runs every 200 ms
          for (int i = 0; i<NUM_JOINTS; i++){
            fast_printf("Attempting Arming motors...\r\n");
            odrive_SetAxisState(i, raw_joints, AXIS_STATE_CLOSED_LOOP_CONTROL);
          }
          last_action_tick = xTaskGetTickCount();
        }
        done = true;
        for (int i = 0; i<NUM_JOINTS; i++){
          if(raw_joints[i].odrive_error != 0){
            fast_printf("ODRIVE ERROR JOINT %d: %b\r\n", i, raw_joints[i].odrive_error);
          }
          if(raw_joints[i].odrive_error != 0 || raw_joints[i].axisState != AXIS_STATE_CLOSED_LOOP_CONTROL){
            // fast_printf("Axis state: %d\r\n", raw_joints[i].axisState);
            done=false;
            break;
          }
        }
        sys_state = SYSTEM_STATE_ARM_MOTORS;
        if(done){
          fast_printf("Armed motors, moving to calibrating offsets\r\n");
          last_action_tick = 0;
          sys_state = SYSTEM_STATE_CALIBRATE_OFFSETS;
        }
        break;

      case SYSTEM_STATE_CALIBRATE_OFFSETS:
        joint_control_feed_watchdog();
        // TODO: Ideally, we would wait until a usb command
        // is received that lets us know that the leg is ready for calibration
        
        bool all_calibrated = true;
        for (int i = 0; i<NUM_JOINTS; i++){
          // Check that they aren't uninitialized
          float pos = raw_joints[i].motor_pos_turns;
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
          fast_printf("All calibrated, running loop\r\n");
          joint_control_hold(raw_joints); // target = where the leg is now
          sys_state = SYSTEM_STATE_RUNNING;
        }
        break;

      case SYSTEM_STATE_RUNNING: {
        for(int i = 0; i<NUM_JOINTS; i++){
          // check timeouts
          if((uint32_t)xLastWakeTime > raw_joints[i].last_msg_tick + pdMS_TO_TICKS(20)){
            fast_printf("JOINT TIMEOUT: JOINT %d\r\n", i);
            fast_printf("Time since last is %d\r\n", xLastWakeTime - raw_joints[i].last_msg_tick);
            sys_state = SYSTEM_STATE_FAULT;
            break;
          }
          // if((uint32_t)xLastWakeTime > joint_command[i].last_msg_tick + pdMS_TO_TICKS(20)){
          //   fast_printf("COMMAND TIMEOUT: JOINT %d\r\n", i);
          //   sys_state = SYSTEM_STATE_FAULT;
          //   break;
          // }
          if(raw_joints[i].odrive_error != 0){
            fast_printf("ERROR on joint %d: %d\r\n", i, (unsigned int) raw_joints[i].odrive_error);
            sys_state = SYSTEM_STATE_FAULT;
            break;
          }
        }
        // The breaks above only leave the for loop: don't command torque after a fault
        if(sys_state != SYSTEM_STATE_RUNNING){
          break;
        }
        joint_control_step(raw_joints, loop_iter%100 == 0);

        // float hip_pos = raw_joints[HIP_INDEX].motor_pos_turns / 14.0f;
        // float hip_vel = raw_joints[HIP_INDEX].motor_vel_turns / 14.0f;
        // float knee_pos = raw_joints[KNEE_INDEX].motor_pos_turns / 14.0f;
        // float knee_vel = raw_joints[KNEE_INDEX].motor_vel_turns / 14.0f;
        float hip_pos = raw_joints[HIP_INDEX].motor_pos_turns;
        float hip_vel = raw_joints[HIP_INDEX].motor_vel_turns;
        float knee_pos = raw_joints[KNEE_INDEX].motor_pos_turns;
        float knee_vel = raw_joints[KNEE_INDEX].motor_vel_turns;
        if(loop_iter%100 == 0){
          print_float("hip_pos:", hip_pos);
          print_float("hip_vel:", hip_vel);
          print_float("knee_pos:", knee_pos);
          print_float("knee_vel:", knee_vel);
        }
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
