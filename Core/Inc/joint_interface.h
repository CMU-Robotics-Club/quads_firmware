#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NUM_JOINTS 2 // 0: Hip, 1: Knee
#define KNEE_INDEX 1
#define HIP_INDEX  0
#define KNEE_NODE_ID 0 // TODO: Match the real one
#define HIP_NODE_ID 1

// Current state
typedef enum {
    SYSTEM_STATE_INIT = 0,
    SYSTEM_STATE_WAIT_CAN,
    SYSTEM_STATE_CLEAR_ERRORS,
    SYSTEM_STATE_ARM_MOTORS,
    SYSTEM_STATE_CALIBRATE_OFFSETS,
    SYSTEM_STATE_RUNNING,
    SYSTEM_STATE_FAULT
} SystemState_t;

typedef struct {
    float torque_limit;
    uint8_t can_id;
    float motor_pos_rad;
    float motor_vel_rad;
    float motor_torque_est;
    float initial_pos_offset;
    uint32_t odrive_error;
    uint8_t odrive_state;
    uint32_t last_msg_tick; // For watchdog
    bool joint_initialized;
    bool invert_direction;
} JointRawFeedback_t;

typedef struct {
    float target_pos_rad;
    float target_vel_rad;
    uint32_t last_msg_tick;
} JointCommand_t;

extern volatile JointRawFeedback_t g_raw_joints[NUM_JOINTS];
extern volatile JointCommand_t g_joint_command[NUM_JOINTS];
