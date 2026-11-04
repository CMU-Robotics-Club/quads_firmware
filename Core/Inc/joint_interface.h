#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NUM_JOINTS 2 // 0: Hip, 1: Knee
#define KNEE_INDEX 1
#define HIP_INDEX  0

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
    uint8_t can_id;
    bool invert_direction;
    float torque_limit;
    float backlash_rad;
    float gear_ratio; // e.g., 14.0f
} JointConfig_t;

typedef struct {
    float motor_pos_rad;
    float motor_vel_rad;
    float motor_torque_est;
    float initial_pos_offset;
    uint32_t odrive_error;
    uint8_t odrive_state;
    uint32_t last_msg_tick; // For watchdog
    bool joint_initialized;
} JointRawFeedback_t;

typedef struct {
    float target_pos_rad;
    float target_vel_rad;
    uint32_t last_msg_tick;
} JointCommand_t;
