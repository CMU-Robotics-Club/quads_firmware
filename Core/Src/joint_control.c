#include "joint_control.h"

#include "joint_units.h"
#include "pd_controller.h"
#include "rtt_log.h"

/* TBD: 14 is what Laika-Software assumes (laika_hardware_interface.cpp divides by 14).
 * Not measured on the leg yet. */
#define JOINT_GEAR_RATIO 14.0f

/* ODrive CAN-Simple, checked against the ODrive 0.6.12 CAN protocol page. */
#define ODRIVE_CMD_SET_CONTROLLER_MODE 0x0B /* 0: control_mode u32, 4: input_mode u32 */
#define ODRIVE_CMD_SET_INPUT_TORQUE    0x0E /* 0: torque float32, Nm */
/* Enum values from Laika-Software odrive_enums.h; TBD: confirm on the ODrives (fw 0.6.11). */
#define ODRIVE_CONTROL_MODE_TORQUE     1U
#define ODRIVE_INPUT_MODE_PASSTHROUGH  1U

/* TBD: first-test values, not tuned. Joint side: Nm/rad, Nm/(rad/s), Nm.
 * kp = 0 gives damping only, which is the first thing to try on the leg. */
static const pd_gains_t gains[NUM_JOINTS] = {
    [KNEE_INDEX] = {.kp = 0.0f, .kd = 0.2f, .tau_max = 1.0f},
    [HIP_INDEX]  = {.kp = 0.0f, .kd = 0.2f, .tau_max = 1.0f},
};

static const char *const q_names[NUM_JOINTS]   = {[KNEE_INDEX] = "knee q", [HIP_INDEX] = "hip q"};
static const char *const qd_names[NUM_JOINTS]  = {[KNEE_INDEX] = "knee qd", [HIP_INDEX] = "hip qd"};
static const char *const tau_names[NUM_JOINTS] = {[KNEE_INDEX] = "knee tau", [HIP_INDEX] = "hip tau"};

static float q_des[NUM_JOINTS]; /* target angle, rad */

/* invert_direction flips the joint's positive direction: feedback here, torque on the way out. */
static float direction(const JointRawFeedback_t *joint)
{
    return joint->invert_direction ? -1.0f : 1.0f;
}

static float joint_q(const JointRawFeedback_t *joint)
{
    return direction(joint) *
           joint_pos_rad(joint->motor_pos_turns, joint->initial_pos_offset, JOINT_GEAR_RATIO);
}

static float joint_qd(const JointRawFeedback_t *joint)
{
    return direction(joint) * joint_vel_rad_s(joint->motor_vel_turns, JOINT_GEAR_RATIO);
}

void joint_control_hold(const JointRawFeedback_t *joints)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        q_des[i] = joint_q(&joints[i]);
#if !PD_DRY_RUN
        uint32_t mode[2] = {ODRIVE_CONTROL_MODE_TORQUE, ODRIVE_INPUT_MODE_PASSTHROUGH};
        odrive_process_tx_msg(i, ODRIVE_CMD_SET_CONTROLLER_MODE, (uint8_t *)mode, sizeof(mode));
#endif
    }
}

void joint_control_step(const JointRawFeedback_t *joints, bool print)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        float q = joint_q(&joints[i]);
        float qd = joint_qd(&joints[i]);
        float tau = pd_torque(&gains[i], q_des[i], q, 0.0f, qd, 0.0f);

#if !PD_DRY_RUN
        float tau_motor = direction(&joints[i]) * motor_torque_nm(tau, JOINT_GEAR_RATIO);
        odrive_process_tx_msg(i, ODRIVE_CMD_SET_INPUT_TORQUE, (uint8_t *)&tau_motor,
                              sizeof(tau_motor));
#endif

        if (print) {
            print_float(q_names[i], q);
            print_float(qd_names[i], qd);
            print_float(tau_names[i], tau);
        }
    }
}
