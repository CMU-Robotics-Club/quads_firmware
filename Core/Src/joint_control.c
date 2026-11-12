#include "joint_control.h"

#include "joint_units.h"
#include "rtt_log.h"

#include <math.h>
#include <stddef.h>

/* 14 is what Laika-Software assumes (laika_hardware_interface.cpp divides by 14).
 * Measured on the test leg by turning each output about 90 degrees by hand: knee 13.9, hip 13.6. */
#define JOINT_GEAR_RATIO 14.0f

/* ODrive CAN-Simple, checked against the ODrive 0.6.12 CAN protocol page. */
#define ODRIVE_CMD_SET_CONTROLLER_MODE 0x0B /* 0: control_mode u32, 4: input_mode u32 */
#define ODRIVE_CMD_SET_INPUT_TORQUE    0x0E /* 0: torque float32, Nm */
/* Enum values from Laika-Software odrive_enums.h; both ODrives read back control_mode 1 and
 * input_mode 1 in odrivetool. */
#define ODRIVE_CONTROL_MODE_TORQUE     1U
#define ODRIVE_INPUT_MODE_PASSTHROUGH  1U

/* TBD: first-test values, not tuned. Joint side: Nm/rad, Nm/(rad/s), Nm.
 * kp = 0 gives damping only, which is the first thing to try on the leg. */
static const pd_gains_t gains[NUM_JOINTS] = {
  [KNEE_INDEX] = {.kp = 180.0f, .kd = 2.8f, .tau_max = 1.5f},
  [HIP_INDEX]  = {.kp = 180.0f, .kd = 2.8f, .tau_max = 1.5f},
};

static const char *const q_names[NUM_JOINTS]   = {[KNEE_INDEX] = "knee q", [HIP_INDEX] = "hip q"};
static const char *const qd_names[NUM_JOINTS]  = {[KNEE_INDEX] = "knee qd", [HIP_INDEX] = "hip qd"};
static const char *const tau_names[NUM_JOINTS] = {[KNEE_INDEX] = "knee tau", [HIP_INDEX] = "hip tau"};

static float q_des[NUM_JOINTS]; /* target angle, rad */

float pd_torque(const pd_gains_t *g, float q_des, float q, float qd_des, float qd, float tau_ff)
{
  /* !(x > 0) is also true for NaN, so a NaN limit disables output too. */
  if (g == NULL || !(g->tau_max > 0.0f) || !isfinite(g->tau_max)) {
    return 0.0f;
  }

  float tau = g->kp * (q_des - q) + g->kd * (qd_des - qd) + tau_ff;

  /* NaN/inf in any input or gain propagates into tau, so one check covers all of them. */
  if (!isfinite(tau)) {
    return 0.0f;
  }
  if (tau > g->tau_max) {
    return g->tau_max;
  }
  if (tau < -g->tau_max) {
    return -g->tau_max;
  }
  return tau;
}

static void send_input_torque(int joint_index, float tau_motor)
{
  odrive_process_tx_msg(joint_index, ODRIVE_CMD_SET_INPUT_TORQUE, (uint8_t *)&tau_motor,
      sizeof(tau_motor));
}

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

void joint_control_hold(const JointRawFeedback_t *joints) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    q_des[i] = joint_q(&joints[i]) + 2.00f;
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

#if PD_DRY_RUN
    send_input_torque(i, 0.0f); /* only feeds the ODrive watchdog */
#else
    send_input_torque(i, direction(&joints[i]) * motor_torque_nm(tau, JOINT_GEAR_RATIO));
#endif

    if (print) {
      print_float(q_names[i], q);
      print_float(qd_names[i], qd);
      print_float(tau_names[i], tau);
    }
  }
}

void joint_control_feed_watchdog(void)
{
  for (int i = 0; i < NUM_JOINTS; i++) {
    send_input_torque(i, 0.0f);
  }
}
