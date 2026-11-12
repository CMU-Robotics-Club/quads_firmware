/*
 * Per-actuator PD loop on top of joint_interface: feedback -> pd_torque -> Set_Input_Torque.
 * Called from ControlTask (control_task.c).
 */
#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include "joint_interface.h"

/* 1: compute and print the PD torque but send only zero torque (to feed the ODrive watchdog),
 *    and ControlTask does not arm the motors.
 * 0: ControlTask arms the motors and the torque is sent to the ODrives.
 * Keep 1 until signs, gear ratio and gains have been checked on the leg by hand. */
#define PD_DRY_RUN 0

typedef struct {
    float kp;      /* Nm/rad */
    float kd;      /* Nm/(rad/s) */
    float tau_max; /* output limit, Nm; must be > 0 and finite, otherwise output is 0 */
} pd_gains_t;

/* tau = kp*(q_des - q) + kd*(qd_des - qd) + tau_ff, clamped to +-tau_max.
 * Returns 0 if g is NULL, tau_max is invalid, or any input makes the result NaN/inf. */
float pd_torque(const pd_gains_t *g, float q_des, float q, float qd_des, float qd, float tau_ff);

/* Call once when entering RUNNING: the target becomes the current position, so the leg holds
 * where it is. With PD_DRY_RUN 0 it also switches the ODrives to torque control. */
void joint_control_hold(const JointRawFeedback_t *joints);

/* Call every control period in RUNNING with this period's feedback copy.
 * print: also log q, qd and tau of every joint this period. */
void joint_control_step(const JointRawFeedback_t *joints, bool print);

/* Send Set_Input_Torque(0) to every joint. Any Set_Input_* feeds the ODrive watchdog, also in
 * IDLE, where an unfed watchdog raises WATCHDOG_TIMER_EXPIRED (seen on fw 0.6.11).
 * Call every control period in the states between WAIT_CAN and RUNNING. */
void joint_control_feed_watchdog(void);

#endif /* JOINT_CONTROL_H */
