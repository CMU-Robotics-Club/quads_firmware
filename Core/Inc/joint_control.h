/*
 * Per-actuator PD loop on top of joint_interface: feedback -> pd_torque -> Set_Input_Torque.
 * Called from ControlTask (control_task.c).
 */
#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include "joint_interface.h"

/* 1: compute and print the PD torque but send nothing, and ControlTask does not arm the motors.
 * 0: ControlTask arms the motors and the torque is sent to the ODrives.
 * Keep 1 until signs, gear ratio and gains have been checked on the leg by hand. */
#define PD_DRY_RUN 1

/* Call once when entering RUNNING: the target becomes the current position, so the leg holds
 * where it is. With PD_DRY_RUN 0 it also switches the ODrives to torque control. */
void joint_control_hold(const JointRawFeedback_t *joints);

/* Call every control period in RUNNING with this period's feedback copy.
 * print: also log q, qd and tau of every joint this period. */
void joint_control_step(const JointRawFeedback_t *joints, bool print);

#endif /* JOINT_CONTROL_H */
