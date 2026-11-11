/*
 * Joint PD controller: torque from position and velocity error.
 *
 * Pure C, no HAL, so it can be host-tested (tests/test_pd_controller.c).
 * Joint-side units: rad, rad/s, Nm. Motor turns and gear ratio are converted by the caller.
 */
#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H

typedef struct {
    float kp;      /* Nm/rad */
    float kd;      /* Nm/(rad/s) */
    float tau_max; /* output limit, Nm; must be > 0 and finite, otherwise output is 0 */
} pd_gains_t;

/* tau = kp*(q_des - q) + kd*(qd_des - qd) + tau_ff, clamped to +-tau_max.
 * Returns 0 if g is NULL, tau_max is invalid, or any input makes the result NaN/inf. */
float pd_torque(const pd_gains_t *g, float q_des, float q, float qd_des, float qd, float tau_ff);

#endif /* PD_CONTROLLER_H */
