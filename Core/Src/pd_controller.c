#include "pd_controller.h"

#include <math.h>
#include <stddef.h>

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
