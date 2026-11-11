#include "pd_controller.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static int near(float a, float b)
{
    return fabsf(a - b) < 1e-5f;
}

int main(void)
{
    const pd_gains_t g = {.kp = 10.0f, .kd = 0.5f, .tau_max = 2.0f};

    /* Zero error: only the feed-forward comes out. */
    assert(near(pd_torque(&g, 1.0f, 1.0f, 0.0f, 0.0f, 0.3f), 0.3f));

    /* Sign: below the target pushes positive (10 * 0.1 = 1.0). */
    assert(near(pd_torque(&g, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f), 1.0f));
    /* Damping: faster than the target pushes negative (0.5 * (0 - 2) = -1.0). */
    assert(near(pd_torque(&g, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f), -1.0f));

    /* Clamp on both sides. */
    assert(pd_torque(&g, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 2.0f);
    assert(pd_torque(&g, -10.0f, 0.0f, 0.0f, 0.0f, 0.0f) == -2.0f);

    /* Bad input -> 0. */
    assert(pd_torque(&g, NAN, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
    assert(pd_torque(&g, 0.0f, 0.0f, 0.0f, INFINITY, 0.0f) == 0.0f);

    /* Bad config -> 0. */
    const pd_gains_t zero_lim = {.kp = 10.0f, .kd = 0.5f, .tau_max = 0.0f};
    const pd_gains_t neg_lim = {.kp = 10.0f, .kd = 0.5f, .tau_max = -2.0f};
    const pd_gains_t nan_lim = {.kp = 10.0f, .kd = 0.5f, .tau_max = NAN};
    const pd_gains_t inf_lim = {.kp = 10.0f, .kd = 0.5f, .tau_max = INFINITY};
    assert(pd_torque(&zero_lim, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
    assert(pd_torque(&neg_lim, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
    assert(pd_torque(&nan_lim, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
    assert(pd_torque(&inf_lim, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
    assert(pd_torque(NULL, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);

    puts("test_pd_controller: PASS");
    return 0;
}
