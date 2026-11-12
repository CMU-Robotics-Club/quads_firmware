#include "joint_units.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int near(float a, float b)
{
    return fabsf(a - b) < 1e-5f;
}

int main(void)
{
    const float gear = 14.0f;

    /* At the zero position the angle is 0, wherever the zero is. */
    assert(near(joint_pos_rad(0.0f, 0.0f, gear), 0.0f));
    assert(near(joint_pos_rad(3.5f, 3.5f, gear), 0.0f));

    /* 14 motor turns = one output turn = 2*pi rad. */
    assert(near(joint_pos_rad(14.0f, 0.0f, gear), JOINT_TWO_PI));
    /* A quarter output turn (3.5 motor turns) past a non-zero zero. */
    assert(near(joint_pos_rad(5.5f, 2.0f, gear), JOINT_TWO_PI / 4.0f));
    /* Sign follows the motor. */
    assert(near(joint_pos_rad(-7.0f, 0.0f, gear), -JOINT_TWO_PI / 2.0f));

    /* 14 motor turns/s = 2*pi rad/s at the output. */
    assert(near(joint_vel_rad_s(14.0f, gear), JOINT_TWO_PI));
    assert(near(joint_vel_rad_s(-7.0f, gear), -JOINT_TWO_PI / 2.0f));

    /* The gearbox multiplies torque, so the motor needs 1/14 of the joint torque. */
    assert(near(motor_torque_nm(14.0f, gear), 1.0f));
    assert(near(motor_torque_nm(-1.4f, gear), -0.1f));

    puts("test_joint_units: PASS");
    return 0;
}
