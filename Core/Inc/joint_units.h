/*
 * Unit conversions between ODrive motor-side values and the actuator output shaft.
 *
 * Pure C, no HAL, so it can be host-tested (tests/test_joint_units.c).
 * gear_ratio is N for an N:1 reduction: the motor turns N times per output turn.
 * These are per-actuator values. The knee/hip linkage coupling is not applied here.
 */
#ifndef JOINT_UNITS_H
#define JOINT_UNITS_H

#define JOINT_TWO_PI 6.28318530717958647692f

/* Output angle in rad, measured from zero_turns (the motor position chosen as zero). */
static inline float joint_pos_rad(float motor_turns, float zero_turns, float gear_ratio)
{
    return JOINT_TWO_PI * (motor_turns - zero_turns) / gear_ratio;
}

/* Output speed in rad/s from motor speed in turns/s. */
static inline float joint_vel_rad_s(float motor_turns_per_s, float gear_ratio)
{
    return JOINT_TWO_PI * motor_turns_per_s / gear_ratio;
}

/* Motor-shaft torque in Nm needed for joint_torque_nm at the output (ideal gearbox, no losses). */
static inline float motor_torque_nm(float joint_torque_nm, float gear_ratio)
{
    return joint_torque_nm / gear_ratio;
}

#endif /* JOINT_UNITS_H */
