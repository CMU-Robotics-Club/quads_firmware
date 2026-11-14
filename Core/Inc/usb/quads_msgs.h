/**
 * @file quads_msgs.h
 * @brief The ONE place to define USB messages (payload structs + message list).
 * @author Ivan Lok
 *
 * Adding a message:
 *   1. Write a packed payload struct below.
 *   2. Add one line to QP_MESSAGES.
 *   3. (optional) Write a validate function in quads_msgs.c, or pass NULL.
 *   4. Mirror the struct in scripts/quads_host.py.
 * Everything else (enum ID, channel object, storage, parser table) is generated.
 *
 * Each message keeps ONE latest value. Data for several joints goes in one struct
 * (an array), so it is always updated together.
 *
 * Reading the latest value anywhere in firmware:
 *   qp_cmd_joints_t sp; uint32_t ver = 0;
 *   if (qp_update(&qp_ch_CMD_JOINTS, &sp, &ver) > 0) { ...new data... }
 */
#ifndef QUADS_MSGS_H
#define QUADS_MSGS_H

#include <stdint.h>

#define QP_MAX_JOINTS 12U   /* protocol always carries 12; firmware currently drives 2 */

/*------------- Payload structs (little-endian, f32) -------------*/

/** One joint's setpoint (index in the array = joint id). */
typedef struct __attribute__((packed)) {
  float q_des;      /**< rad, output side */
  float qd_des;     /**< rad/s */
  float tau_ff;     /**< Nm, feed-forward */
} qp_joint_sp_t;

/** Setpoints for all joints in one frame (144 bytes): the controller never sees a half-updated pose. */
typedef struct __attribute__((packed)) {
  qp_joint_sp_t j[QP_MAX_JOINTS];
} qp_cmd_joints_t;

/** One joint's PD gains. */
typedef struct __attribute__((packed)) {
  float kp;
  float kd;
  float tau_max;    /**< Nm, output clamp */
} qp_joint_gains_t;

/** Gains for all joints in one frame (144 bytes). */
typedef struct __attribute__((packed)) {
  qp_joint_gains_t j[QP_MAX_JOINTS];
} qp_cmd_gains_t;

/*------------- Message list (Jetson -> STM32, IDs 0x01-0x7F) -------------*/
/*
 * NAME   : channel is qp_ch_<NAME>, ID enum is QP_<NAME>
 * ID     : type byte on the wire
 * TYPE   : payload struct, length must match exactly
 * ACK    : true = framework replies ACK and dedupes repeated seq
 * VALID  : uint8_t fn(const void *payload) returning QP_OK / QP_ERR_*, or NULL
 */
#define QP_MESSAGES(X)                                                          \
  /* NAME        ID    TYPE             ACK    VALID          */     \
  X(CMD_JOINTS,  0x01, qp_cmd_joints_t, false, validate_joints)     \
  X(CMD_GAINS,   0x03, qp_cmd_gains_t,  true,  validate_gains)

/*------------- Generated: IDs and channel declarations -------------*/
#define QP_X_ENUM(NAME, ID, TYPE, ACK, VALID) QP_##NAME = (ID),
enum { QP_MESSAGES(QP_X_ENUM) };
#undef QP_X_ENUM

struct qp_channel;
#define QP_X_EXTERN(NAME, ID, TYPE, ACK, VALID) extern struct qp_channel qp_ch_##NAME;
QP_MESSAGES(QP_X_EXTERN)
#undef QP_X_EXTERN

#endif /* QUADS_MSGS_H */
