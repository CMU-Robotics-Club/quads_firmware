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
 * Reading the latest value anywhere in firmware:
 *   qp_cmd_joint_t j; uint32_t ver = 0;
 *   if (qp_update(&qp_ch_CMD_JOINT, 0, &j, &ver) > 0) { ...new data... }
 */
#ifndef QUADS_MSGS_H
#define QUADS_MSGS_H

#include <stdint.h>

#define QP_MAX_JOINTS 12U   /* protocol limit; firmware currently drives 2 */

/*------------- Payload structs (little-endian, f32) -------------*/

/** One joint setpoint. Keyed by joint_id (first byte). */
typedef struct __attribute__((packed)) {
  uint8_t joint_id;
  float   q_des;    /**< rad, output side */
  float   qd_des;   /**< rad/s */
  float   tau_ff;   /**< Nm, feed-forward */
} qp_cmd_joint_t;

/** PD gains for one joint. Keyed by joint_id (first byte). */
typedef struct __attribute__((packed)) {
  uint8_t joint_id;
  float   kp;
  float   kd;
  float   tau_max;  /**< Nm, output clamp */
} qp_cmd_gains_t;

/*------------- Message list (Jetson -> STM32, IDs 0x01-0x7F) -------------*/
/*
 * NAME   : channel is qp_ch_<NAME>, ID enum is QP_<NAME>
 * ID     : type byte on the wire
 * TYPE   : payload struct, length must match exactly
 * SLOTS  : 1 = one latest value; N = N latest values keyed by payload[0] (e.g. joint_id)
 * ACK    : true = framework replies ACK and dedupes repeated seq
 * VALID  : uint8_t fn(const void *payload) returning QP_OK / QP_ERR_*, or NULL
 */
#define QP_MESSAGES(X)                                                          \
  /* NAME       ID    TYPE            SLOTS          ACK    VALID          */   \
  X(CMD_JOINT,  0x01, qp_cmd_joint_t, QP_MAX_JOINTS, false, validate_joint)     \
  X(CMD_GAINS,  0x03, qp_cmd_gains_t, QP_MAX_JOINTS, true,  validate_gains)

/*------------- Generated: IDs and channel declarations -------------*/
#define QP_X_ENUM(NAME, ID, TYPE, SLOTS, ACK, VALID) QP_##NAME = (ID),
enum { QP_MESSAGES(QP_X_ENUM) };
#undef QP_X_ENUM

struct qp_channel;
#define QP_X_EXTERN(NAME, ID, TYPE, SLOTS, ACK, VALID) extern struct qp_channel qp_ch_##NAME;
QP_MESSAGES(QP_X_EXTERN)
#undef QP_X_EXTERN

#endif /* QUADS_MSGS_H */
