/**
 * @file usb_msgs.c
 * @brief Validate functions + generated channel objects for the messages in usb_msgs.h.
 * @author Ivan Lok
 *
 * A validate function runs in UsbTask before a payload is stored. Returning anything
 * but QP_OK drops the payload and, for ACKed messages, tells the host why.
 */
#include "usb_msgs.h"

#include <math.h>
#include <string.h>
#include "usb_proto.h"

/*------------- Validate functions (payload is unaligned: memcpy first) -------------*/

/* Reject NaN / Inf in any joint setpoint. */
static uint8_t validate_joints(const void *payload)
{
  qp_cmd_joints_t c;
  memcpy(&c, payload, sizeof(c));
  for (size_t i = 0; i < QP_MAX_JOINTS; i++) {
    if (!isfinite(c.j[i].q_des) || !isfinite(c.j[i].qd_des) || !isfinite(c.j[i].tau_ff)) {
      return QP_ERR_BAD_ARG;
    }
  }
  return QP_OK;
}

/* Reject NaN / Inf and negative gains or torque limits. */
static uint8_t validate_gains(const void *payload)
{
  qp_cmd_gains_t c;
  memcpy(&c, payload, sizeof(c));
  for (size_t i = 0; i < QP_MAX_JOINTS; i++) {
    const qp_joint_gains_t *g = &c.j[i];
    if (!isfinite(g->kp) || !isfinite(g->kd) || !isfinite(g->tau_max) ||
        g->kp < 0.0f || g->kd < 0.0f || g->tau_max < 0.0f) {
      return QP_ERR_BAD_ARG;
    }
  }
  return QP_OK;
}

/*------------- Generated channel objects -------------*/
/* For each message: compile-time size/ID checks, a static storage buffer of exactly
 * sizeof(TYPE) bytes, and the channel object qp_ch_<NAME> pointing at it. */

#define QP_X_CHANNEL(NAME, ID, TYPE, ACK, VALID)                                         \
  _Static_assert(sizeof(TYPE) <= QP_MAX_PAYLOAD, #NAME " payload too large");          \
  _Static_assert((ID) >= 0x01 && (ID) <= 0x7F, #NAME " ID must be 0x01-0x7F");           \
  static uint8_t NAME##_storage[sizeof(TYPE)];                                           \
  qp_channel_t qp_ch_##NAME = {                                                          \
    .name = #NAME, .type = (ID), .len = sizeof(TYPE),                                    \
    .need_ack = (ACK), .validate = (VALID), .storage = NAME##_storage,                   \
  };
QP_MESSAGES(QP_X_CHANNEL)
#undef QP_X_CHANNEL

/* Table the parser searches by type byte (find_channel in usb_proto.c) */
#define QP_X_PTR(NAME, ID, TYPE, ACK, VALID) &qp_ch_##NAME,
qp_channel_t *const qp_channels[] = { QP_MESSAGES(QP_X_PTR) };
#undef QP_X_PTR

const size_t qp_channel_count = sizeof(qp_channels) / sizeof(qp_channels[0]);

/* Compile-time check for duplicate IDs: two equal IDs = "duplicate case value" error. */
#define QP_X_CASE(NAME, ID, TYPE, ACK, VALID) case (ID): break;
static inline void qp_check_unique_ids(int id) { switch (id) { QP_MESSAGES(QP_X_CASE) default: break; } }
#undef QP_X_CASE
