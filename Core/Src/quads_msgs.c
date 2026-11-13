/**
 * @file quads_msgs.c
 * @brief Validate functions + generated channel objects for the messages in quads_msgs.h.
 * @author Ivan Lok
 */
#include "quads_msgs.h"

#include <math.h>
#include <string.h>
#include "quads_proto.h"

/*------------- Validate functions (payload is unaligned: memcpy first) -------------*/

static uint8_t validate_joint(const void *payload)
{
  qp_cmd_joint_t c;
  memcpy(&c, payload, sizeof(c));
  if (!isfinite(c.q_des) || !isfinite(c.qd_des) || !isfinite(c.tau_ff)) {
    return QP_ERR_BAD_ARG;
  }
  return QP_OK;
}

static uint8_t validate_gains(const void *payload)
{
  qp_cmd_gains_t c;
  memcpy(&c, payload, sizeof(c));
  if (!isfinite(c.kp) || !isfinite(c.kd) || !isfinite(c.tau_max) ||
      c.kp < 0.0f || c.kd < 0.0f || c.tau_max < 0.0f) {
    return QP_ERR_BAD_ARG;
  }
  return QP_OK;
}

/*------------- Generated channel objects -------------*/

#define QP_X_CHANNEL(NAME, ID, TYPE, SLOTS, ACK, VALID)                                  \
  _Static_assert(sizeof(TYPE) <= QP_MAX_PAYLOAD, #NAME " payload too large");          \
  _Static_assert((SLOTS) >= 1U && (SLOTS) <= 255U, #NAME " bad slot count");             \
  _Static_assert((ID) >= 0x01 && (ID) <= 0x7F, #NAME " ID must be 0x01-0x7F");           \
  static uint8_t  NAME##_storage[(SLOTS) * sizeof(TYPE)];                                \
  static uint32_t NAME##_version[SLOTS];                                                 \
  static uint32_t NAME##_stamp[SLOTS];                                                   \
  qp_channel_t qp_ch_##NAME = {                                                          \
    .name = #NAME, .type = (ID), .len = sizeof(TYPE), .slots = (SLOTS),                  \
    .need_ack = (ACK), .validate = (VALID),                                              \
    .storage = NAME##_storage, .version = NAME##_version, .stamp_ms = NAME##_stamp,      \
  };
QP_MESSAGES(QP_X_CHANNEL)
#undef QP_X_CHANNEL

#define QP_X_PTR(NAME, ID, TYPE, SLOTS, ACK, VALID) &qp_ch_##NAME,
qp_channel_t *const qp_channels[] = { QP_MESSAGES(QP_X_PTR) };
#undef QP_X_PTR

const size_t qp_channel_count = sizeof(qp_channels) / sizeof(qp_channels[0]);

/* Compile-time check for duplicate IDs: two equal IDs = "duplicate case value" error. */
#define QP_X_CASE(NAME, ID, TYPE, SLOTS, ACK, VALID) case (ID): break;
static inline void qp_check_unique_ids(int id) { switch (id) { QP_MESSAGES(QP_X_CASE) default: break; } }
#undef QP_X_CASE
