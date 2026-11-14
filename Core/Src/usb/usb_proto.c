/**
 * @file usb_proto.c
 * @brief CDC0 protocol: COBS framing, CRC16, frame parser, channel storage, ACK.
 * @author Ivan Lok
 *
 * RX path (all in UsbTask, entered from tud_cdc_rx_cb):
 *   qp_rx_bytes()  collect bytes until 0x00
 *   -> qp_cobs_decode()  undo COBS
 *   -> handle_frame()    CRC, header, find channel, length, dedupe, validate
 *   -> channel_store()   copy into the channel under a critical section, then ACK
 *
 * One generic path for every message in usb_msgs.h; per-message code is only the
 * optional validate function. Readers (any task) use qp_update().
 */
#include "usb_proto.h"

#include <string.h>

#ifndef QP_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#endif

/*------------- CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xorout) -------------*/

/* Bitwise instead of a 512 B table: frames are small and the rate is low. */
uint16_t qp_crc16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/*------------- COBS -------------*/

/* Each block starts with a code byte = (distance to the next zero) + 1. code_idx is
 * where the current block's code byte goes; it is filled in once the block ends.
 * A run of 254 non-zero bytes closes the block with code 0xFF (no zero implied). */
size_t qp_cobs_encode(const uint8_t *in, size_t len, uint8_t *out)
{
  size_t code_idx = 0;
  size_t o = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; i++) {
    if (in[i] == 0U) {
      out[code_idx] = code;
      code_idx = o++;
      code = 1;
    } else {
      out[o++] = in[i];
      if (++code == 0xFFU) {
        out[code_idx] = code;
        code_idx = o++;
        code = 1;
      }
    }
  }
  out[code_idx] = code;
  return o;
}

/* Reverse of encode: copy code-1 bytes, then insert the implied zero, except after a
 * 0xFF block or at the very end. Every write is bounds-checked against out_max. */
size_t qp_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max)
{
  size_t i = 0;
  size_t o = 0;
  while (i < len) {
    uint8_t code = in[i++];
    if (code == 0U || (size_t)(code - 1U) > (len - i)) {
      return 0;
    }
    for (uint8_t k = 1; k < code; k++) {
      if (o >= out_max) {
        return 0;
      }
      out[o++] = in[i++];
    }
    if (code != 0xFFU && i < len) {
      if (o >= out_max) {
        return 0;
      }
      out[o++] = 0U;
    }
  }
  return o;
}

/*------------- Channel storage access -------------*/

/* Host tests build this file without FreeRTOS */

#ifdef QP_HOST_TEST
#define QP_LOCK()      do { } while (0)
#define QP_UNLOCK()    do { } while (0)
extern uint32_t qp_test_now_ms;
#define QP_NOW_MS()    qp_test_now_ms
#else
#define QP_LOCK()      taskENTER_CRITICAL()
#define QP_UNLOCK()    taskEXIT_CRITICAL()
#define QP_NOW_MS()    ((uint32_t)xTaskGetTickCount())
#endif

/* Writer side (UsbTask). Same lock as qp_update(), so a reader never copies a
 * half-written struct: ControlTask has higher priority and could preempt mid-memcpy. */
static void channel_store(qp_channel_t *ch, const uint8_t *payload)
{
  const uint32_t now = QP_NOW_MS();
  QP_LOCK();
  memcpy(ch->storage, payload, ch->len);
  ch->stamp_ms = now;
  ch->version++;
  QP_UNLOCK();
}

/* Compare with != (not >) so the u32 version wrapping around is harmless. */
int qp_update(qp_channel_t *ch, void *out, uint32_t *last_version)
{
  int is_new = 0;
  QP_LOCK();
  if (ch->version != *last_version) {
    memcpy(out, ch->storage, ch->len);
    *last_version = ch->version;
    is_new = 1;
  }
  QP_UNLOCK();
  return is_new;
}

/* No lock: two aligned 32-bit reads are each atomic on the M7. Worst case the
 * age is off by one update, which is fine for a timeout. */
uint32_t qp_age_ms(const qp_channel_t *ch)
{
  if (ch->version == 0U) {
    return UINT32_MAX;
  }
  return QP_NOW_MS() - ch->stamp_ms;
}

/*------------- Parser state (UsbTask only, so no lock) -------------*/

static qp_stats_t stats;
static bool     have_rx_seq;    /* false until the first valid frame */
static uint16_t last_rx_seq;    /* host seq of the last valid frame, for gap counting */

static uint8_t  enc_buf[QP_MAX_ENCODED];  /* COBS bytes of the frame being received */
static size_t   enc_len;
static bool     enc_overflow;   /* frame too long: drop bytes until the next 0x00 */

void qp_get_stats(qp_stats_t *out)
{
  *out = stats;   /* UsbTask is the only writer; counters are informational */
}

static void send_ack(uint8_t type, uint16_t seq, uint8_t err)
{
  const qp_ack_t ack = { .ack_type = type, .ack_seq = seq, .err = err };
  (void)qp_send(QP_ACK, &ack, sizeof(ack));
}

/* Linear search: fine for a handful of messages; switch to a 128-entry table if it grows. */
static qp_channel_t *find_channel(uint8_t type)
{
  for (size_t i = 0; i < qp_channel_count; i++) {
    if (qp_channels[i]->type == type) {
      return qp_channels[i];
    }
  }
  return NULL;
}

/* Generic path for every message, steps numbered below. */
static void handle_frame(const uint8_t *raw, size_t raw_len)
{
  /* 1. Frame must hold at least a header and a CRC */
  if (raw_len < QP_HDR_LEN + QP_CRC_LEN) {
    stats.cobs_errors++;
    return;
  }
  /* 2. CRC. On failure type and seq can't be trusted, so drop silently (no NACK) */
  const size_t body_len = raw_len - QP_CRC_LEN;
  const uint16_t crc_rx = (uint16_t)(raw[body_len] | ((uint16_t)raw[body_len + 1U] << 8));
  if (qp_crc16(raw, body_len) != crc_rx) {
    stats.crc_errors++;
    return;
  }

  /* 3. Header. memcpy because raw is not aligned for the u16 seq */
  qp_hdr_t hdr;
  memcpy(&hdr, raw, sizeof(hdr));
  const uint8_t *payload = raw + QP_HDR_LEN;
  if ((size_t)hdr.len != body_len - QP_HDR_LEN) {
    stats.bad_len++;
    return;
  }

  /* 4. Loss detection on the host's single seq counter. gap >= 0x8000 means the
   *    seq went backwards (host restarted), which is not counted as loss. */
  if (have_rx_seq) {
    const uint16_t gap = (uint16_t)(hdr.seq - last_rx_seq - 1U);
    if (gap != 0U && gap < 0x8000U) {
      stats.seq_gaps += gap;
    }
  }
  have_rx_seq = true;
  last_rx_seq = hdr.seq;

  /* 5. Unknown type: always ACK, so the host learns it's talking to old firmware */
  qp_channel_t *ch = find_channel(hdr.type);
  if (ch == NULL) {
    stats.unknown_type++;
    send_ack(hdr.type, hdr.seq, QP_ERR_UNKNOWN_TYPE);
    return;
  }

  /* 6. Length must match the struct exactly */
  uint8_t err = QP_OK;
  const uint32_t now = QP_NOW_MS();
  /* 7. ACK channels: compare with the last seq, as a signed 16-bit difference so the
   *    u16 wrap is harmless. Only within QP_DEDUPE_MS: after a pause the host may have
   *    restarted with seq = 0, which must count as new. */
  const bool recent = ch->need_ack && ch->have_seq && (now - ch->last_seq_ms) < QP_DEDUPE_MS;
  const int16_t diff = (int16_t)(uint16_t)(hdr.seq - ch->last_seq);
  if (hdr.len != ch->len) {
    stats.bad_len++;
    err = QP_ERR_BAD_LEN;
  } else if (recent && diff == 0) {
    /* Host resent because our ACK was lost: ACK again, don't store twice */
    stats.dup_frames++;
    send_ack(hdr.type, hdr.seq, ch->last_err);
    return;
  } else if (recent && diff < 0) {
    /* Late resend of an older command: a newer one is already stored, keep it */
    stats.stale_frames++;
    send_ack(hdr.type, hdr.seq, QP_ERR_STALE);
    return;
  } else {
    /* 8. Validate and store */
    if (ch->validate != NULL) {
      err = ch->validate(payload);
    }
    if (err == QP_OK) {
      channel_store(ch, payload);
      stats.rx_frames++;
    } else {
      stats.bad_arg++;
    }
    if (ch->need_ack) {
      ch->have_seq = true;
      ch->last_seq = hdr.seq;
      ch->last_err = err;
      ch->last_seq_ms = now;
    }
  }

  if (ch->need_ack) {
    send_ack(hdr.type, hdr.seq, err);
  }
}

/* Bytes can arrive split across USB packets, so state is kept between calls. */
void qp_rx_bytes(const uint8_t *data, uint32_t n)
{
  static uint8_t raw[QP_MAX_RAW];   /* static: keeps 261 B off the UsbTask stack */
  for (uint32_t i = 0; i < n; i++) {
    const uint8_t b = data[i];
    if (b != 0U) {
      if (enc_len < sizeof(enc_buf)) {
        enc_buf[enc_len++] = b;
      } else {
        enc_overflow = true;   /* keep dropping until the next delimiter */
      }
      continue;
    }
    /* 0x00 = end of frame */
    if (enc_overflow) {
      stats.overflows++;
    } else if (enc_len > 0U) {
      const size_t raw_len = qp_cobs_decode(enc_buf, enc_len, raw, sizeof(raw));
      if (raw_len == 0U) {
        stats.cobs_errors++;
      } else {
        handle_frame(raw, raw_len);
      }
    }
    enc_len = 0;
    enc_overflow = false;
  }
}

/*------------- TX -------------*/

#ifndef QP_HOST_TEST
static uint16_t tx_seq;   /* STM32's own counter, independent of the host's */

int qp_send(uint8_t type, const void *payload, uint8_t len)
{
  uint8_t raw[QP_MAX_RAW];
  uint8_t enc[QP_MAX_ENCODED + 1U];

  const qp_hdr_t hdr = { .type = type, .seq = tx_seq++, .len = len };
  memcpy(raw, &hdr, QP_HDR_LEN);
  if (len > 0U) {
    memcpy(raw + QP_HDR_LEN, payload, len);
  }
  const size_t body_len = QP_HDR_LEN + len;
  const uint16_t crc = qp_crc16(raw, body_len);
  raw[body_len]      = (uint8_t)(crc & 0xFFU);
  raw[body_len + 1U] = (uint8_t)(crc >> 8);

  size_t n = qp_cobs_encode(raw, body_len + QP_CRC_LEN, enc);
  enc[n++] = 0x00U;

  /* All-or-nothing: never put half a frame in the FIFO */
  if (tud_cdc_n_write_available(QUADS_CDC_CMD) < n) {
    stats.tx_drops++;
    return -1;
  }
  tud_cdc_n_write(QUADS_CDC_CMD, enc, (uint32_t)n);
  tud_cdc_n_write_flush(QUADS_CDC_CMD);
  stats.tx_frames++;
  return 0;
}
#endif
