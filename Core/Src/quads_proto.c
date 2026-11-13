/**
 * @file quads_proto.c
 * @brief CDC0 protocol: COBS framing, CRC16, frame parser, message registry, ACK.
 * @author Ivan Lok
 *
 * Parser runs in UsbTask (called from tud_cdc_rx_cb). One generic path for every
 * message in quads_msgs.h; per-message code is only the optional validate function.
 */
#include "quads_proto.h"

#include <string.h>

#ifndef QP_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"
#endif

/*------------- CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xorout) -------------*/

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

/**
 * @brief Decode one COBS block (without the trailing 0x00).
 * @return Decoded length, or 0 on malformed input / overflow.
 */
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

#ifdef QP_HOST_TEST
#define QP_LOCK()      do { } while (0)
#define QP_UNLOCK()    do { } while (0)
#define QP_NOW_MS()    0U
#else
#define QP_LOCK()      taskENTER_CRITICAL()
#define QP_UNLOCK()    taskEXIT_CRITICAL()
#define QP_NOW_MS()    ((uint32_t)xTaskGetTickCount())
#endif

/** @brief Writer side (UsbTask): store one validated payload into its slot. */
static void channel_store(qp_channel_t *ch, uint8_t slot, const uint8_t *payload)
{
  const uint32_t now = QP_NOW_MS();
  QP_LOCK();
  memcpy(ch->storage + (size_t)slot * ch->len, payload, ch->len);
  ch->stamp_ms[slot] = now;
  ch->version[slot]++;
  QP_UNLOCK();
}

int qp_update(qp_channel_t *ch, uint8_t slot, void *out, uint32_t *last_version)
{
  if (slot >= ch->slots) {
    return -1;
  }
  int is_new = 0;
  QP_LOCK();
  const uint32_t ver = ch->version[slot];
  if (ver != *last_version) {
    memcpy(out, ch->storage + (size_t)slot * ch->len, ch->len);
    *last_version = ver;
    is_new = 1;
  }
  QP_UNLOCK();
  return is_new;
}

uint32_t qp_age_ms(const qp_channel_t *ch, uint8_t slot)
{
  if (slot >= ch->slots || ch->version[slot] == 0U) {
    return UINT32_MAX;
  }
  return QP_NOW_MS() - ch->stamp_ms[slot];
}

/*------------- Parser state -------------*/

static qp_stats_t stats;
static bool     have_rx_seq;
static uint16_t last_rx_seq;

static uint8_t  enc_buf[QP_MAX_ENCODED];
static size_t   enc_len;
static bool     enc_overflow;

void qp_get_stats(qp_stats_t *out)
{
  *out = stats;   /* UsbTask is the only writer; counters are informational */
}

static void send_ack(uint8_t type, uint16_t seq, uint8_t err)
{
  const qp_ack_t ack = { .ack_type = type, .ack_seq = seq, .err = err };
  (void)qp_send(QP_ACK, &ack, sizeof(ack));
}

static qp_channel_t *find_channel(uint8_t type)
{
  for (size_t i = 0; i < qp_channel_count; i++) {
    if (qp_channels[i]->type == type) {
      return qp_channels[i];
    }
  }
  return NULL;
}

/** @brief Generic path for every message: CRC -> header -> channel -> length -> validate -> store -> ACK. */
static void handle_frame(const uint8_t *raw, size_t raw_len)
{
  if (raw_len < QP_HDR_LEN + QP_CRC_LEN) {
    stats.cobs_errors++;
    return;
  }
  const size_t body_len = raw_len - QP_CRC_LEN;
  const uint16_t crc_rx = (uint16_t)(raw[body_len] | ((uint16_t)raw[body_len + 1U] << 8));
  if (qp_crc16(raw, body_len) != crc_rx) {
    stats.crc_errors++;
    return;
  }

  qp_hdr_t hdr;
  memcpy(&hdr, raw, sizeof(hdr));
  const uint8_t *payload = raw + QP_HDR_LEN;
  if ((size_t)hdr.len != body_len - QP_HDR_LEN) {
    stats.bad_len++;
    return;
  }

  /* Loss detection on the host's single seq counter */
  if (have_rx_seq) {
    const uint16_t gap = (uint16_t)(hdr.seq - last_rx_seq - 1U);
    if (gap != 0U && gap < 0x8000U) {
      stats.seq_gaps += gap;
    }
  }
  have_rx_seq = true;
  last_rx_seq = hdr.seq;

  qp_channel_t *ch = find_channel(hdr.type);
  if (ch == NULL) {
    stats.unknown_type++;
    send_ack(hdr.type, hdr.seq, QP_ERR_UNKNOWN_TYPE);
    return;
  }

  uint8_t err = QP_OK;
  if (hdr.len != ch->len) {
    stats.bad_len++;
    err = QP_ERR_BAD_LEN;
  } else if (ch->need_ack && ch->have_seq && ch->last_seq == hdr.seq) {
    /* Host resent because our ACK was lost: ACK again, don't store twice */
    stats.dup_frames++;
    send_ack(hdr.type, hdr.seq, QP_OK);
    return;
  } else {
    const uint8_t slot = (ch->slots > 1U) ? payload[0] : 0U;
    if (slot >= ch->slots) {
      err = QP_ERR_BAD_ARG;
    } else if (ch->validate != NULL) {
      err = ch->validate(payload);
    }
    if (err == QP_OK) {
      channel_store(ch, slot, payload);
      stats.rx_frames++;
    } else {
      stats.bad_arg++;
    }
    if (ch->need_ack) {
      ch->have_seq = true;
      ch->last_seq = hdr.seq;
    }
  }

  if (ch->need_ack) {
    send_ack(hdr.type, hdr.seq, err);
  }
}

void qp_rx_bytes(const uint8_t *data, uint32_t n)
{
  static uint8_t raw[QP_MAX_RAW];
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
static uint16_t tx_seq;

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
