/**
 * @file usb_proto.h
 * @brief Quads USB protocol (CDC0): framing, channel objects, read API.
 * @author Ivan Lok
 *
 * Wire format (one frame):
 *   COBS( type u8 | seq u16 LE | len u8 | payload[len] | crc16 u16 LE ) 0x00
 *   - COBS removes every 0x00 from the frame, so 0x00 only marks the end.
 *   - crc16 = CRC16-CCITT (poly 0x1021, init 0xFFFF) over type..payload.
 *   - seq: sender's counter, +1 per frame. Used for loss detection, ACK matching, dedupe.
 *   - type 0x01-0x7F = Jetson -> STM32, 0x80-0xFF = STM32 -> Jetson.
 *
 * Messages are defined in usb_msgs.h. Each one becomes a channel object holding its
 * latest value. UsbTask writes, any task reads with qp_update(): both copy the whole
 * struct inside a short critical section, so a reader never sees a half-written value
 * (ControlTask can preempt UsbTask mid-write; sharing a pointer would tear).
 */
#ifndef USB_PROTO_H
#define USB_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "usb_msgs.h"

#define QP_MAX_PAYLOAD    255U  /* len is u8; per-message length is checked per channel */
#define QP_HDR_LEN        4U
#define QP_CRC_LEN        2U
#define QP_MAX_RAW        (QP_HDR_LEN + QP_MAX_PAYLOAD + QP_CRC_LEN)
#define QP_MAX_ENCODED    (QP_MAX_RAW + (QP_MAX_RAW / 254U) + 1U)

/** Dedupe / stale checks only apply within this window after the last ACKed frame.
 *  Longer than the host's resend timeout, shorter than a host restart. */
#define QP_DEDUPE_MS      1000U

/** STM32 -> Jetson message IDs (not channels: sent by firmware) */
#define QP_ACK            0x83U

/** ACK / validate result codes */
enum {
  QP_OK               = 0,
  QP_ERR_UNKNOWN_TYPE = 1,
  QP_ERR_BAD_LEN      = 2,
  QP_ERR_BAD_ARG      = 3,  /**< rejected by the validate function (NaN, negative gain...) */
  QP_ERR_STALE        = 4,  /**< older seq than the last one on this channel: not applied */
};

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint16_t seq;
  uint8_t  len;
} qp_hdr_t;

/** QP_ACK payload */
typedef struct __attribute__((packed)) {
  uint8_t  ack_type;
  uint16_t ack_seq;
  uint8_t  err;
} qp_ack_t;

/** Channel object: one per message, generated from QP_MESSAGES. Don't touch fields directly. */
typedef struct qp_channel {
  const char *name;
  uint8_t     type;
  uint8_t     len;
  bool        need_ack;
  uint8_t   (*validate)(const void *payload);
  uint8_t    *storage;    /**< len bytes: the latest payload */
  uint32_t    version;    /**< +1 per write, 0 = never written */
  uint32_t    stamp_ms;   /**< tick of last write */
  bool        have_seq;   /**< dedupe state for ACKed channels */
  uint16_t    last_seq;
  uint8_t     last_err;   /**< result sent for last_seq, repeated on a resend */
  uint32_t    last_seq_ms; /**< tick when last_seq arrived */
} qp_channel_t;

/** Link statistics */
typedef struct {
  uint32_t rx_frames;      /**< valid frames stored */
  uint32_t crc_errors;
  uint32_t cobs_errors;    /**< malformed COBS or too short */
  uint32_t overflows;      /**< frame longer than QP_MAX_ENCODED */
  uint32_t bad_len;
  uint32_t unknown_type;
  uint32_t bad_arg;
  uint32_t seq_gaps;       /**< frames the host sent that we never saw */
  uint32_t dup_frames;     /**< repeated seq on ACKed messages (re-ACKed, not re-stored) */
  uint32_t stale_frames;   /**< older seq on ACKed messages (ACK STALE, not stored) */
  uint32_t tx_frames;
  uint32_t tx_drops;       /**< TX FIFO full */
} qp_stats_t;

/**
 * @brief Copy the latest value of a channel if it changed since last_version.
 * @param ch Channel, e.g. &qp_ch_CMD_JOINTS.
 * @param out Buffer of the channel's payload struct; filled only when new data.
 * @param last_version In: version seen last time (start with 0). Out: version copied.
 * @return 1 = new data copied, 0 = nothing new.
 * @note Task context only (FreeRTOS critical section, < 1 us for 144 bytes).
 */
int qp_update(qp_channel_t *ch, void *out, uint32_t *last_version);

/**
 * @brief Milliseconds since the channel was last written (UINT32_MAX if never).
 * @note Use for timeout -> HOLD.
 */
uint32_t qp_age_ms(const qp_channel_t *ch);

/** @brief Feed raw bytes received on CDC0. UsbTask only. */
void qp_rx_bytes(const uint8_t *data, uint32_t n);

/**
 * @brief Encode and send one frame on CDC0. UsbTask only.
 * @return 0 on success, -1 if the TX FIFO had no room (frame dropped).
 */
int qp_send(uint8_t type, const void *payload, uint8_t len);

/** @brief Copy of the link statistics. */
void qp_get_stats(qp_stats_t *out);

/*------------- Building blocks, exposed for tests -------------*/

/** @brief CRC16-CCITT (poly 0x1021, init 0xFFFF). Check value for "123456789" is 0x29B1. */
uint16_t qp_crc16(const uint8_t *data, size_t len);

/**
 * @brief COBS-encode len bytes. The output contains no 0x00.
 * @param out Must hold len + len/254 + 1 bytes.
 * @return Encoded length, without the 0x00 delimiter (the caller appends it).
 */
size_t qp_cobs_encode(const uint8_t *in, size_t len, uint8_t *out);

/**
 * @brief Decode one COBS frame, given without its trailing 0x00.
 * @return Decoded length, or 0 on malformed input or if out_max is too small.
 */
size_t qp_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_max);

/* Generated by usb_msgs.c from QP_MESSAGES */
extern qp_channel_t *const qp_channels[];
extern const size_t qp_channel_count;

#endif /* USB_PROTO_H */
