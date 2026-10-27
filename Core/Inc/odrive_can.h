/*
 * ODrive CAN-Simple protocol: frame layout and parsing.
 *
 * Pure C, no HAL, so it can be host-tested (tests/test_odrive_can.c).
 *
 * Arbitration ID = node_id << 5 | cmd_id (11-bit standard ID: 6-bit node, 5-bit cmd).
 * Layouts verified against Laika-Software can_simple_messages.hpp (ODrive fw v0.6.x).
 */
#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <stdint.h>

#define ODRIVE_CMD_HEARTBEAT   0x01U
#define ODRIVE_CMD_GET_ENCODER 0x09U

#define ODRIVE_CMD_MASK 0x1FU
#define ODRIVE_NODE_SHIFT 5U

/* Filter + mask that match the heartbeat of any node: compare only the 5 cmd_id bits. */
#define ODRIVE_HEARTBEAT_FILTER_ID   ODRIVE_CMD_HEARTBEAT
#define ODRIVE_HEARTBEAT_FILTER_MASK ODRIVE_CMD_MASK

#define ODRIVE_HEARTBEAT_LEN 8U

/* AxisState (ODrive odrive_enums.h) */
#define ODRIVE_AXIS_STATE_UNDEFINED            0U
#define ODRIVE_AXIS_STATE_IDLE                 1U
#define ODRIVE_AXIS_STATE_STARTUP_SEQUENCE     2U
#define ODRIVE_AXIS_STATE_FULL_CALIBRATION     3U
#define ODRIVE_AXIS_STATE_MOTOR_CALIBRATION    4U
#define ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH 6U
#define ODRIVE_AXIS_STATE_ENCODER_OFFSET_CAL   7U
#define ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL  8U
#define ODRIVE_AXIS_STATE_LOCKIN_SPIN          9U
#define ODRIVE_AXIS_STATE_ENCODER_DIR_FIND     10U
#define ODRIVE_AXIS_STATE_HOMING               11U
#define ODRIVE_AXIS_STATE_ENCODER_HALL_POL_CAL 12U
#define ODRIVE_AXIS_STATE_ENCODER_HALL_PHASE_CAL 13U
#define ODRIVE_AXIS_STATE_ANTICOGGING_CAL      14U

typedef struct {
    uint32_t axis_error;      /* bytes 0-3, little-endian */
    uint8_t axis_state;       /* byte 4 */
    uint8_t procedure_result; /* byte 5 */
    uint8_t trajectory_done;  /* byte 6 bit 0 */
} odrive_heartbeat_t;

static inline uint8_t odrive_node_id(uint32_t id) { return (uint8_t)(id >> ODRIVE_NODE_SHIFT); }
static inline uint8_t odrive_cmd_id(uint32_t id) { return (uint8_t)(id & ODRIVE_CMD_MASK); }
static inline uint32_t odrive_make_id(uint8_t node_id, uint8_t cmd_id)
{
    return ((uint32_t)node_id << ODRIVE_NODE_SHIFT) | (cmd_id & ODRIVE_CMD_MASK);
}

/* Parse a received frame as a heartbeat.
 * Returns 0 and fills node_id and hb on success.
 * Returns -1 (outputs untouched) if cmd_id is not heartbeat or len < 8. */
int odrive_parse_heartbeat(uint32_t id, const uint8_t *data, uint32_t len,
                           uint8_t *node_id, odrive_heartbeat_t *hb);

/* Short name for an AxisState value, "?" if unknown. Never NULL. */
const char *odrive_axis_state_str(uint8_t state);

#endif /* ODRIVE_CAN_H */
