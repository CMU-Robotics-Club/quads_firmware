/*
 * ODrive CAN-Simple protocol parsing. Pure C, no HAL (host-tested).
 */
#include "odrive_can.h"

#include <stddef.h>

int odrive_parse_heartbeat(uint32_t id, const uint8_t *data, uint32_t len,
                           uint8_t *node_id, odrive_heartbeat_t *hb)
{
    if (data == NULL || node_id == NULL || hb == NULL) return -1;
    if (odrive_cmd_id(id) != ODRIVE_CMD_HEARTBEAT) return -1;
    if (len < ODRIVE_HEARTBEAT_LEN) return -1;

    /* Assemble byte by byte: little-endian regardless of host endianness, no alignment needs. */
    hb->axis_error = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    hb->axis_state = data[4];
    hb->procedure_result = data[5];
    hb->trajectory_done = data[6] & 0x01U;
    *node_id = odrive_node_id(id);
    return 0;
}

const char *odrive_axis_state_str(uint8_t state)
{
    switch (state) {
    case ODRIVE_AXIS_STATE_UNDEFINED:              return "UNDEFINED";
    case ODRIVE_AXIS_STATE_IDLE:                   return "IDLE";
    case ODRIVE_AXIS_STATE_STARTUP_SEQUENCE:       return "STARTUP_SEQUENCE";
    case ODRIVE_AXIS_STATE_FULL_CALIBRATION:       return "FULL_CALIBRATION";
    case ODRIVE_AXIS_STATE_MOTOR_CALIBRATION:      return "MOTOR_CALIBRATION";
    case ODRIVE_AXIS_STATE_ENCODER_INDEX_SEARCH:   return "ENCODER_INDEX_SEARCH";
    case ODRIVE_AXIS_STATE_ENCODER_OFFSET_CAL:     return "ENCODER_OFFSET_CAL";
    case ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL:    return "CLOSED_LOOP_CONTROL";
    case ODRIVE_AXIS_STATE_LOCKIN_SPIN:            return "LOCKIN_SPIN";
    case ODRIVE_AXIS_STATE_ENCODER_DIR_FIND:       return "ENCODER_DIR_FIND";
    case ODRIVE_AXIS_STATE_HOMING:                 return "HOMING";
    case ODRIVE_AXIS_STATE_ENCODER_HALL_POL_CAL:   return "ENCODER_HALL_POL_CAL";
    case ODRIVE_AXIS_STATE_ENCODER_HALL_PHASE_CAL: return "ENCODER_HALL_PHASE_CAL";
    case ODRIVE_AXIS_STATE_ANTICOGGING_CAL:        return "ANTICOGGING_CAL";
    }
    return "?";
}
