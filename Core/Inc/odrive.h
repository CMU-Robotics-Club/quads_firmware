#pragma once
#include <stdint.h>

typedef enum {
  AXIS_STATE_IDLE = 0,
  AXIS_STATE_CLOSED_LOOP_CONTROL,
} AxisState;

void ODrive_SetAxisState(uint8_t bus_num, uint8_t can_id, AxisState state);
void ODrive_ClearErrors(uint8_t bus_num, uint8_t can_id);
