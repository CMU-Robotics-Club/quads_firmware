#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "odrive.h"

void ODrive_SetAxisState(uint8_t bus_num, uint8_t can_id, AxisState state){
  if(bus_num == 1){
    //TODO
  }
  else{
    printf("ERROR: UNSUPPORTED BUS NUMBER: %d\r\n", bus_num);
  }
}

void ODrive_ClearErrors(uint8_t bus_num, uint8_t can_id){
  
}
