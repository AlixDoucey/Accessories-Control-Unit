#include "software_timer.h"
#include "stm32h5xx_hal.h"

void stim_start(stim *inst) {
  inst->start_tick = HAL_GetTick();
}

uint32_t stim_get(stim *inst){
  return HAL_GetTick() - inst->start_tick;
}
