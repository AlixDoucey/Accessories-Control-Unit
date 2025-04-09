#include "software_timer.h"

// clang-format off

uint32_t stim_ref_clk = 0xFFFFFFFF;

void stim_start(stim *inst) { 
  if (inst->start_tick == 0xFFFFFFFF)
    inst->start_tick = stim_ref_clk; 
}

void stim_stop(stim *inst) {
  inst->start_tick = 0xFFFFFFFF;
}

void stim_reset(stim *inst) { 
  inst->start_tick = stim_ref_clk;
}

uint32_t stim_get(stim *inst) {
  if (inst->start_tick == 0xFFFFFFFF)
    return 0;

  return stim_ref_clk - inst->start_tick;
}
