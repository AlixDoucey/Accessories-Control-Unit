#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>

extern uint32_t stim_ref_clk;

typedef struct {
  uint32_t start_tick;
} stim;

void stim_start(stim *inst);
void stim_stop(stim *inst);
void stim_reset(stim *inst);
uint32_t stim_get(stim *inst);

#endif // TIMERS_H
