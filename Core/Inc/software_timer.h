#ifndef TIMERS_H
#define TIMERS_H
// Sofware timer --> s_timer --> stim

#include "main.h"

typedef struct {
  uint32_t start_tick;
} stim;

void stim_start(stim *inst);
uint32_t stim_get(stim *inst);

#endif // TIMERS_H
