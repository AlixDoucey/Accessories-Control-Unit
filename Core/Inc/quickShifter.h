#ifndef QUICK_SHIFTER_H
#define QUICK_SHIFTER_H

#include "filter.h"
#include "main.h"
#include "stdbool.h"

#define IgnitionCutTime 100
#define UpShiftDeadZone 50
#define DownShiftDeadZone 50

typedef enum __attribute__((packed)) {
  WaitingForInput = 0,
  WaitingForUpShift,
  WaitingForDownShift,
} QuickShifterState;

extern QuickShifterState QsState;

typedef struct {
  float position;
  float calibratedPosition;
  uint16_t pending_samples;
} shifterStruct;

extern shifterStruct shifter;

typedef struct {
  uint8_t engaged;
  uint8_t prevEngaged; // previously engaged
} gearStruct;

extern gearStruct gear;

#define LOADCELL_BUF_LEN 100

typedef struct {
  int32_t buf[LOADCELL_BUF_LEN];
  float val;
  uint16_t pending_samples;
} loadCellStruct;

extern loadCellStruct loadCell;

typedef enum __attribute__((packed)) { Success = 0, Fail } ShiftStatus;

extern bool gearShiftFail;

float shifter_getPosition(filter *inst);
float loadCell_getVal(filter *inst);
void shifter_calibrate();
void shifter_tryRecalibrate();
void gear_get();
void gear_set();
ShiftStatus wait_for_upShift();
ShiftStatus wait_for_downShift();

#endif // QUICK_SHIFTER_H
