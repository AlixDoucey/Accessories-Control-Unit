#ifndef DISPLAY_UPDATE_H
#define DISPLAY_UPDATE_H

#include "driveByWire.h"
#include "quickShifter.h"

typedef struct {
  // Quick Shifter //
  QuickShifterState QsState; // 1 byte
  gearStruct gear;           // 2 bytes
  shifterStruct shifter;     // 8 bytes

  bool gearShiftFail; // 1 byte
  float loadCell;     // 4 bytes

  // Drive by wire //
  positionSensorStruct throttle; // 8 bytes
  float tps;                     // 4 bytes

  // Speeduino //

  // Errors //
  uint8_t err_msg_len;
  char err_msg[50];
} DisplayData;

#endif // DISPLAY_UPDATE_H
