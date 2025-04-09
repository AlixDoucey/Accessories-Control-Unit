#ifndef DISPLAY_UPDATE_H
#define DISPLAY_UPDATE_H

#include "driveByWire.h"
#include "ecu.h"
#include "quickShifter.h"

#pragma pack(1)

typedef struct __attribute__((packed)) {
  // Quick Shifter //
  QuickShifterState QsState; // 1 byte
  gearStruct gear;           // 2 bytes

  float shifter_position;           // 4 bytes
  float shifter_calibratedPosition; // 4 bytes

  bool gearShiftFail; // 1 byte
  float loadCell;     // 4 bytes

  // Drive by wire //
  positionSensorStruct throttle; // 8 bytes
  float tps;                     // 4 bytes

  // Speeduino //
  EcuData speeduino_data;

  // Errors //
  uint8_t err_msg_len;
  char err_msg[60];
} DisplayData;

#pragma pack()

extern void updateDisplay();

#endif // DISPLAY_UPDATE_H
