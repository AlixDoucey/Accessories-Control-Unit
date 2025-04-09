#ifndef DRIVE_BY_WIRE_H
#define DRIVE_BY_WIRE_H

#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)

typedef struct __attribute__((packed)) {
  uint16_t rawPosition;        // 12bits
  uint16_t calibratedPosition; // 12bits
  float position;              // normalized
} positionSensorStruct;

#pragma pack()

extern positionSensorStruct throttle;
extern uint32_t stepPwmData[];

extern float Tps; // normalized

void initThrottle();
float throttleGetPosition();
void throttleCalibrate();
void throttleTryRecalibrate();
void throttleBodySetPosition(float pos);

void stepperCalibrate();

extern bool timerExpired;

#endif // DRIVE_BY_WIRE_H
