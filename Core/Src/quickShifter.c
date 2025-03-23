#include "quickShifter.h"
#include "app_threadx.h"
#include "main.h"
#include "software_timer.h"

#include "tx_api.h"
#include <stdint.h>

QuickShifterState QsState = WaitingForInput;

bool shifting = false;
bool gearShiftFail = false;

shifterStruct shifter;
gearStruct gear;
loadCellStruct loadCell;

float shifter_getPosition(filter *inst) { return filter_process_sample(inst); }
float loadCell_getVal(filter *inst) { return filter_process_sample(inst); }

void shifter_calibrate() { shifter.calibratedPosition = shifter.position; }

void shifter_tryRecalibrate() {
  // TODO: check if the force sensor is near 0 as well;
  if (stim_get(&shifterTimer) > 1500) {
    shifter_calibrate();
  }
}

void gear_get() {
  static const GPIO_TypeDef *ports[] = {GearN_GPIO_Port, Gear1_GPIO_Port, Gear2_GPIO_Port, Gear3_GPIO_Port,
                                        Gear4_GPIO_Port};
  static const uint16_t pins[] = {GearN_Pin, Gear1_Pin, Gear2_Pin, Gear3_Pin, Gear4_Pin};

  gear.engaged = 99; // Default value

  for (int i = 0; i < 5; i++) {
    if (HAL_GPIO_ReadPin(ports[i], pins[i]) == GPIO_PIN_RESET) {
      gear.engaged = i;
      return;
    }
  }
}

void gear_set() {
  switch (gear.engaged) {
  case 0:
    HAL_GPIO_WritePin(GearN_GPIO_Port, GearN_Pin, GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(Gear1_GPIO_Port, Gear1_Pin, GPIO_PIN_SET);
    break;
  case 2:
    HAL_GPIO_WritePin(Gear2_GPIO_Port, Gear2_Pin, GPIO_PIN_SET);
    break;
  case 3:
    HAL_GPIO_WritePin(Gear3_GPIO_Port, Gear3_Pin, GPIO_PIN_SET);
    break;
  case 4:
    HAL_GPIO_WritePin(Gear4_GPIO_Port, Gear4_Pin, GPIO_PIN_SET);
    break;
  }
}

// TODO: add shiftingTimout and update the display

ShiftStatus wait_for_upShift() {
  QsState = WaitingForUpShift;

  do {
    // TODO: include the load cell in the if statement
    // Upshift failed, user shifts back into lower gear
    if (shifter.position < (shifter.calibratedPosition - DownShiftDeadZone)) {
      gear.prevEngaged++;
      return Fail;
    }
    gear_get();
    qs_update_sensors();
  } while ((gear.engaged != (gear.prevEngaged + 1)));

  return Success;
}

ShiftStatus wait_for_downShift() {
  QsState = WaitingForDownShift;

  do {
    // TODO: include the load cell in the if statement
    // Downshift failed, user shifts back into upper gear
    if (shifter.position > (shifter.calibratedPosition + UpShiftDeadZone)) {
      gear.prevEngaged--;
      return Fail;
    }
    gear_get();
    qs_update_sensors();
  } while ((gear.engaged != (gear.prevEngaged - 1)));

  return Success;
}
