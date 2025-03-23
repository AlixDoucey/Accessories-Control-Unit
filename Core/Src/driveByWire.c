#include "driveByWire.h"
#include "main.h"
#include <math.h>

//--User Variables Start--//
const float THROTTLE_PEDAL_DEADZONE = 30;
const float THROTTLE_PEDAL_MAX = 3800.0f;
const uint16_t MICRO_STEPPING = 4;
//--User Variables End--//

const float STEP = 1.8f / MICRO_STEPPING;
const float TOTAL_STEP_COUNT = 146.0f / STEP; // 146.0 -> degrees

positionSensorStruct throttle;

bool timerExpired = 0;

typedef enum {
  CW = 0,
  CCW = 1,
} RotationDirection;

uint16_t stepperPosition = 0;
float Tps = 0.0f;
uint32_t stepPwmData[400];

void initThrottle() {
  throttle.rawPosition = 0;
  throttle.position = 0.0f;
  throttle.calibratedPosition = 0;

  for (uint16_t i = 0; i < TOTAL_STEP_COUNT; i++) {
    stepPwmData[i] = 50;
  }
}

// Throttle pedal //

float throttleNormalizePosition(uint16_t adcAverage) {
  if (adcAverage < throttle.calibratedPosition) {
    adcAverage = throttle.calibratedPosition;
  } else if (adcAverage > THROTTLE_PEDAL_MAX) {
    adcAverage = THROTTLE_PEDAL_MAX;
  }

  return (adcAverage - throttle.calibratedPosition) / (THROTTLE_PEDAL_MAX - throttle.calibratedPosition);
}

float throttleGetPosition() {
  uint32_t adcAverage = 0;

  // Step by 3 to get indices 0, 3, 6, etc.
  for (uint16_t i = 0; i < ADC_BUF_LEN; i += 3) {
    adcAverage += adc_buf[i];
  }

  adcAverage /= (ADC_BUF_LEN / 3);

  throttle.rawPosition = adcAverage; // debug
  return throttleNormalizePosition((uint16_t)adcAverage);
}

void throttleCalibrate() {
  throttle.position = throttleGetPosition();
  throttle.calibratedPosition = throttle.rawPosition + THROTTLE_PEDAL_DEADZONE;
}

void throttleTryRecalibrate() {
  /*
  if (hasThrottleTimerExpired()) {
          timerExpired = 1;
          throttleCalibrate();
  }
  */
}

void delayMicroseconds(uint32_t us) {
  __HAL_TIM_SET_COUNTER(&htim12, 0); // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim12) < us)
    ;
}

// get absolute value //

float absf(float value) {
  if (value < 0) {
    value = value * -1;
  }
  return value;
}

// Stepper //

void stepperSetDirection(RotationDirection dir) {
  HAL_GPIO_WritePin(StepperDir_GPIO_Port, StepperDir_Pin, (uint8_t)dir);
}

void stepperRotateToStep(float newPos) {
  float stepsDelta = roundf(newPos - stepperPosition);
  uint16_t stepCount = absf(stepsDelta);

  if (stepCount != 0) {
    RotationDirection direction;
    if (stepsDelta > 0) {
      direction = CCW;
    } else {
      direction = CW;
    }
    stepperSetDirection(direction);

    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)stepPwmData, stepCount);
    // vTaskSuspend(DriveByWire_Handler);

    if (direction == CCW) {
      stepperPosition += stepCount;
    } else {
      stepperPosition -= stepCount;
    }

    Tps = stepperPosition / TOTAL_STEP_COUNT;
  }
}

// speed 0.0 to 1.0
void stepperStep() {
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)stepPwmData, 5);
  // vTaskSuspend(DriveByWire_Handler);
}

void throttleBodySetPosition(float pos) { stepperRotateToStep(pos * TOTAL_STEP_COUNT); }

void stepperCalibrate() {

  while (HAL_GPIO_ReadPin(ThrottleCalibration_GPIO_Port, ThrottleCalibration_Pin) != GPIO_PIN_RESET) {
    stepperSetDirection(CCW);
    stepperStep();
    // vTaskDelay(20);
  }
  // vTaskDelay(50);
  stepperPosition = 92;

  throttleBodySetPosition(0.0);
}
