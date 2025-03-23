/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NAU7802.h"
#include "main.h"
#include "software_timer.h"
#include "tx_api.h"
// #include "displayUpdate.h"
#include "driveByWire.h"
#include "filter.h"
#include "gps.h"
#include "quickShifter.h"
#include "steeringLeds.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 1024
#define TRACE_BUF_SIZE 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t QuickShifter_stack[THREAD_STACK_SIZE];
uint8_t SteeringLeds_stack[THREAD_STACK_SIZE];
uint8_t DriveByWire_stack[THREAD_STACK_SIZE];
uint8_t Gps_stack[THREAD_STACK_SIZE];
uint8_t DisplayUpdate_stack[THREAD_STACK_SIZE];

TX_THREAD QuickShifter_handler;
TX_THREAD SteeringLeds_handler;
TX_THREAD DriveByWire_handler;
TX_THREAD Gps_handler;
TX_THREAD DisplayUpdate_handler;

uint8_t trace_buf[TRACE_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID QuickShifter_entry(ULONG intial_input);
VOID SteeringLeds_entry(ULONG intial_input);
VOID DriveByWire_entry(ULONG intial_input);
VOID Gps_entry(ULONG intial_input);
VOID DisplayUpdate_entry(ULONG intial_input);

stim shifterTimer;
stim throttleTimer;
stim adcTimer;

VOID dummy_expiration_function(ULONG dummy_parameter) {};
/* USER CODE END PFP */

/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT App_ThreadX_Init(VOID *memory_ptr) {
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */
  /* USER CODE BEGIN App_ThreadX_Init */

  tx_trace_enable(&trace_buf, TRACE_BUF_SIZE, 30);

  /* Create ThreadOne.  */
  tx_thread_create(&QuickShifter_handler, "QuickShifter", QuickShifter_entry, 0x1234, QuickShifter_stack,
                   THREAD_STACK_SIZE, 10, 10, 1, TX_AUTO_START);
  tx_thread_create(&DriveByWire_handler, "DriveByWire", DriveByWire_entry, 0x1234, DriveByWire_stack, THREAD_STACK_SIZE,
                   10, 10, 1, TX_DONT_START);
  tx_thread_create(&Gps_handler, "Gps", Gps_entry, 0x1234, Gps_stack, THREAD_STACK_SIZE, 9, 10, 1, TX_DONT_START);
  tx_thread_create(&DisplayUpdate_handler, "DisplayUpdate", DisplayUpdate_entry, 0x1234, DisplayUpdate_stack,
                   THREAD_STACK_SIZE, 8, 10, 1, TX_DONT_START);
  tx_thread_create(&SteeringLeds_handler, "SteeringLeds", SteeringLeds_entry, 0x1234, SteeringLeds_stack,
                   THREAD_STACK_SIZE, 7, 10, 1, TX_DONT_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
 * @brief  Function that implements the kernel's initialization.
 * @param  None
 * @retval None
 */
void MX_ThreadX_Init(void) {
  /* USER CODE BEGIN Before_Kernel_Start */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
#define MS_TO_TICKS(milliseconds) (((milliseconds) * TX_TIMER_TICKS_PER_SECOND) / 1000UL)

void qs_update_sensors(filter *instShifter, filter *instLoadCell) {
  tx_thread_sleep(MS_TO_TICKS(3));
  shifter.position = shifter_getPosition(instShifter);
  loadCell.val = loadCell_getVal(instLoadCell);
}

void dbw_update_sensors() {
  throttle.position = throttleGetPosition();
  tx_thread_sleep(MS_TO_TICKS(4));
}

/////////////// TASKS ////////////////

VOID QuickShifter_entry(ULONG intial_input) {
#define ignitionCutTime MS_TO_TICKS(100)
  while (1) {
    tx_thread_sleep(MS_TO_TICKS(100));
  }

  filter shifterFilter;
  shifterFilter.pending_samples = &shifter.pending_samples;
  filter_create(&shifterFilter, filter1, UINT16, ADC_BUF_LEN, ADC_NBR_CONVERSION, 3, adc_buf, &pending_samples);

  filter loadCellFilter;
  loadCellFilter.pending_samples = &loadCell.pending_samples;
  filter_create(&loadCellFilter, filter2, INT32, LOADCELL_BUF_LEN, 1, 1, loadCell.buf, &loadCell.pending_samples);

  /*

  NAU_begin(&hi2c1, 1);

  tx_thread_sleep(MS_TO_TICKS(200));
  qs_update_sensors(&shifterFilter, &loadCellFilter);
  shifter_calibrate();

  gear_get();
  gear_set();
  while (1) {
    qs_update_sensors(&shifterFilter, &loadCellFilter);
    shifter_tryRecalibrate();

    // Up shift
    // TODO: include the load cell in the if statement
    if (shifter.position > (shifter.calibratedPosition + UpShiftDeadZone) && gear.engaged != 4) {
      gear.prevEngaged = gear.engaged;

    UpShift:
      if (gear.prevEngaged != 0) {
        HAL_GPIO_WritePin(ClutchOut_GPIO_Port, ClutchOut_Pin, GPIO_PIN_SET);
      }

      if (wait_for_upShift() == Fail) {
        goto DownShift;
      }

      gear_set();

      tx_thread_sleep(ignitionCutTime);

      HAL_GPIO_WritePin(ClutchOut_GPIO_Port, ClutchOut_Pin, GPIO_PIN_RESET);

      while (shifter.position > (shifter.calibratedPosition + 30)) {
        qs_update_sensors(&shifterFilter);
      }

      QsState = WaitingForInput;

      //reset_timer(&shifterTimer);
    }

    // Down shift
    // TODO: include the load cell in the if statement
    if (shifter.position < (shifter.calibratedPosition - DownShiftDeadZone) && gear.engaged != 0) {
      gear.prevEngaged = gear.engaged;

    DownShift:
      if (wait_for_downShift() == Fail) {
        goto UpShift;
      }

      gear_set();

      if (HAL_GPIO_ReadPin(ClutchOut_GPIO_Port, ClutchOut_Pin) == GPIO_PIN_SET) {
        tx_thread_sleep(ignitionCutTime);
        HAL_GPIO_WritePin(ClutchOut_GPIO_Port, ClutchOut_Pin, GPIO_PIN_RESET);
      }

      while (shifter.position < (shifter.calibratedPosition - 30)) {
        qs_update_sensors(&shifterFilter);
      }

      QsState = WaitingForInput;

      //reset_timer(&shifterTimer);
    }
    qs_update_sensors(&shifterFilter);
    */
  //}
}

VOID SteeringLeds_entry(ULONG intial_input) {
  ledInit();

  // LEDs startup animation
  tx_thread_sleep(MS_TO_TICKS(100));
  // Loop through the LEDs from 0 to 10
  for (int i = 0; i < 11; i++) {
    ledSetColor(i, colors[i][0], colors[i][1], colors[i][2]);
    ledSend(0.4);
    tx_thread_sleep(MS_TO_TICKS(100));
  }

  tx_thread_sleep(MS_TO_TICKS(500));

  for (float i = 0.4; i >= 0; i -= 0.0025) {
    ledSend(i);
    tx_thread_sleep(MS_TO_TICKS(5));
  }

  while (1) {
    if (dataSent == false) {
      ledUpdate();
      ledSend(0.4);
      dataSent = true;
    }
    tx_thread_sleep(MS_TO_TICKS(16));
  }
}

VOID DriveByWire_entry(ULONG intial_input) {
  initThrottle();

  tx_thread_sleep(MS_TO_TICKS(250));

  throttleCalibrate();

  tx_thread_suspend(&SteeringLeds_handler);
  tx_thread_suspend(&Gps_handler);
  tx_thread_suspend(&DisplayUpdate_handler);

  stepperCalibrate();
  stim_start(&throttleTimer);

  while (1) {
    // throttleTryRecalibrate();
    throttleBodySetPosition(throttle.position);
    dbw_update_sensors();
  }
}

VOID Gps_entry(ULONG intial_input) {
  // GNSS_Init(&GNSS_Handle, &huart3);
  // vTaskDelay(tickDelay1000);
  // GNSS_LoadConfig(&GNSS_Handle);
  // vTaskDelay(tickDelay100);
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gpsBuf, 1000);

  while (1) {
    // GNSS_GetPVTData(&GNSS_Handle);
    tx_thread_sleep(MS_TO_TICKS(100));
    // GNSS_ParsePVTData(&GNSS_Handle);

    // Store the parsed latitude and longitude into variables
    // latitude = GNSS_Handle.fLat;  // Latitude in decimal degrees
    // longitude = GNSS_Handle.fLon; // Longitude in decimal degrees
  }
}

VOID DisplayUpdate_entry(ULONG intial_input) {
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart4, (uint8_t *)displayBuf, 10);

  while (1) {
    tx_thread_sleep(MS_TO_TICKS(100));
  }
}
/* USER CODE END 1 */
