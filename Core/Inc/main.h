/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern DMA_HandleTypeDef handle_GPDMA2_Channel0;
extern DMA_HandleTypeDef handle_GPDMA2_Channel1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;

// handle_GPDMA1_Channel0 USART1_RX
// handle_GPDMA1_Channel1 USART1_TX
// handle_GPDMA1_Channel2 ADC1

#define ADC_BUF_LEN 150
extern uint16_t adc_buf[];

#define USART1_RX_BUF_LEN 100
#define USART1_TX_BUF_LEN 2
extern uint8_t rx_buf_usart1[USART1_RX_BUF_LEN];

extern uint8_t ADC_NBR_CONVERSION;

extern uint16_t pending_samples;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ThrottlePos_Pin GPIO_PIN_0
#define ThrottlePos_GPIO_Port GPIOC
#define ClutchPos_Pin GPIO_PIN_1
#define ClutchPos_GPIO_Port GPIOC
#define ShifterPos_Pin GPIO_PIN_2
#define ShifterPos_GPIO_Port GPIOC
#define EcuStep_Pin GPIO_PIN_5
#define EcuStep_GPIO_Port GPIOC
#define EcuDir_Pin GPIO_PIN_0
#define EcuDir_GPIO_Port GPIOB
#define StepperDir_Pin GPIO_PIN_2
#define StepperDir_GPIO_Port GPIOB
#define TwoStepButton_Pin GPIO_PIN_9
#define TwoStepButton_GPIO_Port GPIOC
#define ThrottleCalibration_Pin GPIO_PIN_8
#define ThrottleCalibration_GPIO_Port GPIOA
#define ClutchOut_Pin GPIO_PIN_9
#define ClutchOut_GPIO_Port GPIOA
#define Gear4_Pin GPIO_PIN_11
#define Gear4_GPIO_Port GPIOC
#define GearN_Pin GPIO_PIN_12
#define GearN_GPIO_Port GPIOC
#define Gear2_Pin GPIO_PIN_2
#define Gear2_GPIO_Port GPIOD
#define Gear1_Pin GPIO_PIN_3
#define Gear1_GPIO_Port GPIOB
#define Gear3_Pin GPIO_PIN_4
#define Gear3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
