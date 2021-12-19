/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "animation_graph.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define DEBUG_MODE

#define MIN_PWM_VALUE 0
#define MAX_PWM_VALUE 499

#define MIN_COLOR_TEMPERATURE_VALUE 0
#define MAX_COLOR_TEMPERATURE_VALUE 255

#define MIN_BRIGHTNESS_VALUE 0
#define MAX_BRIGHTNESS_VALUE 255

typedef struct {
    uint8_t id;
    uint8_t active;
    uint8_t brightness;
    uint8_t colorTemperature;

    uint16_t warmPwmValue;
    uint16_t coldPwmValue;

    uint16_t targetWarmPwmValue;
    uint16_t targetColdPwmValue;

//    uint16_t warmAnimationStep;
//    uint16_t coldAnimationStep;
//
    uint16_t warmAnimationProgress;
    uint16_t coldAnimationProgress;

    TIM_HandleTypeDef *warmTimerHandle;
    uint32_t warmTimerChannel;
    volatile uint32_t *warmTimerCCR;
    TIM_HandleTypeDef *coldTimerHandle;
    uint32_t coldTimerChannel;
    volatile uint32_t *coldTimerCCR;

    uint16_t animationWarmChart[ANIMATION_STEPS_NUMBER];
    uint16_t animationColdChart[ANIMATION_STEPS_NUMBER];
    bool isAnimationInProgress;
    uint16_t animationStep;
} ChannelState_t;

void timerTick();

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
#define W5500_RESET_Pin GPIO_PIN_4
#define W5500_RESET_GPIO_Port GPIOA
#define BOARD_LED_Pin GPIO_PIN_5
#define BOARD_LED_GPIO_Port GPIOA
#define W5500_CS_Pin GPIO_PIN_0
#define W5500_CS_GPIO_Port GPIOB
#define CH4Cold_Pin GPIO_PIN_14
#define CH4Cold_GPIO_Port GPIOB
#define CH4Warm_Pin GPIO_PIN_15
#define CH4Warm_GPIO_Port GPIOB
#define CH3Cold_Pin GPIO_PIN_8
#define CH3Cold_GPIO_Port GPIOA
#define CH3Warm_Pin GPIO_PIN_9
#define CH3Warm_GPIO_Port GPIOA
#define CH2Cold_Pin GPIO_PIN_6
#define CH2Cold_GPIO_Port GPIOC
#define CH2Warm_Pin GPIO_PIN_7
#define CH2Warm_GPIO_Port GPIOC
#define CH1Cold_Pin GPIO_PIN_10
#define CH1Cold_GPIO_Port GPIOA
#define CH1Warm_Pin GPIO_PIN_11
#define CH1Warm_GPIO_Port GPIOA
#define CH0Cold_Pin GPIO_PIN_8
#define CH0Cold_GPIO_Port GPIOC
#define CH0Warm_Pin GPIO_PIN_9
#define CH0Warm_GPIO_Port GPIOC
#define CH4Button_Pin GPIO_PIN_1
#define CH4Button_GPIO_Port GPIOD
#define CH3Button_Pin GPIO_PIN_2
#define CH3Button_GPIO_Port GPIOD
#define CH2Button_Pin GPIO_PIN_3
#define CH2Button_GPIO_Port GPIOD
#define CH1Button_Pin GPIO_PIN_4
#define CH1Button_GPIO_Port GPIOD
#define CH0Button_Pin GPIO_PIN_5
#define CH0Button_GPIO_Port GPIOD
#define BrokerStatusN_Pin GPIO_PIN_6
#define BrokerStatusN_GPIO_Port GPIOD
#define BrokerStatus_Pin GPIO_PIN_3
#define BrokerStatus_GPIO_Port GPIOB
#define EthernetStatus_Pin GPIO_PIN_4
#define EthernetStatus_GPIO_Port GPIOB
#define EthernetStatusN_Pin GPIO_PIN_5
#define EthernetStatusN_GPIO_Port GPIOB
#define CH4Status_Pin GPIO_PIN_6
#define CH4Status_GPIO_Port GPIOB
#define CH3Status_Pin GPIO_PIN_7
#define CH3Status_GPIO_Port GPIOB
#define CH2Status_Pin GPIO_PIN_8
#define CH2Status_GPIO_Port GPIOB
#define CH1Status_Pin GPIO_PIN_9
#define CH1Status_GPIO_Port GPIOB
#define CH0Status_Pin GPIO_PIN_10
#define CH0Status_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
