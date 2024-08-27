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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HC_SR04.h"
#include "pid.h"
#include "motor.h"
#include "wit.h"
//#include <stdio.h>
#include "RPM_Encoder.h"
#include "cmsis_os.h"
//#include "uartstdio.h"
#include "ctype.h"
#include "math.h"
#include "string.h"
//#include "PS2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define SS_Pin GPIO_PIN_4
#define SS_GPIO_Port GPIOA
#define DIRECTION_3_Pin GPIO_PIN_13
#define DIRECTION_3_GPIO_Port GPIOE
#define DIRECTION_2_Pin GPIO_PIN_14
#define DIRECTION_2_GPIO_Port GPIOE
#define DIRECTION_1_Pin GPIO_PIN_15
#define DIRECTION_1_GPIO_Port GPIOE
#define HC_Output_Pin GPIO_PIN_8
#define HC_Output_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define spi_enable 		HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET)
#define spi_disable  	HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_SET)

//#define Forward 0xef
//#define Backward 0xbf
//#define Left 0x7f
//#define Right 0xdf
//#define IDLE 0xff
//#define Forward_Left 0x6f
//#define Forward_Right 0xcf
//#define Backward_Left 0x3f
//#define Backward_Right 0x9f

#define Select_Hand_1 0xfe
#define Select_Hand_2 0xfd
//#define Select_Hand_3 0xfb


#define Rotate_180 0xef
#define Rotate_mn180 0xbf

#define Rotate_Left 0x7f
#define Rotate_Right 0xdf

#define Lift 0xef
#define Down 0xbf
#define Close 0x7f
#define Open 0xdf

#define Fast 0xf7
#define Slow 0xfb

#define true 1
#define false 0
#define N 500
#define speed 600
#define ticktak 10
#define delay_trans portMAX_DELAY

#define right 1
#define left 0
#define donot 2
#define no 3


#define tolerance 5
#define Angle_FORWARD 60
#define Angle_BACKWARD 240
#define Angle_RIGHT 330
#define Angle_LEFT 150

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern PID_Param_t pid;

extern void pid_config(void);
extern void pid__motion_config(void);

extern double rpm_1, rpm_2, rpm_3;
extern double out_1, out_2, out_3;

extern osThreadId_t taskReadButtonHandle;
extern osMessageQueueId_t AxisDesireQueueHandle;
extern osMessageQueueId_t StableHandle;
extern osThreadId_t CONTROLHandle;
extern osThreadId_t AxisXYDesireHandle;
extern osThreadId_t WitMotionHandle;

typedef struct {                                // object data type
  char buffer;
  uint8_t buffer_index;
} msgQueueObj_t;

typedef enum {FIRST,DESIRE,REPEAT,GO,PAUSE,RESTART} state_t;
typedef enum {FORWARD,BACKWARD,LEFT,RIGHT,NO} run_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
