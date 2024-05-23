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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "RLS_encoder.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_1
#define RS485_RE_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_2
#define RS485_DE_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ENCODER_SEND_LEN 8
#define ENCODER_RECV_LEN 13

extern MOTOR_send cmd;
extern MOTOR_recv data;
extern uint8_t encoder_recv_buf[ENCODER_RECV_LEN];

extern int count;
extern int usart1_sta;

extern int motor_error_count;  // 电机错误次数，累计到10次报错
extern int motor_sta;          // 电机状态，0：停止，1：运动
extern int motor_data_sta;     // 电机参数解析状态，即motor_r->correct的全局变量
extern int rls_error_count;    // 编码器错误次数，累积到10次报错

void Servo_Inquire(float* position, float* speed);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
