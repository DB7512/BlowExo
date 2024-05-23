/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MOTOR_send cmd;             // 以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_recv data;
int blow_position = 0;      // RLS编码器位置
int count = 0;
int usart1_sta = 0;

int motor_sta = 0;          // 电机状态，0：无数据，1：有数据
int rls_sta = 0;            // 编码器状态，0：无数据，1：有数据
int motor_error_count = 0;  // 电机错误次数，累计到10次报错
int motor_data_sta = 0;     // 电机参数解析状态，即motor_r->correct的全局变量，0：解析错误，1：解析成功
int rls_error_count = 0;    // 编码器错误次数，累积到10次报错
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* 电机的当前参数 */
  float motor_position_rad = 0.0;   // 电机位置（rad）
  float motor_speed_rad = 0.0;      // 电机速度（rad/s）
  /* 关节的当前参数 */
  float blow_position_rad = 0.0;    // 肘关节位置（rad）
  float blow_speed_rad = 0.0;       // 肘关节速度（rad/s）
  
  /* 获取电机当前信息 */
  if(motor_sta != 1)
  {
    /** 设置电机工作模式，仅初始查询电机参数
    */
    Servo_Inquire(&motor_position_rad, &motor_speed_rad);
//    printf("motorpos %f \n", data.Pos);
  }
  
  float pos = 0;
  float delta = -0.005;
  
  cmd.id=0;           //给电机控制指令结构体赋值
  cmd.mode=1;
  cmd.T=0;
  cmd.W=-10;
  cmd.Pos=0;
  cmd.K_P=0;
  cmd.K_W=0.05;


  HAL_TIM_Base_Start_IT(&htim2);    // 开启定时器中断
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(motor_error_count > 10)
    {
      printf("motor error \n");
      Error_Handler();
    }
    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
/* 定时器中断回调函数 ----------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    /** 查询电机参数
    */
    modify_data(&cmd);
    // 发送
    SET_485_DE_UP();
    SET_485_RE_UP();
    HAL_UART_Transmit(&huart1, (uint8_t *)&cmd, sizeof(cmd.motor_send_data), 10); 
    // 接收
    SET_485_RE_DOWN();
    SET_485_DE_DOWN();
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&data, sizeof(data.motor_recv_data));
    
    /**查询编码器参数 
    */
    uint8_t encoder_send_buf[ENCODER_SEND_LEN] = {0xFF, 0x03, 0x00, 0x02, 0x00, 0x04, 0xF0, 0x17};
    
    HAL_UART_Transmit(&huart3, encoder_send_buf, ENCODER_SEND_LEN, 10);
    
    HAL_UART_Receive_IT(&huart3, (uint8_t *)encoder_recv_buf, ENCODER_RECV_LEN);
    
    
  }
  
  
  
}

/* 串口中断回调函数 ------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    // 电机串口通信数据处理
    if(sizeof(data.motor_recv_data) > 0)
    {
      // 解析电机状态参数
      uint8_t *rp = (uint8_t *)&data.motor_recv_data;
      if(rp[0] == 0xFD && rp[1] == 0xEE)
      {
        data.correct = 1;
        motor_data_sta = extract_data(&data);       // 均转化为弧度制单位
        if(motor_data_sta == 0)
        {
          motor_error_count++;
        }
        else
        {
          motor_error_count = 0;
          printf("MotorPos %f \n", data.Pos);
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
          if(data.Pos < 20 && data.Pos > -20)
          {
            cmd.id=0;           //给电机控制指令结构体赋值
            cmd.mode=1;
            cmd.T=0;
            cmd.W=0;
            cmd.Pos=0;
            cmd.K_P=0;
            cmd.K_W=0;
            printf("stop \n");
          }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&data, sizeof(data.motor_recv_data));
      }
    }
    else
    {
      motor_error_count++;
    }
  }
  else if(huart->Instance==USART2)
  {
//    Encoder_Send_recv(&blow_position);
//    printf("encoder data %d \n", blow_position);

    
  }
  else if(huart->Instance==USART3)
  {
    if(encoder_recv_buf[0] == 0xFF)
    {
      printf("encoder data %s \n", encoder_recv_buf);
      Extract_Encoder_Data(encoder_recv_buf, &blow_position);
//      printf("encoder data %d \n", blow_position);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
  }
}

/* 查询电机当前位置和速度 */
void Servo_Inquire(float* position, float* speed)
{
  cmd.id   = 0;           //给电机控制指令结构体赋值
  cmd.mode = 1;
  cmd.T    = 0;
  cmd.W    = 0;
  cmd.Pos  = 0.0;
  cmd.K_P  = 0.1;
  cmd.K_W  = 0.01;
//  SERVO_Send_recv(&cmd, &data);
//  *position = data.Pos;
//  *speed = data.W;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
