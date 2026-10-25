/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "string.h"
#include "stdio.h"
#include <math.h>
#include "delta.h"
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
volatile PID_t position_pid;
volatile PID_t speed_pid;
volatile uint16_t previous_count = 0;
volatile uint16_t current_count = 0;
volatile int32_t delta_count = 0;
volatile int32_t total_count = 0;
float_t target_position;
float_t dt = 0.02f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  target_position = 13 * 30 * 2 * 2;

  /*初始化pid参数*/
  position_pid.Kd = 0.01;
  position_pid.Ki = 0.5;
  position_pid.Kp = 2;
  position_pid.output_max = 1000;
  position_pid.output_min = -1000;

  speed_pid.Kd = 0.01;
  speed_pid.Ki = 0.5;
  speed_pid.Kp = 2;
  speed_pid.output_max = 1000;
  speed_pid.output_min = -1000;

  PID_Reset(&position_pid);
  PID_Reset(&speed_pid);

  /*启动3个TIM*/
  if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); //启动TIM3——PWM输出
  
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  if(HAL_TIM_Base_Start(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); //启动TIM1——获取编码器CNT
  
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  } //启动TIM2——定时中断

  /*写入初始化方向*/
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    //Tip：PID与PWM是自动执行的，每时每刻都在做事
    /*获取当下的编码器COUNT*/
    current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    delta_count = Encoder_Delta(current_count, previous_count);
    total_count += delta_count;
    
    /*启动PID算法，双环PID串联，并计算相关值*/
    float current_speed = delta_count / dt;

    float target_speed = PID_Update(&position_pid, target_position, total_count, dt);
    float temporary_target = PID_Update(&speed_pid, target_speed, current_speed, dt);
    
    /*判断方向并给CCR赋值*/
    if(temporary_target >= 0)
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, SET);
      
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, temporary_target);
    }
    else if(temporary_target < 0)
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, RESET);

      temporary_target = 0 - temporary_target;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, temporary_target);
    }

    /*更新count*/
    previous_count = current_count;

    /*检测是否稳定在目标值附近 + 不同的运动模式*/
    static uint8_t model = 1;
    static uint32_t start_wait_ms = 0;

    float position_error = target_position - (float)total_count;

    switch(model)
    {
      //所谓model，其实是2个不同运动状态间的检查点，检查上一个运动是否合规，并开启下一个运动
      case 1: //承正转2圈，启等待300ms
        if(Motor_IsArrived(position_error, current_speed))
        {
          model = 2;
          start_wait_ms = HAL_GetTick();
        }
        break;

      case 2: //承等待300ms，启反转2圈
        if((uint32_t)HAL_GetTick() - start_wait_ms >= 300)
        {
          target_position = 0U;
          model = 3;
        }
        break;

      case 3: //承反转2圈，启停止
        if(Motor_IsArrived(position_error, current_speed))
        {
          model = 4;
        }
        break;
    }

  }
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
