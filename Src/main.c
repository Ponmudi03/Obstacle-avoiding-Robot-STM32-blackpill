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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IN1_Pin GPIO_PIN_0
#define IN2_Pin GPIO_PIN_1
#define IN3_Pin GPIO_PIN_2
#define IN4_Pin GPIO_PIN_3
#define IN5_Pin GPIO_PIN_4
#define IN6_Pin GPIO_PIN_5
#define IN7_Pin GPIO_PIN_6
#define IN8_Pin GPIO_PIN_7
#define TRIG_PIN GPIO_PIN_15
#define TRIG_PORT GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;


osThreadId defaultTaskHandle;
osThreadId ultrasonic_readHandle;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
uint8_t Distance1  = 0;
uint8_t Distance2 = 0;
uint8_t Get_Averaged_Distance(void);
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void HS04_READ(void const * argument);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void forward(void);
void reverse(void);
void right(void);
void left(void);
void stop(void);
void servo0(void);
void servo90(void);
void servo180(void);





void delay_us(uint16_t us)
 {
   __HAL_TIM_SET_COUNTER(&htim1, 0);  // Set the counter value to 0
   HAL_TIM_Base_Start(&htim1);  // Start the timer

   while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // Wait until the counter reaches the us input value

   HAL_TIM_Base_Stop(&htim1);  // Stop the timer
 }

 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 {
     if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
     {
         if (Is_First_Captured == 0) // if the first value is not captured
         {
             IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
             Is_First_Captured = 1;  // set the first captured as true
             // Now change the polarity to falling edge
             __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
         }
         else if (Is_First_Captured == 1)   // if the first is already captured
         {
             IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
             __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
             if (IC_Val2 > IC_Val1)
             {
                 Difference = IC_Val2 - IC_Val1;
             }
             else if (IC_Val1 > IC_Val2)
             {
                 Difference = (0xffff - IC_Val1) + IC_Val2;
             }

             Distance = (Difference * 0.034f) / 2.0f; // Use floating point arithmetic
             Is_First_Captured = 0; // set it back to false

             // Set polarity to rising edge
             __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
             __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
         }
     }
 }

 void HCSR04_Read(void)
 {
     HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
     delay_us(10);  // wait for 10 us
     HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

     __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
     HAL_Delay(60); // Add a delay of at least 60ms (or more) between successive measurements
 }
 uint8_t Get_Averaged_Distance(void)
 {
     uint8_t num_samples = 5;
     uint32_t sum = 0;
     for (uint8_t i = 0; i < num_samples; i++)
     {
         HCSR04_Read();
         HAL_Delay(100);  // wait for the reading to complete
         sum += Distance;
     }
     return sum / num_samples;
 }

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
 int main(void) {
     HAL_Init();
     SystemClock_Config();
     MX_GPIO_Init();
     MX_TIM3_Init();
     MX_TIM1_Init();
     MX_USART1_UART_Init();

     // RTOS initialization
     osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
     defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

     osThreadDef(ultrasonic_read, HS04_READ, osPriorityRealtime, 0, 128);
     ultrasonic_readHandle = osThreadCreate(osThread(ultrasonic_read), NULL);

     osKernelStart(); // Start the RTOS scheduler

     while (1) {
         // Main loop logic
         start:
         HAL_Delay(200);
         forward();
         HAL_Delay(200);

         if ((Distance >= 14 && Distance <= 16) || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) {
             stop();
             HAL_Delay(1000);
             reverse();
             HAL_Delay(1000);
             stop();

             // Measure distances at different angles
             servo0();
             HAL_Delay(1000);
             HCSR04_Read(); // You may want to trigger an immediate read here if necessary
             HAL_Delay(100);
             Distance1 = Distance;

             servo180();
             HAL_Delay(1000);
             HCSR04_Read(); // Trigger another read if necessary
             HAL_Delay(100);
             Distance2 = Distance;

             servo90();
             HAL_Delay(1000);

             // Decide direction based on distances
             if (Distance1 > Distance2) {
                 right();
                 HAL_Delay(1000);
                 goto start;
             } else if (Distance2 > Distance1) {
                 left();
                 HAL_Delay(1000);
                 goto start;
             }
         }
     }
 }

  /* USER CODE END 3 */
void forward(void)
{
	 HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET); // IN1
	    HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
	    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET); // IN3
	    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
	    HAL_GPIO_WritePin(GPIOA, IN5_Pin, GPIO_PIN_SET); // IN1
	   	    HAL_GPIO_WritePin(GPIOA, IN6_Pin, GPIO_PIN_RESET); // IN2
	   	    HAL_GPIO_WritePin(GPIOA, IN7_Pin, GPIO_PIN_SET); // IN3
	   	    HAL_GPIO_WritePin(GPIOA, IN8_Pin, GPIO_PIN_RESET); // IN4

}
void reverse(void)
{
	 HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
	    HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET); // IN2
	    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
	    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET); // IN4
	    HAL_GPIO_WritePin(GPIOA, IN5_Pin, GPIO_PIN_RESET); // IN1
	    	    HAL_GPIO_WritePin(GPIOA, IN6_Pin, GPIO_PIN_SET); // IN2
	    	    HAL_GPIO_WritePin(GPIOA, IN7_Pin, GPIO_PIN_RESET); // IN3
	    	    HAL_GPIO_WritePin(GPIOA, IN8_Pin, GPIO_PIN_SET); // IN4
}
void right(void)
{
	 HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_SET); // IN1
		    HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
		    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
		    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_SET); // IN4
		    HAL_GPIO_WritePin(GPIOA, IN5_Pin, GPIO_PIN_RESET); // IN1
		   	    HAL_GPIO_WritePin(GPIOA, IN6_Pin, GPIO_PIN_SET); // IN2
		   	    HAL_GPIO_WritePin(GPIOA, IN7_Pin, GPIO_PIN_SET); // IN3
		   	    HAL_GPIO_WritePin(GPIOA, IN8_Pin, GPIO_PIN_RESET); // IN4
}
void left(void)
{
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
		    HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_SET); // IN2
		    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_SET); // IN3
		    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
		    HAL_GPIO_WritePin(GPIOA, IN5_Pin, GPIO_PIN_SET); // IN1
		   	    HAL_GPIO_WritePin(GPIOA, IN6_Pin, GPIO_PIN_RESET); // IN2
		   	    HAL_GPIO_WritePin(GPIOA, IN7_Pin, GPIO_PIN_RESET); // IN3
		   	    HAL_GPIO_WritePin(GPIOA, IN8_Pin, GPIO_PIN_SET); // IN4
}
void stop(void)
{
	HAL_GPIO_WritePin(GPIOA, IN1_Pin, GPIO_PIN_RESET); // IN1
			    HAL_GPIO_WritePin(GPIOA, IN2_Pin, GPIO_PIN_RESET); // IN2
			    HAL_GPIO_WritePin(GPIOA, IN3_Pin, GPIO_PIN_RESET); // IN3
			    HAL_GPIO_WritePin(GPIOA, IN4_Pin, GPIO_PIN_RESET); // IN4
			    HAL_GPIO_WritePin(GPIOA, IN5_Pin, GPIO_PIN_RESET); // IN1
			   	    HAL_GPIO_WritePin(GPIOA, IN6_Pin, GPIO_PIN_RESET); // IN2
			   	    HAL_GPIO_WritePin(GPIOA, IN7_Pin, GPIO_PIN_RESET); // IN3
			   	    HAL_GPIO_WritePin(GPIOA, IN8_Pin, GPIO_PIN_RESET); // IN4
}
void servo0(void)
{
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
}
void servo90(void)
{
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
}
void servo180(void)
{
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2500);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_HS04_READ */
/**
* @brief Function implementing the ultrasonic_read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HS04_READ */
void HS04_READ(void const * argument)
{
    for (;;)
    {
        HCSR04_Read();
        osDelay(1000);
    }
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
