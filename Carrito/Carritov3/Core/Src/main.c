/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t state = 0;


volatile uint8_t pulses_rf;
volatile uint8_t pulses_lf;
volatile uint8_t pulses_rb;
volatile uint8_t pulses_lb;
uint8_t pulsesperturn = 20;
float rpm_rf;
float rpm_lf;
float rpm_rb;
float rpm_lb;

uint8_t current_tick_rf;
uint8_t current_tick_lf;
uint8_t current_tick_rb;
uint8_t current_tick_lb;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
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
	char rpmdata[1400];
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);


  uint32_t start_time, elapsed_time, current_time;





  uint8_t last_ticked_rf = 0;
  uint8_t last_ticked_lf = 0;
  uint8_t last_ticked_rb = 0;
  uint8_t last_ticked_lb = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 170);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 170);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 170);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 170);



	  switch(state){

		  case 1:
			  //Forward path
			  //Poner en libreria

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //izq atras
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //izq atras
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //derehca atras
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0); //derecha atrasa
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0); //derecha enfrente
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); //derecha enfrente
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //izq enfrente
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // ezq enfrente

			  break;

		  case 2:
			  //Right turn
			  //Poner en libreria

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);


			  break;

		  default:
			  break;


	  }

	  current_time = HAL_GetTick();
	  elapsed_time = current_time - start_time;


	  if(elapsed_time > 1000){

		  rpm_rf = ((current_tick_rf - last_ticked_rf) * 1000 *	60) / (20 * 1000);
		  rpm_lf = ((current_tick_lf - last_ticked_lf) * 1000 *	60) / (20 * 1000);
		  rpm_rb = ((current_tick_rb - last_ticked_rb) * 1000 *	60) / (20 * 1000);
		  rpm_lb = ((current_tick_lb - last_ticked_lb) * 1000 *	60) / (20 * 1000);



		  last_ticked_rf = current_tick_rf;
		  last_ticked_lf = current_tick_lf;
		  last_ticked_rb = current_tick_rb;
		  last_ticked_lb = current_tick_lb;


		  current_tick_rf = 0;
		  current_tick_lf = 0;
		  current_tick_rb = 0;
		  current_tick_lb = 0;


		 uint32_t millis = HAL_GetTick();
		 uint8_t num_chars =  sprintf(rpmdata, "Time: %ul \n\r Right Front: %.2f \n\r Left Front: %.2f \n\r  Right Back: %.2f \n\r Left Back: %.2f \n\r", millis, rpm_rf , rpm_lf, rpm_rb, rpm_lb);
		 //uint8_t num_chars =  sprintf(rpmdata, "%.2f , %.2f , %.2f, , %.2f , %.2f", start_time, rpm_rf , rpm_lf, rpm_rb, rpm_lb);

		  HAL_UART_Transmit(&huart1, &rpmdata, num_chars, 10);




	  }

		  if(state == 0){
			  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1){
				  start_time = HAL_GetTick();
				  elapsed_time = 0;
				  state++;
			  }
		  }
		  else if(state == 1){
			  if(elapsed_time > 5000){
				  start_time = HAL_GetTick();
				  elapsed_time = 0;
				  state++;
			  }
		  }
		  else if(state == 2){
			  if(elapsed_time > 1000){

				  state = 1;
			  }
		  }


	  /*
	  // Parte 1: Mover hacia adelante
	          start_time = HAL_GetTick();
	          elapsed_time = 0;
	          while (elapsed_time < 5000) {
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //izq atras
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //izq atras
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //derehca atras
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0); //derecha atrasa
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0); //derecha enfrente
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); //derecha enfrente
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //izq enfrente
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // ezq enfrente
	              current_time = HAL_GetTick();
	              elapsed_time = current_time - start_time;
	          }

	          // Parte 2: Girar a la derecha
	          start_time = HAL_GetTick();
	          elapsed_time = 0;
	          while (elapsed_time < 5000) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	              current_time = HAL_GetTick();
	              elapsed_time = current_time - start_time;
	          }
	          */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

	HAL_UART_TxCpltCallback(&huart1);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  if(GPIO_Pin == GPIO_PIN_15){
	  //Right front wheel
	  pulses_rf++;
	  current_tick_rf += pulses_rf;
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

  }
  if(GPIO_Pin == GPIO_PIN_14){
	  //Left front wheel
	  pulses_lf++;
	  current_tick_lf += pulses_lf;
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

  }
  if(GPIO_Pin == GPIO_PIN_13){
	  //Right back wheel
	  pulses_rb++;
	  current_tick_rb += pulses_rb;
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

  }
  if(GPIO_Pin == GPIO_PIN_12){
	  //Left back wheel
	  pulses_lb++;
	  current_tick_lb += pulses_lb;
	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
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
