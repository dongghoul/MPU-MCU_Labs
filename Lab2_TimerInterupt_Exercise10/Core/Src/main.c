/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//counter to change the frequency of 4 seven segment leds to 1Hz
const int COUNTER_SCAN_1HZ = 25;
int timer7seg_counter = 0;
int timer7seg_flag = 0;
int timerClock_counter = 0;
int timerClock_flag = 0;
int timerDOT_counter = 0;
int timerDOT_flag = 0;
int timerMatrix_flag = 0;
int timerMatrix_counter = 0;
int TIMER_CYCLE = 10;

void setTimerClock(int duration){
	timerClock_counter = duration / TIMER_CYCLE;
	timerClock_flag = 0;
}

void setTimerDOT(int duration){
	timerDOT_counter = duration / TIMER_CYCLE;
	timerDOT_flag = 0;
}

void setTimer7SEG(int duration){
	timer7seg_counter = duration / TIMER_CYCLE;
	timer7seg_flag = 0;
}

void setTimerMaxtrix(int duration){
	timerMatrix_counter = duration / TIMER_CYCLE;
	timerMatrix_flag = 0;
}

void timerRun(){
	if (timerClock_counter > 0){
		timerClock_counter--;
		if (timerClock_counter == 0) timerClock_flag = 1;
	}
	if (timerDOT_counter > 0){
		timerDOT_counter--;
		if (timerDOT_counter == 0) timerDOT_flag = 1;
	}
	if (timer7seg_counter > 0){
		timer7seg_counter--;
		if (timer7seg_counter == 0) timer7seg_flag = 1;
	}
	if (timer7seg_counter > 0){
		timerMatrix_counter--;
		if (timerMatrix_counter == 0) timerMatrix_flag = 1;
	}
}

void display7SEG(int num){
	//outputs: SEG7 -> SEG0
	//outputs with value of 0 (RESET) will be turn on
	//outputs with value of 1 (SET) will be turn off
	if (num == 0){ //100 0000
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin
								|SEG3_Pin|SEG4_Pin|SEG5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG6_Pin, GPIO_PIN_SET);
	}
	else if (num == 1){ //111 1001
		HAL_GPIO_WritePin(GPIOB, SEG1_Pin|SEG2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG3_Pin|SEG4_Pin
								|SEG5_Pin|SEG6_Pin, GPIO_PIN_SET);
	}
	else if (num == 2){ //010 0100
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG3_Pin
								|SEG4_Pin|SEG6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG2_Pin|SEG5_Pin, GPIO_PIN_SET);
	}
	else if (num == 3){ //011 0000
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin
								|SEG3_Pin|SEG6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG4_Pin|SEG5_Pin, GPIO_PIN_SET);
	}
	else if (num == 4){ //001 1001
		HAL_GPIO_WritePin(GPIOB, SEG1_Pin|SEG2_Pin|SEG5_Pin
								|SEG6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG3_Pin|SEG4_Pin, GPIO_PIN_SET);
	}
	else if (num == 5){ //001 0010
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG2_Pin|SEG3_Pin
								|SEG5_Pin|SEG6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG1_Pin|SEG4_Pin, GPIO_PIN_SET);
	}
	else if (num == 6){ //000 0010
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG2_Pin|SEG3_Pin
								|SEG4_Pin|SEG5_Pin|SEG6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG1_Pin, GPIO_PIN_SET);
	}
	else if (num == 7){ //111 1000
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG3_Pin|SEG4_Pin|SEG5_Pin
								|SEG6_Pin, GPIO_PIN_SET);
	}
	else if (num == 8){ //000 0000
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin
								|SEG3_Pin|SEG4_Pin|SEG5_Pin
								|SEG6_Pin, GPIO_PIN_RESET);
	}
	else if (num == 9){ //001 0000
		HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin
								|SEG3_Pin|SEG5_Pin|SEG6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SEG4_Pin, GPIO_PIN_SET);
	}
}

const int MAX_LED = 4;
int index_led = 0;
int led_buffer[4] = {1, 2, 3, 4};
void update7SEG(int index){
	switch (index){
		case 0:
			//turn on first 7seg led only
			HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
			break;
		case 1:
			//turn on second 7seg led only
			HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
			break;
		case 2:
			//turn on third 7seg led only
			HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);
			break;
		case 3:
			//turn on fourth 7seg led only
			HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
	//display number according to led_buffer and index
	display7SEG(led_buffer[index]);
}

int hour = 23, minute = 58, second = 50;

void updateClockBuffer(){
	led_buffer[0] = hour / 10;
	led_buffer[1] = hour % 10;
	led_buffer[2] = minute / 10;
	led_buffer[3] = minute % 10;
}

const int MAX_LED_MATRIX = 8;
int index_led_matrix = 0;
int index_led_matrix_offset = 0;
uint8_t matrix_buffer[8] = {0xFF, 0x01, 0x00, 0xEC,
						    0xEC, 0x00, 0x01, 0xFF};
void updateLEDMatrix(int index, int offset){
	//set 8 high bits (PB15 -> PB8) of GPIOB's output data register to according buffer
	GPIOB->ODR = (GPIOB->ODR & 0x00FF) | (matrix_buffer[(index + offset) % MAX_LED_MATRIX] << 8);
	//scan each column one-by-one
	switch (index){
		case 0:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM2_Pin|ENM3_Pin
									|ENM4_Pin|ENM5_Pin|ENM6_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM0_GPIO_Port, ENM0_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, ENM0_Pin|ENM2_Pin|ENM3_Pin
									|ENM4_Pin|ENM5_Pin|ENM6_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM1_GPIO_Port, ENM1_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM0_Pin|ENM3_Pin
									|ENM4_Pin|ENM5_Pin|ENM6_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM2_GPIO_Port, ENM2_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM2_Pin|ENM0_Pin
									|ENM4_Pin|ENM5_Pin|ENM6_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM3_GPIO_Port, ENM3_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM2_Pin|ENM3_Pin
									|ENM0_Pin|ENM5_Pin|ENM6_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM4_GPIO_Port, ENM4_Pin, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM2_Pin|ENM3_Pin
									|ENM4_Pin|ENM0_Pin|ENM6_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM5_GPIO_Port, ENM5_Pin, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM2_Pin|ENM3_Pin
									|ENM4_Pin|ENM5_Pin|ENM0_Pin
									|ENM7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM6_GPIO_Port, ENM6_Pin, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOA, ENM1_Pin|ENM2_Pin|ENM3_Pin
									|ENM4_Pin|ENM5_Pin|ENM6_Pin
									|ENM0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ENM7_GPIO_Port, ENM7_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  setTimerClock(10);
  setTimerDOT(1000);
  setTimer7SEG(10);
  setTimerMaxtrix(10);

  int ledDOT_state = 0;
  int scan_counter = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (timerClock_flag == 1){
		  setTimerClock(1000);
		  second++;
		  if (second >= 60){
			  second = 0;
			  minute++;
		  }
		  if (minute >= 60){
			  minute = 0;
			  hour++;
		  }
		  if (hour >= 24){
			  hour = 0;
		  }
		  updateClockBuffer();
	  }

	  if (timer7seg_flag == 1){
		  //set timer for 1Hz frequency
		  setTimer7SEG(250);
		  //updated index_led to stay in valid range, which is from 0 to 3
		  index_led %= MAX_LED;
		  //update 7seg leds
		  update7SEG(index_led++);
	  }

	  if (timerDOT_flag == 1){
		  //1 second blinking
		  setTimerDOT(1000);
		  ledDOT_state = 1 - ledDOT_state;
	  }

	  switch (ledDOT_state){
		  case 0:
			  //turn DOT leds on
			  HAL_GPIO_WritePin(DOT_GPIO_Port, DOT_Pin, RESET);
			  break;
		  case 1:
			  //turn DOT leds off
			  HAL_GPIO_WritePin(DOT_GPIO_Port, DOT_Pin, SET);
			  break;
		  default:
			  break;
	  }

	  if (timerMatrix_flag == 1){
		  setTimerMaxtrix(50);
		  //increase scan counter for each column scanned
		  scan_counter++;
		  index_led_matrix %= MAX_LED_MATRIX;
		  updateLEDMatrix(index_led_matrix++, index_led_matrix_offset);
	  }

	  if (scan_counter >= 8){
		  //increase offset after scanning all 8 columns
		  index_led_matrix_offset++;
		  index_led_matrix_offset %= MAX_LED_MATRIX;
		  //reset scan counter
		  scan_counter = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENM0_Pin|ENM1_Pin|DOT_Pin|LED_RED_Pin
                          |EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG0_Pin|SEG1_Pin|SEG2_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|SEG3_Pin|SEG4_Pin|SEG5_Pin
                          |SEG6_Pin|ROW0_Pin|ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENM0_Pin ENM1_Pin DOT_Pin LED_RED_Pin
                           EN0_Pin EN1_Pin EN2_Pin EN3_Pin
                           ENM2_Pin ENM3_Pin ENM4_Pin ENM5_Pin
                           ENM6_Pin ENM7_Pin */
  GPIO_InitStruct.Pin = ENM0_Pin|ENM1_Pin|DOT_Pin|LED_RED_Pin
                          |EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |ENM2_Pin|ENM3_Pin|ENM4_Pin|ENM5_Pin
                          |ENM6_Pin|ENM7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin ROW2_Pin
                           ROW3_Pin ROW4_Pin ROW5_Pin ROW6_Pin
                           ROW7_Pin SEG3_Pin SEG4_Pin SEG5_Pin
                           SEG6_Pin ROW0_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|ROW2_Pin
                          |ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin
                          |ROW7_Pin|SEG3_Pin|SEG4_Pin|SEG5_Pin
                          |SEG6_Pin|ROW0_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timerRun();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
