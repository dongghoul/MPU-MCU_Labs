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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display7SEG(int num){
	//leds with value of 0 will be turn on
	//leds with value of 1 will be turn off
	if (num == 0){ //1111 1111 1100 0000
		GPIOB->ODR = 0xFFC0;
	}
	else if (num == 1){ //1111 1111 1111 1001
		GPIOB->ODR = 0xFFF9;
	}
	else if (num == 2){ //1111 1111 1010 0100
		GPIOB->ODR = 0xFFA4;
	}
	else if (num == 3){ //1111 1111 1011 0000
		GPIOB->ODR = 0xFFB0;
	}
	else if (num == 4){ //1111 1111 1001 1001
		GPIOB->ODR = 0xFF99;
	}
	else if (num == 5){ //1111 1111 1001 0010
		GPIOB->ODR = 0xFF92;
	}
	else if (num == 6){ //1111 1111 1000 0010
		GPIOB->ODR = 0xFF82;
	}
	else if (num == 7){ //1111 1111 1111 1000
		GPIOB->ODR = 0xFFF8;
	}
	else if (num == 8){ //1111 1111 1000 0000
		GPIOB->ODR = 0xFF80;
	}
	else if (num == 9){ //1111 1111 1001 0000
		GPIOB->ODR = 0xFF90;
	}
}

void trafficLight(int *counter, int *state, GPIO_TypeDef* red, uint16_t redPin
									, GPIO_TypeDef* yellow, uint16_t yellowPin
									, GPIO_TypeDef* green, uint16_t greenPin)
{
	if (*state == 0){
		//turn red led on
		HAL_GPIO_WritePin(red, redPin, GPIO_PIN_SET);
		//switch to green led after 5 seconds
		if (*counter <= 0){
			*counter = 3; //3 seconds of green
			*state = 1;
			HAL_GPIO_WritePin(red, redPin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(green, greenPin, GPIO_PIN_SET);
		}
	}
	else if (*state == 1){
		//turn green led on
		HAL_GPIO_WritePin(green, greenPin, GPIO_PIN_SET);
		//switch to yellow led after 3 seconds
		if (*counter <= 0){
			*counter = 2; //2 seconds of yellow
			*state = 2;
			HAL_GPIO_WritePin(green, greenPin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(yellow, yellowPin, GPIO_PIN_SET);
		}
	}
	else if (*state == 2){
		//turn yellow led on
		HAL_GPIO_WritePin(yellow, yellowPin, GPIO_PIN_SET);
		//switch to red led after 2 seconds
		if (*counter <= 0){
			*counter = 5; //5 seconds of red
			*state = 0;
			HAL_GPIO_WritePin(yellow, yellowPin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(red, redPin, GPIO_PIN_SET);
		}
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
  /* USER CODE BEGIN 2 */
  int horizontal_counter = 5;
  int horizontal_state = 0; //horizontal road starts with 5 seconds of red
  int vertical_counter = 3;
  int vertical_state = 1; //vertical road starts with 3 seconds of green
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	trafficLight(&horizontal_counter, &horizontal_state
								 , HORIZONTAL_RED_GPIO_Port, HORIZONTAL_RED_Pin
								 , HORIZONTAL_YELLOW_GPIO_Port, HORIZONTAL_YELLOW_Pin
								 , HORIZONTAL_GREEN_GPIO_Port, HORIZONTAL_GREEN_Pin);
	trafficLight(&vertical_counter, &vertical_state
								 , VERTICAL_RED_GPIO_Port, VERTICAL_RED_Pin
								 , VERTICAL_YELLOW_GPIO_Port, VERTICAL_YELLOW_Pin
								 , VERTICAL_GREEN_GPIO_Port, VERTICAL_GREEN_Pin);
	//display counter for horizontal road
	display7SEG(horizontal_counter);
	horizontal_counter--;
	vertical_counter--;
	HAL_Delay(1000);
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
  HAL_GPIO_WritePin(GPIOA, HORIZONTAL_RED_Pin|HORIZONTAL_YELLOW_Pin|HORIZONTAL_GREEN_Pin|VERTICAL_RED_Pin
                          |VERTICAL_YELLOW_Pin|VERTICAL_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a_Pin|b_Pin|c_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HORIZONTAL_RED_Pin HORIZONTAL_YELLOW_Pin HORIZONTAL_GREEN_Pin VERTICAL_RED_Pin
                           VERTICAL_YELLOW_Pin VERTICAL_GREEN_Pin */
  GPIO_InitStruct.Pin = HORIZONTAL_RED_Pin|HORIZONTAL_YELLOW_Pin|HORIZONTAL_GREEN_Pin|VERTICAL_RED_Pin
                          |VERTICAL_YELLOW_Pin|VERTICAL_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a_Pin b_Pin c_Pin d_Pin
                           e_Pin f_Pin g_Pin */
  GPIO_InitStruct.Pin = a_Pin|b_Pin|c_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
