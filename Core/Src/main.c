/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "Display_API.h"

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

struct display_data {

	uint8_t fsm;
	uint8_t index;

} display_handle = { .fsm = 0, .index = 7, };

int i = 0;
int number;
int var = 0;
int idx = 0;
int idx2 = 0;
int unid, dez, cent;
int fsm = 0;
int index1 = 0;
int display_clock = 0;
int downcounter=0;

int digits[11][8] = {

{ 0, 1, 1, 1, 1, 1, 1, 0 }, //digit 0
		{ 0, 0, 1, 1, 0, 0, 0, 0 }, //digit 1
		{ 0, 1, 1, 0, 1, 1, 0, 1 }, //digit 2
		{ 0, 1, 1, 1, 1, 0, 0, 1 }, //digit 3
		{ 0, 0, 1, 1, 0, 0, 1, 1 }, //digit 4
		{ 0, 1, 0, 1, 1, 0, 1, 1 }, //digit 5
		{ 0, 1, 0, 1, 1, 1, 1, 1 }, //digit 6
		{ 0, 1, 1, 1, 0, 0, 0, 0 }, //digit 7
		{ 0, 1, 1, 1, 1, 1, 1, 1 }, //digit 8
		{ 0, 1, 1, 1, 1, 0, 1, 1 },  //digit 9
		{ 0, 0, 0, 0, 0, 0, 0, 0 }  //null
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void DigitExtract(int);
void Display(int,int);
int AnalogHandler(int);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t readValue;

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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_GPIO_WritePin(GPIOC, DIG1, 0);
	HAL_GPIO_WritePin(GPIOC, DIG2, 0);
	HAL_GPIO_WritePin(GPIOC, DIG3, 0);
	HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		HAL_ADC_PollForConversion(&hadc1, 1000);
		readValue = HAL_ADC_GetValue(&hadc1);

		var = AnalogHandler(readValue);

		DigitExtract(var);
		Display(unid, 1);
		Display(dez, 2);
		if(cent==0){
			cent=10;
		}
		Display(cent, 3);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIGIT3_Pin|DIGIT2_Pin|DIGIT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHCP_pin_Pin|STCP_pin_Pin|DS_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIGIT3_Pin DIGIT2_Pin DIGIT1_Pin */
  GPIO_InitStruct.Pin = DIGIT3_Pin|DIGIT2_Pin|DIGIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHCP_pin_Pin STCP_pin_Pin DS_pin_Pin */
  GPIO_InitStruct.Pin = SHCP_pin_Pin|STCP_pin_Pin|DS_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int AnalogHandler(int Value){

	Value = Value*100/4095;

	if ( Value > 100){
		Value = 100;
	}else if ( Value < 0){
		Value = 0;
	}

	return Value;

}

//void Display(struct display_data *handle){
void Display(int value, int digit) {

	/* Versão com delay*/


	downcounter=100;while(downcounter>0){};
		HAL_GPIO_WritePin(GPIOB, STCP_pin, 0); // INICIO DA MENSAGEM
//		HAL_Delay(0.05);

	for (int index = 7; index >= 0; index--) {


			HAL_GPIO_WritePin(GPIOB, SHCP_pin, 0); //CLOCK LOW
			downcounter=10;while(downcounter>0);
			//HAL_Delay(0.01);

			if (digits[value][index] == 1) {
				HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data HIGH
			}

			else if (digits[value][index] == 0) {
				HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data LOW
			}
			HAL_GPIO_WritePin(GPIOB, SHCP_pin, 1); //CLOCK HIGH
			downcounter=10;while(downcounter>0);
			//HAL_Delay(0.01);
	}

	HAL_GPIO_WritePin(GPIOB, STCP_pin, 1);  // fim da mensagem (SEND ou LATCH)

	if (digit == 1) {
		HAL_GPIO_WritePin(GPIOC, DIG1, 0);			//HAL_Delay(.1);
		HAL_GPIO_WritePin(GPIOC, DIG2, 0);			//HAL_Delay(.1);
		HAL_GPIO_WritePin(GPIOC, DIG3, 1);			//HAL_Delay(.1);
		downcounter=10;while(downcounter>0);
	} else if (digit == 2) {
		HAL_GPIO_WritePin(GPIOC, DIG1, 0);			//HAL_Delay(.1);
		HAL_GPIO_WritePin(GPIOC, DIG2, 1);			//HAL_Delay(.1);
		HAL_GPIO_WritePin(GPIOC, DIG3, 0);			//HAL_Delay(.1);
		downcounter=10;while(downcounter>0);
	} else {
		HAL_GPIO_WritePin(GPIOC, DIG1, 1);			//HAL_Delay(.1);
		HAL_GPIO_WritePin(GPIOC, DIG2, 0);			//HAL_Delay(.1);
		HAL_GPIO_WritePin(GPIOC, DIG3, 0);			//HAL_Delay(.1);
		downcounter=10;while(downcounter>0);
	}







	/* Versão não funcional

	if (display_clock == 0) {
		HAL_GPIO_WritePin(GPIOB, STCP_pin, 0); // INICIO DA MENSAGEM
	}

	else if (display_clock == 2 || display_clock == 4 || display_clock == 6
			|| display_clock == 8 || display_clock == 10 || display_clock == 12
			|| display_clock == 14 || display_clock == 14) {

		//INTERVALO
	}

	else if (display_clock >= 15) {

		if (digits[value][index1] == 1) {
			HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data HIGH
		}

		else if (digits[value][index1] == 0) {
			HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data LOW
		}

		//HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, STCP_pin, 1); // FIM DA MENSAGEM
	}

	else {

		if (digits[value][index1] == 1) {
			HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data HIGH
		}

		else if (digits[value][index1] == 0) {
			HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data LOW
		}
	}
		 */

}



void DigitExtract(int num) {

//	int unid, dez, cent;
	unid = num % 10;
	num = num / 10;
	dez = num % 10;
	cent = num / 10;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) //frequencia do timer é de 5 kHz, st=200us
	{

		//atualizar a contagem a cada 1000 ms
		if (idx < 501) {

			idx++;
		} else {
			//var = var + 1;

			if (var > 500) {
			//	var = 0;
			}

			idx = 1;
		}

		//cada 1 ms
//		if (idx2 < 5) {
//
//			idx2++;
//		} else {
//
//			idx2 = 0;
//
//			if (downcounter > 0) {
//				downcounter--;
//			}
//		}

	}

	if (htim->Instance == TIM3) //frequencia do timer é de 50 kHz, time=20us
	{

		if (downcounter > 0) {
			downcounter--;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
