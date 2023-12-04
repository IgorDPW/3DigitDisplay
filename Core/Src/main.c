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
#include "math.h"
#include "Display_API.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Parâmetros utilizados para leitura dos sensores analógicos */
#define BSP_ADC_CHANNEL_SIZE 3
#define BSP_ADC_FILTER_SAMPLES_SIZE 6 //valor anterior = 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

/* Buffer utilizado pelo DMA para armazenar os valores lidos pelo ADC */
static volatile uint16_t Buffer_ADC_DMA[BSP_ADC_CHANNEL_SIZE];

/* Buffer utilizado para armazenar as ultimas x amostras obtidas pelo ADC */
static volatile uint16_t Buffer_ADC_Filter[BSP_ADC_CHANNEL_SIZE][BSP_ADC_FILTER_SAMPLES_SIZE];

uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t counterOutside = 0; //For testing only
uint32_t counterInside = 0; //For testing only

int i = 0;
int number;
int16_t var;
int16_t Value[3];
int idx = 0;
int idx2 = 0;
int unid, dez, cent,unid_vet[8],dez_vet[8],cent_vet[8];
int fsm = 0;
int index1 = 0;
int display_clock = 0;
int downcounter = 0;
int downcounter_timer4= 0;
int counter_timer4= 0;
int myIndex=7;

int brilho =23;
int color,Red=100,Green=200,Blue=255;

enum color {
	branco,
	eco,
	verde,
	amarelo,
	vermelho,
	azul,
};

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
		{ 0, 1, 1, 1, 1, 0, 1, 1 }, //digit 9
		{ 0, 0, 0, 0, 0, 0, 0, 0 }  //null
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DigitExtract(int);
void Display(int,int);
void AnalogHandler(void);
void LEDHandler(int);
void ColorModeSelect(void);
void Set_LED (int, int , int , int );
void Set_Brightness(int);
void WS2512_Send (void);
void ADC_Handler(void);
void Analog_Buffer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_LED 20			//numero máximo de leds para acender em sequencia
#define MAX_Brightness 45	// brilho máximo entre 0 e 45
#define USE_BRIGHTNESS 1

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];	//para o brilho

uint16_t readValue[3];

int32_t SpeedMode;
int32_t Sensor_in;
int32_t Sensor_Out;

int datasentflag = 0;


void ADC_Select_CH0 (void){
	/** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CH1 (void){
	/** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Select_CH9 (void){
	/** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_GPIO_WritePin(GPIOC, DIG1, 1);
	HAL_GPIO_WritePin(GPIOC, DIG2, 2);
	HAL_GPIO_WritePin(GPIOC, DIG3, 3);
	HAL_ADC_Start(&hadc1);

	 WS2512_Send();

	 counter_timer4=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		ADC_Handler();						//realiza captação dos valores analógicos

		Analog_Buffer();						//Buffer para estabilização dos sinais

		AnalogHandler();					//Conversão dos sinais

		var = Value[1];						//variável utilizada para acionamento display
		DigitExtract(var);					//Atribuição dos valores usados nos 3 digitos 7 segmentos

		ColorModeSelect();					//Identificação do Modo de atuação do SPEED

		var = Value[2];						//variável utilizada para acionamento dos LEDs
		LEDHandler(var);					//Acionamento dos LEDs WS2812  utilizando o sinal do Speed

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_0;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_9;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIGIT3_Pin|DIGIT2_Pin|DIGIT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHCP_pin_Pin|STCP_pin_Pin|DS_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ORIGINAL_Pin|ECO_Pin|SPORT_Pin|PERFORMANCE_Pin
                          |TRACK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIGIT3_Pin DIGIT2_Pin DIGIT1_Pin */
  GPIO_InitStruct.Pin = DIGIT3_Pin|DIGIT2_Pin|DIGIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTAO_Pin */
  GPIO_InitStruct.Pin = BOTAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOTAO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHCP_pin_Pin STCP_pin_Pin DS_pin_Pin */
  GPIO_InitStruct.Pin = SHCP_pin_Pin|STCP_pin_Pin|DS_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ORIGINAL_Pin ECO_Pin SPORT_Pin PERFORMANCE_Pin
                           TRACK_Pin */
  GPIO_InitStruct.Pin = ORIGINAL_Pin|ECO_Pin|SPORT_Pin|PERFORMANCE_Pin
                          |TRACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ADC_Handler() {

	ADC_Select_CH0();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);		//TODO definir se está correto. Padrão = 1000
	readValue[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	readValue[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH9();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	readValue[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

}

void AnalogHandler() {

	//Adequa a escala de cada sinal

		////conversao de 0 a 100% para uso do pedal Ranger
		int max1 = 2290;
		int min1 = 560;

		int max2 = 2370;
		int min2 = 545;

		Value[1] = ( Sensor_in - min1 ) *100 / (max1 - min1 );
		if ( Value[1] > 100){
			Value[1] = 100;
		}else if ( Value[1] < 0){
			Value[1] = 0;
		}

		Value[2] = ( Sensor_Out - min2 ) *100 / (max2 - min2 );
		if ( Value[2] > 100){
			Value[2] = 100;
		}else if ( Value[2] < 0){
			Value[2] = 0;
		}

}

//TODO Documentar
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//Rotina de tratamento da interrupção externa

	currentMillis = HAL_GetTick();

	if ((currentMillis - previousMillis) > 200) {

		//brilho = brilho + 5;

//		switch (color) {
//
//		case branco:
//			Red = 255;
//			Green = 255;
//			Blue = 255;
//
//			color = verde;
//
//			break;
//
//		case eco:
//			Red = 0;
//			Green = 110;
//			Blue = 255;
//
//			color = amarelo;
//
//		case verde:
//			Red = 0;
//			Green = 255;
//			Blue = 0;
//
//			color = amarelo;
//			break;
//
//		case amarelo:
//			Red = 255;
//			Green = 255;
//			Blue = 0;
//
//			color = vermelho;
//			break;
//
//		case vermelho:
//			Red = 255;
//			Green = 0;
//			Blue = 0;
//
//			color = azul;
//			break;
//
//		case azul:
//			Red = 0;
//			Green = 0;
//			Blue = 255;
//
//			color = branco;
//
//			break;
//
//		default:
//			break;

//		}


//		if (brilho >= 45) {
//			brilho = 1;
//		}



		previousMillis = currentMillis;

	}
}

//TODO Documentar método
void ColorModeSelect() {

	//Define o modo de atuação SPEED

	if(SpeedMode<430){			//modo original
		Value[0] = 0;
	} else if(SpeedMode<800){	//modo Eco
		Value[0] = 1;
	} else if(SpeedMode<1200){	//modo Sport
		Value[0] = 2;
	} else if(SpeedMode<1510){	//modo Performance
		Value[0] = 3;
	} else if(SpeedMode<2000){	//modo Track
		Value[0] = 4;
	} else if(SpeedMode<2500){	//modo Valet
			Value[0] = 4;
	}

	switch (Value[0]) {

	case branco:
		Red = 255;
		Green = 255;
		Blue = 255;

		HAL_GPIO_WritePin(GPIOA, ORIGINAL_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ECO_Pin | SPORT_Pin | PERFORMANCE_Pin | GPIO_PIN_5,
				GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

		break;

	case eco:
		Red = 0;
		Green = 180;
		Blue = 255;

		HAL_GPIO_WritePin(GPIOA, ECO_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ORIGINAL_Pin | SPORT_Pin | PERFORMANCE_Pin | GPIO_PIN_5,
				GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);


		break;

	case verde:
		Red = 0;
		Green = 255;
		Blue = 0;

		HAL_GPIO_WritePin(GPIOA, SPORT_Pin , GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ECO_Pin | ORIGINAL_Pin | PERFORMANCE_Pin | GPIO_PIN_5,
				GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

		break;

	case amarelo:
		Red = 255;
		Green = 255;
		Blue = 0;

		HAL_GPIO_WritePin(GPIOA, PERFORMANCE_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ECO_Pin | SPORT_Pin | ORIGINAL_Pin | GPIO_PIN_5,
				GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

		break;

	case vermelho:
		Red = 255;
		Green = 0;
		Blue = 0;

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ECO_Pin | SPORT_Pin | PERFORMANCE_Pin | ORIGINAL_Pin,
				GPIO_PIN_SET);

		break;

	case azul:
		Red = 0;
		Green = 0;
		Blue = 255;

		HAL_GPIO_WritePin(GPIOA, ORIGINAL_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ECO_Pin | SPORT_Pin | PERFORMANCE_Pin | GPIO_PIN_5,
				GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

		break;

	default:

		HAL_GPIO_WritePin(GPIOA, ORIGINAL_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOA,
				ECO_Pin | SPORT_Pin | PERFORMANCE_Pin | GPIO_PIN_5,
				GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

		break;

	}


}

void LEDHandler(int Valor) {


	//brilho = Value * MAX_Brightness / 100;

	//lógica para acionamento sequencial
	if (Valor == 0 ) {
		Set_LED(0, 0, 0, 0);
		Set_LED(1, 0, 0, 0);
		Set_LED(2, 0, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	}
	if (Valor >= 1 && Valor < 6) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, 0, 0, 0);
		Set_LED(2, 0, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);



	} else if (Valor >= 6 && Valor < 12) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, 0, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 12 && Valor < 18) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, 0, 0, 0);
		Set_LED(3, 0, 0, 0);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 18 && Valor < 24) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, 0, 0, 0);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 24 && Valor < 30) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, 0, 0, 0);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 30 && Valor < 36) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, 0, 0, 0);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 36 && Valor < 42) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, 0, 0, 0);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 42 && Valor < 48) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, 0, 0, 0);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 48 && Valor < 54) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, 0, 0, 0);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 54 && Valor < 60) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, 0, 0, 0);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 60 && Valor < 66) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, Red, Green, Blue);
		Set_LED(11, 0, 0, 0);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 66 && Valor < 72) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, Red, Green, Blue);
		Set_LED(11, Red, Green, Blue);
		Set_LED(12, 0, 0, 0);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 72 && Valor < 78) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, Red, Green, Blue);
		Set_LED(11, Red, Green, Blue);
		Set_LED(12, Red, Green, Blue);
		Set_LED(13, 0, 0, 0);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 78 && Valor < 84) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10,Red, Green, Blue);
		Set_LED(11, Red, Green, Blue);
		Set_LED(12, Red, Green, Blue);
		Set_LED(13, Red, Green, Blue);
		Set_LED(14, 0, 0, 0);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 84 && Valor < 90) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, Red, Green, Blue);
		Set_LED(11, Red, Green, Blue);
		Set_LED(12, Red, Green, Blue);
		Set_LED(13, Red, Green, Blue);
		Set_LED(14, Red, Green, Blue);
		Set_LED(15, 0, 0, 0);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 90 && Valor < 96) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, Red, Green, Blue);
		Set_LED(11, Red, Green, Blue);
		Set_LED(12, Red, Green, Blue);
		Set_LED(13, Red, Green, Blue);
		Set_LED(14, Red, Green, Blue);
		Set_LED(15, Red, Green, Blue);
		Set_LED(16, 0, 0, 0);
		Set_LED(17, 0, 0, 0);

	} else if (Valor >= 96 && Valor < 101) {
		Set_LED(0, Red, Green, Blue);
		Set_LED(1, Red, Green, Blue);
		Set_LED(2, Red, Green, Blue);
		Set_LED(3, Red, Green, Blue);
		Set_LED(4, Red, Green, Blue);
		Set_LED(5, Red, Green, Blue);
		Set_LED(6, Red, Green, Blue);
		Set_LED(7, Red, Green, Blue);
		Set_LED(8, Red, Green, Blue);
		Set_LED(9, Red, Green, Blue);
		Set_LED(10, Red, Green, Blue);
		Set_LED(11, Red, Green, Blue);
		Set_LED(12, Red, Green, Blue);
		Set_LED(13, Red, Green, Blue);
		Set_LED(14, Red, Green, Blue);
		Set_LED(15, Red, Green, Blue);
		Set_LED(16, Red, Green, Blue);
		Set_LED(17, 0, 0, 0);
	}


	Set_Brightness(brilho);
	WS2512_Send();
	HAL_Delay(50);
}

//TODO Documentar método
void Analog_Buffer(){

	/* Processa as novas amostras */
	int32_t accumulator[BSP_ADC_CHANNEL_SIZE] = { 0 };
	static int16_t filter_sample_index = 0;

	//guarda a amostra atual
	Buffer_ADC_Filter[0][filter_sample_index] = readValue[0];
	Buffer_ADC_Filter[1][filter_sample_index] = readValue[1];
	Buffer_ADC_Filter[2][filter_sample_index] = readValue[2];

	//Processa o novo valor filtrado considerando as ultimas x amostras
	for (int sample_idx = 0; sample_idx < BSP_ADC_FILTER_SAMPLES_SIZE;++sample_idx)
	{
		accumulator[0] += Buffer_ADC_Filter[0][sample_idx];
		accumulator[1] += Buffer_ADC_Filter[1][sample_idx];
		accumulator[2] += Buffer_ADC_Filter[2][sample_idx];
	}

	//wrap around no buffer
	filter_sample_index++;
	if (filter_sample_index >= BSP_ADC_FILTER_SAMPLES_SIZE)
	{
		filter_sample_index = 0;
	}


	//calcula a média dos valores
	SpeedMode = accumulator[0];
	SpeedMode = (SpeedMode / BSP_ADC_FILTER_SAMPLES_SIZE);

	Sensor_in = accumulator[1];
	Sensor_in = (Sensor_in / BSP_ADC_FILTER_SAMPLES_SIZE);

	Sensor_Out= accumulator[2];
	Sensor_Out = (Sensor_Out / BSP_ADC_FILTER_SAMPLES_SIZE);

}

//TODO Documentar método
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag = 1;
}

//TODO Documentar método
void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}


//TODO Documentar método
void DigitExtract(int num) {

//	int unid, dez, cent;
	unid = num % 10;
	num = num / 10;
	dez = num % 10;
	cent = num / 10;
}

//TODO Documentar método
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	//************************************ TIMER 4 ************************************

	if (htim->Instance == TIM4) //frequencia do timer4 é de 5 kHz, sample time = 200us
	{


		//atualização do valor do sinal analógico a cada 500 ms

		if (idx < 200) {

			idx++;
		} else {


			idx = 1;
		}



		//atualização do contador timer4

		if (downcounter_timer4 > 0) {
			downcounter_timer4--;
		}

		counter_timer4++;
	}


	//************************************ TIMER 3 ************************************

	if (htim->Instance == TIM3) { //frequencia do timer3 é de 50 kHz, time=20us

		if (downcounter > 0) {
			downcounter--;
		}


		else { 					//downcounter liberado




			// INICIO DO CICLO DE 100 Hz para acionamento dos 3 DISPLAYs 7 SEGMENTOS


			switch (fsm) {			//fsm é o index da mensagem entregue ao CI HC595 para multiplexação

			case 0: //inicio da mensagem 1
			case 2:
			case 4:
			case 6:
			case 8:
			case 10:
			case 12:
			case 14:

				HAL_GPIO_WritePin(GPIOB, STCP_pin, 0);		//inicio da mensagem
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK

				if (digits[cent][myIndex] == 1) {
					HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data HIGH
				} else {
					HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data LOW
				}

				break;

			case 1:
			case 3:
			case 5:
			case 7:
			case 9:
			case 11:
			case 13:
			case 15:

				myIndex--;
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				break;

			case (16):

				//+15	final da mensagem 1 e liga digit 1

				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				HAL_GPIO_WritePin(GPIOB, STCP_pin, 1);		//FIM da mensagem
				HAL_GPIO_WritePin(GPIOC, DIG1, 1);
				myIndex = 7;

				break;

			case 165:

				//+150	desliga digito 1

				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				HAL_GPIO_WritePin(GPIOC, DIG1, 0);

				break;

			case 215: //+50	inicio da mensagem 2
			case 217:
			case 219:
			case 221:
			case 223:
			case 225:
			case 227:
			case 229:

				HAL_GPIO_WritePin(GPIOB, STCP_pin, 0);		//inicio da mensagem
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK

				if (digits[dez][myIndex] == 1) {
					HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data HIGH
				} else {
					HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data LOW
				}

				break;

			case 216:
			case 218:
			case 220:
			case 222:
			case 224:
			case 226:
			case 228:
			case 230:

				myIndex--;
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				break;

			case 231:

				//+15 final da mensagem 2 e liga digit 2

				myIndex = 7;
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				HAL_GPIO_WritePin(GPIOB, STCP_pin, 1);		//FIM da mensagem
				HAL_GPIO_WritePin(GPIOC, DIG2, 1);

				break;

			case 380:

				//+150 desliga digito 2

				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				HAL_GPIO_WritePin(GPIOC, DIG2, 0);

				break;

			case 430:	//+50	inicio da mensagem 3
			case 432:
			case 434:
			case 436:
			case 438:
			case 440:
			case 442:
			case 444:

				HAL_GPIO_WritePin(GPIOB, STCP_pin, 0);		//inicio da mensagem
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK

				if (digits[unid][myIndex] == 1) {
					HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data HIGH
				} else {
					HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data LOW
				}

				break;

			case 431:
			case 433:
			case 435:
			case 437:
			case 439:
			case 441:
			case 443:
			case 445:

				myIndex--;
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				break;

			case 446:

				//+15 final da mensagem 3 e liga digit 3

				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				HAL_GPIO_WritePin(GPIOB, STCP_pin, 1);		//FIM da mensagem
				HAL_GPIO_WritePin(GPIOC, DIG3, 1);
				myIndex = 7;

				break;

			case 595:

				//+150 desliga digito 3

				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK
				HAL_GPIO_WritePin(GPIOC, DIG3, 0);

				break;

			case 597:

				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);		//CLOCK

				//FIM DO CICLO

				downcounter = 50;		//tempo morto de 1ms
				fsm = -1;
				myIndex = 7;
				break;

			default:
				HAL_GPIO_TogglePin(GPIOB, SHCP_pin);	//CLOCK

				break;

			}

			// fim do ciclo interno dos 3 digitos

			fsm++;

		}
	}

}

#define PI 3.14159265

void Set_Brightness(int brightness) // 0~45 linearização do brilho
{
#if USE_BRIGHTNESS

	if (brightness > 45)
		brightness = 45;
	for (int i = 0; i < MAX_LED; i++) {
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j = 1; j < 4; j++) {
			float angle = 90 - brightness; // em graus
			angle = angle * PI / 180; //em radianos
			LED_Mod[i][j] = (LED_Data[i][j]) / (tan(angle));
		}
	}
}

#endif

uint16_t pwmData[(24*MAX_LED+50)];

void WS2512_Send(void) {
	uint32_t indx = 0;
	uint32_t color;

//	for (int i=0; i<50; i++)
//	{
//		pwmData[indx] = 0;
//		indx++;
//	}

	for (int i = 0; i < MAX_LED; i++)
	{
		color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));

		for (int i = 23; i >= 0; i--) {
			if (color & (1 << i)) {
				pwmData[indx] = 60;		// pulso alto, 2/3 de 90, aprox 68%
			}

			else
				pwmData[indx] = 30;	// pulso baixo, 1/3 de 90, aprox 32%

			indx++;
		}
	}

	for (int i = 0; i < 50; i++)//intervalor de tempo de 50us antes da próxima msg
			{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwmData, indx);
	while (!datasentflag) {
	};
	datasentflag = 0;
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
