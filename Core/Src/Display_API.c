/**
 ******************************************************************************
 * @file           : Display_API.c
 * @brief          : Funções que realizam tarefas específicas do Diplay Ditial 7 Segmentos,

 *  Created on: Nov 21, 2023
 *  Author: Igor
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Display_API.h"


/* Private define ------------------------------------------------------------*/

//#define

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//int digits[10][8] =
//{
//	{0,1,1,1,1,1,1,0}, //digit 0
//	{0,0,1,1,0,0,0,0}, //digit 1
//	{0,1,1,0,1,1,0,1}, //digit 2
//	{0,1,1,1,1,0,0,1}, //digit 3
//	{0,0,1,1,0,0,1,1}, //digit 4
//	{0,1,0,1,1,0,1,1}, //digit 5
//	{0,1,0,1,1,1,1,1}, //digit 6
//	{0,1,1,1,0,0,0,0}, //digit 7
//	{0,1,1,1,1,1,1,1}, //digit 8
//	{0,1,1,1,1,0,1,1}  //digit 9
//};

/* External variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
  * @brief
  * @param
  * @retval
  */



void DisplayDigit(int value, int digit) {

//	HAL_GPIO_WritePin(GPIOB, STCP_pin, 0); // INICIO DA MENSAGEM
//
//	for (int index = 7; index >= 0; index--) {
//
//		HAL_GPIO_WritePin(GPIOB, SHCP_pin, 0);//CLOCK LOW
//		//HAL_Delay(1);
//
//		if (digits[value][index] == 1) {
//			HAL_GPIO_WritePin(GPIOB, DS_pin, 1); //data LOW
//		}
//
//		else if (digits[value][index] == 0) {
//			HAL_GPIO_WritePin(GPIOB, DS_pin, 0); //data HIGH
//		}
//
//		//HAL_Delay(1);
//		HAL_GPIO_WritePin(GPIOB, SHCP_pin, 1); //CLOCK HIGH
//
//	}
//
//	HAL_GPIO_WritePin(GPIOB, STCP_pin, 1);  // fim da mensagem (SEND ou LATCH)


//	if(digit==1){
//		HAL_GPIO_WritePin(GPIOC, DIG1, 0);
//		HAL_GPIO_WritePin(GPIOC, DIG2, 0);
//		HAL_GPIO_WritePin(GPIOC, DIG3, 1);
//	}
//	else if(digit==2){
//		HAL_GPIO_WritePin(GPIOC, DIG1, 0);
//		HAL_GPIO_WritePin(GPIOC, DIG2, 1);
//		HAL_GPIO_WritePin(GPIOC, DIG3, 0);
//	}
//	else{
//		HAL_GPIO_WritePin(GPIOC, DIG1, 1);
//		HAL_GPIO_WritePin(GPIOC, DIG2, 0);
//		HAL_GPIO_WritePin(GPIOC, DIG3, 0);
//	}

}

/*****END OF FILE****/
