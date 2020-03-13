/*
 * main.c
 *
 *  Created on: Mar 11, 2020
 *      Author: DELL
 */



#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#define usTIM TIM1

void SystemClock_Config(void);
void GPIO_Init(void);
void TIMER1_Init(void);
void UART2_Init(void);
void TIMER2_Init(void);
void Error_handler(void);
void usDelay(uint32_t uSec);

TIM_HandleTypeDef htimer1;
TIM_HandleTypeDef htimer2;
UART_HandleTypeDef huart2;




uint8_t icFlag = 0;
uint8_t captureIdx=0;
uint32_t edge1Time=0, edge2Time=0;

const float speedOfSound = 0.0343/2;

float distance;
char uartBuf[100];
int main(void)
{
//uint32_t noTicks=0;
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	UART2_Init();
	TIMER1_Init();
	TIMER2_Init();

		 while (1)
		  {
				//Set TRIG to LOW for few uSec
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_RESET);
				usDelay(3);

				//*** START Ultrasonic measurement routine ***//
				//1. Output 10 usec TRIG
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET);
				usDelay(10);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_RESET);

				//2. ECHO signal pulse width

				//Start IC timer
				HAL_TIM_IC_Start_IT(&htimer2, TIM_CHANNEL_1);
				//Wait for IC flag
				uint32_t startTick = HAL_GetTick();
				do
				{
					if(icFlag) break;
				}while((HAL_GetTick() - startTick) < 500);  //500ms
				icFlag = 0;
				HAL_TIM_IC_Stop_IT(&htimer2, TIM_CHANNEL_1);

				//Calculate distance in cm
				if(edge2Time > edge1Time)
				{

					distance = ((edge2Time - edge1Time) + 0.0f)*speedOfSound;
				}
				else
				{
					distance = 0.0f;
				}

				//Print to UART terminal for debugging
				             sprintf(uartBuf, "Distance (cm)  = %.1f\r\n", distance);
		 		             if(distance <= 10)

				  				{
				  				 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				  			    }
				  				else
				  				{
				  				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				  				}

				    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);

			     	HAL_Delay(1000);

		  }

}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_handler();
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
    Error_handler();
  }
}
			   /**
			     * @brief System Clock Configuration
			     * @retval None
			     */

void GPIO_Init(void)
{
	GPIO_InitTypeDef ledgpio;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);


	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);

	ledgpio.Pin = GPIO_PIN_8;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);

}

void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

void TIMER1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	htimer1.Instance = TIM1;
	htimer1.Init.Prescaler = 84-1;
	htimer1.Init.CounterMode= TIM_COUNTERMODE_UP;
	htimer1.Init.Period = 0;
	htimer1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htimer1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if( HAL_TIM_Base_Init(&htimer1) != HAL_OK )
	{
		Error_handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	 if (HAL_TIM_ConfigClockSource(&htimer1, &sClockSourceConfig) != HAL_OK)
	 {
	    Error_handler();
	 }
	 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	 if (HAL_TIMEx_MasterConfigSynchronization(&htimer1, &sMasterConfig) != HAL_OK)
	 {
	   Error_handler();
	 }

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		if(captureIdx == 0) //First edge
		{
			edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);// __HAL_TIM_GetCounter(&hTIMER2);// ;

			captureIdx = 1;
		}
		else if(captureIdx == 1) //Second edge
		{
			edge2Time =  HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			captureIdx = 0;
			icFlag = 1;
		}
}





void TIMER2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};


  htimer2.Instance = TIM2;
  htimer2.Init.Prescaler = 84-1;
  htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimer2.Init.Period = 1000000-1; //1sec
  htimer2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htimer2) != HAL_OK)
  {
    Error_handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htimer2, &sClockSourceConfig) != HAL_OK)
  {
    Error_handler();
  }
  if (HAL_TIM_IC_Init(&htimer2) != HAL_OK)
  {
    Error_handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htimer2, &sMasterConfig) != HAL_OK)
  {
    Error_handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htimer2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_handler();
  }

}


void UART2_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}


}


void Error_handler(void)
{

}
