/*
 * main.c
 *
 *  Created on: Mar 13, 2020
 *      Author: DELL
 */



#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#define usTIM TIM1

void SystemClockConfig(uint8_t clock_freq);
void GPIO_Init(void);
void TIMER1_Init(void);
void UART2_Init(void);
void TIMER2_Init(void);
void Error_handler(void);
void usDelay(uint32_t uSec);


TIM_HandleTypeDef htimer1;
TIM_HandleTypeDef htimer2;
UART_HandleTypeDef huart2;

uint32_t input_captures[2] = {0};
uint8_t count=1;

const float speedOfSound = 0.0343/2;
uint8_t icFlag = 0;
float distance;
char uartBuf[100];
int main(void)
{
	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_84_MHZ);
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
				HAL_TIM_IC_Start_IT(&htimer2,TIM_CHANNEL_1);
				//Wait for IC flag
				uint32_t startTick = HAL_GetTick();
				do
				{
				if(icFlag) break;
				}while((HAL_GetTick() - startTick) < 500);  //500ms
				icFlag = 0;
				HAL_TIM_IC_Stop_IT(&htimer2, TIM_CHANNEL_1);
				if(	input_captures[1] >input_captures[0])
					{

					distance = ((input_captures[1] - input_captures[0]) + 0.0f)*speedOfSound;
					}
					else
					{
					distance = 0.0f;
					}
						sprintf(uartBuf, "Distance (cm)  = %.2f\r\n", distance);
						if (distance<=10)
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

void SystemClockConfig(uint8_t clock_freq )
{
	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;


	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	Osc_Init.HSIState = RCC_HSI_ON;
	//Osc_Init.LSEState = RCC_LSE_ON;
	Osc_Init.HSICalibrationValue = 16;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	switch(clock_freq)
	 {
	  case SYS_CLOCK_FREQ_50_MHZ:
		  Osc_Init.PLL.PLLM = 8;
		  Osc_Init.PLL.PLLN = 50;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		//  Osc_Init.PLL.PLLR = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
	     break;

	  case SYS_CLOCK_FREQ_84_MHZ:
		  Osc_Init.PLL.PLLM = 16;
		  Osc_Init.PLL.PLLN = 336;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV4;
		  Osc_Init.PLL.PLLQ = 7;
		//  Osc_Init.PLL.PLLR = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
	     break;

	  case SYS_CLOCK_FREQ_120_MHZ:
		  Osc_Init.PLL.PLLM = 8;
		  Osc_Init.PLL.PLLN = 120;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		//  Osc_Init.PLL.PLLR = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
	     break;

	  default:
	   return ;
	 }

		if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
	{
			Error_handler();
	}



	if (HAL_RCC_ClockConfig(&Clock_Init, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_handler();
	}


	/*Configure the systick timer interrupt frequency (for every 1 ms) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(hclk_freq/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	 if(count == 1) //First edge
	 {
		 input_captures[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		 count = 2;
	 }
	 else if (count == 2)//second edge
	 {
		 input_captures[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		 count =1;
		 icFlag = 1;
	 }



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



