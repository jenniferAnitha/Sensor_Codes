/*
 * main.c
 *
 *  Created on: 07-Mar-2020
 *      Author: DELL
 */


#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"

void SystemClockConfig(uint8_t clock_freq );
void GPIO_Init(void);
void TIMER1_Init(void);
void UART2_Init(void);
void Error_handler(void);
TIM_HandleTypeDef htimer1;
UART_HandleTypeDef huart2;

void delay_us(uint32_t us);
const float speedOfSound = 0.0340/2;
const float error_Factor=0.65;
float distance;
char uartBuf[100];
int main(void)
{
	uint32_t noTicks=0;
	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_84_MHZ);
	GPIO_Init();
	UART2_Init();
	TIMER1_Init();
	HAL_TIM_Base_Start(&htimer1);
	while(1)
	{
		// Set the TRIG pin to low for few seconds
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
		delay_us(3);

		// Start the ultrasonic routine

		//1. Output 10Microsecond in TRIG
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		delay_us(10);

		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
		//2. Measure the echo signal
		// wait for echo pin rising edge
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)== GPIO_PIN_RESET);

		//3. Start measuring echo pulse width in usec

		noTicks=0;

		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)== GPIO_PIN_SET)
		{
			noTicks++;
			delay_us(2);//2.8 us

		};


		//4. Estimate the distance in Cm

		distance = ((noTicks + 0.0f)*2.8*speedOfSound)- error_Factor;

		//5. Print to UART terminal for debugging
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

void SystemClockConfig(uint8_t clock_freq )
{
	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;


	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	Osc_Init.HSIState = RCC_HSI_ON;
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

	ledgpio.Pin = GPIO_PIN_9;
	ledgpio.Mode = GPIO_MODE_INPUT;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
	}


void delay_us(uint32_t us)
{
	//Set the counter value to 0
	__HAL_TIM_SET_COUNTER(&htimer1,0);

	//wait for the counter to reach the us input in the parameter
	while(__HAL_TIM_GET_COUNTER(&htimer1) < us);
}


void TIMER1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	htimer1.Instance = TIM1;
	htimer1.Init.Prescaler = 84-1;
	htimer1.Init.CounterMode= TIM_COUNTERMODE_UP;
	htimer1.Init.Period = 0xFFFFFFFF-1;
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
