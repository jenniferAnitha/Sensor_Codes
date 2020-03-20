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

void SystemClock_Config(void);
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
	SystemClock_Config();
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
	htimer1.Init.Period = 0xFFFF-1;
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
