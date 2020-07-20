
/*
 * main.c
 *
 *  Created on: May 23, 2020
 *      Author: DELL
 */
#include<string.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"


void SystemClock_Config(void);
void GPIO_Init(void);
void Error_handler(void);
extern void initialise_monitor_handles(void);


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();


	while(1)
	{

		  initialise_monitor_handles();

		  	  	  uint32_t value = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6);

		  	  	  printf("%ld ",value);

		  	  	  if(value == 0)
		  	  	  {
		  	  		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		  	  		  printf("Object is detected\n");
		  	  	  }

		  	  	  else
		  	  	  {
		  	  		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		  	  		 printf("Object is not detected\n");
		  	  	  }

	  }
	}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	void Error_handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
	 void Error_handler();
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


void GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_6;
	ledgpio.Mode = GPIO_MODE_INPUT;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD,&ledgpio);

	ledgpio.Pin = GPIO_PIN_4;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB,&ledgpio);
}

void Error_handler(void)
{
	while(1);
}

