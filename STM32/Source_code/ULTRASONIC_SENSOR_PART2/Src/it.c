 /* it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: Dell
 */


#include "main.h"
extern TIM_HandleTypeDef htimer1;
extern TIM_HandleTypeDef htimer2;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM1_IRQHandler(void)
	{

		HAL_TIM_IRQHandler(&htimer1);
	}


void TIM2_IRQHandler  (void)
	{

		HAL_TIM_IRQHandler(&htimer2);
	}
