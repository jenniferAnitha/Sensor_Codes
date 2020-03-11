
/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */


#include "main.h"

extern TIM_HandleTypeDef htimer1;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM4_IRQHandler(void)
	{

		HAL_TIM_IRQHandler(&htimer1);
	}
