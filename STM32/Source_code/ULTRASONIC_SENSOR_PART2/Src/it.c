
/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */


#include "main.h"

extern TIM_HandleTypeDef htimer3;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM3_IRQHandler  (void)
	{

		HAL_TIM_IRQHandler(&htimer3);
	}
