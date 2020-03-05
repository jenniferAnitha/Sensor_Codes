/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include "main.h"


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void EXTI9_5_IRQHandler(void)
{

	 if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6)!= RESET)
	 {

		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
	 }

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}
