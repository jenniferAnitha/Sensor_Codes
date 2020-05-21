/*
 * it.c
 *
 *  Created on: May 20, 2020
 *      Author: DELL
 */



#include "main.h"


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

