/*
 * it.c
 *
 *  Created on: June 3, 2020
 *      Author: DELL
 */



#include "main.h"


void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

