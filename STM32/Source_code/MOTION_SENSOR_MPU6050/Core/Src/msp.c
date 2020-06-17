/*
 * msp.c
 *
 *  Created on: May 20, 2020
 *      Author: DELL
 */


#include "stm32f4xx_hal.h"

extern void _Error_Handler(char *, int);

#include "main.h"

void HAL_MspInit(void)
{
	  __HAL_RCC_SYSCFG_CLK_ENABLE();
	  __HAL_RCC_PWR_CLK_ENABLE();
 //Here will do low level processor specific inits.
	//1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2. Enable the required system exceptions of the arm cortex mx processor
	SCB->SHCSR |= 0x7 << 16; //usage fault, memory fault and bus fault system exceptions

	//3. configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
	HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
	HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);
}



void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	 GPIO_InitTypeDef gpio_uart;
	 //here we are going to do the low level inits. of the USART2 peripheral

	 //1. enable the clock for the USART2 peripheral as well as for GPIOA peripheral
	 __HAL_RCC_USART2_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	 //2 . Do the pin muxing configurations
	 gpio_uart.Pin = GPIO_PIN_2;
	 gpio_uart.Mode =GPIO_MODE_AF_PP;
	 gpio_uart.Pull = GPIO_PULLUP;
	 gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
	 gpio_uart.Alternate =  GPIO_AF7_USART2; //UART2_TX
	 HAL_GPIO_Init(GPIOA,&gpio_uart);

	 gpio_uart.Pin = GPIO_PIN_3; //UART2_RX
	 HAL_GPIO_Init(GPIOA,&gpio_uart);
	 //3 . Enable the IRQ and set up the priority (NVIC settings )
	 HAL_NVIC_EnableIRQ(USART2_IRQn);
	 HAL_NVIC_SetPriority(USART2_IRQn,15,0);

}


/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {

    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  }

}


/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	 GPIO_InitTypeDef gpio_i2c;
	 //here we are going to do the low level inits. of the USART2 peripheral

	 //1. enable the clock for the USART2 peripheral as well as for GPIOA peripheral
	 __HAL_RCC_I2C2_CLK_ENABLE();
	 __HAL_RCC_GPIOB_CLK_ENABLE();

	 //2 . Do the pin muxing configurations
	 gpio_i2c.Pin = GPIO_PIN_6;
	 gpio_i2c.Mode =GPIO_MODE_AF_PP;
	 gpio_i2c.Pull = GPIO_PULLUP;
	 gpio_i2c.Speed = GPIO_SPEED_FREQ_LOW;
	 gpio_i2c.Alternate =  GPIO_AF4_I2C2; //I2C1_SCL
	 HAL_GPIO_Init(GPIOB,&gpio_i2c);

	 gpio_i2c.Pin = GPIO_PIN_7; //I2C1_SDA
	 HAL_GPIO_Init(GPIOB,&gpio_i2c);
	 //3 . Enable the IRQ and set up the priority (NVIC settings )
	 HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	 HAL_NVIC_SetPriority(I2C1_EV_IRQn,15,0);

}


/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {

    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7|GPIO_PIN_6);

  }

}
