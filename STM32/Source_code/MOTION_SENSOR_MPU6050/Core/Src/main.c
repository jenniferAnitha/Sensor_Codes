
/*
 * main.c
 *
 *  Created on: June 3, 2020
 *      Author: DELL
 */



#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "defines.h"
#include "_stm32_mpu6050.h"
#include "stm32f407xx.h"

void SystemClockConfig(uint8_t clock_freq );
void GPIO_Init(void);
void I2C1_Init(void);
void UART2_Init(void);
void Error_handler(void);




I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
char uartBuf[100];
int16_t X=0, Y=0, Z=0, x=0, y=0, z=0;
float   Temperature=0;
_MPU6050_t mpu6050_sensor;

int main(void)

{

	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_84_MHZ );
	GPIO_Init();
    I2C1_Init();
	UART2_Init();
	/* Initialize MPU6050 sensor 0, address = 0xD0, AD0 pin on sensor is low */
	 while(_MPU6050_Init(&mpu6050_sensor,_MPU6050_Device_0, _MPU6050_Accelerometer_4G, _MPU6050_Gyroscope_500s) != _MPU6050_Result_Ok)
	 	 {

	 	 }
	 	 HAL_Delay(200);

	while(1)
	{

		 HAL_Delay(300);
			  	 /* Read all data of mpu6050 sensor */
		 _MPU6050_ReadAll_Data(&mpu6050_sensor);

			  		X = mpu6050_sensor.Accelerometer_x;
			  		Y = mpu6050_sensor.Accelerometer_y;
			  		Z = mpu6050_sensor.Accelerometer_z;
			  	    x = mpu6050_sensor.Gyroscope_x;
			  		y = mpu6050_sensor.Gyroscope_y;
			  		z = mpu6050_sensor.Gyroscope_z;
			  		Temperature = mpu6050_sensor.Temperature;


			  		/* Send the data to serial buffer */
			  		sprintf(uartBuf,"Accelerometer values- X:%d Y:%d  Z:%d\r\n",X, Y, Z);
			  		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 1000);
			  		sprintf(uartBuf,"Gyroscope values-  X:%d Y:%d Z:%d Temperature: %f\r\n",x, y, z, Temperature);
			  		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 1000);

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
		  Osc_Init.PLL.PLLM = 8;
		  Osc_Init.PLL.PLLN = 84;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void I2C1_Init(void)
{
	hi2c1.Instance=I2C1;
	hi2c1.Init.ClockSpeed=100000;
	hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1=0;
	hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2=0;
	hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
	if(HAL_I2C_Init(&hi2c1)!=HAL_OK)
	{
		Error_handler();
	}
}




/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

	void GPIO_Init(void)
	{
	 GPIO_InitTypeDef ledpin;

	    __HAL_RCC_GPIOB_CLK_ENABLE();

	    /*Configure GPIO pin Output Level */
	     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);


	    ledpin.Pin = GPIO_PIN_12;
	    ledpin.Mode = GPIO_MODE_OUTPUT_PP;
	    ledpin.Speed=GPIO_SPEED_FREQ_LOW;
	    ledpin.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB,&ledpin);
	}



/**
 * @brief  This function is executed in case of error occurrence.
 *  @retval None
 */

void Error_handler(void)
{


}





