/*
 * LCD.h
 *
 *  Created on: Jun 22, 2020
 *      Author: user 4
 */


#include<LCD.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>
/****************************PIN DEFINATION*************************/
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_8
#define EN_GPIO_Port GPIOC


#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80


#define SET 1
#define RESET 0

/******************Initializing LCD*****************************************************/


void lcd_init(void)

{

	  // dislay initialisation
		send_command(0x20);//Initialize the lcd in 4 bit operation
		send_command(0x28);//2 lines 16*2
		send_command(0x01);//Display on cursor off
		send_command(0x0e);// Auto increment cursor
		send_command(0x06);// Display clear
		send_command(0x80);//first line first position





}


/*********************Send Command*******************************************************************/

void send_command(char cmd)
{


    // higher nibble
	 GPIOB->ODR=(GPIOB->ODR & ~0xF000)|((cmd&0xF0)<<8);
	 HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, SET);
	 HAL_Delay(1);
	 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, RESET);
	 HAL_Delay(1);


	 // lower nibble
	  GPIOB->ODR=(GPIOB->ODR & ~0xF000)|((cmd &0x0F)<<12);
	  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, SET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, RESET);
	  HAL_Delay(1);



}


/***********************Send Data****************************************************************************/
void send_data(char data)
{

	//higher nibble
    GPIOB->ODR=(GPIOB->ODR & ~0xF000)|((data&0xF0)<<8);
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, SET);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, RESET);
    HAL_Delay(1);

	// lower nibble
	GPIOB->ODR=(GPIOB->ODR & ~0xF000)|((data&0x0F)<<12);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, SET);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, SET);
	HAL_Delay(1);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, RESET);
	HAL_Delay(1);


}



/*************************Lcd print***************************************************************************/

void lcd_print(char *str)

{

	do
	{

	send_data((uint8_t)*str++);

    }
	while(*str!='\0');
}
/**********************************************************************************************************/



//*********************************** Put a custom character on specific LCD display location*****************/
void lcd_put_custom_char(uint8_t x, uint8_t y, uint8_t location)
{

 send_command(0xC0);
 for(int i=1; i<3;i++)
 {
    send_data(location);
    location=location+1;
 }

}

//*****************************//


//************************************************3.To create a custom Characters//
void lcd_create_custom_char(uint8_t location, uint8_t* data_bytes)
{
	int i;

	// We only have 8 locations 0-7 for custom chars
	location &= 0x07;

	// Set CGRAM address
	send_command(0x40 | (location << 3));

	// Write 8 bytes custom char pattern
	for (i = 0; i < 8; i++)
	{
		send_data(data_bytes[i]);
	}
}




