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
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_6
#define EN_GPIO_Port GPIOC
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB

#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80


#define SET 1
#define RESET 0

/******************Initializing LCD*****************************************************/


void lcd_init(void)

{
	send_command(0X30);/*we are setting 8-bit mode lcd having 1 line and we are initializing it to be 5×7 character display*/
	send_command(0x38); /* Select the LCD 2 rows and 5*7 matrix Display*/
	send_command(0x01); /*Clear screen*/
	send_command (0x0e); /* used to on the Display on Cursor on*/
	send_command(0x06); /*Increment cursor (shift cursor to right)*/
	send_command(0x80);/*Force cursor to beginning ( 1st line)*/





}

/*********************Send Command*******************************************************************/

void send_command(char cmd)
{
      //move the 8bit command into GPIOB (PB8-PB15)
	GPIOB->ODR=(cmd<<8);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	//This function is used to make the LCD latch the data into its internal registers
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, RESET);
}


/***********************Send Data****************************************************************************/
void send_data(char data)
{
//   //move the 8bit data  into GPIOB (PB8-PB15)
	GPIOB->ODR=(data<<8);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, SET);
	//This function is used to make the LCD latch the data into its internal registers
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, RESET);
}
/************************Print String********************************************************************/


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
 for(int i=1; i<=3;i++)
 {
    send_data(location);
    location=location+1;
 }

}



/*
To create a custom Characters
*/
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
