/*
 * LCD.h
 *
 *  Created on: Jun 22, 2020
 *      Author: user 4
 */
#include<stdint.h>
#ifndef INC_LCD_H_
#define INC_LCD_H_

void lcd_init(void);

void send_command(char cmd);

void send_data(char data);

void lcd_print(char *str);

void lcd_create_custom_char(uint8_t location, uint8_t* data_bytes);

void lcd_put_custom_char(uint8_t x, uint8_t y, uint8_t location);

#endif /* INC_LCD_H_ */
