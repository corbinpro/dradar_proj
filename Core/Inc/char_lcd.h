/*
 * char_lcd.h
 *
 *  Created on: Jul 14, 2025
 *      Author: corbinpro
 */

#ifndef INC_CHAR_LCD_H_
#define INC_CHAR_LCD_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
#include "main.h"

//private macros
// 1602 I2C address
#define I2C_ADDR 0x27 // I2C address of the PCF8574
// 1602 dimensions
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD
// 1602 message bit numbers
#define DC_BIT 0 // Data/Command bit (register select bit)
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

//PFP
void CharLCD_Write_Nibble (uint8_t nibble, uint8_t dc);
void CharLCD_Send_Cmd(uint8_t cmd);
void CharLCD_Send_Data(uint8_t data);
void CharLCD_Init();
void CharLCD_Write_String(char *str);
void CharLCD_Set_Cursor(uint8_t row, uint8_t column);
void CharLCD_Clear(void);
void RecieveHandles(I2C_HandleTypeDef*);


#endif /* INC_CHAR_LCD_H_ */
