//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
#ifndef _LCD_3_WIRE_H        // Блокируем повторное включение этого модуля
#define _LCD_3_WIRE_H 
//-----------------------------------------------------------------------------

#include "main.h"

//-----------------------------------------------------------------------------
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

//flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

//------------------------
#define LCD_I2C			  I2C2 //I2C1
#define LCD_I2C_ADDRESS	 (0x27 << 1)//(0x38<<1)

#define LCD_RS 	   (1 << 0)// номер бита порта, к которой подключенеа линия R/S ЖКИ.
#define LCD_RW 	   (1 << 1)// номер бита порта, к которой подключенеа линия R/W ЖКИ.
#define LCD_E 	   (1 << 2)// номер бита порта, к которой подключенеа линия E ЖКИ.
#define LCD_LED	   (1 << 3)// номер бита порта, к которой подключенеа линия LED ЖКИ.

#define LCD_DATA    1
#define LCD_CMD     0
//------------------------
#define LCD_STRING_1  0x80
#define LCD_STRING_2  0xC0
//------------------------
//#define LCD_DELAY     0x1

//-----------------------------------------------------------------------------
void lcd_init        (void);
void lcd_clear       (void);
void lcd_set_cursor  (uint8_t str, uint8_t pos); 
void lcd_print_ch    (char ch);
void lcd_print_string(char *txt);
void lcd_BinToDec    (uint16_t var, uint8_t num);

void lcd_BacklightON(void);
void lcd_BacklightOFF(void);
//void lcd_user_sumbol   (void);
//-----------------------------------------------------------------------------
#endif                       // Закрывающий #endif к блокировке повторного включения
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
