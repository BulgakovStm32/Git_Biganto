
#ifndef _LcdGraphic_H
#define _LcdGraphic_H
//*******************************************************************************************
//*******************************************************************************************
//#include "Rtc.h"
//#include "lcd12864.h"
//#include "LM6063D.h"
//#include "Lcd_TIC32.h"
#include "ssd1306.h"
//*******************************************************************************************
//*******************************************************************************************
#define LCD_X_RES 			   128	 //разрешение экрана
#define LCD_Y_RES 			   64
#define LCD_VIDEO_BUFFER_SIZE  ((LCD_X_RES * LCD_Y_RES) / 8)  //
#define LCD_TEXT_BUFFER_SIZE   22
//
#define	X0	0xb8
#define	Y0	0x40
//режимы отображения пикселя - используются в графических функциях
#define PIXEL_OFF	        0
#define PIXEL_ON	        1
#define PIXEL_XOR	        2
//режимы отображения символов - используются в функциях вывода текстов.
#define LCD_CHAR_SIZE_NORM	0
#define LCD_CHAR_SIZE_BIG	1
#define LCD_CHAR_SIZE_BOLD	2
//*******************************************************************************************
//*******************************************************************************************
void     Lcd_Init		     (void);
void     Lcd_Update          (void);
uint8_t* Lcd_pVideoBuffer    (void);
void     Lcd_ClearVideoBuffer(void);
uint8_t* UartTextBuf(void);

void Lcd_Filling(uint8_t byte);
void Lcd_Pixel  (uint8_t x, uint8_t y, uint8_t mode);
void Lcd_Line   (int x1, int y1, int x2, int y2, uint8_t mode);
void Lcd_Circle (uint8_t x, uint8_t y, uint8_t radius, uint8_t mode);
void Lcd_Bar    (int x1, int y1, int x2, int y2, uint8_t persent);

void Lcd_GotoXYFont  (uint8_t x, uint8_t y);
void Lcd_Chr         (uint16_t ch);
void Lcd_SetCursor   (uint8_t x, uint8_t y);
void Lcd_ChrBold     (int ch);
void Lcd_StringBold  (uint8_t x, uint8_t y);
void Lcd_ChrBig      (int ch);
void Lcd_StringBig   (uint8_t x, uint8_t y);
uint8_t Lcd_Print    (char *txt);
uint8_t Lcd_PrintBold(char *txt);
uint8_t Lcd_PrintBig (char *txt);
void Lcd_Image       (const uint8_t *imageData);

uint8_t Lcd_BinToDec(uint16_t var, uint8_t num, uint8_t charSize);
void    Lcd_u8ToHex (uint8_t hexChar);
void    Lcd_u32ToHex(uint32_t varHex);
//*******************************************************************************************
//*******************************************************************************************
#endif /*_LcdGraphic_H*/



