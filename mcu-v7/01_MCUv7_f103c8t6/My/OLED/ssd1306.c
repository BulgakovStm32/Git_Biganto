
//*******************************************************************************************
//*******************************************************************************************

#include "ssd1306.h"

//*******************************************************************************************
//*******************************************************************************************
static uint8_t   *pVideoBuffer;//указатель на видеобуфер.
static SSD1306_t SSD1306;
//********************************************
//extern const uint16_t Font7x10[];
//extern const uint16_t Font11x18[];
//extern const uint16_t Font16x26[];
//
//FontDef_t Font_7x10 = {
//	7,
//	10,
//	Font7x10
//};
////**********************
//FontDef_t Font_11x18 = {
//	11,
//	18,
//	Font11x18
//};
////**********************
//FontDef_t Font_16x26 = {
//	16,
//	26,
//	Font16x26
//};
//*******************************************************************************************
//*******************************************************************************************
/**
 * @brief  Writes multi bytes to slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  *data: pointer to data array to write it to slave
 * @param  count: how many bytes will be written
 * @retval None
 */
static void ssd1306_I2C_WriteDataBuf(uint8_t *pData, uint16_t len){

	I2C_Master_Write(SSD1306_I2C, SSD1306_I2C_ADDR, 0x40, pData, len);
}
//***********************************************************************
/**
 * @brief  Writes single byte to slave
 * @param  *I2Cx: I2C used
 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  data: data to be written
 * @retval None
 */
static void ssd1306_I2C_WriteCMD(uint8_t cmd){

	I2C_Master_Write(SSD1306_I2C, SSD1306_I2C_ADDR, 0, &cmd, 1);
}
//*******************************************************************************************
//*******************************************************************************************
uint8_t SSD1306_Init(I2C_TypeDef *i2c) {

	/* Init I2C */
	I2C_Master_Init(i2c, 0);

	/* Init LCD */
	ssd1306_I2C_WriteCMD(0xAE); //display off
	ssd1306_I2C_WriteCMD(0x20); //Set Memory Addressing Mode
	ssd1306_I2C_WriteCMD(0x00); //00,Horizontal Addressing Mode;
							    //01,Vertical Addressing Mode;
							    //10,Page Addressing Mode (RESET);
							    //11,Invalid

	ssd1306_I2C_WriteCMD(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

	ssd1306_I2C_WriteCMD(0xC8); //Set COM Output Scan Direction
	ssd1306_I2C_WriteCMD(0x00); //---set low column address
	ssd1306_I2C_WriteCMD(0x10); //---set high column address
	ssd1306_I2C_WriteCMD(0x40); //--set start line address

	ssd1306_I2C_WriteCMD(0x81); //--set contrast control register
	ssd1306_I2C_WriteCMD(50);

	ssd1306_I2C_WriteCMD(0xA1); //--set segment re-map 0 to 127


	ssd1306_I2C_WriteCMD(0xA6); //--set normal display
	ssd1306_I2C_WriteCMD(0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_I2C_WriteCMD(0x3F); //

	ssd1306_I2C_WriteCMD(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_I2C_WriteCMD(0xD3); //-set display offset
	ssd1306_I2C_WriteCMD(0x00); //-not offset
	ssd1306_I2C_WriteCMD(0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_I2C_WriteCMD(0xF0); //--set divide ratio
	ssd1306_I2C_WriteCMD(0xD9); //--set pre-charge period
	ssd1306_I2C_WriteCMD(0x22); //

	ssd1306_I2C_WriteCMD(0xDA); //--set com pins hardware configuration
	//ssd1306_I2C_WriteCMD(0x12);//SSD1306 128x64
	ssd1306_I2C_WriteCMD(0x22);//SSD1306 128x32

	ssd1306_I2C_WriteCMD(0xDB); //--set vcomh
	ssd1306_I2C_WriteCMD(0x20); //0x20,0.77xVcc

	ssd1306_I2C_WriteCMD(0x8D); //--set DC-DC enable
	ssd1306_I2C_WriteCMD(0x14); //

	ssd1306_I2C_WriteCMD(0xAF); //--turn on SSD1306 panel
	//-------------------------
	//работа по прерываниям I2C.

//	uint8_t InitBuf[28] = {
//
//		0xAE, //display off
//		0x20, //Set Memory Addressing Mode
//		0x01, //00,Horizontal Addressing Mode;
//			  //01,Vertical Addressing Mode;
//			  //10,Page Addressing Mode (RESET);
//			  //11,Invalid
//		0xB0, //Set Page Start Address for Page Addressing Mode,0-7
//		//
//		0xC8, //Set COM Output Scan Direction
//		0x00, //---set low column address
//		0x10, //---set high column address
//		0x40, //--set start line address
//		//
//		0x81, //--set contrast control register
//		50,
//		//
//		0xA1, //--set segment re-map 0 to 127
//		0xA6, //--set normal display
//		0xA8, //--set multiplex ratio(1 to 64)
//		0x3F, //
//		//
//		0xA4, //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
//		0xD3, //-set display offset
//		0x00, //-not offset
//		0xD5, //--set display clock divide ratio/oscillator frequency
//		0xF0, //--set divide ratio
//		0xD9, //--set pre-charge period
//		0x22, //
//		//
//		0xDA, //--set com pins hardware configuration
//		0x12,
//		//
//		0xDB, //--set vcomh
//		0x20, //0x20,0.77xVcc
//		//
//		0x8D, //--set DC-DC enable
//		0x14, //
//		//
//		0xAF, //--turn on SSD1306 panel
//	};
//	//------------
//	I2C_Int_StartSendData(SSD1306_I2C, SSD1306_I2C_ADDR, InitBuf, 28);

	pVideoBuffer = Lcd_pVideoBuffer();

//	msDelay(1000);

	/* Clear screen */
//	SSD1306_Fill(SSD1306_COLOR_WHITE);
	/* Update screen */
//	SSD1306_UpdateScreen();
	/* Set default values */
//	SSD1306.CurrentX = 0;
//	SSD1306.CurrentY = 0;
	/* Initialized OK */
//	SSD1306.Initialized = 1;
	/* Return OK */
	return 1;
}
//***********************************************************************
void SSD1306_UpdateScreen(uint8_t *pBuf) {

//	//Передача данных для дисплея 1,25"
//	for(uint8_t m = 0; m < 8; m++)
//	{
//		ssd1306_I2C_WriteCMD(0xB0 + m);//Set Page Start Address for Page Addressing Mode,0-7
//		ssd1306_I2C_WriteCMD(0x02);    //Set low column address ,смещение вывода изображениея на 2 столбца.
//		ssd1306_I2C_WriteCMD(0x10);    //Set high column address
//		/* Write multi data */
//		ssd1306_I2C_WriteDataBuf(&pBuf[SSD1306_WIDTH * m], SSD1306_WIDTH);
//	}


	static uint16_t m = 0;
	if(m < 8)
	{
		ssd1306_I2C_WriteCMD(0xB0 + m);//Set Page Start Address for Page Addressing Mode,0-7
		ssd1306_I2C_WriteCMD(0x00);    //Set low column address ,смещение вывода изображениея на 2 столбца.
		ssd1306_I2C_WriteCMD(0x10);    //Set high column address
		/* Write multi data */
		ssd1306_I2C_WriteDataBuf(&pBuf[SSD1306_WIDTH * m], SSD1306_WIDTH);
		m++;
	}
	else m = 0;


//    //Передача данных для дисплея 0,95"
//	ssd1306_I2C_WriteCMD(0x20);//настройка адресации
//	ssd1306_I2C_WriteCMD(0x00);//
//
//	ssd1306_I2C_WriteCMD(0x21);//установка столбца
//	ssd1306_I2C_WriteCMD(0);   //Начальный столбец.
//	ssd1306_I2C_WriteCMD(127); //Конечный столбец.
//
//	ssd1306_I2C_WriteCMD(0x22);//установка страницы
//	ssd1306_I2C_WriteCMD(0);   //Начальная страница.
//	ssd1306_I2C_WriteCMD(7);   //Конечная страница.
//
//	ssd1306_I2C_WriteDataBuf(pBuf, 1024);

}
//***********************************************************************
void SSD1306_ToggleInvert(void) {
	uint16_t i;

	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;

	/* Do memory toggle */
//	for (i = 0; i < sizeof(SSD1306_Buffer); i++)
//		{
//			SSD1306_Buffer[i] = ~SSD1306_Buffer[i];
//		}

	for(i = 0; i < SSD1306_VIDEO_BUFFER_SIZE; i++)
		{
			*(pVideoBuffer + i) = ~(*(pVideoBuffer + i));
		}
}
//***********************************************************************
void SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Set memory */
//	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));

	memset(pVideoBuffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, SSD1306_VIDEO_BUFFER_SIZE);
}
//***********************************************************************
void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color){

	/* Error */
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;	/* Error */
	/* Check if pixels are inverted */
	if (SSD1306.Inverted) color = (SSD1306_COLOR_t)!color;

	/* Set color */
//	if(color == SSD1306_COLOR_WHITE) SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |=   1 << (y % 8);
//	else                             SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	/* Set color */
	if(color == SSD1306_COLOR_WHITE) *(pVideoBuffer + (x + (y / 8) * SSD1306_WIDTH)) |=   1 << (y % 8);
	else                             *(pVideoBuffer + (x + (y / 8) * SSD1306_WIDTH)) &= ~(1 << (y % 8));
}
//***********************************************************************
void SSD1306_GotoXY(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}
//***********************************************************************
char SSD1306_Putc(char ch, FontDef_t* Font, SSD1306_COLOR_t color) {
	uint32_t i, b, j;

	/* Check available space in LCD */
	if (
		SSD1306_WIDTH <= (SSD1306.CurrentX + Font->FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font->FontHeight)
	) {
		/* Error */
		return 0;
	}

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t) color);
			} else {
				SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR_t)!color);
			}
		}
	}

	/* Increase pointer */
	SSD1306.CurrentX += Font->FontWidth;

	/* Return character written */
	return ch;
}
//***********************************************************************
char SSD1306_Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}

		/* Increase string pointer */
		str++;
	}

	/* Everything OK, zero should be returned */
	return *str;
}
//***********************************************************************
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c){

	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if(x0 >= SSD1306_WIDTH)  x0 = SSD1306_WIDTH - 1;
	if(x1 >= SSD1306_WIDTH)  x1 = SSD1306_WIDTH - 1;
	if(y0 >= SSD1306_HEIGHT) y0 = SSD1306_HEIGHT - 1;
	if(y1 >= SSD1306_HEIGHT) y1 = SSD1306_HEIGHT - 1;

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			SSD1306_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			SSD1306_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		SSD1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}
//***********************************************************************
void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c) {
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw 4 lines */
	SSD1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}
//***********************************************************************
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c) {
	uint8_t i;

	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		SSD1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}
//***********************************************************************
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color) {
	/* Draw lines */
	SSD1306_DrawLine(x1, y1, x2, y2, color);
	SSD1306_DrawLine(x2, y2, x3, y3, color);
	SSD1306_DrawLine(x3, y3, x1, y1, color);
}
//***********************************************************************
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}
//***********************************************************************
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x0 + x, y0 + y, c);
        SSD1306_DrawPixel(x0 - x, y0 + y, c);
        SSD1306_DrawPixel(x0 + x, y0 - y, c);
        SSD1306_DrawPixel(x0 - x, y0 - y, c);

        SSD1306_DrawPixel(x0 + y, y0 + x, c);
        SSD1306_DrawPixel(x0 - y, y0 + x, c);
        SSD1306_DrawPixel(x0 + y, y0 - x, c);
        SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}
//***********************************************************************
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}
//***********************************************************************
void SSD1306_ON(void) {
	ssd1306_I2C_WriteCMD(0x8D);
	ssd1306_I2C_WriteCMD(0x14);
	ssd1306_I2C_WriteCMD(0xAF);
}
//***********************************************************************
void SSD1306_OFF(void) {
	ssd1306_I2C_WriteCMD(0x8D);
	ssd1306_I2C_WriteCMD(0x10);
	ssd1306_I2C_WriteCMD(0xAE);
}
//*******************************************************************************************
//*******************************************************************************************


