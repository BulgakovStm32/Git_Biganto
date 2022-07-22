
#include "Lcd_i2c_1602.h"

//************************************************************************************************************
//************************************************************************************************************

//************************************************************************************************************
//************************************************************************************************************
static void lcd_send_tetrad(uint8_t byte, uint8_t type){
  
	uint8_t temp = 0;
	//-------------------------
	temp = byte & 0xF0;
	//управление ножкой RS.
	if(type) temp |=  LCD_RS;
	else	 temp &= ~LCD_RS;

	//Передаем тетраду.
	if(I2C_StartAndSendDeviceAddr(LCD_I2C, LCD_I2C_ADDRESS)) return;
	I2C_SendData(LCD_I2C, &temp, 1);

	//Шлем строб E в дисплей.
	temp |= LCD_E;
	I2C_SendData(LCD_I2C, &temp, 1);

	temp &= ~(LCD_E); 
	I2C_SendData(LCD_I2C, &temp, 1);

	I2C_Stop(LCD_I2C);
}
//***************************************************************
static void lcd_send(uint8_t byte, uint8_t type){
  
	lcd_send_tetrad(byte, type);       //Передаем старшую тетраду.
	lcd_send_tetrad((byte << 4), type);//Передаем младшую тетраду.
}
//************************************************************************************************************
//************************************************************************************************************
//ПП инициалации LCD.
void lcd_init(void){

	msDelay(250);
	//we start in 8bit mode, try to set 4 bit mode
	lcd_send(0x03, LCD_CMD);
	msDelay(5);
	//second try
	lcd_send(0x03, LCD_CMD);
	msDelay(5);
	//third go!
	lcd_send(0x03, LCD_CMD);
	msDelay(1);
	//finally, set to 4-bit interface
	lcd_send(0x02, LCD_CMD);
	// set # lines, font size, etc.
	lcd_send((LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS), LCD_CMD);
	// turn the display on with no cursor or blinking default
	lcd_send((LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF), LCD_CMD);
	// clear it off
	lcd_clear();
}
//***************************************************************
void lcd_clear(void){
  
	 lcd_send(LCD_CLEARDISPLAY, LCD_CMD);
	 msDelay(4);
}
//***************************************************************
//Ф-я установки курсора.
//str - номер строки (верхняя - 0, нижняя - 1):
//pos - позиция курсора в строке(1-16).
void lcd_set_cursor(uint8_t str, uint8_t pos){
	
	lcd_send((pos-1)+str, LCD_CMD);
}
//***************************************************************
void lcd_print_ch(char ch){

	lcd_send(ch, LCD_DATA);
}
//***************************************************************
// ПП передачи строки в дисплей.
void lcd_print_string(char *txt){
	
	while(*txt)
    {
      lcd_send(*txt, LCD_DATA);
      txt++;
    }
}
//***************************************************************
void lcd_BinToDec(uint16_t var, uint8_t num){
  
  uint8_t decArray[5];
  uint8_t i;
  //--------------------
  decArray[4] = (uint8_t)(var / 10000);
  var %= 10000;

  decArray[3] = (uint8_t)(var / 1000);
  var %= 1000;

  decArray[2] = (uint8_t)(var / 100);
  var %= 100;

  decArray[1] = (uint8_t)(var / 10);
  decArray[0] = (uint8_t)(var % 10);
  //--------------------
  //Вывод на дисплей
  for(i = 0; i < num; i++)
    {
      lcd_send(decArray[num - i - 1] + '0', LCD_DATA);
    }
}
//***************************************************************
void lcd_BacklightON(void){

	lcd_send(LCD_BACKLIGHT, LCD_DATA);
}
//***************************************************************
void lcd_BacklightOFF(void){

	lcd_send(LCD_NOBACKLIGHT, LCD_DATA);
}
//***************************************************************
////ПП создания символа в LCD.
//void LCD_USER_SUMBOL(void){
	////----------------------------
		////Символ градуса Цельсия.
		//LCD_CMD_SEND (0b01000000);
	//
		//LCD_DATA_SEND(0b00001100);
		//LCD_DATA_SEND(0b00010010);
		//LCD_DATA_SEND(0b00010010);
		//LCD_DATA_SEND(0b00001100);
		//LCD_DATA_SEND(0b00000000);
		//LCD_DATA_SEND(0b00000000);
		//LCD_DATA_SEND(0b00000000);
		//LCD_DATA_SEND(0b00000000);
		////----------------------------
		//LCD_DATA_SEND(0b00001110);
		//LCD_DATA_SEND(0b00010101);
		//LCD_DATA_SEND(0b00011011);
		//LCD_DATA_SEND(0b00001110);
		//LCD_DATA_SEND(0b00000000);
		//LCD_DATA_SEND(0b00010001);
		//LCD_DATA_SEND(0b00001110);
		//LCD_DATA_SEND(0b00010001);
	////----------------------------	
//
	////----------------------------
//
	////----------------------------
//
	////----------------------------
//
	////----------------------------
//
	////----------------------------
//
	////----------------------------
//}
//************************************************************************************************************
//************************************************************************************************************








