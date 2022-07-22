
#include "LM6063D.h"
  
//extern SPI_HandleTypeDef hspi2;
//-----------------------------------------------------------------------------
static void Lm6063LcdPause(uint16_t pause){
  
  while(pause != 0) {--pause;}
}
//-----------------------------------------------------------------------------
static void Lm6063LcdGpioInit(void){
  
	//Включаем тактирование порта
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN);
	//--------------------
	// A1   - PB10.
	// /RES - PB12.
	// /CS  - PB11.
	//выход, режим - push-pull.
	GPIOB->CRH &= ~(GPIO_CRH_CNF10 |
					GPIO_CRH_CNF11 |
					GPIO_CRH_CNF12 );
	//тактирование 50МГц.
	GPIOB->CRH |= (GPIO_CRH_MODE10 |
                   GPIO_CRH_MODE11 |
                   GPIO_CRH_MODE12 );

	Lm6063LcdResHight;
	Lm6063LcdCsHight;
}	
//-----------------------------------------------------------------------------
//Передача байта в дисплей.
static void Lm6063LcdSendByte(uint8_t byte, uint8_t cmd){
  
  Lm6063LcdCsLow;          //Enable display controller (active low)
  if(cmd) Lm6063LcdA1Hight;//данные.
  else    Lm6063LcdA1Low;  //команда.
	
  Spi2TxRxByte(byte);      //
  //HAL_SPI_Transmit(&hspi2, &byte, 1, 10);
  
  Lm6063LcdCsHight;        //Disable display controller
}
//-----------------------------------------------------------------------------
//инициализация SPI и дисплея
void Lm6063LcdInit(void){
  
	Spi2Init();
	Lm6063LcdGpioInit();
	
	msDelay(5);//delay_ms(1);
 
  //дернули ресет
//  Lm6063LcdResHight;
//	Lm6063LcdPause(10);
	
  Lm6063LcdResLow; 
  Lm6063LcdPause(50);
	
  Lm6063LcdResHight;
  Lm6063LcdPause(50);
  
  Lm6063LcdSendByte(0xaf, LM6063_CMD);	//Display OFF
  Lm6063LcdSendByte(0x40, LM6063_CMD);
  //--------------------
  //ADC select
  //Sets the display RAM address SEG output correspondence 
	Lm6063LcdSendByte(0xa0, LM6063_CMD);//normal
	//Lm6063LcdSendByte(0xa1, LM6063_CMD);//reverse 
  //--------------------
  //
	Lm6063LcdSendByte(0xa6, LM6063_CMD);
	Lm6063LcdSendByte(0xa4, LM6063_CMD);
	Lm6063LcdSendByte(0xa2, LM6063_CMD);
	
  //--------------------
  //Common Output Mode Select
	//Lm6063LcdSendByte(0xc0, LM6063_CMD);//Normal
  Lm6063LcdSendByte(0xc8, LM6063_CMD);//Reverse 
	//--------------------
  
	Lm6063LcdSendByte(0x2f, LM6063_CMD);
	Lm6063LcdSendByte(0x25, LM6063_CMD);
	Lm6063LcdSendByte(0xf8, LM6063_CMD);
	
	Lm6063LcdSendByte(0x00, LM6063_CMD);
  //Lm6063LcdSendByte(0x01, LM6063_CMD);
	//--------------------
	//Контрастность - Electronic volume mode set
	Lm6063LcdSendByte(0x81, LM6063_CMD);//Команда
	Lm6063LcdSendByte(0x22, LM6063_CMD);//Значение
	
	//--------------------	
	Lm6063LcdSendByte(0xaf, LM6063_CMD);
}
//-----------------------------------------------------------------------------
//Copies the LCD cache into the device RAM
void Lm6063LcdUpdate(uint8_t *displayBuf){
  
  uint8_t  i, j;
  //--------------------
	Lm6063LcdSendByte(0x40, LM6063_CMD);	//Set Display Start Line = com0
	for(i = 0; i < 8; i++)
    {
      Lm6063LcdSendByte(0xB0|i, LM6063_CMD); //Set Page Address as ComTable
      Lm6063LcdSendByte(0x10,   LM6063_CMD); //Set Column Address = 0
      Lm6063LcdSendByte(0x04,   LM6063_CMD); //Colum from 4 -> 132 auto add
      for(j = 0; j < 128; j++)
        {
          Lm6063LcdSendByte(*(displayBuf++), LM6063_DATA);
        }
    }	
}
//-----------------------------------------------------------------------------



















