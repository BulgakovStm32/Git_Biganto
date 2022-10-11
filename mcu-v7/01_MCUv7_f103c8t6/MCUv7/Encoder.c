/*
 * Encoder.c
 *
 *  Created on: 5 авг. 2022 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "Encoder.h"

//*******************************************************************************************
//*******************************************************************************************
static uint32_t encoderResolution = 0; //Разрешение энкодера (Кол-во двоичных разрядов).
static uint32_t encoderType       = 0; //тип кода (Грей или бинарный) энкодера

static uint32_t encoderState 	  = 0;
//*******************************************************************************************
//*******************************************************************************************
static uint32_t _encoder_GrayToBin(uint32_t grayCode){

	grayCode ^= grayCode >> 1;
	grayCode ^= grayCode >> 2;
	grayCode ^= grayCode >> 4;
	grayCode ^= grayCode >> 8;
	grayCode ^= grayCode >> 16;
	return grayCode;
}
//*******************************************************************************************
//*******************************************************************************************
void ENCODER_Init(void){

	SPI_Init(ENCODER_SPI);
	encoderState = ENCODER_READY;
	ENCODER_GetCode();	//Холостое чтение для очистки регистров SPI
	//volatile uint32_t temp = ENCODER_GetCode();	//Холостое чтение для очистки регистров SPI
}
//**********************************************************
//Получение значение поворота энкодера.
uint32_t ENCODER_GetCode(void){

	if(encoderState == ENCODER_NO_INIT) return 0;
	//Чтение и выравнивание данных из энкодера.
	//uint32_t encoderVal = (SPI_Rx3Byte(ENCODER_SPI) >> 7) & 0x0000FFFF; //Разрешения энкодера 16 бит.
	//return _encoder_GrayToBin(encoderVal);                            	//Преобразование кода Грея в двоичный код.
	return ((SPI_Rx3Byte(ENCODER_SPI) >> 7) & 0x0000FFFF);//Разрешения энкодера 16 бит.
}
//**********************************************************
void ENCODER_GetCodeEncoder(uint8_t *buf){

	uint32_t encoder = ENCODER_GetCode();

	buf[0] = (uint8_t) encoder;
	buf[1] = (uint8_t)(encoder >> 8);
	buf[2] = (uint8_t)(encoder >> 16);
	buf[3] = (uint8_t)(encoder >> 24);
}
//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************



//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************

















