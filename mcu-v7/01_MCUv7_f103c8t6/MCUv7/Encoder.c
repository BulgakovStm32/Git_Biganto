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

static uint8_t  encoderResolution = 0; //Разрешение энкодера (Кол-во двоичных разрядов).
static uint8_t  encoderType       = 0; //тип кода (Грей или бинарный) энкодера
static uint32_t encoderShift	  = 0;
static uint32_t encoderMask		  = 0;
static uint32_t encoderState 	  = 0;
static float encoderAngleQuant	  = 0;//количество градусов в одном наге энкодера.

//*******************************************************************************************
//*******************************************************************************************
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
//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void ENCODER_Init(void){

	SPI_Init(ENCODER_SPI);

	//Заводские настройки.
	ENCODER_SetConfig((ENCODER_DEFAULT_TYPE << 8) |
					   ENCODER_DEFAULT_RESOLUTION_BIT);
	encoderState = ENCODER_READY;
	//Холостое чтение для очистки регистров SPI
	ENCODER_GetCode();
}
//**********************************************************
void ENCODER_SetConfig(uint16_t config){

	//младший байт config - разрешение энкодера (Кол-во двоичных разрядов).
	//старший байт config - тип кода (Грей или бинарный) энкодера.
	encoderResolution =  config & 0x00FF;
	encoderType       = (config >> 8) & 0x00FF;

	if(encoderResolution == 0) 							encoderResolution = ENCODER_RESOLUTION_BIT_MIN;
	if(encoderResolution >= ENCODER_RESOLUTION_BIT_MAX) encoderResolution = ENCODER_RESOLUTION_BIT_MAX;
	encoderMask       = ((1 << encoderResolution) - 1);
	encoderAngleQuant = (360.0 / (1 << encoderResolution));

	//Опеределение на сколько нужно сдвинуть вычитанные из энкодера данные.
	//ENCODER_SPI_READ_BIT - столько всего бит вычитывается из энкодера.
	//resolution - разрешение энкодера
	//Минус 1 т.к. первый вычитанный бит незначащий.
	encoderShift = ENCODER_SPI_READ_BIT_NUM - encoderResolution - 1;
}
//**********************************************************
uint16_t ENCODER_GetConfig(void){

	//младший байт - разрешение энкодера (Кол-во двоичных разрядов).
	//старший байт - тип кода (Грей или бинарный) энкодера.

	return (uint16_t)( (uint8_t)(encoderType << 8) | (uint8_t)encoderResolution);
}
//**********************************************************
float ENCODER_GetAngleQuant(void){

	return encoderAngleQuant;
}
//**********************************************************
//Получение значение поворота энкодера.
int32_t ENCODER_GetCode(void){

	uint32_t encoderVal;
	//--------------------------
	if(encoderState == ENCODER_NO_INIT) return 0;

	encoderVal  = SPI_Rx3Byte(ENCODER_SPI);  //Чтение данных из энкодера. Вычитываем 3 байта.
	encoderVal  = encoderVal >> encoderShift;//Делаем нужное кол-во бит.
	encoderVal &= encoderMask; 				 //маской убираем незначащий бит.
	//Тип кода энкодера.
	if(encoderType == ENCODER_TYPE_BIN)  return encoderVal;
	if(encoderType == ENCODER_TYPE_GRAY) return _encoder_GrayToBin(encoderVal);
	return -1;//ошибка типа
}
//**********************************************************
void ENCODER_BuildPackCode(uint8_t *buf){

	uint32_t encoder = ENCODER_GetCode();
	//--------------------------
	buf[0] = (uint8_t) encoder;
	buf[1] = (uint8_t)(encoder >> 8);
	buf[2] = (uint8_t)(encoder >> 16);
	buf[3] = (uint8_t)(encoder >> 24);
}
//**********************************************************
void ENCODER_BuildPackConfig(uint8_t *buf){

	uint16_t config = ENCODER_GetConfig();
	//--------------------------
	buf[0] = (uint8_t) config;
	buf[1] = (uint8_t)(config >> 8);
}
//**********************************************************

//**********************************************************

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************




















