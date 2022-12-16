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
static uint32_t encoderOffset     = 0;
static uint32_t encoderAngleQuantX1e6 = 0;//количество градусов в одном шаге энкодера *1000000.
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

	//Смещение относительно нуля.
	encoderOffset = ENCODER_GetCode();
}
//**********************************************************
void ENCODER_SetConfig(uint16_t config){

	//младший байт config - разрешение энкодера (Кол-во двоичных разрядов).
	//старший байт config - тип кода (Грей или бинарный) энкодера.
	encoderResolution =  config & 0x00FF;
	encoderType       = (config >> 8) & 0x00FF;

	if(encoderResolution == 0) 						   encoderResolution = ENCODER_RESOLUTION_BIT_MIN;
	if(encoderResolution > ENCODER_RESOLUTION_BIT_MAX) encoderResolution = ENCODER_RESOLUTION_BIT_MAX;

	uint32_t res = (1 << encoderResolution);
	encoderMask  = (res - 1);
	encoderAngleQuantX1e6 = (360 * 1000000 + res/2) / res;//кол-во градусов в одном шаге энкодера *1000000

	//Опеределение на сколько нужно сдвинуть вычитанные из энкодера данные.
	//ENCODER_SPI_READ_BIT - столько всего бит вычитывается из энкодера.
	//resolution 		   - разрешение энкодера
	//Минус 1 т.к. первый вычитанный бит незначащий.
	encoderShift = ENCODER_SPI_READ_BIT_NUM - encoderResolution - 1;
}
//**********************************************************
uint16_t ENCODER_GetConfig(void){

	//младший байт - разрешение энкодера (Кол-во двоичных разрядов).
	//старший байт - тип кода (Грей или бинарный) энкодера.

	return (uint16_t)((uint8_t)(encoderType << 8) | (uint8_t)encoderResolution);
}
//**********************************************************
uint32_t ENCODER_GetAngleQuant(void){

	return encoderAngleQuantX1e6;
}
//**********************************************************
uint32_t ENCODER_GetOffset(void){

	return encoderOffset;
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
//получение угла энкодера.
uint32_t ENCODER_GetAngle(void){

	return (ENCODER_GetCode() * encoderAngleQuantX1e6 + 500) / 1000;
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

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Ф-ии для отладки работы энкодера
static EncoderDebug_t encoderDebug = {.code   = 0,
									  .offset = 0,
									  .angle  = 0.0,
									  .RPM 	  = 0.0};
static float 	oldAngle   = 0.0;
static float 	deltaAngle = 0.0;
static uint32_t sysTick    = 0;
static uint8_t  txBuf[64]  = {0};
//*******************************************************************************************
//*******************************************************************************************
static void BinToDecWithoutDot(uint32_t var, uint8_t* buf){

	*(buf+0) = (uint8_t)(var / 100000) + '0';
	var %= 100000;

	*(buf+1) = (uint8_t)(var / 10000) + '0';
	var %= 10000;

	*(buf+2) = (uint8_t)(var / 1000) + '0';
	var %= 1000;

	//*(buf+3) = ',';

	*(buf+3) = (uint8_t)(var / 100) + '0';
	var %= 100;

	*(buf+4) = (uint8_t)(var / 10) + '0';
	*(buf+5) = (uint8_t)(var % 10) + '0';
}
//************************************************************
static void BinToDecWithDot(uint32_t var, uint8_t* buf){

	*(buf+0) = (uint8_t)(var / 100000) + '0';
	var %= 100000;

	*(buf+1) = (uint8_t)(var / 10000) + '0';
	var %= 10000;

	*(buf+2) = (uint8_t)(var / 1000) + '0';
	var %= 1000;

	*(buf+3) = ',';

	*(buf+4) = (uint8_t)(var / 100) + '0';
	var %= 100;

	*(buf+5) = (uint8_t)(var / 10) + '0';
	*(buf+6) = (uint8_t)(var % 10) + '0';
}
//************************************************************
static void BuildAndSendTextBuf(uint32_t timeStamp, uint32_t encodTicks, uint32_t angle, uint32_t speed){

	BinToDecWithDot(timeStamp, txBuf);
	txBuf[7] = '\n';//'\t';

	BinToDecWithoutDot(encodTicks, txBuf+8);
	txBuf[14] = '\n';//'\t';

	BinToDecWithDot(angle, txBuf+15);
	txBuf[22] = '\n';//'\t';

	BinToDecWithDot(speed, txBuf+23);
	txBuf[30] = '\n';//'\t';

	//USART1_TX -> DMA1_Channel4
	//USART2_TX -> DMA1_Channel7
//	DMAxChxStartTx(DMA1_Channel7, txBuf, 31);
}
//************************************************************
EncoderDebug_t* ENCODER_DBG_Data(void){

	return &encoderDebug;
}
//************************************************************
//Работа с энкодером. Для отладки данные предаются по USART.
void ENCODER_DBG_CalcAngleAndSpeed(void){

	static uint32_t mSecCount = 0;
	//--------------------------
	sysTick++;
	if(sysTick > 999999) sysTick = 0;
	//--------------------------
	if(++mSecCount >= 100)//расчет скорости производится каждые 100мс.
	{
		mSecCount = 0;
		//__disable_irq();
		//LED_ACT_High();
		//Чтение заначения энкодера. ~9,5 мкС
		encoderDebug.code  = ENCODER_GetCode() - encoderDebug.offset;
		if(encoderDebug.code < 0) encoderDebug.code = -encoderDebug.code;
		//LED_ACT_Low();
		//Расчет значений для расчета скрости.
		encoderDebug.angle = ENCODER_GetAngleQuant() * encoderDebug.code;//расчет угла поворота вала энкодера.
		if(oldAngle > encoderDebug.angle) oldAngle -= 360.0;	         //Это нужно для корректного расчета скорости при переходе от 359 к 0 градусов.
		deltaAngle = encoderDebug.angle - oldAngle;         		     //приращение угла

		//Расчет скорости вращения.
		encoderDebug.RPM = deltaAngle * QUANT_FOR_100mS;
		oldAngle = encoderDebug.angle;
		//Передаем данные по USART на компьютер.
		BuildAndSendTextBuf(sysTick,
							encoderDebug.code,
							(uint32_t)(encoderDebug.angle * 1000),
							(uint32_t)(encoderDebug.RPM   * 1000));
		//__enable_irq();
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************




















