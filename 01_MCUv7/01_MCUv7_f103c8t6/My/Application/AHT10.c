/*
 * AHT10.c
 *
 *  Created on: 25 янв. 2021 г.
 *      Author: belyaev
 */
//AHT10 - датчик температуры и влажности.
//*******************************************************************************************
//*******************************************************************************************

#include "AHT10.h"

//*******************************************************************************************
//*******************************************************************************************

static AHT10_t AHT10Str;
static uint8_t TemperatureSign = 0;

//*******************************************************************************************
//*******************************************************************************************
void AHT10_Init(void){

}
//************************************************************
void AHT10_SoftReset(void){

	uint8_t cmd = 0xBA;//soft reset command
	//-------------------
	I2C_StartAndSendDeviceAddr(I2C2, AHT10_ADDR);
	I2C_SendData(I2C2, &cmd, 1);
	I2C_Stop(I2C2);

	msDelay(50);
}
//************************************************************
AHT10_t* AHT10(void){

	return &AHT10Str;
}
//************************************************************
void AHT10_ReadData(void){

	uint8_t  cmd           = 0xAC;//start measurment command
	uint8_t  rxBuf[6]      = {0}; //
	uint32_t AHT10_ADC_Raw = 0;   //
	int32_t  temp          = 0;   //
	//-------------------
	I2C_StartAndSendDeviceAddr(I2C2, AHT10_ADDR);
	I2C_SendData(I2C2, &cmd, 1);
	I2C_Stop(I2C2);

	//msDelay(100);
	//msDelay(10);

	I2C_StartAndSendDeviceAddr(I2C2, (AHT10_ADDR | 0x01));
	I2C_ReadData(I2C2, rxBuf, 6);
	//I2C_Stop(I2C2);

	//Расчет влажности
	AHT10_ADC_Raw   = (rxBuf[1] << 16) | (rxBuf[2] << 8) | (rxBuf[3] & 0xF0);
	AHT10_ADC_Raw >>= 4;
	AHT10_ADC_Raw  /= 100;
	AHT10Str.Humidity = (uint16_t)AHT10_ADC_Raw;

	AHT10_ADC_Raw  = ((rxBuf[3] & 0x0F) << 16) | (rxBuf[4] << 8) | rxBuf[5];
	//Расчет температуры.
	AHT10_ADC_Raw *= 19;
	temp = AHT10_ADC_Raw - 5000000;
	//Знак температуры
	if(temp & 0x80000000)
		{
			TemperatureSign = AHT10_SIGN_NEGATIVE;
			temp *= -1;
		}
	else	TemperatureSign = AHT10_SIGN_POSITIVE;

	AHT10_ADC_Raw = temp / 10000;

	AHT10Str.Temperature = (uint16_t)AHT10_ADC_Raw;
}
//**********************************************************
uint8_t AHT10_GetTemperatureSign(void){

	return TemperatureSign;
}
//*******************************************************************************************
//*******************************************************************************************



