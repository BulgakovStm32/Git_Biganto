/*
 * Protocol_I2C.c
 *
 *  Created on: 22 авг. 2022 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "Protocol_I2C.h"

//*******************************************************************************************
//*******************************************************************************************

static I2C_IT_t I2cWire;
static uint32_t ledFlag = 0;
//*******************************************************************************************
//*******************************************************************************************
static void _protocol_RequestParsing(void){

	MCU_Request_t  *request  = (MCU_Request_t *) I2C_IT_GetpRxBuf(&I2cWire);
	MCU_Response_t *response = (MCU_Response_t *)I2C_IT_GetpTxBuf(&I2cWire);
	uint32_t temp = 0;
	//--------------------------
	//Проверка CRC.
	volatile uint8_t crcCalc = CRC8_FastCalculate((uint8_t*)request, request->Count+1);
	volatile uint8_t crcReq  = request->Payload[request->Count-1];
	if(crcCalc != crcReq) return;//если CRC не совпадает то выходим.
	//--------------------------
	I2cWire.timeOut = 0; //Сброс таймаута.
	ledFlag = 1;		 //Индикация приема пакета.

	//DELAY_microS(50);//Отладка.
	//Разбор пришедшего запроса
	switch(request->CmdCode)
	{
//		//-------------------
//		case(cmdGetCurrentPosition):
//			//LED_ACT_Toggel();
//			response->Count   = 15; 				  //Размер пакета в байтах (без поля COUNT)
//			response->CmdCode = cmdGetCurrentPosition;//код командыё
//		break;
//		//-------------------
//		case(cmdGetCurrentAcceleration):
//		//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdGetCurrentVelocity):
//		//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdSetTargetPosition):
//		//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdSetMaxAcceleration):
//		//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdSetMaxVelocity):
//				//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdSetMicrostep):
//				//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdResetTMC):
//				//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdResetPosition):
//				//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdEmergencyStop):
//				//LED_ACT_Toggel();
//		break;
		//-------------------
		case(cmdArduinoMicroTS):
			response->Count   = 5; //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmdArduinoMicroTS;
			*(uint32_t*)&response->Payload[0] = RTOS_GetTickCount();
		break;
		//-------------------
//		case(cmdArduinoMeasurePulseCalibration):
//				//LED_ACT_Toggel();
//		break;
//		//-------------------
//		case(cmdArduinoGetPulseCalibration):
//				//LED_ACT_Toggel();
//		break;
		//-------------------
		case(cmdGetEncoderPosition):
			response->Count   = 15; 				  //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmdGetEncoderPosition;//код команды

			response->Payload[0] = 0xA0; //u8 SPI status      - заглушка
			response->Payload[1] = 0xA1; //u8 TMC reg address - заглушка

			*(uint32_t*)&response->Payload[6]  = DELAY_microSecCount(); 		//u32 beginTS
			temp = (uint32_t)(ENCODER_DEGREE_QUANT * ENCODER_GetCode() * 1000); //расчет угла поворота вала энкодера.
			*(uint32_t*)&response->Payload[2]  = temp;	     					//u32 data
			*(uint32_t*)&response->Payload[10] = DELAY_microSecCount(); 		//u32 endTS
		break;
		//-------------------
		case(cmdGetTemperature):
			response->Count   = 5; 			      // Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmdGetTemperature;// Команда
			TEMPERATURE_SENSE_BuildPack(request->Payload[0], response->Payload);
		break;
		//-------------------
		case(cmdGetSupplyVoltage):
			response->Count   = 5; 			        // Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmdGetSupplyVoltage;// Команда
			*(uint32_t*)&response->Payload[0] = POWER_GetSupplyVoltage();
		break;
		//*******************************************
		//*******************************************
		//Отладочные команды
		case(cmdGetSenseState):					 // получение состояния оптических сенсоров.
			response->Count   = 5; 			     // Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmdGetSenseState;// Команда
			*(uint32_t*)&response->Payload[0] = OPT_SENS_GetState();
		break;
		//-------------------
		//Отладочные команды
		case(cmdGetResetCount):					 // получение счетчика переинициализации I2С.
			response->Count   = 5; 			     // Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmdGetResetCount;// Команда
			*(uint32_t*)&response->Payload[0] = I2cWire.resetCount;
		break;
		//-------------------

		//*******************************************
		//*******************************************
		default: return;
		//-------------------
	}
	//Расчет CRC
	response->Payload[response->Count-1] = CRC_Calculate((uint8_t*)response, response->Count+1);//crc для пакета(с полем COUNT).
	//Кол-во байтов в ответе(с полями COUNT и CRC).
	I2C_IT_SetTxSize(&I2cWire, response->Count+1+1);
//	I2C_DMA_Write(&I2cWire);
}
//************************************************************
static void _protocol_TxParsing(void){

}
//*******************************************************************************************
//*******************************************************************************************
void PROTOCOL_I2C_Init(void){

	//Инициализация I2C Slave для работы по прерываниям.
	I2cWire.i2c		  = I2C2;
	I2cWire.i2cMode	  = I2C_MODE_SLAVE;
	I2cWire.GpioRemap = I2C_GPIO_NOREMAP;
	I2cWire.slaveAddr = 0x05;
	I2cWire.rxBufSize = I2C_IT_RX_BUF_SIZE_DEFAULT;
	I2cWire.txBufSize = I2C_IT_RX_BUF_SIZE_DEFAULT;
	I2cWire.i2cSlaveRxCpltCallback = _protocol_RequestParsing;
	I2cWire.i2cSlaveTxCpltCallback = _protocol_TxParsing;
	I2C_IT_Init(&I2cWire);
	//I2C_DMA_Init(&I2cWire);
}
//**********************************************************
void PROTOCOL_I2C_IncTimeoutAndResetI2c(void){

	//Счетсик I2cWire.timeOut сбрасывается в _protocol_RequestParsing
	//при совпадении CRC.
	I2cWire.timeOut++;
	if(I2cWire.timeOut >= PROTOCOL_I2C_REQUEST_TIMEOUT_mS)
	{
		I2cWire.timeOut = 0;
		I2cWire.resetCount++;
		I2C_IT_Init(&I2cWire);
		//I2C_DMA_Init(&I2cWire);
		LED_ACT_Toggel();//Индикация отсутствия обмена.
	}
	//Индикация обмена. Мигаем светодиодом.
	static uint32_t ledCount = 0;
	if(ledFlag)
	{
		if(++ledCount >= 50)
		{
			ledCount = 0;
			ledFlag  = 0;
			LED_ACT_Toggel();
		}
	}
}
//**********************************************************
uint32_t PROTOCOL_I2C_GetResetCount(void){

	return I2cWire.resetCount;
}
//*******************************************************************************************
//*******************************************************************************************

