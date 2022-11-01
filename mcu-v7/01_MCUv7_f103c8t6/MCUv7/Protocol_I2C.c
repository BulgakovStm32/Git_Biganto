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

static I2C_IT_t 		   I2cWire;
static ProtocolFlag_t 	   protocolFlags;
static MCU_SystemCtrlReg_t systemCtrlReg;
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
static void _protocol_RequestParsing(void){

	volatile MCU_Request_t  *request  = (MCU_Request_t *) I2C_IT_GetpRxBuf(&I2cWire);
	volatile MCU_Response_t *response = (MCU_Response_t *)I2C_IT_GetpTxBuf(&I2cWire);
	uint32_t temp;
	uint8_t  cmd;
	//--------------------------
	//Проверка CRC.
	volatile uint8_t crcCalc = CRC8_FastCalculate((uint8_t*)request, request->Count+1);
	volatile uint8_t crcReq  = request->Payload[request->Count-1];
	if(crcCalc != crcReq) return;//если CRC не совпадает то выходим.
	//--------------------------
	I2cWire.timeOut 	= 0; 		//Сброс таймаута.
	protocolFlags.f_Led	= FLAG_SET;	//Индикация приема пакета.
	cmd = request->CmdCode;			//
	//DELAY_microS(50);//Отладка.
	//Разбор пришедшего запроса
	switch(cmd)
	{
		//*******************************************
		//*******************************************
		case(cmdSetMicrostep):	//Задать микрошаг.
			temp = request->Payload[0];
			MOTOR_SetMicrostepMode(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		case(cmdSetReducerRate): //Задать передаточное число редуктора
			temp = request->Payload[0];
			MOTOR_SetReducerRate(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		case(cmdSetAccelerationTime): //Задать время ускорение, в мС.
			temp = *(uint32_t*)&request->Payload[0];
			MOTOR_SetAccelerationTime(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		case(cmdSetMaxVelocity): //Задать максимальную скорость в RPM
			temp = *(uint32_t*)&request->Payload[0];
			MOTOR_SetVelocity(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		case(cmdMotorTorqueCtrl): //Вкл./Откл. момента удержания мотора.
			temp = request->Payload[0];
			if(temp) MOTOR_TorqueControl(MOTOR_TORQUE_ON);
			else 	 MOTOR_TorqueControl(MOTOR_TORQUE_OFF);

//			if(temp)
//			{
//				MOTOR_TorqueControl(MOTOR_TORQUE_ON);
//				protocolCmdFlags.f_cmdMotorTorqueCtrl = FLAG_SET;
//			}
//			else protocolCmdFlags.f_cmdMotorTorqueCtrl = FLAG_CLEAR;

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		case(cmdSetTargetPosition): //Задать целевое положение, в градсах.
			temp = *(uint32_t*)&request->Payload[0];
			if(MOTOR_GetSpeedRampState() == STOP)
			{
				MOTOR_SetPosition((int32_t)temp);
				RTOS_SetTask(MOTOR_StartRotate, 0, 0);//запуск вращения.
			}
			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------


		//*******************************************
		//*******************************************
		case(cmdGetMillisCount):
			response->Count   = 5;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = RTOS_GetTickCount();
		break;
		//-------------------
		case(cmdGetEncoderAngle):
			response->Count   = 15;  //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды

			response->Payload[0] = 0xA0; //u8 SPI status      - заглушка
			response->Payload[1] = 0xA1; //u8 TMC reg address - заглушка

			*(uint32_t*)&response->Payload[6]  = DELAY_microSecCount(); 		//u32 beginTS
			temp = (uint32_t)(ENCODER_GetAngleQuant() * ENCODER_GetCode() * 1000.0); //расчет угла поворота вала энкодера.
			*(uint32_t*)&response->Payload[2]  = temp;	     					//u32 data
			*(uint32_t*)&response->Payload[10] = DELAY_microSecCount(); 		//u32 endTS
		break;
		//-------------------
		case(cmdGetTemperature):
			response->Count   = 5;   // Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
			TEMPERATURE_SENSE_BuildPack(request->Payload[0], response->Payload);
		break;
		//-------------------
		case(cmdGetSystemCtrlReg):
			response->Count   = 5;   //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = systemCtrlReg.BLK;
		break;
		//*******************************************
		//*******************************************
		//Отладочные команды
		case(cmdGetSupplyVoltage):	 //Получение напряжения питания платы MCU, в мВ.
			response->Count   = 5; 	 //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = POWER_GetSupplyVoltage();
		break;
		//-------------------
		case(cmdGetSenseState):		 //получение состояния оптических сенсоров.
			response->Count   = 5; 	 //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = OPT_SENS_GetState();
		break;
		//-------------------
		//Отладочные команды
		case(cmdGetResetCount):		 //получение счетчика переинициализации I2С.
			response->Count   = 5; 	 //Размер пакета в байтах (без поля COUNT)
			response->CmdCode = cmd; //код команды.
			*(uint32_t*)&response->Payload[0] = I2cWire.resetCount;
		break;
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
//*******************************************************************************************
//*******************************************************************************************
void PROTOCOL_I2C_Init(void){

	systemCtrlReg.BLK = 0;

	//Инициализация I2C Slave для работы по прерываниям.
	I2cWire.i2c		  = I2C2;
	I2cWire.i2cMode	  = I2C_MODE_SLAVE;
	I2cWire.GpioRemap = I2C_GPIO_NOREMAP;
	I2cWire.i2cSpeed  = 400000;
	I2cWire.slaveAddr = 0x06;
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
	if(protocolFlags.f_Led == FLAG_SET)
	{
		if(++ledCount >= 50)
		{
			ledCount = 0;
			protocolFlags.f_Led == FLAG_CLEAR;
			LED_ACT_Toggel();
		}
	}
}
//**********************************************************
uint32_t PROTOCOL_I2C_GetResetCount(void){

	return I2cWire.resetCount;
}
//**********************************************************
ProtocolFlag_t* PROTOCOL_I2C_Flags(void){

	return &protocolFlags;
}
//**********************************************************
MCU_SystemCtrlReg_t* PROTOCOL_I2C_SystemCtrlReg(void){

	return &systemCtrlReg;
}
//*******************************************************************************************
//*******************************************************************************************

