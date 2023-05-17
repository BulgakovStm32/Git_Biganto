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
static protocolFlag_t 	   protocolFlags;
static MCU_SystemCtrlReg_t systemCtrlReg;

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
static uint8_t _protocol_CRC_Calc(uint8_t *pBlock, uint8_t len){

	return CRC8_FastCalculate(pBlock, len);
}
//************************************************************
static void _protocol_RequestParsing(void){

//	volatile MCU_Request_t  *request  = (MCU_Request_t *) I2C_IT_GetpRxBuf(&I2cWire);
//	volatile MCU_Response_t *response = (MCU_Response_t *)I2C_IT_GetpTxBuf(&I2cWire);

	MCU_Request_t  *request  = (MCU_Request_t *) I2C_IT_GetpRxBuf(&I2cWire);
	MCU_Response_t *response = (MCU_Response_t *)I2C_IT_GetpTxBuf(&I2cWire);
	uint32_t temp;
	uint8_t  cmd;
	//--------------------------
	//Проверка CRC.
//	volatile uint8_t crcCalc = CRC8_FastCalculate((uint8_t*)request, request->Count+1);
//	volatile uint8_t crcReq  = request->Payload[request->Count-1];

	uint8_t crcCalc = _protocol_CRC_Calc((uint8_t*)request, request->Count+1);
	uint8_t crcReq  = request->Payload[request->Count-1];
	if(crcCalc != crcReq) return;//если CRC не совпадает то выходим.
	//--------------------------
	I2cWire.timeOut 	= 0; 		//Сброс таймаута.
	protocolFlags.f_Led	= FLAG_SET;	//Индикация приема пакета.

	//DELAY_microS(50);//Отладка.
	//--------------------------
	//Разбор пришедшего запроса
	cmd = request->CmdCode;			//
	switch(cmd)
	{
		//-----------------------------------------------
		//---------------Команды мотора.-----------------
		//Задать микрошаг.
		case(cmdSetMicrostep):
			temp = request->Payload[0];
			MOTOR_SetMicrostepMode(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Задать передаточное число редуктора
		case(cmdSetReducerRate):
			temp = request->Payload[0];
			//MOTOR_SetReducerRate(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Задать время ускорение, в мС.
		case(cmdSetAccelerationTime):
			temp = *(uint32_t*)&request->Payload[0];
			//MOTOR_SetAccelerationTime(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Задать максимальную скорость в RPM
		case(cmdSetMaxVelocity):
			temp = *(uint32_t*)&request->Payload[0];
			//MOTOR_SetVelocity(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Вкл./Откл. момента удержания мотора.
		case(cmdMotorTorqueCtrl):
			temp = request->Payload[0];
//			if(temp) MOTOR_TorqueControl(MOTOR_TORQUE_ON);
//			else 	 MOTOR_TorqueControl(MOTOR_TORQUE_OFF);
			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Задать целевое положение, в градсах.
		case(cmdSetTargetPosition):
//			temp = *(uint32_t*)&request->Payload[0];
//			if(MOTOR_GetSpeedRampState() == STOP)
//			{
//				MOTOR_SetTargetPosition((int32_t)temp);
//				RTOS_SetTask(MOTOR_StartRotate, 0, 0);//запуск вращения.
//			}
			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Получить текущее положение, в градсах.

		//-------------------
		//Получить время в течении которого будет происходить ускорение, в мС.

		//-------------------
		//Получить максимальную скорость, в оборотах в минуту (RPM)


		//-----------------------------------------------
		//--------------Команды энкодера-----------------
		//Задать разрешение и тип кода (Грей или бинарный) энкодера.
		case(cmdSetEncoderConfig):
			temp = *(uint16_t*)&request->Payload[0];
			ENCODER_SetConfig(temp);

			//Ответ на команду.
			response->Count   = 2;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
		break;
		//-------------------
		//Получить параметры энкодера(разрешение и тип кода).
		case(cmdGetEncoderConfig):
			response->Count   = 4;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
			*(uint16_t*)&response->Payload[0] = ENCODER_GetConfig();
		break;
		//-------------------
		//Получить код энкодера, т.е. значение от 0 до 2^(разрешение_энкодера).
		case(cmdGetEncoderCode):
			response->Count   = 5;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = ENCODER_GetCode();
		break;
		//-------------------
		//Получить угол энкодера в градусах от 0 до 360 градусов.
		case(cmdGetEncoderAngle):
			response->Count   = 15;  //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды

			response->Payload[0] = 0xA0; //u8 SPI status      - заглушка
			response->Payload[1] = 0xA1; //u8 TMC reg address - заглушка

			*(uint32_t*)&response->Payload[6]  = DELAY_GetuSecCount(); 		//u32 beginTS
			//temp = ENCODER_GetAngle(); //расчет угла поворота вала энкодера.
			temp = MOTOR_GetMotorPosition();

			*(uint32_t*)&response->Payload[2]  = temp;	     					//u32 data
			*(uint32_t*)&response->Payload[10] = DELAY_GetuSecCount(); 		//u32 endTS
		break;
		//-----------------------------------------------
		//---------Команды датчиков температуры----------
		//Получить значение от датчика температуры.
		case(cmdGetTemperature):
			response->Count   = 5;   //Размер пакета в байтах (c полем CRC, но без поля COUNT)
			response->CmdCode = cmd; //код команды
			TEMPERATURE_SENSE_BuildPack(request->Payload[0], response->Payload);
		break;
		//-------------------
		//Команда опрашивает шину на наличие тепературных датчиков.


		//-----------------------------------------------
		//--------------Остальные команды----------------
		//Получить текущее значение миллисекундного таймера MCU
		case(cmdGetMillisCount):
			response->Count   = 5;   //Размер пакета в байтах (без полей COUNT и CRC)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = RTOS_GetTickCount();
		break;
		//-------------------
		//Чтение системного регистра управления.
		case(cmdGetSystemCtrlReg):
			response->Count   = 5;   //Размер пакета в байтах (без полей COUNT и CRC)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = systemCtrlReg.BLK;
		break;
		//-----------------------------------------------
		//--------------Отладочные команды---------------
		case(cmdGetSupplyVoltage):	 //Получение напряжения питания платы MCU, в мВ.
			response->Count   = 5; 	 //Размер пакета в байтах (без полей COUNT и CRC)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = POWER_GetSupplyVoltageSMA();//POWER_GetSupplyVoltage();
		break;
		//-------------------
		case(cmdGetSenseState):		 //получение состояния оптических сенсоров.
			response->Count   = 5; 	 //Размер пакета в байтах (без полей COUNT и CRC)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = OPT_SENS_GetState();
		break;
		//-------------------
		//Отладочные команды
		case(cmdGetResetCount):		 //получение счетчика переинициализации I2С.
			response->Count   = 5; 	 //Размер пакета в байтах (без полей COUNT и CRC)
			response->CmdCode = cmd; //код команды.
			*(uint32_t*)&response->Payload[0] = I2cWire.resetCount;
		break;
		//*******************************************
		//*******************************************
		default:
			//Получить текущее значение миллисекундного таймера MCU
			response->Count   = 5;   //Размер пакета в байтах (без полей COUNT и CRC)
			response->CmdCode = cmd; //код команды
			*(uint32_t*)&response->Payload[0] = RTOS_GetTickCount();
		//-------------------
	}
	//--------------------------
	//Расчет CRC
	crcCalc = _protocol_CRC_Calc((uint8_t*)response, response->Count+1);//crc для пакета(с полем COUNT)
	response->Payload[response->Count-1] = crcCalc;
	//--------------------------
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
	I2cWire.i2c		  = I2C2;				//используемый порт I2C
	I2cWire.i2cMode	  = I2C_MODE_SLAVE;		//режим SLAVE
	I2cWire.GpioRemap = I2C_GPIO_NOREMAP;	//нет ремапа ножек порта I2C
	I2cWire.i2cSpeed  = PROTOCOL_I2C_SPEED;	//скорость работы порта I2C
	I2cWire.slaveAddr = PROTOCOL_I2C_ADDR;	//адрес устройства на нине I2C

	I2cWire.rxBufSize = I2C_IT_RX_BUF_SIZE_DEFAULT;
	I2cWire.txBufSize = I2C_IT_RX_BUF_SIZE_DEFAULT;
	I2cWire.i2cSlaveRxCpltCallback = _protocol_RequestParsing;
	I2cWire.i2cSlaveTxCpltCallback = _protocol_TxParsing;
	I2C_IT_Init(&I2cWire);
	//I2C_DMA_Init(&I2cWire);
}
//**********************************************************
void PROTOCOL_I2C_CheckTimeout(void){

	//Индикация отсутствия обмена. Мигаем светодиодом с периодом 1000 мс.
	//Счетсик I2cWire.timeOut сбрасывается в _protocol_RequestParsing
	//при совпадении CRC.
	if(++I2cWire.timeOut >= PROTOCOL_I2C_REQUEST_TIMEOUT_mS)
	{
		I2cWire.timeOut = 0;
		I2cWire.resetCount++;	//
		I2C_IT_Init(&I2cWire);	//повторная инициализация I2C
		//I2C_DMA_Init(&I2cWire);
		LED_ACT_Toggel();
		return;
	}
	//Индикация обмена. Мигаем светодиодом с периодом 50 мс.
	static uint32_t ledCount = 0;
	if(protocolFlags.f_Led == FLAG_SET)
	{
		if(++ledCount >= 50)//периодг мигания 50 мс.
		{
			ledCount = 0;
			protocolFlags.f_Led = FLAG_CLEAR;
			LED_ACT_Toggel();
		}
	}
}
//**********************************************************
uint32_t PROTOCOL_I2C_GetResetCount(void){

	return I2cWire.resetCount;
}
//**********************************************************
protocolFlag_t* PROTOCOL_I2C_Flags(void){

	return &protocolFlags;
}
//**********************************************************
MCU_SystemCtrlReg_t* PROTOCOL_I2C_SystemCtrlReg(void){

	return &systemCtrlReg;
}
//*******************************************************************************************
//*******************************************************************************************

