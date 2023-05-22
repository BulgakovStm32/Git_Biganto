/*
 * Protocol_I2C.h
 *
 *  Created on: 22 авг. 2022 г.
 *      Author: belyaev
 */
#ifndef MCUV7_PROTOCOL_I2C_H
#define MCUV7_PROTOCOL_I2C_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define PROTOCOL_I2C_ADDR					0x06	//Адрес на шине I2C
#define PROTOCOL_I2C_SPEED					400000	//Скорсть работы шины I2C

#define PROTOCOL_I2C_REQUEST_TIMEOUT_mS		1000	//
//****************************************************
//Структура запроса.
#pragma pack(push, 1)//размер выравнивания в 1 байт
typedef struct{
	uint8_t  CmdCode;    	// Код команды.
	uint8_t  Count;       	// Размер блока.
	uint8_t	 Payload[62];	// Данные
}MCU_Request_t;
#pragma pack(pop)//вернули предыдущую настройку.
//****************************************************
//Структура ответа.
#pragma pack(push, 1)//размер выравнивания в 1 байт
typedef struct{
	uint8_t  Count;       	// Размер блока.
	uint8_t  CmdCode;    	// Код команды.
	uint8_t	 Payload[62];	// Данные
}MCU_Response_t;
#pragma pack(pop)//вернули предыдущую настройку.
//****************************************************
typedef union{
	struct{
		uint32_t PwrEvent :2;		//PWR_EVENT (R) - событие выключения
									//0 - ничего
									//1 - произошёл запрос на выключение
									//2 - происходит выключение

		uint32_t f_PwrOff :1;		//PWR_OFF (R) - бит текущего состояния выключения питания
									//0 - остальные режимы работы
									//1 - пришла команда cmdTurnOffPower,
		//uint32_t dummy    :28;
	}Flags;
	uint32_t BLK;
}MCU_SystemCtrlReg_t;
//****************************************************
//флаги приема команд.
typedef struct{
//	uint32_t f_CmdMotorTorqueCtrl   :1; //пришла команда Вкл./Откл. момента удержания ШД.
//	uint32_t f_CmdSetTargetPosition :1; //пришла команда на запуск вращения.
	uint32_t f_Led  				:1;
	uint32_t f_CmdTurnOff 			:1; //пришла команда отключения питания.
	uint32_t dummy	     	 	   	:29;
}protocolFlag_t;
//*******************************************************************************************
//*******************************************************************************************
void 	 	    PROTOCOL_I2C_Init(void);
void 	 	    PROTOCOL_I2C_CheckTimeout(void);
uint32_t 		PROTOCOL_I2C_GetResetCount(void);
protocolFlag_t* PROTOCOL_I2C_Flags(void);
MCU_SystemCtrlReg_t* PROTOCOL_I2C_SystemCtrlReg(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_PROTOCOL_I2C_H */



