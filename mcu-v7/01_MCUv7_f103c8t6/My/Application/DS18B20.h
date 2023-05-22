/*
 * DS18B20.h
 *
 *  Created on: 20 дек. 2020 г.
 *      Author: Zver
 */
#ifndef DS18B20_H_
#define DS18B20_H_
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define DELAY_WRITE_1        	5
#define DELAY_WRITE_1_PAUSE  	60

#define DELAY_WRITE_0        	65
#define DELAY_WRITE_0_PAUSE  	5

#define DELAY_READ_SLOT      	2 //1
#define DELAY_BUS_RELAX      	15
#define DELAY_READ_PAUSE     	55//45
//**********************************
#define DS18B20_NO_SENSE        -255

#define DS18B20_SCALE_9_bit  	(uint16_t)(0.5    * 10000) // 9 бит - 0,5    градуса
#define DS18B20_SCALE_10_bit 	(uint16_t)(0.25   * 10000) //10 бит - 0,25   градуса
#define DS18B20_SCALE_11_bit 	(uint16_t)(0.125  * 10000) //11 бит - 0,125  градуса
#define DS18B20_SCALE_12_bit 	(uint16_t)(0.0625 * 10000) //12 бит - 0,0625 градуса


#define OWI_ROM_SEARCH_FINISHED     0x00    //!< Search finished return code.
#define OWI_ROM_SEARCH_FAILED       0xff    //!< Search failed return code.

#define SEARCH_SUCCESSFUL     		0x00
#define SEARCH_CRC_ERROR      		0x01
#define SEARCH_ERROR          		0xff
//**********************************
//Команды DS18B20
typedef enum {
	SEARCH_ROM 		 = 0xF0,	//определить адреса 1-Wire устройств, подключенных к шине.
	READ_ROM		 = 0x33,	//считывать 64-разрядный код устройства.
	SKIP_ROM         = 0xCC,	//обращения ко всем 1-Wire устройствам, подключенным к шине.
	MATCH_ROM        = 0x55,	//обращение к конкретному 1-Wire устройству.

	ALARM_SAERCH     = 0xEC,	//отвечают на данную команду устройства с установленным флагом аварии.

	CONVERT_T        = 0x44,
	READ_SCRATCHPAD  = 0xBE,
	WRITE_SCRATCHPAD = 0x4E,
	COPY_SCRATCHPAD  = 0x48,
	RECALL_E2		 = 0xB8,
	READ_PWR_SUPPLY  = 0xB4,

	TH_REGISTER      = 0x4B,
	TL_REGISTER      = 0x46,
}DS18B20_Cmd_Enum;
//**********************************
//Разрешение DS18B20
typedef enum {
	DS18B20_REOLUTION_9_BIT  = 0x1F,
	DS18B20_REOLUTION_10_BIT = 0x3F,
	DS18B20_REOLUTION_11_BIT = 0x5F,
	DS18B20_REOLUTION_12_BIT = 0x7F
}ds18b20_resolution_t;
//**********************************
//Блокнот DS18B20
#pragma pack(push, 1)//размер выравнивания в 1 байт
typedef union{
	struct{
		uint8_t temperatureLsb;
		uint8_t temperatureMsb;
		uint8_t ThReg;
		uint8_t TlReg;
		uint8_t configReg;
		uint8_t reserv1;
		uint8_t reserv2;
		uint8_t reserv3;
		uint8_t crc;
	}Str;
	uint8_t Blk[9];
}ds18b20_scratchPad_t;
#pragma pack(pop)//вернули предыдущую настройку.
//**********************************
//Структура данных от датчика температуры DS18B20
typedef struct{
	uint8_t					id[8];		//64 бита индентификатора
	uint8_t			  		number;		//Номер датчика
	ds18b20_resolution_t	resolution;	//точность измерения
	ds18b20_scratchPad_t	scratchPad;	//блокнот
	int16_t					temperature;
}ds18b20_t;
//*******************************************************************************************
//*******************************************************************************************
void 	 ONE_WIRE_Init(GPIO_TypeDef *port, uint32_t pin);
uint8_t  ONE_WIRE_SearchRom(uint8_t *idBuf, uint8_t lastDeviation);
uint8_t  ONE_WIRE_SearchDevices(ds18b20_t *devices, uint8_t numDevices, uint8_t *numCount);
void 	 ONE_WIRE_MatchRom(uint8_t *idArray);
uint8_t	 ONE_WIRE_SendCmd(uint8_t *idArray, DS18B20_Cmd_Enum cmd);
void 	 ONE_WIRE_ReadDeviceData(ds18b20_t *device);

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
#define ONE_WIRE_PORT	GPIOB
#define ONE_WIRE_PIN	14


void 	TEMPERATURE_SENSE_Init(void);
void 	TEMPERATURE_SENSE_SetResolution(uint8_t sensorNumber, ds18b20_resolution_t resolution);
void 	TEMPERATURE_SENSE_ReadData(uint8_t sensorNumber);
int16_t TEMPERATURE_SENSE_GetTemp(uint8_t sensorNumber);
void 	TEMPERATURE_SENSE_BuildPack(uint8_t sensorNumber, uint8_t *buf);

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
#endif /* DS18B20_H_ */





