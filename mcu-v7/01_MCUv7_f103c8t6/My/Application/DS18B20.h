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
#define DS18B20_READY			0
#define DS18B20_BUSY			1

#define DS18B20_SIGN_POSITIVE	0
#define DS18B20_SIGN_NEGATIVE   1

//#define DELAY_T_CONVERT      760000
//#define DELAY_RESET_PULSE    500
//#define DELAY_PROTECTION     5

#define DELAY_WRITE_1        5
#define DELAY_WRITE_1_PAUSE  60

#define DELAY_WRITE_0        65
#define DELAY_WRITE_0_PAUSE  5

#define DELAY_READ_SLOT      1
#define DELAY_BUS_RELAX      15
#define DELAY_READ_PAUSE     55//45

//**********************************
typedef enum {
	SEARCH_ROM 		 = 0xFF,//определить адреса 1-Wire устройств, подключенных к шине.
	READ_ROM		 = 0x33,//считывать 64-разрядный код устройства.
	SKIP_ROM         = 0xCC,//обращения ко всем 1-Wire устройствам, подключенным к шине.
	MATCH_ROM        = 0x55,//обращение к конкретному 1-Wire устройству.
	ALARM_SAERCH     = 0xEC,//отвечают на данную команду устройства с установленным флагом аварии.

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
typedef enum {
  DS18B20_Resolution_9_bit  = 0x1F,
  DS18B20_Resolution_10_bit = 0x3F,
  DS18B20_Resolution_11_bit = 0x5F,
  DS18B20_Resolution_12_bit = 0x7F
}DS18B20_Resolution_Enum;
//**********************************
typedef struct{
	GPIO_TypeDef		   *GPIO_PORT;
	uint32_t	  			GPIO_PIN;
	uint32_t				SensorNumber;	 //SENSOR_NUMBER;
	DS18B20_Resolution_Enum Resolution;  	 //RESOLUTION;
	uint32_t				TemperatureSign; //TEMPERATURE_SIGN;
	uint32_t				Temperature;     //TEMPERATURE;
}DS18B20_t;
//*******************************************************************************************
//*******************************************************************************************
void 	 TemperatureSens_GpioInit(DS18B20_t *sensor);
void 	 TemperatureSens_SetResolution(DS18B20_t *sensor);
void 	 TemperatureSens_Init(DS18B20_t *sensor);
void 	 TemperatureSens_StartConvertTemperature(DS18B20_t *sensor);
void     TemperatureSens_ReadTemperature(DS18B20_t *sensor);
uint32_t TemperatureSens_Sign(DS18B20_t *sensor);
uint32_t TemperatureSens_Temperature(DS18B20_t *sensor);


//*******************************************************************************************
//*******************************************************************************************
void TEMPERATURE_SENSE_Init(void);
void TEMPERATURE_SENSE1_ReadTemperature(void);
void TEMPERATURE_SENSE2_ReadTemperature(void);
DS18B20_t* TEMPERATURE_SENSE_GetSens(uint32_t numSensor);
void TEMPERATURE_SENSE_BuildPack(uint32_t numSensor, uint8_t *buf);

//*******************************************************************************************
//*******************************************************************************************
#endif /* DS18B20_H_ */





