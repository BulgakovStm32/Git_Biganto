/*
 * Encoder.h
 *
 *  Created on: 5 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef MCUV7_ENCODER_H_
#define MCUV7_ENCODER_H_
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define ENCODER_SPI						SPI1
#define ENCODER_SPI_READ_BIT_NUM		(3 * 8) //Выиитывается 24 бита

#define ENCODER_TYPE_BIN				0
#define ENCODER_TYPE_GRAY				1

#define ENCODER_RESOLUTION_BIT_MIN		1
#define ENCODER_RESOLUTION_BIT_MAX		23
//-----------------------------------
#define ENCODER_DEFAULT_RESOLUTION_BIT	16
#define ENCODER_DEFAULT_TYPE			ENCODER_TYPE_BIN

//-----------------------------------
typedef enum {
	ENCODER_NO_INIT = 0,
	ENCODER_READY,
}EncoderState_t;
//*******************************************************************************************
//*******************************************************************************************
void 	 ENCODER_Init(void);
void     ENCODER_SetConfig(uint16_t config);
uint16_t ENCODER_GetConfig(void);
uint32_t ENCODER_GetAngleQuant(void);
uint32_t ENCODER_GetOffset(void);
int32_t  ENCODER_GetCode(void);
uint32_t ENCODER_GetAngle(void);

void 	 ENCODER_BuildPack(uint8_t *buf);
//*******************************************************************************************
//*******************************************************************************************
#define _RPM					60
#define QUANT_FOR_100mS     	((_RPM * (1000/100) * 1000) / 360.0)
#define QUANT_FOR_10mS      	((_RPM * (1000/10)  * 1000) / 360.0)
#define QUANT_FOR_64mS      	((_RPM * (1000/64)  * 1000) / 360.0)
//********************************************************
typedef struct{
	int32_t  code;
	uint32_t offset;
	float 	 angle;
	float 	 RPM;
}EncoderDebug_t;
//********************************************************
//Ф-ии для отладки работы энкодера
EncoderDebug_t* ENCODER_DBG_Data(void);
void 			ENCODER_DBG_CalcAngleAndSpeed(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_ENCODER_H_ */
