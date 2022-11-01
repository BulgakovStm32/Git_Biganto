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
float 	 ENCODER_GetAngleQuant(void);
int32_t  ENCODER_GetCode(void);


void 	 ENCODER_BuildPack(uint8_t *buf);
//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_ENCODER_H_ */
