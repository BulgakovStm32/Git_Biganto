/*
 * Encoder.h
 *
 *  Created on:
 *      Author:
 */

#ifndef _Encoder_H
#define _Encoder_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define ENCODER_NO_TURN     	0
#define ENCODER_TURN        	1
#define ENCODER_TURN_RIGHT  	2
#define ENCODER_TURN_LEFT		3

#define ENCODER_BUTTON_TIMEOUT	10
//**********************************
//Состояние кнопки энкодера.


//**********************************
typedef struct{
	GPIO_TypeDef *GPIO_PORT_A;
	uint32_t	  GPIO_PIN_A;

	GPIO_TypeDef *GPIO_PORT_B;
	uint32_t	  GPIO_PIN_B;

	GPIO_TypeDef *GPIO_PORT_BUTTON;
	uint32_t	  GPIO_PIN_BUTTON;
	//------------
	uint8_t		  ENCODER_STATE;
	uint8_t		  BUTTON_STATE;
	uint32_t	  ENCODER_NUMBER;
}Encoder_t;
//*******************************************************************************************
//*******************************************************************************************
void Encoder_Init(Encoder_t *encoder);
void Encoder_ScanLoop(Encoder_t *encoder);
void Encoder_IncDecParam(Encoder_t *encoder, uint16_t *parameter, uint32_t step, uint16_t min, uint16_t max);
//*******************************************************************************************
//*******************************************************************************************
#endif //_Encoder_H


