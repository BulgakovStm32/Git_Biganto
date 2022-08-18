/*
 * Buttons.h
 *
 *  Created on: 17 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef BUTTONS_H
#define BUTTONS_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
typedef enum{
	BUTTON_RELEASE = 0, //отпускание кнопки
	BUTTON_PRESS,		//нажатие на кнопку
	BUTTON_CLICK,		//клик по кнопке
	BUTTON_LONG_PRESS,	//удержание дольше timeout - длительное нажатие
	BUTTON_HOLD,		//удержание кнопки
}ButtonState_t;

//**********************************
typedef struct{
	GPIO_TypeDef 	*GPIO_PORT;
	uint32_t	 	 GPIO_PIN;
	ButtonState_t	 State;
}Button_t;

//**********************************
#define MCU_PWR_BUTTON_GPIO	GPIOB
#define MCU_PWR_BUTTON_PIN	3

//*******************************************************************************************
//*******************************************************************************************
void 		  BUTTON_Init(Button_t *btn);
ButtonState_t BUTTON_GetState(Button_t *btn);
void 		  BUTTON_CheckLoop(Button_t *btn);
//*******************************************************************************************
//*******************************************************************************************
#endif /* MY_APPLICATION_BUTTONS_H_ */
