/*
 * OpticSensors.h
 *
 *  Created on: 10 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef MCUV7_OPTICSENSORS_H
#define MCUV7_OPTICSENSORS_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
//SENS_LED    - PB6 - выход
#define	SENS_LED_GPIO		GPIOB
#define SENS_LED_PIN		GPIO_ODR_ODR6
//SENS_LENS_E - PB3 - вход
#define	SENS_LENS_E_GPIO	GPIOB
#define SENS_LENS_E_PIN		GPIO_IDR_IDR3
//SENS_BAT_E  - PB4 - вход
#define	SENS_BAT_E_GPIO		GPIOB
#define SENS_BAT_E_PIN		GPIO_IDR_IDR4
//*******************************************************************************************
//*******************************************************************************************
void 	 OPT_SENS_Init(void);
void 	 OPT_SENS_CheckLoop(void);
uint32_t OPT_SENS_GetState(uint32_t sens);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_OPTICSENSORS_H_ */


