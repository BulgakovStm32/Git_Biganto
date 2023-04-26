/*
 * GpioMCU.c
 *
 *  Created on: 11 янв. 2023 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "GpioMCU.h"

//*******************************************************************************************
//*******************************************************************************************

//**********************************************************

//*******************************************************************************************
//*******************************************************************************************
void GPIO_MCU_Init(void){

	// MCU_EN - PB2
	STM32_GPIO_InitForOutputPushPull(MCU_EN_GPIO_PORT, MCU_EN_GPIO_PIN);
	// LED_ACT - PA15
	STM32_GPIO_InitForOutputPushPull(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
	// GPS_EN - PB13
	STM32_GPIO_InitForOutputPushPull(GPS_EN_GPIO_PORT, GPS_EN_GPIO_PIN);
	// LIDAR_EN - PB5
	STM32_GPIO_InitForOutputPushPull(LIDAR_EN_GPIO_PORT, LIDAR_EN_GPIO_PIN);
	// BB_PWR_BTN - PA8
	STM32_GPIO_InitForOutputPushPull(BB_PWR_BTN_GPIO_PORT, BB_PWR_BTN_GPIO_PIN);
	// FAN_EN - PB12
	STM32_GPIO_InitForOutputPushPull(FAN_EN_GPIO_PORT, FAN_EN_GPIO_PIN);
	// PWR_BTN_LED - PB9
	STM32_GPIO_InitForOutputPushPull(PWR_BTN_LED_GPIO_PORT, PWR_BTN_LED_GPIO_PIN);
	// LAMP_PWM - PB1
	STM32_GPIO_InitForOutputPushPull(LAMP_PWM_GPIO_PORT, LAMP_PWM_GPIO_PIN);
}
//**********************************************************


//**********************************************************


//*******************************************************************************************
//*******************************************************************************************
