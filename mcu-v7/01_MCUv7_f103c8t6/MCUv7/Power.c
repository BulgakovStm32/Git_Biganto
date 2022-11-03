/*
 * Power.c
 *
 *  Created on: 30 авг. 2022 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "Power.h"

//*******************************************************************************************
//*******************************************************************************************

static PowerFlag_t PowerFlags;


//*******************************************************************************************
//*******************************************************************************************
void POWER_Init(void){

	ADC_Init();
	GPIO_InitForInputPullUp(MCU_PWR_BTN_GPIO, MCU_PWR_BTN_PIN);
	GPIO_InitForInputPullDown(BB_PWR_OK_GPIO, BB_PWR_OK_PIN);
}
//**********************************************************
uint32_t POWER_GetSupplyVoltage(void){

//	uint32_t avrMeas = 0;
//	//-------------------
//	for(uint32_t i = 0; i < 8; i++)
//	{
//		avrMeas += ADC_GetMeas(ADC_DC_IN_DIV_CH);
//		DELAY_microS(500);
//	}
//	avrMeas = avrMeas/8;
//	return (avrMeas * DIVISION_FACTOR);

	return ADC_GetMeas(DC_IN_DIV_ADC_CH) * DIVISION_FACTOR;
}
//**********************************************************
void POWER_SupplyVoltageCheck(void){

	uint32_t supplyVoltage = POWER_GetSupplyVoltage();
	//-------------------
	//Напряжение НИЖЕ миннимального (10,8В) напряжения АКБ.
	if(supplyVoltage <= BATTERY_VOLTAGE_MIN) POWER_Flags()->f_BatteryLow = FLAG_SET;
	//Напряжение АКБ НИЖЕ 12В
	else if(supplyVoltage <= BATTERY_VOLTAGE_WARNING) POWER_Flags()->f_BatteryWarning = FLAG_SET;
	//Напряжение АКБ ВЫШЕ 12в
	else POWER_Flags()->f_BatteryWarning = FLAG_CLEAR;
}
//**********************************************************
PowerFlag_t* POWER_Flags(void){

	return &PowerFlags;
}
//**********************************************************
PwrButtonState_t POWER_PwrButton(void){

	if(MCU_PWR_BTN_GPIO->IDR & (1<<MCU_PWR_BTN_PIN)) return RELEASE;
	else											 return PRESS;
}
//************************************************************
void POWER_PwrButtonLed(uint32_t sate){

	if(sate) PWR_BTN_LED_High();
	else 	 PWR_BTN_LED_Low();
}
//**********************************************************
uint32_t POWER_GetBigBoardPwr(void){

	return (BB_PWR_OK_GPIO->IDR & (1<<BB_PWR_OK_PIN));
}
//*******************************************************************************************
//*******************************************************************************************
