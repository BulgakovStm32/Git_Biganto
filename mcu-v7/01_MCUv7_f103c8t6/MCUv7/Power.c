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
//*******************************************************************************************
//*******************************************************************************************
void POWER_Init(void){

	ADC_Init();
	GPIO_InitForInputPullUp(MCU_PWR_BTN_GPIO, MCU_PWR_BTN_PIN);
	GPIO_InitForInputPullDown(BB_PWR_OK_GPIO, BB_PWR_OK_PIN);

	//Иницияализация ШИМ для управления LAMP. Используется вывод PB1(TIM3_CH4).
	TIM3_InitForPWM();
}
//**********************************************************
uint32_t POWER_GetSupplyVoltage(void){

	return ADC_GetMeas_mV(DC_IN_DIV_ADC_CH) * DIVISION_FACTOR;
}
//**********************************************************
//скользящее среднее.
uint32_t POWER_GetSupplyVoltageSMA(void){

	//скользящее среднее.
	#define WINDOW_SIZE		8	//размер окна усреднения(кратно степени 2)

	static uint32_t window[WINDOW_SIZE] = {0};
	static uint32_t winIndex = 0;
		   uint32_t avrMeas  = 0;
	//-------------------
	window[winIndex] = POWER_GetSupplyVoltage();
	winIndex++;
	winIndex &= (WINDOW_SIZE - 1);

	for(uint32_t i = 0; i < WINDOW_SIZE; i++)
	{
		avrMeas += window[i];
	}
	avrMeas /= WINDOW_SIZE;

	return avrMeas;
}
//**********************************************************
void POWER_CheckSupplyVoltage(void){

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
//**********************************************************
//Задать ширину ШИМ от 0 до 100
//0   - на выходе ностоянно 0В
//100 - на выходе постоянно 3,3В
void POWER_SetPWMwidthForLamp(uint32_t width){

	TIM3_SetPWMwidth(width);
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************














