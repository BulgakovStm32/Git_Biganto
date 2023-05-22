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
static pwrFlag_t pwrFlags;

//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void POWER_Init(void){

	STM32_GPIO_InitForAnalogInput(DC_IN_DIV_GPIO_PORT, DC_IN_DIV_GPIO_PIN);
	STM32_ADC_Init();

	STM32_GPIO_InitForInputPullUp(MCU_PWR_BTN_GPIO, MCU_PWR_BTN_PIN);

	//Линии Big Board.
	STM32_GPIO_InitForInputPullDown (BB_PWR_OK_GPIO_PORT,  BB_PWR_OK_GPIO_PIN);
	//STM32_GPIO_InitForOutputPushPull(BB_PWR_BTN_GPIO_PORT, BB_PWR_BTN_GPIO_PIN);
	//STM32_GPIO_InitForOutputPushPull(BB_PWR_EN_GPIO_PORT,  BB_PWR_EN_GPIO_PIN);

	//Иницияализация ШИМ для управления LAMP. Используется вывод PB1(TIM3_CH4).
	TIM3_InitForPWM(10000);
}
//**********************************************************
void POWER_TurnOff(void){

	PROTOCOL_I2C_Flags()->f_CmdTurnOff = FLAG_SET;
}
//**********************************************************
uint32_t POWER_GetSupplyVoltage(void){

	return STM32_ADC_GetMeas_mV(DC_IN_DIV_ADC_CH) * DIVISION_FACTOR;
}
//**********************************************************
uint32_t POWER_GetSupplyVoltageSMA(void){

	//return Filter_SMA((uint16_t)POWER_GetSupplyVoltage());
	//return Filter_SMAv2((uint16_t)POWER_GetSupplyVoltage());
	return Filter_SMAv4((uint16_t)POWER_GetSupplyVoltage());
	//return Filter_Average((uint16_t)POWER_GetSupplyVoltage());
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
pwrFlag_t* POWER_Flags(void){

	return &pwrFlags;
}
//**********************************************************
pwrButtonState_t POWER_PwrButton(void){

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

	return (BB_PWR_OK_GPIO_PORT->IDR & (1<<BB_PWR_OK_GPIO_PIN));
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














