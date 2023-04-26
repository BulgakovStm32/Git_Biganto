/*
 * Power.h
 *
 *  Created on: 30 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef MCUV7_POWER_H
#define MCUV7_POWER_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
// MCU_PWR_PBN - PB7 - вход. Кнопка включения питания.
#define MCU_PWR_BTN_GPIO		GPIOB
#define MCU_PWR_BTN_PIN			7
// BB_PWR_OK - PA12 - вход. Линия питания Big Board.
#define BB_PWR_OK_GPIO			GPIOA
#define BB_PWR_OK_PIN			12
// DC_IN_DIV - PB0 - аналоговый вход.
#define DC_IN_DIV_GPIO_PORT		GPIOB
#define DC_IN_DIV_GPIO_PIN		0

#define DC_IN_DIV_ADC_CH		8		//Канал АЦП
#define DIVIDER_HIGH_RESISTER	10000	//Сопротивление верхнего резистора делителя, в Омах
#define DIVIDER_LOW_RESISTER	1000	//Сопротивление нижнего резистора делителя, в Омах
#define DIVISION_FACTOR		    (((DIVIDER_HIGH_RESISTER + DIVIDER_LOW_RESISTER/2) / DIVIDER_LOW_RESISTER) + 1)//коэффициент деления делителя

#define BATTERY_VOLTAGE_MIN		10800	//Минимально напряжение питания, в мВ
#define BATTERY_VOLTAGE_WARNING	12000	//Напряжение при котором нужно обратить внимание на заряд АКБ, в мВ.
//**********************************
typedef enum{
	RELEASE = 0, //отпускание кнопки
	PRESS,       //нажатие на кнопку
//	CLICK,		 //клик по кнопке
//	LONG_PRESS,	 //удержание дольше timeout - длительное нажатие
//	HOLD,		 //удержание кнопки
}pwrButtonState_t;
//**********************************
//Флаги питания
typedef struct{
	uint32_t 	f_PowerOff   	 : 1; //флаг отключения питания. выставляется после нажатия кнопки питания.
	uint32_t 	f_BatteryWarning : 1; //Напряжение АКБ ниже 12В
	uint32_t 	f_BatteryLow 	 : 1; //Напряжение АКБ ниже минимально допустимого (10,8В)
}pwrFlag_t;
//*******************************************************************************************
//*******************************************************************************************
void	 	     POWER_Init(void);
void 			 POWER_TurnOff(void);

uint32_t 	     POWER_GetSupplyVoltage(void);
uint32_t 		 POWER_GetSupplyVoltageSMA(void);
void 			 POWER_CheckSupplyVoltage(void);
pwrFlag_t*     	 POWER_Flags(void);
pwrButtonState_t POWER_PwrButton(void);
void 			 POWER_PwrButtonLed(uint32_t sate);
uint32_t 		 POWER_GetBigBoardPwr(void);

void POWER_SetPWMwidthForLamp(uint32_t width);
//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_POWER_H_ */
