/*
 *
 *
 *  Created on:
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "Blink.h"

//*******************************************************************************************
//*******************************************************************************************

static volatile uint32_t blinkReg   = 0;
static volatile uint32_t blinkCount = 0;

//*******************************************************************************************
//*******************************************************************************************
void Blink_ResetCount(void){

	blinkReg   = 0;
	blinkCount = 0;
}
//***********************************************************
//Процесс для синхронного мигания светодиодами.
void Blink_Loop(void){
	
	++blinkCount;
	if((blinkCount % 50)   == 0) blinkReg ^= (1 << INTERVAL_50_mS);
	if((blinkCount % 100)  == 0) blinkReg ^= (1 << INTERVAL_100_mS);
	if((blinkCount % 250)  == 0) blinkReg ^= (1 << INTERVAL_250_mS);
	if((blinkCount % 500)  == 0) blinkReg ^= (1 << INTERVAL_500_mS);
	if((blinkCount % 1000) == 0)
	{
		blinkReg  ^= (1 << INTERVAL_1000_mS);
		blinkCount = 0;
	}
}
//***********************************************************
uint32_t Blink(BlinkIntervalEnum_t interval){
	
	if(blinkReg & (1 << interval)) return 1;
	return 0;
}
//***********************************************************
//Пример использования.
//if(Led_Blink(RTOS_GetTickCount(), 500, 10)) PWR_BTN_LED_Low();
//else 										  PWR_BTN_LED_High();

uint32_t Blink_WithVariablePeriod(uint32_t millisCount, uint32_t period, uint32_t switch_on_time){

	static uint32_t millisOld = 0;
	static uint32_t flag      = 0;
	//-------------------
	if((millisCount - millisOld) >= (flag ? (period - switch_on_time) : switch_on_time))
	{
		millisOld = millisCount;
		flag = !flag;
	}
	return flag;
}
//************************************************************

//*******************************************************************************************
//*******************************************************************************************


