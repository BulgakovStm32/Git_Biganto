
#include "sys_tick_ST.h"

//-----------------------------------------------------------------------------
//Инициализация системного таймера SysTick.
void SysTick_Init(void){

	SysTick->LOAD = TimerTick;	//Загрузка значения.
	SysTick->VAL  = TimerTick;	//Обнуление таймера. 
                              //При записи любого значения этот регист обнуляется, с обнулением флага COUNTFLAG. 
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | //Тактирование частотой процессора (24МГц).
					SysTick_CTRL_TICKINT_Msk   | //Рразрешение прерывания.
					SysTick_CTRL_ENABLE_Msk;     //Резрешение таймеру считать.
}
//-----------------------------------------------------------------------------
//Прерываени системного таймера SysTick.
//void SysTick_Handler(void){
//	
//	//static	uint32_t	mSec_count = 0;
//	//static	uint16_t	temp = 0;
//	//static	uint16_t	i = 0;
//	//-------------------- 
//	//mSec_count++; // отсчет времени дл¤ ф-ии задержки Delay_mSec().
//	
//	LC1SostGreenLedToggel;
//	LC1SostRedLedToggel;
//  
//  
//	
//// 	StpicCSLow;
//// 	SPI2->DR = i;								//загружаем данные для передачи 
//// 	while (!(SPI2->SR & SPI_SR_TXE));	//ожидание окончания передачи
//// 	StpicCSHight;
//// 	StpicCSLow;
//// 	
//// 	i ^= 0xFFFF;
//	//-------------------- 
//}
//-----------------------------------------------------------------------------
