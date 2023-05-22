/*
 * iwdg_ST.c
 *
 *  Created on: 9 мар. 2023 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "iwdg_ST.h"

//*******************************************************************************************
//*******************************************************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
// Function    : IWDG_Init()
// Description : Инициализация сторожевого таймера IWDG
// Parameters  : period - время, по прошествии которого сторожевой таймер
//				 перезагрузит микроконтроллер(от 7мс до 26200мс)
// RetVal      :
//*****************************************
void IWDG_Init(uint32_t period){

	/* Enable the peripheral clock RTC */
	RCC->CSR |= RCC_CSR_LSION;
	while((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY)
	{
		/* add time out here for a robust application */
	}

	IWDG->KR = IWDG_WRITE_ACCESS;	// Ключ для доступа к таймеру

	//Bits 2:0 PR[2:0]: Prescaler divider
	//These bits are write access protected seeSection 19.3.2. They are written by software to
	//select the prescaler divider feeding the counter clock. PVU bit of IWDG_SR must be reset in
	//order to be able to change the prescaler divider.
	//000: divider /4
	//001: divider /8
	//010: divider /16
	//011: divider /32
	//100: divider /64
	//101: divider /128
	//110: divider /256
	//111: divider /256
	IWDG->PR  = 7; 								//Для IWDG_PR=7 Tmin=6,4мс RLR=Tмс*40/256
	IWDG->RLR = (period * 40 + 256/2) / 256;	//Загрузить регистр перезагрузки
	IWDG->KR  = IWDG_REFRESH;					//Перезагрузка
	IWDG->KR  = IWDG_START; 					//Пуск таймера
}
//**********************************************************
// Function    : IWDG_Reset()
// Description : Сброс сторожевого таймера IWDG
// Parameters  :
// RetVal      :
//*****************************************
void IWDG_Reset(void){

	IWDG->KR = IWDG_REFRESH;
}
//**********************************************************
// Function    : IWDG_Stop() !!!! Невозможно остановить IWDG после пуска !!!!
// Description : Останов сторожевого таймера IWDG
// Parameters  :
// RetVal      :
//*****************************************
void IWDG_Stop(void){

//	RCC->CSR &= ~RCC_CSR_LSION;
//	DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

//	IWDG->KR  = IWDG_WRITE_ACCESS;	// Ключ для доступа к таймеру
//	IWDG->RLR = 0;
//	IWDG->KR  = IWDG_REFRESH;		// Перезагрузка

//	IWDG->KR = IWDG_WRITE_ACCESS;	// Ключ для доступа к таймеру
	IWDG->KR = 0xFFFFFFFF;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
// Function    : WWDG_Init() !!!! НЕОТЛАЖЕН !!!
// Description : Инициализация оконного сторожевого таймера WWDG
// Parameters  : period - время, по прошествии которого сторожевой таймер
//				 перезагрузит микроконтроллер(от 7мс до 26200мс)
// RetVal      :
//*****************************************
void WWDG_Init(uint32_t period){

	/* Enable the peripheral clock WWDG */
	RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

	/* Configure WWDG */
	/* (1) set prescaler to have a rollover each about 5.5ms, set window value (about 2.25ms) */
	/* (2) Refresh WWDG before activate it */
	/* (3) Activate WWDG */

	//Bits 8:7 WDGTB[1:0]: Timer base
	//The time base of the prescaler can be modified as follows:
	//00: CK Counter Clock (PCLK1 div 4096) div 1
	//01: CK Counter Clock (PCLK1 div 4096) div 2
	//10: CK Counter Clock (PCLK1 div 4096) div 4
	//11: CK Counter Clock (PCLK1 div 4096) div 8
	WWDG->CFR |= (3 << WWDG_CFR_WDGTB_Pos);			// Задать частоту тактирования WWDG
	WWDG->CFR |= (0x41 << WWDG_CFR_W_Pos);			// Задать окно

	WWDG->CR |= WWDG_REFRESH; 						// Разрешить работу WWDG и установить начальное	значение
	WWDG->CR |= WWDG_CR_WDGA; 						/* (3) */
}
//**********************************************************
// Function    : WWDG_Reset()
// Description : Сброс оконного сторожевого таймера WWDG
// Parameters  :
// RetVal      :
//*****************************************
void WWDG_Reset(void){

	WWDG->CR |= WWDG_REFRESH; /* Refresh WWDG */
}
//**********************************************************
// Function    : WWDG_Stop()
// Description : Останов оконного сторожевого таймера WWDG
// Parameters  :
// RetVal      :
//*****************************************
void WWDG_Stop(void){

	RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN; // Отключить тактирование WWDG
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************



















