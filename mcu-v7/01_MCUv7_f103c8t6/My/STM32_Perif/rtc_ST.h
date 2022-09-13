
#ifndef _rtc_ST_H
#define _rtc_ST_H
//-----------------------------------------------------------------------------

//#include "stm32f10x.h"
//#include "stm32f0xx.h"
//#include "gpio_ST.h"
#include "main.h"

//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
void      RTC_Init(void);
uint32_t  RTC_GetCounter(void);
void      RTC_SetCounter(uint32_t value);

void RTC_IT_Handler(void);
//-----------------------------------------------------------------------------
#endif 


