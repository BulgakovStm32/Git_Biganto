
#ifndef _Blink_H
#define _Blink_H
//-----------------------------------------------------------------------------

//#include "stm32f10x.h"
//#include "stm32f0xx.h"
//#include "stm32f103xb.h"
#include "main.h"

//-----------------------------------------------------------------------------
typedef enum{
	INTERVAL_50_mS = 0,
	INTERVAL_100_mS,
	INTERVAL_250_mS,
	INTERVAL_500_mS,
	INTERVAL_1000_mS,
}BlinkIntervalEnum_t;
//-----------------------------------------------------------------------------
void     Blink_Loop(void);
uint16_t Blink(BlinkIntervalEnum_t interval);
//-----------------------------------------------------------------------------
#endif /*_Blink_H*/


