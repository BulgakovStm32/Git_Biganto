
#ifndef _LM6063D_H
#define _LM6063D_H

//-----------------------------------------------------------------------------

#include "main.h"

//-----------------------------------------------------------------------------
//Сигнал A1.
// A1=H - transferring the Display data.
// A1=L - transferring the Control data.
#define Lm6063LcdA1Hight   (GPIOB->BSRR = GPIO_BSRR_BS10)
#define Lm6063LcdA1Low     (GPIOB->BSRR = GPIO_BSRR_BR10)
//Сигнал /RES.
// /RES=L - initialization is executed.
// /RES=H - normal running.
#define Lm6063LcdResHight   (GPIOB->BSRR = GPIO_BSRR_BS12)
#define Lm6063LcdResLow    	(GPIOB->BSRR = GPIO_BSRR_BR12)
//Сигнал /CS.
// /CS=L - Enable access to LCD module.
// /CS=H - Disable access to LCD module.
#define Lm6063LcdCsHight  	 (GPIOB->BSRR = GPIO_BSRR_BS11)
#define Lm6063LcdCsLow   	 (GPIOB->BSRR = GPIO_BSRR_BR11)
//Сигнал LED.
//#define Lm6063LcdLedOn     (GPIOB->BSRR = GPIO_BSRR_BS3) 
//#define Lm6063LcdLedOff    (GPIOB->BSRR = GPIO_BSRR_BR3)
//#define Lm6063LcdLedToggel (GPIOB->ODR  = GPIOB->ODR ^ GPIO_ODR_ODR3)

#define LM6063_CMD     0
#define LM6063_DATA    1

#define LM6063_X_RES   		128	 //разрешение экрана
#define LM6063_Y_RES 	 	64
#define LM6063_CACHE_SIZE	((LM6063_X_RES * LM6063_Y_RES) / 8)//
//-----------------------------------------------------------------------------
void Lm6063LcdInit  (void);
void Lm6063LcdUpdate(uint8_t *displayBuf);
//-----------------------------------------------------------------------------
#endif //_LM6063D_H



