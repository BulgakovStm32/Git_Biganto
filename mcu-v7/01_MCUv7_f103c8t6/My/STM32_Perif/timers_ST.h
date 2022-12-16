/*
 * timers_ST.h
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: Zver
 */

#ifndef TIMERS_ST_H_
#define TIMERS_ST_H_
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************


//*******************************************************************************************
//*******************************************************************************************
void TIM3_InitForPWM(void);
void TIM3_SetPWMwidth(uint32_t width);

void TIM1_Init(uint32_t freq_Hz);
void TIM2_Init(void);

void TIMx_Enable(TIM_TypeDef *tim);
void TIMx_Disable(TIM_TypeDef *tim);
void TIMx_SetARR(TIM_TypeDef *tim, uint32_t val);
//*******************************************************************************************
//*******************************************************************************************
#endif /* TIMERS_ST_H_ */
