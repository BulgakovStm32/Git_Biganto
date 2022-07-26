/*
 * Filters.h
 *
 *  Created on: 25 янв. 2021 г.
 *      Author: belyaev
 */

#ifndef FILTERS_H_
#define FILTERS_H_
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************



//*******************************************************************************************
//*******************************************************************************************
uint16_t Average(uint16_t inValue, uint8_t div);//усреднение по div выборкам
uint16_t Filter_LowPass(uint16_t inValue);	    //Цифровой фильтр НЧ
uint16_t Filter_SMA(uint16_t inValue);		    //Алгоритм скользящего среднего (Simple Moving Average)
uint16_t Filter_EMA(uint16_t inValue);			//Эспоненциальное скользящее среднее (Exponential Moving Average, EMA).

//*******************************************************************************************
//*******************************************************************************************
#endif /* FILTERS_H_ */




