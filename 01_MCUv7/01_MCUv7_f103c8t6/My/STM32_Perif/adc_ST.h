
#ifndef _adc_ST_H
#define _adc_ST_H
//-----------------------------------------------------------------------------

//#include "stm32f10x.h"
#include "main.h"

//-----------------------------------------------------------------------------
#define ADC_REF			  3270UL //Опорное напряжение
#define ADC_RESOLUTION	  4096UL
//#define ADC_QUANT		  ((ADC_REF * 10000) / ADC_RESOLUTION)

#define AdcMeasQuantity   16     //Количество измерений для усреднения
#define AdcMeasShiftDev   4      //Количество сдвигов для деления
//--------------------
//Каналы АЦП.
#define Ia_ADC_CH   0
#define Ib_ADC_CH   1
#define Ic_ADC_CH   2
//-----------------------------------------------------------------------------
//Аппатартный уровень.
void     Adc_Init(void);
uint32_t Adc_GetMeas(uint8_t channel);
uint16_t Adc_GetRegDR(ADC_TypeDef *adc);

uint16_t Adc_GetRegJDRx(ADC_TypeDef *adc, uint8_t ch);
//-----------------------------------------------------------------------------
#endif /*_adc_ST_HH*/
