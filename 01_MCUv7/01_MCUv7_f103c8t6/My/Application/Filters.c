/*
 * Filters.c
 *
 *  Created on: 25 янв. 2021 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "Filters.h"

//*******************************************************************************************
//*******************************************************************************************
uint16_t Average(uint16_t inValue, uint8_t div){

	static uint8_t  avrCount    = 0;
	static uint32_t sum         = 0;
	static uint16_t avrOldValue = 0;
	//-------------------
	if(avrCount < div)
		{
			avrCount++;
			sum += inValue;
		}
	else
		{
			avrCount    = 0;
			avrOldValue = sum / div;
			sum         = 0;
			return avrOldValue;
		}
	return avrOldValue;
}
//************************************************************
//Цифровой фильтр НЧ

#define SPS 200UL //Частота дискретизации АЦП в Гц.
#define Trc 0.1f //постоянная времени фильтра в Секундах
#define K  (SPS*Trc)

uint16_t Filter(uint16_t inValue){

	static uint32_t Dacc = 0;
	static uint16_t Dout  = 0;
	//-------------------
	Dacc = Dacc + inValue - Dout;
	Dout = Dacc >> 4; // /(uint16_t)K;
	return Dout;
}
//************************************************************
#define FILTER_SMA_WINDOW_SIZE	64
//Алгоритм скользящего среднего (Simple Moving Average)
uint16_t Filter_SMA(uint16_t inValue){

	static uint16_t filterBuf[FILTER_SMA_WINDOW_SIZE] = {0};
	       uint32_t outVal = 0;
	//-------------------
	filterBuf[FILTER_SMA_WINDOW_SIZE - 1] = inValue;
	//Расчет среднего.
	for(uint16_t j = 0; j < FILTER_SMA_WINDOW_SIZE; j++)
		{
			outVal += filterBuf[j];
		}
	outVal /= FILTER_SMA_WINDOW_SIZE;
	//сдвинем масив на один элемент влево.
	for(uint16_t i = 0; i < FILTER_SMA_WINDOW_SIZE; i++)
		{
			filterBuf[i] = filterBuf[i+1];
		}
	return (uint16_t)outVal;
}
//************************************************************
#define NUM_READ 16
//оптимальное бегущее среднее арифметическое
uint16_t runMiddleArifmOptim(uint16_t inValue){

  static uint16_t t = 0;
  static uint16_t arr[NUM_READ] = {0};
  static uint32_t average = 0;
  //-------------------
  if(++t >= NUM_READ) t = 0;// перемотка t
  average -= arr[t];        // вычитаем старое
  average += inValue;       // прибавляем новое
  arr[t]   = inValue;       // запоминаем в массив
  return (uint16_t)(average / NUM_READ);
}
//************************************************************
//Эспоненциальное скользящее среднее (Exponential Moving Average, EMA).
uint16_t Filter_EMA(uint16_t inValue){

  static uint16_t filVal = 0;
  //-------------------
  filVal += ((inValue - filVal) >> 3);
  return (uint16_t)filVal;
}
//************************************************************



//*******************************************************************************************
//*******************************************************************************************

