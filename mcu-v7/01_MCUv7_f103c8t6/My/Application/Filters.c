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
#define AVR_DIV		16

//суммирует несколько измерений, и только после этого выдаёт результат.
//Между расчётами выдаёт предыдущий результат:
uint16_t Filter_Average(uint16_t inVal){

	static uint8_t  count = 0;
	static uint32_t sum   = 0;
	static uint16_t out   = 0;
	//-------------------
	sum += inVal;
	count++;
	if(count >= AVR_DIV)
	{
		out = (sum + AVR_DIV/2) / AVR_DIV;
		sum   = 0;
		count = 0;
	}
	return out;
}
//************************************************************
//************************************************************
//Цифровой фильтр НЧ

#define SPS 		   166U 	//Частота дискретизации АЦП в Гц.
#define Trc 		   0.1f  	//постоянная времени фильтра в Секундах
#define K  	(uint16_t)(SPS*Trc) //

uint16_t Filter_LowPass(uint16_t inVal){

	static int32_t acc = 0;
	static int32_t out = 0;
	//-------------------
	acc += inVal - out;
	out = (acc + K/2) / K;
	return (uint16_t)out;
}
//************************************************************
//************************************************************
#define SMA_WINDOW_SIZE		16 //размер окна усреднения(кратно степени 2)

//Алгоритм скользящего среднего (Simple Moving Average)
uint16_t Filter_SMA(uint16_t inValue){

	static uint16_t window[SMA_WINDOW_SIZE] = {0};
	       uint32_t out = 0;
	//-------------------
	window[SMA_WINDOW_SIZE-1] = inValue;
	//Расчет среднего.
	for(uint16_t j = 0; j < SMA_WINDOW_SIZE; j++)
	{
		out += window[j];
	}
	out = (out + SMA_WINDOW_SIZE/2) / SMA_WINDOW_SIZE;
	//сдвинем масив на один элемент влево.
	for(uint16_t i = 0; i < SMA_WINDOW_SIZE-1; i++)
	{
		window[i] = window[i+1];
	}
	return (uint16_t)out;
}
//************************************************************
//Алгоритм скользящего среднего v2 (Simple Moving Average)
uint16_t Filter_SMAv2(uint16_t inVal){

	static uint16_t window[SMA_WINDOW_SIZE] = {0};
	static uint16_t winIndex = 0;
		   uint32_t avrMeas  = 0;
	//-------------------
	window[winIndex] = inVal;
	winIndex++;						   //инкремент индекса
	winIndex &= (SMA_WINDOW_SIZE - 1); //автоматическое обнуление
	//Расчет среднеарифметического значения.

	for(uint32_t i = 0; i < SMA_WINDOW_SIZE; i++)
	{
		avrMeas += window[i];
	}
	avrMeas = (avrMeas + SMA_WINDOW_SIZE/2) / SMA_WINDOW_SIZE;
	return avrMeas;
}
//************************************************************
//оптимальное бегущее среднее арифметическое
//Возможно не верная реализация!!!
//Неясно, как происходит прасчет среднего и для чео нужен буфер window!!!
uint16_t Filter_SMAv3(uint16_t inVal){

	static uint16_t window[SMA_WINDOW_SIZE] = {0};
	static uint16_t winIndex = 0;
	static  int32_t avrMeas  = 0;
	//-------------------
	winIndex++;					     //инкремент индекса
	winIndex &= (SMA_WINDOW_SIZE-1); //автоматическое обнуление индекса
	avrMeas -= window[winIndex];     //вычитаем старое
	avrMeas += inVal;                //прибавляем новое
	window[winIndex] = inVal;        //запоминаем в массив
	return(uint16_t)((avrMeas + SMA_WINDOW_SIZE/2) / SMA_WINDOW_SIZE);
}
//************************************************************
//Рекурсивный фильтр скользящего среднего
//https://habr.com/ru/post/325590/
uint16_t Filter_SMAv4(uint16_t inVal){

	static uint16_t window[SMA_WINDOW_SIZE] = {0};
	static uint16_t winIndex = 0;
	static  int32_t avrMeas  = 0;
	//-------------------
	avrMeas = avrMeas + (inVal - window[winIndex]);
	window[winIndex] = inVal;
	winIndex++;					     //инкремент индекса
	winIndex &= (SMA_WINDOW_SIZE-1); //автоматическое обнуление индекса
	return(uint16_t)((avrMeas + SMA_WINDOW_SIZE/2) / SMA_WINDOW_SIZE);
}
//************************************************************
//************************************************************
#define EMA_WINDOW_SIZE		64 //размер окна усреднения(кратно степени 2)

//Эспоненциальное скользящее среднее (Exponential Moving Average, EMA).
uint16_t Filter_EMA(uint32_t inVal){

	const uint16_t SCALE = 1024;
	const uint16_t DIV   = 2;
	static int32_t out   = 0;
		   int32_t delta;
	//-------------------
	//filVal += ((inVal - filVal) + WINDOW_SIZE/2) / WINDOW_SIZE;
	//return (uint16_t)filVal;

	inVal *= SCALE;
	out   *= SCALE;
	delta  = inVal - out;
	delta  = (delta + DIV/2) / DIV;
	out   += (delta + EMA_WINDOW_SIZE/2) / EMA_WINDOW_SIZE;
	out    = (out + SCALE/2) / SCALE;
	return (uint16_t)out;
}
//************************************************************
//Эспоненциальное скользящее среднее (Exponential Moving Average, EMA).
uint16_t Filter_EMAv2(uint32_t inVal){

	const  uint16_t SCALE = 1024;
	//static uint32_t inOld = 0;
	static  int32_t out   = 0;
	 	    int32_t delta;
	//-------------------
	//k = 2/(N+1)
	//out = outOld + (inVal - outOld) * k;

	inVal *= SCALE;
	out   *= SCALE;
	delta  = inVal - out;
	out   += ((delta * 1 + EMA_WINDOW_SIZE/2) / EMA_WINDOW_SIZE);
	out    = ((out + SCALE/2) / SCALE);
	return (uint16_t)out;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************











