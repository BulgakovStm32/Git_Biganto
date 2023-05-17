/*
 * Opticsensors.c
 *
 *  Created on: 10 авг. 2022 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "OpticSensors.h"

//*******************************************************************************************
//*******************************************************************************************
static uint32_t cycleCount = 0;

static uint32_t OptSensState = 0; //Младшие 4 бит - состояние SENS_LENS
								  //Старшие 4 бит - состояние SENS_BAT

//*******************************************************************************************
//*******************************************************************************************
void OPT_SENS_Init(void){

	STM32_GPIO_InitForOutputPushPull(SENS_LED_GPIO_PORT, SENS_LED_GPIO_PIN);   //SENS_LED    - PB6 - выход
	STM32_GPIO_InitForInputPullUp(SENS_LENS_E_GPIO_PORT, SENS_LENS_E_GPIO_PIN);//SENS_LENS_E - PB3 - вход
	STM32_GPIO_InitForInputPullUp(SENS_BAT_E_GPIO_PORT,  SENS_BAT_E_GPIO_PIN); //SENS_BAT_E  - PB4 - вход
	SENS_LED_Low();
}
//**********************************************************
//Проверяем состояние фотодиода. По идее, если есть периодическая засветка фотодиода светодиодом
//(такое может быть только в случае нахождения крышки объектива перед датчиком), то через 32 чикла
// в регистре OptSensState будет значение 0b1010101010101....(0x55555555).
//Т.е. когда значение OptSensState = 0х5555, значит перед датчиком н аходится какая-то поверхность.
void OPT_SENS_CheckLoop(void){

	static uint32_t flag 	   = 0;
//	static uint32_t cycleCount = 0;
	//-------------------
	//Такая конструкция позволяет считываь состояние фотодиода в середине состояния светодиода.
	//Так должно быть более надежным считывание состояния.
	//!!!!!!В этом варианте происходит неконтролируемая смена состояния с 0х5555 на 0хАААА через некоторое время.
	if(!flag) //Генерация меандра для светодиодов датчиков.
	{
		flag = 1;
		SENS_LED_Toggel();
	}
	else //Проверяем состояние фотодиода.
	{
		flag = 0;
		if(++cycleCount > 15)//Считаем цыклы
		{
			cycleCount = 0;
			//SENS_LED_Low();
		}
		//Датчик крышки объектива
		if(SENS_LENS_E_GPIO_PORT->IDR & (1<<SENS_LENS_E_GPIO_PIN)) OptSensState |=  (0x1 << cycleCount);
		else						   							   OptSensState &= ~(0x1 << cycleCount);
		//Датчик наличия АКБ.
		if(SENS_BAT_E_GPIO_PORT->IDR & (1<<SENS_BAT_E_GPIO_PIN)) OptSensState |=  (0x10000 << cycleCount);
		else													 OptSensState &= ~(0x10000 << cycleCount);
	}


//	if(++cycleCount > 15) cycleCount = 0;//Считаем цыклы
//	//Датчик крышки объектива
//	if(SENS_LENS_E_GPIO_PORT->IDR & (1<<SENS_LENS_E_GPIO_PIN)) OptSensState |=  (0x1 << cycleCount);
//	else						   							   OptSensState &= ~(0x1 << cycleCount);
//	//Датчик наличия АКБ.
//	if(SENS_BAT_E_GPIO_PORT->IDR & (1<<SENS_BAT_E_GPIO_PIN)) OptSensState |=  (0x10000 << cycleCount);
//	else													 OptSensState &= ~(0x10000 << cycleCount);
//	SENS_LED_Toggel();
}
//**********************************************************
uint32_t OPT_SENS_GetState(void){

	uint32_t temp = OptSensState;
	OptSensState = 0;
//	cycleCount   = 0;
//	SENS_LED_Low();
	return temp;
}
//**********************************************************

//*******************************************************************************************
//*******************************************************************************************





