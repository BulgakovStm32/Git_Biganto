/*
 * Motor.c
 *
 *  Created on: 5 авг. 2022 г.
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************

#include "Motor.h"

//*******************************************************************************************
//*******************************************************************************************
//переменные для работы протокола
static uint32_t targetPosition    = 0;  //целевое положение
static uint32_t velocityMax       = 0;	//максимальная скорость
static uint32_t accelerationMax   = 0;	//максимальное ускорение
static uint32_t accelerationStart = 0;	//не янсно что это !!!!!!


//Рабочие переменные
static uint32_t     microSteps 	   = 0; //количество микрошагов.
static MotorState_t motorState	   = MOTOR_NO_INIT;

static uint32_t quantsNum   = 0; //Количество квантов времени за время разгона или торможения
static uint32_t quantsCount = 0;
static uint32_t arrV0       = 0; //

static uint32_t ppsVstart	= 0;
static uint32_t ppsVmax	    = 0;
static uint32_t pps_step    = 0;
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){
	DRV_DIR_High();
	MOTOR_SetMicrostep(STEP_32);
	TIM2_Init();
	DRV_EN_Low(); 	 //включение драйвера.
	DRV_RESET_High();//

	motorState = MOTOR_READY;
}
//**********************************************************
//Задать микрошаг
void MOTOR_SetMicrostep(MotorStepMode_t steps){

	switch(steps){
	//-------------------
	case(STEP_1):
		DRV_MODE2_Low();
		DRV_MODE1_Low();
		DRV_MODE0_Low();
		microSteps = STEP_1;
	break;
	//-------------------
	case(STEP_2):
		DRV_MODE2_Low();
		DRV_MODE1_Low();
		DRV_MODE0_High();
		microSteps = STEP_2;
	break;
	//-------------------
	case(STEP_4):
		DRV_MODE2_Low();
		DRV_MODE1_High();
		DRV_MODE0_Low();
		microSteps = STEP_4;
	break;
	//-------------------
	case(STEP_8):
		DRV_MODE2_Low();
		DRV_MODE1_High();
		DRV_MODE0_High();
		microSteps = STEP_8;
	break;
	//-------------------
	case(STEP_16):
		DRV_MODE2_High();
		DRV_MODE1_Low();
		DRV_MODE0_Low();
		microSteps = STEP_16;
	break;
	//-------------------
	case(STEP_32):
		DRV_MODE2_High();
		DRV_MODE1_Low();
		DRV_MODE0_High();
		microSteps = STEP_32;
	break;
	//-------------------
	default:
		microSteps = STEP_ERR;
	break;
	//-------------------
	}
}
//**********************************************************
uint32_t MOTOR_GetMicrostep(void){

	return microSteps;
}
//**********************************************************
//Получить текущий угол
uint32_t MOTOR_GetCurrentPosition(void){

	return 0;
}
//**********************************************************
//Получить текущее ускорение
uint32_t MOTOR_GetCurrentAcceleration(void){

	return 0;
}
//**********************************************************
//Получить текущую скорость
uint32_t MOTOR_GetCurrentVelocity(void){

	return 0;
}
//**********************************************************
//Задать целевое положение
void MOTOR_SetTargetPosition(uint32_t position){

}
//**********************************************************
//Задать максимальное ускорение
void MOTOR_SetMaxAcceleration(uint32_t acceleration){

}
//**********************************************************
//Задать максимальную скорость
void MOTOR_SetMaxVelocity(uint32_t vel_rpm){

	uint32_t pps = (vel_rpm * MOTOR_STEPS_PER_TURN * microSteps) / 60; //PPS - число микрошагов в секунду
	if(pps > 0xFFFF) pps =  0xFFFF;
	TIM2->ARR   = (uint16_t)(MOTOR_FREQ_TIM_Hz / pps);
	TIM2->CCR1  = (uint16_t)(TIM2->ARR / 2);
	TIM2->CR1  |= TIM_CR1_CEN;
}
//**********************************************************
//Сбросить чип
void MOTOR_ResetDrv(void){

}
//**********************************************************
//Сбросить позицию
void MOTOR_ResetPosition(void){

}
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_CalculateAccelDecel(uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t timeAccel_ms){

	//Расчет частоты тактирования для заданной в RPM скорости.
	ppsVstart = (Vstart_rpm * MOTOR_STEPS_PER_TURN * microSteps) / 60;
	ppsVmax   = (Vmax_rpm   * MOTOR_STEPS_PER_TURN * microSteps) / 60;
	//Находим количество квантов времени за время разгона.
	quantsNum = timeAccel_ms / MOTOR_QUANT_TIME;
	//Находим приращение pps которое нужно сделать на каждом кванте времени разгона.
	pps_step  = (ppsVmax - ppsVstart) / quantsNum;
	//Уточнение количества квантов
	quantsNum = (ppsVmax - ppsVstart) / pps_step;
	//Начальное значение загружаемое в регистр ARR для скорости Vstart
	if(Vstart_rpm == 0) arrV0 = 0;
	else 				arrV0 = MOTOR_FREQ_TIM_Hz / ppsVstart;
	//Запуск таймера
//	TIM2->ARR   = (uint16_t)arrV0;
//	TIM2->CCR1  = (uint16_t)(TIM2->ARR / 2);
//	TIM2->CR1  |= TIM_CR1_CEN;



//	arrV0     = 100000U / ppsV0;
//	arrVmax   = 100000U / ppsVmax;
//	quantsNum = (tAccel * 1000) / MOTOR_QUANT_TIME;
//	arr_step  = (arrV0 - arrVmax) / quantsNum;
//	//Если arr_step=0 значит за заданное время tAccel не получится дойти от V0 до Vmax
//	//Раньше наступит время торможения.
//	if(arr_step == 0) return ;
//	quantsNum = (arrV0 - arrVmax) / arr_step;//точный расчет нужного количества квантов.

//	TIM2->ARR   = (uint16_t)arrV0;
//	TIM2->CCR1  = (uint16_t)(TIM2->ARR - 20);
//	TIM2->CR1  |= TIM_CR1_CEN;

//	motorState = MOTOR_CALC_OK;
}
//**********************************************************
void MOTOR_SpinStart(MotorState_t spinMode){

	motorState  = spinMode;
	//DRV_RESET_High();
	DRV_EN_Low();			  //включение драйвера.
	TIM2->CR1 |= TIM_CR1_CEN; //включение таймера
}
//**********************************************************
void MOTOR_AccelDecelLoop(void){

	static uint32_t msCount = 0;
		   uint32_t tempARR = 0;
	//-------------------
	//Если расчеты не готовы то ничего не делаем и выходим.
	if(motorState == MOTOR_READY)return;

	if(++msCount >= MOTOR_QUANT_TIME)
	{
		msCount = 0;
		//Считаем кванты времени и расчитываем скорость.
		if(quantsCount < quantsNum)
		{
			quantsCount++;

			switch(motorState){
				//----------
				//положительное приращение скорости(ускорение) на данном кванте времени
				case(MOTOR_ACCEL):
					ppsVstart = ppsVstart + pps_step;
					tempARR   = MOTOR_FREQ_TIM_Hz / ppsVstart;	//расчет значения для таймера
				break;
				//----------
				//отрицательное приращение скорости(замедление) на данном кванте времени
				case(MOTOR_DECEL):
					ppsVstart = ppsVstart - pps_step;
					if(ppsVstart == 0) tempARR = 0;
					else               tempARR = MOTOR_FREQ_TIM_Hz / ppsVstart;	//расчет значения для таймера
				break;
				//----------
				default:
					tempARR = 0;
					motorState = MOTOR_READY;
				break;
				//----------
			}
			//Обновляем значение таймера.
			if(tempARR != 0)
			{
				TIM2->ARR  = (uint16_t)tempARR;
				TIM2->CCR1 = (uint16_t)(TIM2->ARR / 2);
			}
			else
			{
				TIM2->ARR  = 1;//The counter is blocked while the auto-reload value is null.
				TIM2->CCR1 = 0;
				//DRV_RESET_Low();
				DRV_EN_High();
			}
		}
		else
		{
			motorState  = MOTOR_READY;
			quantsCount = 0;
		}
	}

}
//*******************************************************************************************
//*******************************************************************************************











