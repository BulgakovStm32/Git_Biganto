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
//static uint32_t targetPosition    = 0;  //целевое положение
//static uint32_t velocityMax       = 0;	//максимальная скорость
//static uint32_t accelerationMax   = 0;	//максимальное ускорение
//static uint32_t accelerationStart = 0;	//не янсно что это !!!!!!

//Рабочие переменные
static uint32_t     microSteps 	   = 0; //количество микрошагов.
static MotorState_t motorState	   = MOTOR_NO_INIT;

static uint32_t quantsNum   = 0; //Количество квантов времени за время разгона или торможения
static uint32_t quantsCount = 0;

static int32_t  spsVstart	= 0;
static uint32_t spsVmax	    = 0;
static uint32_t sps_step    = 0;
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){
	DRV_DIR_High();
	MOTOR_SetMicrostep(STEP_32);

	TIM2_Init(); //Таймер генерации импульсов STEP
//	TIM1_Init(); //Таймер отсчета временных интервалов расчета скорости.

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

	uint32_t pps = (vel_rpm * MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60; //PPS - число микрошагов в секунду
																				 //+30 для правильного округления при целочисленном делении.
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
static uint32_t _motor_CalcARR(uint32_t sps){

	return (MOTOR_FREQ_TIM_Hz + sps/2) / sps;// делениа на 2 для правильного округления при целочисленном делении
}
//**********************************************************
///*
// * расчет время в мс на выход до стационарного режима
// */
//uint32_t MOTOR_CalcAccelMaxToAccelTime(uint32_t Vmax_rpm, uint32_t Accel){
//
//	//время в мс на выход до стационарного режима
//	 return (uint32_t)((float)(Vmax_rpm / Accel) * 1000);
//}
//**********************************************************
void MOTOR_CalcAccelDecel(uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t timeAccel_ms){

	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60;//+30 для правильного округления при целочисленном делении.
	//-------------------
	//Расчет шагов в секунду (sps) для заданной в RPM скорости.
	spsVstart = Vstart_rpm * rpmToSpsCoefficient;
	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;
	//Находим количество квантов времени за время разгона.
	quantsNum = timeAccel_ms / MOTOR_QUANT_TIME_mS;
	//Находим приращение pps которое нужно сделать на каждом кванте времени разгона/торможения.
	sps_step  = (spsVmax - spsVstart) / quantsNum;
	//Уточнение количества квантов
	quantsNum = (spsVmax - spsVstart) / sps_step;

//	//Начальное значение загружаемое в регистр ARR для скорости Vstart
//	if(Vstart_rpm == 0) arrV0 = 0;
//	else 				arrV0 = _motor_CalcARR(spsVstart);
//	//Запуск таймера
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
	if(motorState == MOTOR_READY)return;//Если расчеты не готовы то ничего не делаем и выходим.
	if(++msCount >= MOTOR_QUANT_TIME_mS)//Отсчитываем нужный квант времени.
	{
		//LED_ACT_Toggel();
		msCount = 0;
		//Считаем кванты времени и расчитываем скорость.
		if(quantsCount < quantsNum)
		{
			quantsCount++;

			switch(motorState){
				//----------
				//положительное приращение скорости(ускорение) на данном кванте времени
				case(MOTOR_ACCEL):
					spsVstart = spsVstart + sps_step; //положительное приращение скорости(ускорение)
					//if(quantsCount == quantsNum) spsVstart = spsVmax;//Загрузка spsVmax на последнем кванте.
					if(spsVstart > spsVmax) spsVstart = spsVmax;//проверка на максимум.
					tempARR   = _motor_CalcARR(spsVstart);//расчет значения для таймера
				break;
				//----------
				//отрицательное приращение скорости(замедление) на данном кванте времени
				case(MOTOR_DECEL):
					spsVstart = spsVstart - sps_step;//отрицательное приращение скорости(замедление)
					if(spsVstart <= 0) tempARR = 0;	 //Проверка на минимум
					else               tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера
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











