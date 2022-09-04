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
static uint32_t     microSteps = 0; //количество микрошагов.
static MotorState_t motorState = MOTOR_NO_INIT;

static int32_t  allSteps   = 0;
static uint32_t accelSteps = 0; //Количество шагов за время разгона или торможения
static uint32_t runSteps   = 0;
static uint32_t decelSteps = 0;

static uint32_t stepCount  = 0;

static int32_t  spsVstart  = 0;
static uint32_t spsVmax	   = 0;
static uint32_t sps_step   = 0;
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){

	MOTOR_SetMicrostep(STEP_32);
	TIM2_Init(); //Таймер генерации импульсов STEP
//	TIM1_Init(); //Таймер отсчета временных интервалов расчета скорости.
	motorState = MOTOR_STOP;
}
//**********************************************************
//Задать микрошаг
void MOTOR_SetMicrostep(MotorStepMode_t steps){

	microSteps = steps;
	switch(steps){
	//-------------------
	case(STEP_1):
		DRV_MODE2_Low();
		DRV_MODE1_Low();
		DRV_MODE0_Low();
	break;
	//-------------------
	case(STEP_2):
		DRV_MODE2_Low();
		DRV_MODE1_Low();
		DRV_MODE0_High();
	break;
	//-------------------
	case(STEP_4):
		DRV_MODE2_Low();
		DRV_MODE1_High();
		DRV_MODE0_Low();
	break;
	//-------------------
	case(STEP_8):
		DRV_MODE2_Low();
		DRV_MODE1_High();
		DRV_MODE0_High();
	break;
	//-------------------
	case(STEP_16):
		DRV_MODE2_High();
		DRV_MODE1_Low();
		DRV_MODE0_Low();
	break;
	//-------------------
	case(STEP_32):
		DRV_MODE2_High();
		DRV_MODE1_Low();
		DRV_MODE0_High();
	break;
	//-------------------
	default:
		microSteps = STEP_ERR;
	break;
	//-------------------
	}
}
//**********************************************************
MotorStepMode_t MOTOR_GetMicrostep(void){

	return microSteps;
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
//Расчет значения загружаемого в регистр сравнения ARR таймера.
//sps (step per second) - скорость в шагах в секунду.
static void _motor_CalcARRandSetTimer(uint32_t sps){

	uint32_t temp;
	//-------------------
	if(sps > 0) temp = (MOTOR_FREQ_TIM_Hz + sps/2) / sps;// делениа на 2 для правильного округления при целочисленном делении
	else		temp = 0;								 //The counter is blocked while the auto-reload value is null.
	TIM2->ARR  = (uint16_t)temp;						 //расчет значения для таймера
	TIM2->CCR1 = (uint16_t)(TIM2->ARR / 2);				 //
}
//**********************************************************
//step         – число шагов для перемещения:
//				 отричательное значение шагов - вращение по часовой стрелке;
//				 положительное значение шагов - вращение против часовой стрелке.
//Vstart_rpm   - скорость, с которой стартует ШД, может быть от 0 до Vstart_max(смотрим документацию на ШД)
//Vmax_rpm     - скорость которую хотим достигнуть(смотрим документацию на ШД).
//AccelTime_mS - время(в мС), за которое хотим выйти от Vstart_rpm на Vmax_rpm, т.е. это ускорение.
//				 Такое же время используется для замедления.
void MOTOR_CalcAccelDecel(int32_t step, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS){

	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60;//+30 для правильного округления при целочисленном делении.
	uint32_t deltaSpsV;
	//-------------------
	//Направление вращения.
	if(step < 0) DRV_DIR_High();
	else		 DRV_DIR_Low();

	//Расчет шагов в секунду (sps) для заданной в RPM скорости.
	//т.е. с такой частотой нужно генерировать импульсы STEP
	//что бы была скорость заданная в RPM.
	spsVstart = Vstart_rpm * rpmToSpsCoefficient;
	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;
	deltaSpsV = spsVmax - spsVstart;

	//Находим количество квантов времени за время разгона.
	//т.е. это количество шагов кототое нужно сделать для достижения скорсти Vmax_rpm
	//за время разгона timeAccel_ms
	accelSteps  = accelTime_mS / MOTOR_QUANT_TIME_mS;
	accelSteps += accelTime_mS % MOTOR_QUANT_TIME_mS;//прибавляем остаток.

	//Находим приращение sps которое нужно сделать на каждом
	//шаге (кванте времени) разгона/торможения.
	sps_step = (deltaSpsV + accelSteps/2) / accelSteps;

	//Уточнение количества шагов (квантов времени)
	allSteps   = step;								 //общее кол-во шагов
	accelSteps = (deltaSpsV + sps_step/2) / sps_step;//кол-во шагов (квантов времени) за время разгона.
	decelSteps = accelSteps;						 //кол-во шагов (квантов времени) за время торможения.
	runSteps   = allSteps - accelSteps - decelSteps; //Кол-во шагов которое нужно сделать на участке RUN
	//-------------------
}
//**********************************************************
//angle        – угол на который нужно повернуть вал
//Vstart_rpm   - скорость, с которой стартует ШД, может быть от 0 до Vstart_max(смотрим документацию на ШД)
//Vmax_rpm     - скорость которую хотим достигнуть(смотрим документацию на ШД).
//accelTime_mS – время за которое необходимо разогнаться от Vstart_rpm до Vmax_rpm, в мС.
void MOTOR_Move(int32_t angle, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS){

	int32_t step = angle * MOTOR_FULL_STEPS_PER_TURN * microSteps / 360;//Кол-во шагов для поворота на заданный угол.
	MOTOR_CalcAccelDecel(step, Vstart_rpm, Vmax_rpm, accelTime_mS);		//Расчет значений для ускорения/замедления.
	DRV_EN_Low(); 	 //включение драйвера.
	DRV_RESET_High();//

	_motor_CalcARRandSetTimer(spsVstart);//стартуем с минимальной скорости.
	TIM2->CR1 |= TIM_CR1_CEN; 			 //включение таймера

	motorState = MOTOR_ACCEL;
}
//**********************************************************
void MOTOR_AccelDecelLoop(void){

	static uint32_t msCount = 0;
//		   uint32_t tempARR = 0;
	//-------------------
	if(motorState == MOTOR_STOP)return;//Если расчеты не готовы то ничего не делаем и выходим.
	if(++msCount >= MOTOR_QUANT_TIME_mS)//Отсчитываем нужный квант времени.
	{
		//LED_ACT_Toggel();
		msCount = 0;

		//********************************************************
		//********************************************************
		//Считаем кванты времени и расчитываем скорость.
		stepCount++;
		switch(motorState){
			//----------
			//положительное приращение скорости(ускорение) на данном кванте времени
			case(MOTOR_ACCEL):
				spsVstart = spsVstart + sps_step; 			 //положительное приращение скорости(ускорение)
				if(spsVstart > spsVmax) spsVstart = spsVmax; //проверка на максимум.
					 if(stepCount >= decelSteps) motorState = MOTOR_DECEL;
				else if(stepCount >= accelSteps) motorState = MOTOR_RUN;
			break;
			//----------
			//Постоянная скорость.
			case(MOTOR_RUN):
				spsVstart = spsVmax;
				if(stepCount >=	runSteps) motorState = MOTOR_DECEL;
			break;
			//----------
			//отрицательное приращение скорости(замедление) на данном кванте времени
			case(MOTOR_DECEL):
				spsVstart = spsVstart - sps_step; //отрицательное приращение скорости(замедление)
				if(spsVstart <= 0)
				{
					spsVstart = 0;
					motorState = MOTOR_STOP;
				}
				else if(stepCount >= allSteps) motorState = MOTOR_STOP;
			break;
			//----------
			case(MOTOR_STOP):
				stepCount = 0;
				spsVstart = 0;

				TIM2->CR1 &= ~TIM_CR1_CEN; //Откл. таймера
				DRV_EN_High();			   //Отключение
				DRV_RESET_Low();		   //и сброс драйвера
			break;
			//----------
			default:
				stepCount = 0;
				spsVstart = 0;

				TIM2->CR1 &= ~TIM_CR1_CEN; //Откл. таймера
				DRV_EN_High();			   //Отлк.
				DRV_RESET_Low();		   //и сброс драйвера
			//----------
		}
		//Обновляем значение таймера.
		_motor_CalcARRandSetTimer(spsVstart);

//		TIM2->ARR  = (uint16_t)_motor_CalcARR(spsVstart);//расчет значения для таймера
//		TIM2->CCR1 = (uint16_t)(TIM2->ARR / 2);
//		tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера
//		if(tempARR != 0)
//		{
//			TIM2->ARR  = (uint16_t)tempARR;
//			TIM2->CCR1 = (uint16_t)(TIM2->ARR / 2);
//		}
//		else
//		{
//			TIM2->ARR  = 1;//The counter is blocked while the auto-reload value is null.
//			TIM2->CCR1 = 0;
//			//DRV_RESET_Low();
//			DRV_EN_High();
//		}
		//********************************************************
		//********************************************************
	}
}
//*******************************************************************************************
//*******************************************************************************************











