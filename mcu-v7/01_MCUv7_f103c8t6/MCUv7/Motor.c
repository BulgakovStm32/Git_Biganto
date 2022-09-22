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
//Рабочие переменные
static MotorStepMode_t	microSteps = 0; //количество микрошагов.
static MotorState_t 	motorState = MOTOR_NO_INIT;


static uint32_t 		motorStepCount = 0;
static uint32_t 		allSteps	   = 0;

SpeedRampData_t srd;
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){

	GPIO_InitForOutputPushPull(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN);
	DRV_EN_High();	  //Отключения драйвера.

	GPIO_InitForOutputPushPull(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN);
	DRV_RESET_High();//Сброс драйвера.

	GPIO_InitForOutputPushPull(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN);
	DRV_STEP_Low();

	GPIO_InitForOutputPushPull(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN);

	TIM1_Init(MOTOR_FREQ_TIM_Hz); //Таймер отсчета временных интервалов расчета скорости.
	MOTOR_SetMicrostep(STEP_32);  //Микрошаги
	srd.run_state = STOP;		  //Tells what part of speed ramp we are in.
	motorState    = MOTOR_STOP;
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
//**********************************************************
void MOTOR_Disable(void){

	TIMx_Disable(TIM1);
	DRV_STEP_Low();
	DRV_RESET_Low();
	DRV_EN_High();	  //Отключения драйвера.
}
//**********************************************************
void MOTOR_Enable(void){

	TIMx_Enable(TIM1);
	DRV_EN_Low();	  //Включение драйвера.
}
////*******************************************************************************************
////*******************************************************************************************
//static uint32_t _motor_CalcARR(uint32_t sps){
//
//	return (MOTOR_FREQ_TIM_Hz + sps/2) / sps;// делениа на 2 для правильного округления при целочисленном делении
//}
////**********************************************************
/////*
//// * расчет время в мс на выход до стационарного режима
//// */
////uint32_t MOTOR_CalcAccelMaxToAccelTime(uint32_t Vmax_rpm, uint32_t Accel){
////
////	//время в мс на выход до стационарного режима
////	 return (uint32_t)((float)(Vmax_rpm / Accel) * 1000);
////}
////**********************************************************
//void MOTOR_CalcAccelDecel(uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t timeAccel_ms){
//
//	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60;//+30 для правильного округления при целочисленном делении.
//	//-------------------
//	//Расчет шагов в секунду (sps) для заданной в RPM скорости.
//	spsVstart = Vstart_rpm * rpmToSpsCoefficient;
//	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;
//	//Находим количество квантов времени за время разгона.
//	quantsNum = timeAccel_ms / MOTOR_QUANT_TIME_mS;
//	//Находим приращение pps которое нужно сделать на каждом кванте времени разгона/торможения.
//	sps_step  = (spsVmax - spsVstart) / quantsNum;
//	//Уточнение количества квантов
//	quantsNum = (spsVmax - spsVstart) / sps_step;
//
////	//Начальное значение загружаемое в регистр ARR для скорости Vstart
////	if(Vstart_rpm == 0) arrV0 = 0;
////	else 				arrV0 = _motor_CalcARR(spsVstart);
////	//Запуск таймера
////	TIM2->ARR   = (uint16_t)arrV0;
////	TIM2->CCR1  = (uint16_t)(TIM2->ARR / 2);
////	TIM2->CR1  |= TIM_CR1_CEN;
//
//
//
////	arrV0     = 100000U / ppsV0;
////	arrVmax   = 100000U / ppsVmax;
////	quantsNum = (tAccel * 1000) / MOTOR_QUANT_TIME;
////	arr_step  = (arrV0 - arrVmax) / quantsNum;
////	//Если arr_step=0 значит за заданное время tAccel не получится дойти от V0 до Vmax
////	//Раньше наступит время торможения.
////	if(arr_step == 0) return ;
////	quantsNum = (arrV0 - arrVmax) / arr_step;//точный расчет нужного количества квантов.
//
////	TIM2->ARR   = (uint16_t)arrV0;
////	TIM2->CCR1  = (uint16_t)(TIM2->ARR - 20);
////	TIM2->CR1  |= TIM_CR1_CEN;
//
////	motorState = MOTOR_CALC_OK;
//}
////**********************************************************
//void MOTOR_SpinStart(MotorState_t spinMode){
//
//	motorState  = spinMode;
//	//DRV_RESET_High();
//	DRV_EN_Low();			  //включение драйвера.
//	TIM2->CR1 |= TIM_CR1_CEN; //включение таймера
//}
////**********************************************************
//void MOTOR_AccelDecelLoop(void){
//
//	static uint32_t msCount = 0;
//		   uint32_t tempARR = 0;
//	//-------------------
//	if(motorState == MOTOR_READY)return;//Если расчеты не готовы то ничего не делаем и выходим.
//	if(++msCount >= MOTOR_QUANT_TIME_mS)//Отсчитываем нужный квант времени.
//	{
//		//LED_ACT_Toggel();
//		msCount = 0;
//		//Считаем кванты времени и расчитываем скорость.
//		if(quantsCount < quantsNum)
//		{
//			quantsCount++;
//
//			switch(motorState){
//				//----------
//				//положительное приращение скорости(ускорение) на данном кванте времени
//				case(MOTOR_ACCEL):
//					spsVstart = spsVstart + sps_step; //положительное приращение скорости(ускорение)
//					//if(quantsCount == quantsNum) spsVstart = spsVmax;//Загрузка spsVmax на последнем кванте.
//					if(spsVstart > spsVmax) spsVstart = spsVmax;//проверка на максимум.
//					tempARR   = _motor_CalcARR(spsVstart);//расчет значения для таймера
//				break;
//				//----------
//				//отрицательное приращение скорости(замедление) на данном кванте времени
//				case(MOTOR_DECEL):
//					spsVstart = spsVstart - sps_step;//отрицательное приращение скорости(замедление)
//					if(spsVstart <= 0) tempARR = 0;	 //Проверка на минимум
//					else               tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера
//				break;
//				//----------
//				default:
//					tempARR = 0;
//					motorState = MOTOR_READY;
//				break;
//				//----------
//			}
//			//Обновляем значение таймера.
//			if(tempARR != 0)
//			{
//				TIM2->ARR  = (uint16_t)tempARR;
//				TIM2->CCR1 = (uint16_t)(TIM2->ARR / 2);
//			}
//			else
//			{
//				TIM2->ARR  = 1;//The counter is blocked while the auto-reload value is null.
//				TIM2->CCR1 = 0;
//				//DRV_RESET_Low();
//				DRV_EN_High();
//			}
//		}
//		else
//		{
//			motorState  = MOTOR_READY;
//			quantsCount = 0;
//		}
//	}
//}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Реализация управления мотора по статье
//"AVR446 Линейное управление скоростью шагового двигателя"
//http://avrdoc.narod.ru/index/0-7
//https://embeddeddesign.org/stepper-motor-controller/ - вариант реализации

//SpeedRampData_t srd;



static float fastsqrt(float val){

    long tmp = *(long*)&val;
    //----------------------
    tmp -= 127L << 23; /* Remove IEEE bias from exponent (-2^23) */
    /* tmp is now an appoximation to logbase2(val) */
    tmp  = tmp  >> 1;  /* divide by 2 */
    tmp += 127L << 23; /* restore the IEEE bias from the exponent (+2^23) */
    return *(float*)&tmp;
}
//*******************************************************************************************
/*! \brief Move the stepper motor a given number of steps.
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step   Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 */
void MOTOR_SpeedCntrMove(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed){

	uint32_t max_s_lim;	// ! Number of steps before we hit max speed.
	uint32_t accel_lim;	// ! Number of steps before we must start deceleration
						// (if accel does not hit max speed).
	//----------------------
	//Направление вращения.
	if(step < 0) //Против часовой стрелки
	{
		DRV_DIR_High();
		step = -step;
	}
	else DRV_DIR_Low(); //По часовой стрелке
	//Сохраним сколько всего нам нужно сделать шагов.
	allSteps = step;
	//------------------------------------------------------------
	// If moving only 1 step.
	if(step == 1)
	{
		srd.accel_count = -1; 	// Move one step...
		srd.run_state   = DECEL;// ...in DECEL state.
		// Just a short delay so main() can act on 'running'.
		//srd.step_delay = 1000;

		DRV_EN_Low(); 	 		 //включение драйвера.
		//DRV_RESET_Low();//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
		TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
	}
	//------------------------------------------------------------
	// Only move if number of steps to move is not zero.
	else if(step != 0)
	{
		// Refer to documentation for detailed information about these calculations.

		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt)/ w
		srd.min_delay = A_T_x100 / speed; //Верно

		// Set accelration by calc the first (c0) step delay .
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
		//srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100; //Верно
		srd.step_delay = (T1_FREQ_148 * fastsqrt(A_SQ / accel))/100;

		// Find out after how many steps does the speed hit the max speed limit.
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);

		// If we hit max speed limit before 0,5 step it will round to 0.
		// But in practice we need to move atleast 1 step to get any speed at all.
		if(max_s_lim == 0) max_s_lim = 1;

		// Find out after how many steps we must start deceleration.
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = ((long)step*decel) / (accel+decel);
		// We must accelrate at least 1 step before we can start deceleration.
		if(accel_lim == 0) accel_lim = 1;

		// Use the limit we hit first to calc decel.
		if(accel_lim <= max_s_lim) srd.decel_val = -(step - accel_lim);
		//else					   srd.decel_val = -(((long)max_s_lim*accel)/decel);
		else					   srd.decel_val = -((uint32_t)max_s_lim*accel/decel);

		// We must decelrate at least 1 step to stop.
		if(srd.decel_val == 0) srd.decel_val = -1;

		// Find step to start decleration.
		srd.decel_start = step + srd.decel_val;

		// If the maximum speed is so low that we dont need to go via accelration state.
		if(srd.step_delay <= srd.min_delay)
		{
			srd.step_delay = srd.min_delay;
			srd.run_state  = RUN;
		}
		else srd.run_state = ACCEL;
		// Reset counter.
		srd.accel_count = 0;

		DRV_EN_Low(); 	 		 //включение драйвера.
		//DRV_RESET_Low();//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
		//DRV_RESET_High();
		TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
	}
	//------------------------------------------------------------
}
//**********************************************************
uint32_t MOTOR_GetSpeedRampState(void){

	return srd.run_state;
}
//**********************************************************
void MOTOR_SetSpeedRampState(uint32_t state){

	srd.run_state = state;
}
//**********************************************************
void MOTOR_TimerITHandler(void){

		   volatile uint32_t new_step_delay;   // Holds next delay period.
	static volatile  int32_t last_accel_delay; // Remember the last step delay used when accelrating.
//	static volatile uint32_t step_count = 0;   // Counting steps when moving.
	static volatile uint32_t rest 	    = 0;   // Keep track of remainder from new_step-delay calculation to incrase accurancy
	//----------------------
	//LED_ACT_Toggel();//Отладка.
	TIM1->ARR = (uint16_t)srd.step_delay;//Обновляем значение таймера.
	//DRV_STEP_Low();						 //
	//------------------------------------------------------------
	switch(srd.run_state){
		//-------------------
		case STOP:
			last_accel_delay = 0;
			rest       		 = 0;
			//motorStepCount   = 0;
			//DRV_EN_High();	  //Отключение драйвера.
			//DRV_RESET_Low();//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
			TIM1->CR1 &= ~TIM_CR1_CEN;//CEN: Counter enable

			//Если была пауза во время варщения.
//			if(motorStepCount < allSteps)
//			{
//				//motorStepCount = allSteps - motorStepCount;
//
//				uint32_t RPM            = 35;    	//Скорость оборотов в минуту
//				uint32_t accelTime_mSec = 1500;		//время ускорения в милисекундах
//				uint32_t angle		    = 120*1; 	//Угол на который нужно переместить вал камеры в градусах.
//				uint32_t reducerRate    = 6;		//передаточное число редуктора = 6.
//				//-----------------------------------------------------
//				//Вариант расчета значений когда ускорение задается как время ускорения в секундах
//				//uint32_t accel_RPMM  = 4000; //Ускорение оборотов в минуту за минуту
//				uint32_t accel_RPMM = (uint32_t)(RPM / ((float)accelTime_mSec / 1000 / 60.0));
//				//-----------------------------------------------------
//				//Пример расчета:
//				//Макс.скорость - 10 RPM,   это = 10*2*Pi = 62.8319 рад/мин, или 1,0472 рад/сек.   1,0472/ 0,01 = 104,72 ~ 105 (in 0.01*rad/sec).
//				//Ускорение     - 20 RPM^2, это = 20*2*Pi = 125.6637 рад/мин^2, или 0,035 рад/сек^2. 0,035 / 0,01 = 3,49   ~ 3	 (in 0.01*rad/sec^2).
//				//Замедление    - 20 RPM^2, расчет тотже что и при ускорении.
//				//Шаги - мне нужно повернуться на 360 градусов. Это 6400 шагов при microstep = 1/32
//
//				// param step   Number of steps to move (pos - CW, neg - CCW).
//				// param accel  Accelration to use, in 0.01*rad/sec^2.
//				// param decel  Decelration to use, in 0.01*rad/sec^2.
//				// param speed  Max speed, in 0.01*rad/sec.
//
//				uint32_t microSteps 	   = (uint32_t)MOTOR_FULL_STEPS_PER_TURN * MOTOR_GetMicrostep();
//				uint32_t steps			   = (uint32_t)(angle      * reducerRate * microSteps / (float)360.0);//кол-во шагов необходимое длч перемещение на угол angle
//				uint32_t vel_rad_per_sec   = (uint32_t)(RPM        * reducerRate * _2PI / 60   * 100);		  //скорость рад в сек
//				uint32_t accel_rad_per_sec = (uint32_t)(accel_RPMM * reducerRate * _2PI / 3600 * 100);		  //Ускорение рад в секунду за секунд
//
//				MOTOR_SpeedCntrMove(1* steps,    //param step   Number of steps to move
//									accel_rad_per_sec, //param accel  Accelration to use, in 0.01*rad/sec^2.
//									accel_rad_per_sec, //param decel  Decelration to use, in 0.01*rad/sec^2.
//									vel_rad_per_sec);  //param speed  Max speed, in 0.01*rad/sec.
//			}
			motorStepCount = 0;
		break;
		//-------------------
		case ACCEL:
			DRV_STEP_High();

			motorStepCount++;
			srd.accel_count++;

			new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest           = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);

			// Chech if we should start decelration.
			if(motorStepCount >= srd.decel_start)
			{
				srd.accel_count = srd.decel_val;
				srd.run_state   = DECEL;
			}
			// Chech if we hitted max speed.
			else if(new_step_delay <= srd.min_delay)
			{
				last_accel_delay = new_step_delay;
				new_step_delay   = srd.min_delay;
				rest 			 = 0;
				srd.run_state 	 = RUN;
			}
		break;
		//-------------------
		case RUN:
			DRV_STEP_High();

			motorStepCount++;
			new_step_delay = srd.min_delay;

			// Chech if we should start decelration.
			if(motorStepCount >= srd.decel_start)
			{
				//srd.accel_count = srd.decel_val;
				// Start decelration with same delay as accel ended with.
				new_step_delay = last_accel_delay;
				srd.run_state = DECEL;
			}
		break;
		//-------------------
		case DECEL:
			DRV_STEP_High();

			motorStepCount++;
			srd.decel_val++;
			srd.accel_count = -srd.decel_val;

			new_step_delay = srd.step_delay + (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest           = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);

			// Check if we at last step
			if(srd.decel_val >= 0) srd.run_state = STOP;
		break;
		//-------------------
	}
	srd.step_delay = new_step_delay;
	DRV_STEP_Low();
	//------------------------------------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Рабочие переменные
//static int32_t  allSteps   = 0;
static uint32_t accelSteps = 0; //Количество шагов за время разгона или торможения
static uint32_t runSteps   = 0;
static uint32_t decelSteps = 0;

static uint32_t stepCount  = 0;

static int32_t  spsVstart  = 0;
static uint32_t spsVmax	   = 0;
static uint32_t sps_step   = 0;
//*******************************************************************************************
//*******************************************************************************************
//Расчет значения загружаемого в регистр сравнения ARR таймера.
//sps (step per second) - скорость в шагах в секунду.
static uint32_t _motor_CalcARR(uint32_t sps){

	return (MOTOR_FREQ_TIM_Hz + sps/2) / sps;// делениа на 2 для правильного округления при целочисленном делении
}
//**********************************************************
//step         – число шагов для перемещения:
//				 отричательное значение шагов - вращение по часовой стрелке;
//				 положительное значение шагов - вращение против часовой стрелке.
//Vstart_rpm   - скорость, с которой стартует ШД, может быть от 0 до Vstart_max(смотрим документацию на ШД)
//Vmax_rpm     - скорость которую хотим достигнуть(смотрим документацию на ШД).
//AccelTime_mS - время(в мС), за которое хотим выйти от Vstart_rpm на Vmax_rpm, т.е. это ускорение.
//				 Такое же время используется для замедления.
void MOTOR_CalcAccelDecel(uint32_t step, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS){

	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60;//+30 для правильного округления при целочисленном делении.
	uint32_t deltaSpsV;
	//-------------------
	//	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60;//+30 для правильного округления при целочисленном делении.
	//	//-------------------
	//	//Расчет шагов в секунду (sps) для заданной в RPM скорости.
	//	spsVstart = Vstart_rpm * rpmToSpsCoefficient;
	//	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;

	//	//Находим количество квантов времени за время разгона.
	//	quantsNum = timeAccel_ms / MOTOR_QUANT_TIME_mS;

	//	//Находим приращение pps которое нужно сделать на каждом кванте времени разгона/торможения.
	//	sps_step  = (spsVmax - spsVstart) / quantsNum;

	//	//Уточнение количества квантов
	//	quantsNum = (spsVmax - spsVstart) / sps_step;


	//Расчет шагов в секунду (sps) для заданной в RPM скорости.
	//т.е. с такой частотой нужно генерировать импульсы STEP
	//что бы была скорость заданная в RPM.
	spsVstart = Vstart_rpm * rpmToSpsCoefficient;
	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;
	deltaSpsV = spsVmax - spsVstart;

	//Находим количество шагов кототое нужно сделать для достижения скорсти Vmax_rpm
	//за время разгона timeAccel_ms
	accelSteps  = accelTime_mS / MOTOR_QUANT_TIME_mS;

	//Находим приращение sps которое нужно сделать на каждом
	//шаге (кванте времени) разгона/торможения.
	sps_step = (deltaSpsV + accelSteps/2) / accelSteps;

	//Уточнение количества шагов (квантов времени)
	allSteps   = step;								 //общее кол-во шагов
	accelSteps = (deltaSpsV + sps_step/2) / sps_step;//кол-во шагов (квантов времени) за время разгона.
	decelSteps = accelSteps;						 //кол-во шагов (квантов времени) за время торможения.
	runSteps   = allSteps - accelSteps - decelSteps; //Кол-во шагов которое нужно сделать на участке RUN
	//-------------------
	TIM2_Init();
}
//**********************************************************
//angle        – угол на который нужно повернуть вал
//Vstart_rpm   - скорость, с которой стартует ШД, может быть от 0 до Vstart_max(смотрим документацию на ШД)
//Vmax_rpm     - скорость которую хотим достигнуть(смотрим документацию на ШД).
//accelTime_mS – время за которое необходимо разогнаться от Vstart_rpm до Vmax_rpm, в мС.
void MOTOR_Move(int32_t angle, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS){

	//Направление вращения.
	if(angle < 0)//Против часовой стрелки
	{
		DRV_DIR_High();
		angle = -angle;
	}
	else DRV_DIR_Low(); //По часовой стрелке

	//Кол-во шагов для поворота на заданный угол.
	uint32_t step = (angle * MOTOR_FULL_STEPS_PER_TURN * microSteps / 360);

	MOTOR_CalcAccelDecel(step, Vstart_rpm, Vmax_rpm, accelTime_mS);

	DRV_EN_Low(); 	 		 //включение драйвера.
	DRV_RESET_High();		 //
	//TIM2->ARR  = 2;//srd.step_delay;
	TIM2->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
	motorState = MOTOR_ACCEL;
}
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_TimerITHandler2(void){

	static uint32_t msCount = 0;
		   uint32_t tempARR = 0;
	//-------------------
	if(++msCount >= MOTOR_QUANT_TIME_mS)//Отсчитываем нужный квант времени.
	{
		//LED_ACT_Toggel();
		msCount = 0;
		//Считаем кванты времени и расчитываем скорость.
		if(stepCount < allSteps)
		{
			switch(motorState){
				//----------
				//положительное приращение скорости(ускорение) на данном кванте времени
				case(MOTOR_ACCEL):
					stepCount++;
					spsVstart = spsVstart + sps_step; //положительное приращение скорости(ускорение)
					//if(quantsCount == quantsNum) spsVstart = spsVmax;//Загрузка spsVmax на последнем кванте.
					if(spsVstart > spsVmax) spsVstart = spsVmax;//проверка на максимум.
					tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера

					if(stepCount >= accelSteps) motorState = MOTOR_RUN;
				break;
				//----------
				//Постоянная скорость.
				case(MOTOR_RUN):
					stepCount++;
					spsVstart = spsVmax;
					tempARR   = _motor_CalcARR(spsVstart);//расчет значения для таймера
					if(stepCount >=	(runSteps+accelSteps)) motorState = MOTOR_DECEL;
				break;
				//----------
				//отрицательное приращение скорости(замедление) на данном кванте времени
				case(MOTOR_DECEL):
					stepCount++;
					spsVstart = spsVstart - sps_step;//отрицательное приращение скорости(замедление)
					if(spsVstart <= 0) tempARR = 0;	 //Проверка на минимум
					else               tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера
					if(stepCount >= allSteps) motorState = MOTOR_STOP;
				break;
				//----------
				default:
					tempARR    = 0;
					stepCount  = 0;
					motorState = MOTOR_STOP;
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
			motorState  = MOTOR_STOP;
			stepCount = 0;
		}
	}

}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************






















