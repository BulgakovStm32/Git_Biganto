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

SpeedRampData_t srd;

//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){

	MOTOR_SetMicrostep(STEP_32);
	TIM1_Init(); //Таймер отсчета временных интервалов расчета скорости.
	GPIO_InitForOutputPushPull(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN);
	motorState = MOTOR_STOP;

	srd.run_state = STOP;// Tells what part of speed ramp we are in.
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
	//------------------------------------------------------------
	// If moving only 1 step.
	if(step == 1)
	{
		srd.accel_count = -1; 	// Move one step...
		srd.run_state   = DECEL;// ...in DECEL state.
		// Just a short delay so main() can act on 'running'.
		srd.step_delay = 1000;

//		status.running = TRUE;//Это нужно для вывода сообщения в консоль.
//		OCR1A = 10;
//		// Run Timer/Counter 1 with prescaler = 8.
//		TCCR1B |= ((0<<CS12)|(1<<CS11)|(0<<CS10));

		DRV_EN_Low(); 	 		 //включение драйвера.
		DRV_RESET_High();		 //
		TIM1->ARR  = 2;//srd.step_delay;
		//TIM1->ARR = (uint16_t)srd.step_delay;
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
		srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100; //Верно

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
		DRV_RESET_High();		 //
		TIM1->ARR  = 2;//srd.step_delay;
		//TIM1->ARR = (uint16_t)srd.step_delay;
		TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
	}
	//------------------------------------------------------------
}
//**********************************************************
void MOTOR_TimerITHandler(void){

		   volatile uint32_t new_step_delay;   // Holds next delay period.
	static volatile  int32_t last_accel_delay; // Remember the last step delay used when accelrating.
	static volatile uint32_t step_count = 0;   // Counting steps when moving.
	static volatile uint32_t rest 	    = 0;   // Keep track of remainder from new_step-delay calculation to incrase accurancy
	//----------------------
	LED_ACT_Toggel();//Отладка.
	//Обновляем значение таймера.
//	OCR1A = srd.step_delay;
	TIM1->ARR = (uint16_t)srd.step_delay;
	//------------------------------------------------------------
	switch(srd.run_state){
		//-------------------
		case STOP:
			last_accel_delay = 0;
			step_count 		 = 0;
			rest       		 = 0;
			//Отключение драйвера.
			DRV_STEP_Low();
			DRV_EN_High();
			//Stop Timer/Counter.
			TIM1->ARR  = 0; 		  //The counter is blocked while the auto-reload value is null.
			TIM1->CR1 &= ~TIM_CR1_CEN;//CEN: Counter enable
		break;
		//-------------------
		case ACCEL:
//			sm_driver_StepCounter(srd.dir);
			DRV_STEP_High();

			step_count++;
			srd.accel_count++;

			new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest           = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);

			// Chech if we should start decelration.
			if(step_count >= srd.decel_start)
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
			//sm_driver_StepCounter(srd.dir);
			DRV_STEP_High();

			step_count++;
			new_step_delay = srd.min_delay;

			// Chech if we should start decelration.
			if(step_count >= srd.decel_start)
			{
				//srd.accel_count = srd.decel_val;
				// Start decelration with same delay as accel ended with.
				new_step_delay = last_accel_delay;
				srd.run_state = DECEL;
			}
		break;
		//-------------------
		case DECEL:
			//sm_driver_StepCounter(srd.dir);
			DRV_STEP_High();

			//step_count++;
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
	//Расчет шагов в секунду (sps) для заданной в RPM скорости.
	//т.е. с такой частотой нужно генерировать импульсы STEP
	//что бы была скорость заданная в RPM.
	spsVstart = Vstart_rpm * rpmToSpsCoefficient;
	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;
	deltaSpsV = spsVmax - spsVstart;

	//Находим количество шагов кототое нужно сделать для достижения скорсти Vmax_rpm
	//за время разгона timeAccel_ms
	//accelSteps  = accelTime_mS / MOTOR_QUANT_TIME_mS; //Вар1
	//accelSteps = deltaSpsV * 1000 / accelTime_mS; //Вар2

	// Find out after how many steps does the speed hit the max speed limit.
	// max_s_lim = speed^2 / (2*alpha*accel)
	//max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);
	uint32_t accel = (deltaSpsV*1000/accelTime_mS) / 1000;//это ускорение в шага в секуну^2
	accelSteps = (long)spsVmax*spsVmax / (long)(((long)ALPHA*2*accel));//Вар3

	//Находим приращение sps которое нужно сделать на каждом
	//шаге (кванте времени) разгона/торможения.
	sps_step = (deltaSpsV + accelSteps/2) / accelSteps;

	//Уточнение количества шагов (квантов времени)
	allSteps   = step;								 //общее кол-во шагов
//	accelSteps = (deltaSpsV + sps_step/2) / sps_step;//кол-во шагов (квантов времени) за время разгона.
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

	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microSteps + 30) / 60;//+30 для правильного округления при целочисленном делении.
	uint32_t deltaSpsV;
	uint32_t accel;
	uint32_t decel;
	//-------------------
	//Направление вращения.
		if(angle < 0)//Против часовой стрелки
		{
			DRV_DIR_High();
			angle = -angle;
		}
		else DRV_DIR_Low(); //По часовой стрелке

	//Кол-во шагов для поворота на заданный угол.
	uint32_t step  = angle * MOTOR_FULL_STEPS_PER_TURN * microSteps / 360;

	//Ускорение.
	spsVstart = Vstart_rpm * rpmToSpsCoefficient;//Перевод RPM в SPS
	spsVmax   = Vmax_rpm   * rpmToSpsCoefficient;
	deltaSpsV = spsVmax - spsVstart;
	accel     = (deltaSpsV*1000) / accelTime_mS;
	accel     = accel * 2 * 3141; //Перевод в радианы в микросекунду^2.
	accel	  = accel / 1000;     //Перевод в радианы в секунду^2.

	//Замедление.
	decel = accel;

	//Скорость
	//Vmax_rpm = Vmax_rpm * 2 * 3141 / 1000;

	MOTOR_SpeedCntrMove(step, 100, 100, 840);







//	MOTOR_CalcAccelDecel(step, Vstart_rpm, Vmax_rpm, accelTime_mS);		 //Расчет значений для ускорения/замедления.
//	DRV_EN_Low(); 	 		 //включение драйвера.
//	DRV_RESET_High();		 //
//	TIM1->ARR  = (uint16_t)_motor_CalcARR(spsVstart);//стартуем с минимальной скорости.
//	TIM1->CR1 |= TIM_CR1_CEN;//включение таймера
//	motorState = MOTOR_ACCEL;
}
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_TimerITHandler2(void){

	//********************************************************
	//********************************************************
	LED_ACT_Toggel();//Отладка.

	//Обновляем значение таймера.
//	OCR1A = srd.step_delay;
//	TIM1->ARR = (uint16_t)srd.step_delay;

	//Обновляем значение таймера.
//	TIM1->ARR = (uint16_t)_motor_CalcARR(spsVstart);
	//------------------------------------------------------------
	switch(motorState){
		//----------
		//положительное приращение скорости(ускорение) на данном кванте времени
		case(MOTOR_ACCEL):
			DRV_STEP_High();

			stepCount++;
			spsVstart = spsVstart + sps_step; 			 //положительное приращение скорости(ускорение)
			if(spsVstart > spsVmax) spsVstart = spsVmax; //проверка на максимум.
			if(stepCount >= accelSteps) motorState = MOTOR_RUN;
		break;
		//----------
		//Постоянная скорость.
		case(MOTOR_RUN):
			DRV_STEP_High();

			stepCount++;
			spsVstart = spsVmax;
			if(stepCount >=	(runSteps+accelSteps)) motorState = MOTOR_DECEL;
		break;
		//----------
		//отрицательное приращение скорости(замедление) на данном кванте времени
		case(MOTOR_DECEL):
			DRV_STEP_High();

			stepCount++;
			spsVstart = spsVstart - sps_step; //отрицательное приращение скорости(замедление)
			if(spsVstart <= 0)
			{
				spsVstart = 0;
				motorState = MOTOR_STOP;
			}
			else if(stepCount >= allSteps) motorState = MOTOR_STOP;
		break;
		//----------
		//Остановка мотора.
		case(MOTOR_STOP):
			stepCount = 0;
			spsVstart = 0;

			//Отключение драйвера.
			DRV_STEP_Low();
			DRV_EN_High();
			//Stop Timer/Counter.
			TIM1->ARR  = 0; 		  //The counter is blocked while the auto-reload value is null.
			TIM1->CR1 &= ~TIM_CR1_CEN;//CEN: Counter enable
		break;
		//----------
	}
	//Обновляем значение таймера.
	TIM1->ARR = (uint16_t)_motor_CalcARR(spsVstart);
	DRV_STEP_Low();
	//********************************************************
	//********************************************************
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************






















