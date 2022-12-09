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
static MotorState_t motorState = STOP;

//Переменные для работы рампы разгона/торможения шагового двигателя(ШД).
static SpeedRampData_t speedRamp; //структура разгона/торможения
static uint32_t targetSteps = 0;  //количество шагов, которое нужно пройти ШД для премещения на заданный угол.
static uint32_t stepCount   = 0;  //счетчик шагов ШД.

//Настраиваемые параметры для работы рампы разгона/торможения шагового двигателя(ШД).
static MotorStepMode_t microStepsMode    = 0; //кол-во микрошагов на шаг ШД.
static uint32_t 	   microStepsPerTurn = 0; //кол-во микрошагов на один оборот ШД.

static uint32_t	torqueState	   = 0; //управление моментом удержания после остановки вращения.
static uint32_t reducerRate    = 0; //передаточное число редуктора.
static uint32_t accelTime	   = 0; //время ускорения в милисекундах
static uint32_t targetVelocity = 0; //cкорость, оборотов в минуту (RPM)
static int32_t  targetAngle	   = 0; //угол на который нужно переместить вал камеры в градусах.

static uint32_t stepsInOneDegreeX1000 = 0;//кол-во шагов в одном градусе *1000. Умножение на 1000 нужно для увеличения точности расчетов.
static uint32_t degreeInOneStepX1000  = 0;//кол-во градусов в одном шаге *1000.

//Значения, который пересчитываются каждый раз при изменении количества микрошагов ШД.
//#define ALPHA 	 (_2PI / SPR)             	  // 2*pi/spr
//#define A_T_x100	((long)(ALPHA*T1_FREQ*100))   // (ALPHA / T1_FREQ)*100
//#define A_SQ 		 (long)(ALPHA*2*10000000000)  // ALPHA*2*10000000000
//#define A_x20000 	 (int)(ALPHA*20000)           // ALPHA*20000

static float    ALPHA    = 0.0;
static long     A_T_x100 = 0.0;
static long     A_SQ     = 0.0;
static uint32_t A_x20000 = 0;
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){

	//Инициализация портов микроконтроллера.
	GPIO_InitForOutputPushPull(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN);
	GPIO_InitForOutputPushPull(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN);
	GPIO_InitForOutputPushPull(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN);
	GPIO_InitForOutputPushPull(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN);

	//Таймер отсчета временных интервалов расчета скорости.
	TIM1_Init(MOTOR_T_FREQ_Hz);

	//Заводские настройки.
	MOTOR_TorqueControl(MOTOR_TORQUE_OFF);			   //Откл. момента ШД.
	MOTOR_DriverReset();							   //сброс внутренней логики драйвера мотора DRV8825.
	MOTOR_SetMicrostepMode(MOTOR_DEFAULT_MICROSTEP);   //Режим микрошаги
	reducerRate    		=  MOTOR_DEFAULT_REDUCER_RATE; //передаточное число редуктора.
	accelTime      		=  MOTOR_DEFAULT_ACCEL_TIME_mS;//время ускорения в милисекундах
	targetVelocity 		=  MOTOR_DEFAULT_RPM;		   //cкорость, оборотов в минуту (RPM)
	speedRamp.run_state = STOP;						   //машина состояния
}
//**********************************************************
//Задать микрошаг
void MOTOR_SetMicrostepMode(MotorStepMode_t steps){

	if(speedRamp.run_state != STOP)return;
	microStepsMode = steps;
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
			MOTOR_SetMicrostepMode(MOTOR_DEFAULT_MICROSTEP);
		return;
		//-------------------
	}
	//Значения, который пересчитываются каждый раз при изменении количества микрошагов ШД.
	//#define ALPHA 	 (_2PI / SPR)             	  // 2*pi/spr
	//#define A_T_x100	((long)(ALPHA*T1_FREQ*100))   // (ALPHA / T1_FREQ)*100
	//#define A_SQ 		 (long)(ALPHA*2*10000000000)  // ALPHA*2*10000000000
	//#define A_x20000 	 (int)(ALPHA*20000)           // ALPHA*20000

	microStepsPerTurn = MOTOR_FULL_STEPS_PER_TURN * microStepsMode;
	ALPHA    = _2PI / microStepsPerTurn;
	A_T_x100 = (long)(ALPHA * T1_FREQ * 100);
	A_SQ 	 = (long)(ALPHA * 2 * 10000000000);
	A_x20000 = (uint32_t)(ALPHA * 20000);
}
//**********************************************************
//передаточное чисо редуктора
void MOTOR_SetReducerRate(uint32_t rate){

	if(speedRamp.run_state != STOP)return;
	if(rate > MOTOR_REDUCER_RATE_MAX) rate = MOTOR_REDUCER_RATE_MAX;
	if(rate < MOTOR_REDUCER_RATE_MIN) rate = MOTOR_REDUCER_RATE_MIN;
	reducerRate = rate;
}
//**********************************************************
//время ускорения/замедлениия в мС
void MOTOR_SetAccelerationTime(uint32_t accelTime_mS){

	if(speedRamp.run_state != STOP)return;
	if(accelTime_mS > MOTOR_ACCEL_TIME_mS_MAX)  accelTime_mS = MOTOR_ACCEL_TIME_mS_MAX;
	if(accelTime_mS < MOTOR_ACCEL_TIME_mS_MIN )	accelTime_mS = MOTOR_ACCEL_TIME_mS_MIN;
	accelTime = accelTime_mS;
}
//**********************************************************
//скорость в RPM
void MOTOR_SetVelocity(uint32_t maxVel){

	if(speedRamp.run_state != STOP)return;
	if(maxVel > MOTOR_RPM_MAX) maxVel = MOTOR_RPM_MAX;
	if(maxVel < MOTOR_RPM_MIN) maxVel = MOTOR_RPM_MIN;
	targetVelocity = maxVel;
}
//**********************************************************
void MOTOR_SetTargetPosition(int32_t angle){

	if(speedRamp.run_state != STOP)return;
	targetAngle = angle;
}
//**********************************************************
MotorStepMode_t MOTOR_GetMicrostepMode(void){

	return microStepsMode;
}
//**********************************************************
uint32_t MOTOR_GetReducerRate(void){

	return reducerRate;
}
//**********************************************************
uint32_t MOTOR_GetAccelerationTime(void){

	return accelTime;
}
//**********************************************************
uint32_t MOTOR_GetVelocity(void){

	return targetVelocity;
}
//**********************************************************
int32_t MOTOR_GetTargetPosition(void){

	return targetAngle;
}
//**********************************************************
uint32_t MOTOR_GetMotorPosition(void){

	return (uint32_t)(((uint64_t)stepCount * degreeInOneStepX1000 + 500) / 1000);
}
//**********************************************************
//Управление моментом удержания ШД.
void MOTOR_TorqueControl(uint32_t state){

	torqueState = state;
	if(state == MOTOR_TORQUE_ON) DRV_EN_Low();  //Вкл. драйвера.
	else				   		 DRV_EN_High(); //Откл. драйвера.
}
//**********************************************************
//сброс внутренней логики драйвера мотора DRV8825.
void MOTOR_DriverReset(void){

	DRV_RESET_Low();
	DELAY_microS(10);
	DRV_RESET_High();
}
//**********************************************************
void MOTOR_EmergencyStopRotation(void){

	MOTOR_SetSpeedRampState(DECEL);
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
	uint32_t accel_lim;	// ! Number of steps before we must start deceleration (if accel does not hit max speed).
	//----------------------
	//Направление вращения.
	if(step < 0) //По часовой стрелки
	{
		DRV_DIR_High();
		step = -step;
	}
	else DRV_DIR_Low(); //против часовой стрелки
	//Сохраним сколько всего нам нужно сделать шагов.
	targetSteps = step;
	//----------------------
	// If moving only 1 step.
	if(step == 1)
	{
		speedRamp.accel_count = -1; 	// Move one step...
		speedRamp.run_state   = DECEL;// ...in DECEL state.
		// Just a short delay so main() can act on 'running'.
		//srd.step_delay = 1000;

		//DRV_EN_Low(); 	 		 //включение драйвера.
		//DRV_RESET_Low();//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
		TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
	}
	//----------------------
	// Only move if number of steps to move is not zero.
	else if(step != 0)
	{
		// Refer to documentation for detailed information about these calculations.

		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt)/ w
		speedRamp.min_delay = A_T_x100 / speed; //Верно

		// Set accelration by calc the first (c0) step delay .
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
		//speedRamp.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100; //Верно
		speedRamp.step_delay = (T1_FREQ_148 * fastsqrt(A_SQ / accel))/100;

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
		if(accel_lim <= max_s_lim) speedRamp.decel_val = -(step - accel_lim);
		//else					   speedRamp.decel_val = -(((long)max_s_lim*accel)/decel);
		else					   speedRamp.decel_val = -((uint32_t)max_s_lim*accel/decel);

		// We must decelrate at least 1 step to stop.
		if(speedRamp.decel_val == 0) speedRamp.decel_val = -1;

		// Find step to start decleration.
		speedRamp.decel_start = step + speedRamp.decel_val;

		// If the maximum speed is so low that we dont need to go via accelration state.
		if(speedRamp.step_delay <= speedRamp.min_delay)
		{
			speedRamp.step_delay = speedRamp.min_delay;
			speedRamp.run_state  = RUN;
		}
		else speedRamp.run_state = ACCEL;
		// Reset counter.
		speedRamp.accel_count = 0;
		//----------------------
		//Включение таймера и драйвера мотора.
		//DRV_RESET_Low();//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
		//DRV_RESET_High();
		DRV_EN_Low();  			 //Вкл. драйвера.
		TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
		stepCount  = 0;
	}
	//----------------------
}
//**********************************************************
MotorState_t MOTOR_GetSpeedRampState(void){

	return speedRamp.run_state;
}
//**********************************************************
void MOTOR_SetSpeedRampState(MotorState_t state){

	speedRamp.run_state = state;
}
//**********************************************************
void MOTOR_TimerITHandler(void){

		   volatile uint32_t new_step_delay;   	   // Holds next delay period.
	static volatile  int32_t last_accel_delay = 0; // Remember the last step delay used when accelrating.
	static volatile uint32_t rest = 0;   	   	   // Keep track of remainder from new_step-delay calculation to incrase accurancy
	//----------------------
	//Обновляем значение таймера.
	TIM1->ARR = (uint16_t)speedRamp.step_delay;
	//Машина состояний рампы разгона/торможения мотора.
	switch(speedRamp.run_state){
		//-------------------
		case STOP:
			TIM1->CR1 &= ~TIM_CR1_CEN; 						  //Откл. таймера.
			if(torqueState == MOTOR_TORQUE_OFF) DRV_EN_High();//Откл. драйвера мотора.
			//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
			//DRV_RESET_Low();

			stepCount++;//прибавим последний шаг.
			//Сброс рабочих переменных.
			last_accel_delay = 0;
			rest       		 = 0;
			//stepCount 		 = 0;
		break;
		//-------------------
		case ACCEL:
			DRV_STEP_High(); //делаем шаг.

			stepCount++;
			speedRamp.accel_count++;

			//Расчет времени до следующего шага.
			new_step_delay = speedRamp.step_delay - (((2 * (long)speedRamp.step_delay) + rest)/(4 * speedRamp.accel_count + 1));
			rest           = ((2 * (long)speedRamp.step_delay)+rest)%(4 * speedRamp.accel_count + 1);

			// Chech if we should start decelration.
			if(stepCount >= speedRamp.decel_start)
			{
				speedRamp.accel_count = speedRamp.decel_val;
				speedRamp.run_state   = DECEL;
			}
			// Chech if we hitted max speed.
			else if(new_step_delay <= speedRamp.min_delay)
			{
				last_accel_delay    = new_step_delay;
				new_step_delay      = speedRamp.min_delay;
				rest 			    = 0;
				speedRamp.run_state = RUN;
			}
		break;
		//-------------------
		case RUN:
			DRV_STEP_High();

			stepCount++;
			new_step_delay = speedRamp.min_delay;

			// Chech if we should start decelration.
			if(stepCount >= speedRamp.decel_start)
			{
				//srd.accel_count = srd.decel_val;
				// Start decelration with same delay as accel ended with.
				new_step_delay = last_accel_delay;
				speedRamp.run_state = DECEL;
			}
		break;
		//-------------------
		case DECEL:
			DRV_STEP_High();

			stepCount++;
			speedRamp.decel_val++;
			speedRamp.accel_count = -speedRamp.decel_val;

			new_step_delay = speedRamp.step_delay + (((2 * (long)speedRamp.step_delay) + rest)/(4 * speedRamp.accel_count + 1));
			rest           = ((2 * (long)speedRamp.step_delay) + rest)%(4 * speedRamp.accel_count + 1);

			// Check if we at last step
			if(speedRamp.decel_val >= 0) speedRamp.run_state = STOP;
		break;
		//-------------------
	}
	speedRamp.step_delay = new_step_delay;
	DRV_STEP_Low();
	//------------------------------------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_StartRotate(void){

//	uint32_t RPM            = MOTOR_GetVelocity();        //38;    //Скорость оборотов в минуту
//	uint32_t accelTime_mSec = MOTOR_GetAccelerationTime();//500;   //время ускорения в милисекундах
//	uint32_t reducerRate    = MOTOR_GetReducerRate();     //6;     //передаточное число редуктора = 6.
//	int32_t  angle		    = MOTOR_GetPosition();        //360*2; //Угол на который нужно переместить вал камеры в градусах.

	//Константы для расчета
	const float radSecSec = _2PI / 3600 * 100;	//рад в секунду за секунд, in 0.01*rad/sec^2. 3600 - это 60 секунд в квадрате.
	const float radSec    = _2PI / 60   * 100;	//рад в сек, in 0.01*rad/sec.

	//uint32_t accel_RPMM  = 4000; //Ускорение оборотов в минуту за минуту
	//Вариант расчета значений когда ускорение задается как время ускорения в мС.
	uint32_t accel_RPMM   = (uint32_t)(targetVelocity / ((float)accelTime / 1000 / 60.0));

	uint32_t microSteps   = microStepsPerTurn * reducerRate;				 //кол-во микрошагов с учетом передаточного числа редуктора.
	degreeInOneStepX1000  = (360 * 1000000 + microSteps/2) / microSteps;	 //кол-во градусов в одном шаге *1000. Умножение на 1000 нужно для увеличения точности расчетов.
	stepsInOneDegreeX1000 = (microSteps * 1000) / 360;						 //кол-во шагов в одном градусе *1000. Умножение на 1000 нужно для увеличения точности расчетов.
	int32_t  steps = (int32_t) (targetAngle * stepsInOneDegreeX1000) / 1000; //кол-во шагов необходимое для перемещение на угол angle
	uint32_t accel = (uint32_t)(accel_RPMM     * reducerRate * radSecSec);   //Ускорение рад в секунду за секунд
	uint32_t speed = (uint32_t)(targetVelocity * reducerRate * radSec);		 //скорость рад в сек

	//MOTOR_TorqueControl(MOTOR_TORQUE_ON);  //вкл. момента.
	MOTOR_SpeedCntrMove(steps, 	//param step   Number of steps to move
						accel, 	//param accel  Accelration to use, in 0.01*rad/sec^2.
						accel, 	//param decel  Decelration to use, in 0.01*rad/sec^2.
						speed);	//param speed  Max speed, in 0.01*rad/sec.
	//-----------------------------------------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Рабочие переменные
//static int32_t  allSteps   = 0;
static uint32_t accelSteps = 0; //Количество шагов за время разгона или торможения
static uint32_t runSteps   = 0;
static uint32_t decelSteps = 0;

static int32_t  spsVstart  = 0;
static uint32_t spsVmax	   = 0;
static uint32_t sps_step   = 0;
//*******************************************************************************************
//*******************************************************************************************
//Расчет значения загружаемого в регистр сравнения ARR таймера.
//sps (step per second) - скорость в шагах в секунду.
static uint32_t _motor_CalcARR(uint32_t sps){

	return (MOTOR_T_FREQ_Hz + sps/2) / sps;// делениа на 2 для правильного округления при целочисленном делении
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

	uint32_t rpmToSpsCoefficient = (MOTOR_FULL_STEPS_PER_TURN * microStepsMode + 30) / 60;//+30 для правильного округления при целочисленном делении.
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
	targetSteps = step;								  	 //общее кол-во шагов
	accelSteps  = (deltaSpsV + sps_step/2) / sps_step;	 //кол-во шагов (квантов времени) за время разгона.
	decelSteps  = accelSteps;						  	 //кол-во шагов (квантов времени) за время торможения.
	runSteps    = targetSteps - accelSteps - decelSteps; //Кол-во шагов которое нужно сделать на участке RUN
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
	uint32_t step = (angle * MOTOR_FULL_STEPS_PER_TURN * microStepsMode / 360);

	MOTOR_CalcAccelDecel(step, Vstart_rpm, Vmax_rpm, accelTime_mS);

	DRV_EN_Low(); 	 		 //включение драйвера.
	DRV_RESET_High();		 //
	//TIM2->ARR  = 2;//srd.step_delay;
	TIM2->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
	motorState = ACCEL;
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
		if(stepCount < targetSteps)
		{
			switch(motorState){
				//----------
				//положительное приращение скорости(ускорение) на данном кванте времени
				case(ACCEL):
					stepCount++;
					spsVstart = spsVstart + sps_step; //положительное приращение скорости(ускорение)
					//if(quantsCount == quantsNum) spsVstart = spsVmax;//Загрузка spsVmax на последнем кванте.
					if(spsVstart > spsVmax) spsVstart = spsVmax;//проверка на максимум.
					tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера

					if(stepCount >= accelSteps) motorState = RUN;
				break;
				//----------
				//Постоянная скорость.
				case(RUN):
					stepCount++;
					spsVstart = spsVmax;
					tempARR   = _motor_CalcARR(spsVstart);//расчет значения для таймера
					if(stepCount >=	(runSteps+accelSteps)) motorState = DECEL;
				break;
				//----------
				//отрицательное приращение скорости(замедление) на данном кванте времени
				case(DECEL):
					stepCount++;
					spsVstart = spsVstart - sps_step;//отрицательное приращение скорости(замедление)
					if(spsVstart <= 0) tempARR = 0;	 //Проверка на минимум
					else               tempARR = _motor_CalcARR(spsVstart);//расчет значения для таймера
					if(stepCount >= targetSteps) motorState = STOP;
				break;
				//----------
				default:
					tempARR    = 0;
					stepCount  = 0;
					motorState = STOP;
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
			motorState = STOP;
			stepCount  = 0;
		}
	}

}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************






















