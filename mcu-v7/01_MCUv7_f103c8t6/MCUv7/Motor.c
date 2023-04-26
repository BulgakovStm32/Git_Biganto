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
//флаги
static MotorFlag_t  motorFlags;

//Переменные для работы рампы разгона/торможения шагового двигателя(ШД).
static SpeedRampData_t speedRamp; //структура переменных разгона/торможения
static int32_t  targetSteps = 0;  //количество шагов, которое нужно пройти ШД для премещения на заданный угол.
static uint32_t stepCount   = 0;  //счетчик шагов ШД.

//Переменные для отработки режима Пауза/Возобновление вращения.
static uint32_t pausa_steps         = 0; //шаг на котором началось замедление.
static int32_t  pausa_reverseSteps  = 0; //кол-во шагов назад для возвращения в точку начала замедления
static int32_t  pausa_stepsToTarget = 0; //кол-во шагов, которое нужно пройти до целеыого положения.

//Настраиваемые параметры для работы рампы разгона/торможения шагового двигателя(ШД).
static MotorStepMode_t uStepsMode    = 0; //кол-во микрошагов на шаг ШД.
static uint32_t 	   uStepsPerTurn = 0; //кол-во микрошагов на один оборот ШД.

static uint32_t	torqueState	   = 0; //управление моментом удержания после остановки вращения.
static uint32_t reducerRate    = 0; //передаточное число редуктора.
static uint32_t accelTime	   = 0; //время ускорения в милисекундах
static uint32_t targetVelocity = 0; //cкорость, оборотов в минуту (RPM)
static int32_t  targetAngle	   = 0; //угол на который нужно переместить вал камеры в градусах.

static uint32_t stepsInOneDegreeX1e6 = 0;//кол-во шагов в одном градусе *10000. Умножение на 10000 нужно для увеличения точности расчетов.
static uint32_t degreeInOneStepX1e8  = 0;//кол-во градусов в одном шаге *10е7.

//Значения, который пересчитываются каждый раз при изменении количества микрошагов ШД.
//#define ALPHA 	 (_2PI / SPR)             	  // 2*pi/spr
//#define A_T_x100	((long)(ALPHA*T1_FREQ*100))   // (ALPHA / T1_FREQ)*100
//#define A_SQ 		 (long)(ALPHA*2*10000000000)  // ALPHA*2*10000000000
//#define A_x20000 	 (int)(ALPHA*20000)           // ALPHA*20000

static float    ALPHA    = 0.0;//Угол (в рад.) поворота вала мотора при одном шаге с учетом микрошагов.
static uint32_t A_T_x100 = 0;  //Максимальнач скорость рад/сек при заданной частоте тактирования T1_FREQ таймера
static uint32_t A_SQ     = 0;  //Константа для расчета задержки с0
static uint32_t A_x20000 = 0;  //???
//**********************************************************
//Константы для расчета.

//Компенсации ошибки при расчете c0.
const uint32_t T1_FREQ_148 = (uint32_t)( ((uint32_t)(T1_FREQ * 0.676) + 50) / 100 );//так точнее

//360 градусов умноженные на 1e8.это нужно для повышения точности при целоцисленной расчетах.
const uint64_t degree360x1e8 = (uint64_t)360 * 1e8;

//2Pi rad
const float _2PI = 2 * M_PI;
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void _motor_TimerInit(uint32_t freg){

	TIM1_Init(freg);
}
//**********************************************************
void _motor_TimerEnable(void){

	TIMx_Enable(MOTOR_TIMER); //Counter enable
}
//**********************************************************
void _motor_TimerDisable(void){

	TIMx_Disable(MOTOR_TIMER); //Counter disable
}
//**********************************************************
void _motor_SetTimerCount(uint32_t val){

	TIMx_SetARR(MOTOR_TIMER, val-1);
}
//**********************************************************
//Направление вращения.
void _motor_Direction(uint32_t dir){

	//Направление вращения.
		 if(dir == MOTOR_DIR_CCW) DRV_DIR_Low(); 	//против часовой стрелки
	else if(dir == MOTOR_DIR_CW)  DRV_DIR_High(); 	//по часовой стрелке
	else if(dir == MOTOR_DIR_INV) DRV_DIR_Toggel(); //Инверсия Направления вращения.
}
//**********************************************************
//Реализация управления мотора по статье
//"AVR446 Линейное управление скоростью шагового двигателя"
//http://avrdoc.narod.ru/index/0-7
//https://embeddeddesign.org/stepper-motor-controller/ - вариант реализации

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
void _motor_SpeedCntrMove(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed){

	uint32_t max_s_lim;	// ! Number of steps before we hit max speed.
	uint32_t accel_lim;	// ! Number of steps before we must start deceleration (if accel does not hit max speed).
	//----------------------
	//Направление вращения определяется в ф-ии MOTOR_Rotation.
//	if(step < 0) //По часовой стрелке
//	{
//		step = -step;
//		_motor_Direction(MOTOR_CW);
//	}
//	else _motor_Direction(MOTOR_CCW); //против часовой стрелки

	if(step < 0) step = -step; //Уберем знак.
	targetSteps = step;		   //Сохраним сколько всего нам нужно сделать шагов.
	//----------------------
	// If moving only 1 step.
	if(step == 1)
	{
		speedRamp.accel_count = -1;    // Move one step...
		speedRamp.run_state   = DECEL; // ...in DECEL state.
		// Just a short delay so main() can act on 'running'.
		//srd.step_delay = 1000;

		//DRV_EN_Low();    //включение драйвера.
		//DRV_RESET_Low(); //сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
		_motor_TimerEnable(); //Counter enable
	}
	//----------------------
	// Only move if number of steps to move is not zero.
	else if(step != 0) //здесь step положительное число
	{
		// Refer to documentation for detailed information about these calculations.

		/*
		 * Set max speed limit, by calc min_delay to use in timer.
		 * min_delay = (alpha / tt) / w
		 * Расчет минимальной задержки, т.е. задержка для максимальной скорости.
		 */
		//speedRamp.min_delay = A_T_x100 / speed; //Верно
		speedRamp.min_delay = (A_T_x100 + speed/2) / speed; //так точнее

		/*
		 * Set accelration by calc the first (c0) step delay.
		 * step_delay = 1/tt * sqrt(2*alpha/accel)
		 * step_delay = ( tfreq* 0.676 / 100 ) * 100 * sqrt((2 * alpha * 10000000000) / (accel * 100)) / 10000
		 * Расчет значения задержки с0.
		 */
		uint32_t a_sq = (A_SQ + accel/2) / accel;
		speedRamp.step_delay = (T1_FREQ_148 * sqrtf(a_sq) + 50) / 100;//так точнее

		/*
		 * Find out after how many steps does the speed hit the max speed limit.
		 * max_s_lim = speed^2 / (2*alpha*accel)
		 * max_s_lim – число шагов, необходимое для достижения требуемой скорости, т.е. кол-во шагов за время ускорения.
		 */
		uint32_t alphaAccel = (A_x20000 * accel + 50) / 100;
		max_s_lim = (uint32_t)(((uint64_t)speed * speed + alphaAccel/2) / alphaAccel); //так точнее

		/*
		 * If we hit max speed limit before 0,5 step it will round to 0.
		 * But in practice we need to move atleast 1 step to get any speed at all.
		 */
		if(max_s_lim == 0) max_s_lim = 1;

		/*
		 * Find out after how many steps we must start deceleration.
		 * n1 = (n1+n2)decel / (accel + decel)
		 * accel_lim – число шагов до начала замедления (игнорирование требуемой скорости).
		 */
		//accel_lim = ((long)step * decel) / (accel + decel);
		uint32_t summAccelDecel = accel + decel;							     //так точнее
		accel_lim = (uint32_t)(((uint64_t)step * decel + summAccelDecel/2) / summAccelDecel);//

		//We must accelrate at least 1 step before we can start deceleration.
		if(accel_lim == 0) accel_lim = 1;

		//Use the limit we hit first to calc decel.
		if(accel_lim <= max_s_lim) speedRamp.decel_val = -(step - accel_lim);
		else					   speedRamp.decel_val = -((max_s_lim * accel + decel/2) / decel);

		//We must decelrate at least 1 step to stop.
		if(speedRamp.decel_val == 0) speedRamp.decel_val = -1;

		//Find step to start decleration.
		speedRamp.decel_start = step + speedRamp.decel_val;

		//If the maximum speed is so low that we dont need to go via accelration state.
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
		//MOTOR_DriverReset(); //сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
		DRV_STEP_Low();			 		      	   //Ножку STEP к земле.
		DRV_EN_Low();  			 		  	  	   //Вкл. драйвера.
		stepCount = 0;					      	   //сброс общего счетчика шагов
		_motor_SetTimerCount(speedRamp.step_delay);//задержка с0 перед первым шагом.
		_motor_TimerEnable();					   //пуск таймера.
	}
	//----------------------
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void){

	//Инициализация портов микроконтроллера.
//	STM32_GPIO_InitForOutputPushPull(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN);
//	STM32_GPIO_InitForOutputPushPull(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN);
//	STM32_GPIO_InitForOutputPushPull(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN);
//	STM32_GPIO_InitForOutputPushPull(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN);

	//Выводы управления драйвером мотора.
	STM32_GPIO_InitForOutputPushPull(DRV_MODE2_GPIO_PORT, DRV_MODE2_GPIO_PIN);// DRV_MODE2 - PC13
	STM32_GPIO_InitForOutputPushPull(DRV_MODE1_GPIO_PORT, DRV_MODE1_GPIO_PIN);// DRV_MODE1 - PC14
	STM32_GPIO_InitForOutputPushPull(DRV_MODE0_GPIO_PORT, DRV_MODE0_GPIO_PIN);// DRV_MODE0 - PC15
	STM32_GPIO_InitForOutputPushPull(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN);  // DRV_STEP - PA0
	STM32_GPIO_InitForOutputPushPull(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN);      // /DRV_EN - PA1
	STM32_GPIO_InitForOutputPushPull(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN);	  // DRV_DIR - PA4
	STM32_GPIO_InitForOutputPushPull(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN);// /DRV_RESET - PA7

	//Таймер отсчета временных интервалов расчета скорости.
	_motor_TimerInit(MOTOR_TIMER_FREQ_Hz);

	//Заводские настройки.
	MOTOR_TorqueControl(MOTOR_TORQUE_OFF);			   //Откл. момент удержания.
	MOTOR_DriverReset();							   //сброс внутренней логики драйвера мотора DRV8825.
	MOTOR_SetMicrostepMode(MOTOR_DEFAULT_MICROSTEP);   //Режим микрошаги
	reducerRate    		=  MOTOR_DEFAULT_REDUCER_RATE; //передаточное число редуктора.
	accelTime      		=  MOTOR_DEFAULT_ACCEL_TIME_mS;//время ускорения в милисекундах
	targetVelocity 		=  MOTOR_DEFAULT_RPM;		   //cкорость, оборотов в минуту (RPM)
	speedRamp.run_state = STOP;						   //машина состояния
}
//**********************************************************
//Задать микрошаг
void MOTOR_SetMicrostepMode(MotorStepMode_t uSteps){

	if(speedRamp.run_state != STOP) return;//при вращении ничего не делаем и выходим
	uStepsMode = uSteps;
	switch(uSteps){
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
	uStepsPerTurn = MOTOR_FULL_STEPS_PER_TURN * uStepsMode;//кол-во шагов мотора на один оборот с учетом выбранного режима микрошага.
	ALPHA    =  _2PI / uStepsPerTurn;									   //Угол (в рад.) поворота вала мотора при одном шаге.
	A_T_x100 = (_2PI * T1_FREQ * 100 + uStepsPerTurn/2) / uStepsPerTurn;   //Максимальная скорость (в рад/сек) при заданной частоте тактирования T1_FREQ таймера
	A_SQ     = (_2PI * 2 * 10000000000 + uStepsPerTurn/2) / uStepsPerTurn; //Константа для расчета задержки с0
	A_x20000 = (_2PI * 20000 + uStepsPerTurn/2) / uStepsPerTurn;		   //???
}
//**********************************************************
//передаточное чисо редуктора
void MOTOR_SetReducerRate(uint32_t rate){

	if(speedRamp.run_state != STOP) return;//при вращении ничего не делаем и выходим
	if(rate > MOTOR_REDUCER_RATE_MAX) rate = MOTOR_REDUCER_RATE_MAX;
	if(rate < MOTOR_REDUCER_RATE_MIN) rate = MOTOR_REDUCER_RATE_MIN;
	reducerRate = rate;
}
//**********************************************************
//время ускорения/замедлениия в мС
void MOTOR_SetAccelerationTime(uint32_t accelTime_mS){

	if(speedRamp.run_state != STOP) return;//при вращении ничего не делаем и выходим
	if(accelTime_mS > MOTOR_ACCEL_TIME_mS_MAX)  accelTime_mS = MOTOR_ACCEL_TIME_mS_MAX;
	if(accelTime_mS < MOTOR_ACCEL_TIME_mS_MIN )	accelTime_mS = MOTOR_ACCEL_TIME_mS_MIN;
	accelTime = accelTime_mS;
}
//**********************************************************
//скорость в RPM
void MOTOR_SetVelocity(uint32_t vel){

	if(speedRamp.run_state != STOP)return; //при вращении ничего не делаем и выходим
	if(vel > MOTOR_RPM_MAX) vel = MOTOR_RPM_MAX;
	if(vel < MOTOR_RPM_MIN) vel = MOTOR_RPM_MIN;
	targetVelocity = vel;
}
//**********************************************************
//Задать угол поворота вала мотора.
void MOTOR_SetTargetPosition(int32_t angle){

	if(speedRamp.run_state != STOP) return; //при вращении ничего не делаем и выходим

	motorFlags.f_Pausa        = FLAG_CLEAR; //Сброс переменных Паузы/Запуска варщения
	motorFlags.f_PausaWaiting = FLAG_CLEAR; //при установке нового угла поворота
	motorFlags.f_Start        = FLAG_CLEAR;

	targetAngle = angle;
}
//*******************************************************************************************
//*******************************************************************************************
MotorStepMode_t MOTOR_GetMicrostepMode(void){

	return uStepsMode;
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

	return (uint32_t)(((uint64_t)stepCount * degreeInOneStepX1e8 + 50000) / 100000);
}
//**********************************************************
MotorState_t MOTOR_GetSpeedRampState(void){

	return speedRamp.run_state;
}
//*******************************************************************************************
//*******************************************************************************************
//сброс внутренней логики драйвера мотора DRV8825.
void MOTOR_DriverReset(void){

	DRV_RESET_Low();
	DELAY_uS(10);
	DRV_RESET_High();
}
//**********************************************************
void MOTOR_DriverDisable(void){

	_motor_TimerDisable();
	DRV_STEP_Low();
	DRV_RESET_Low();
	DRV_EN_High();	  //Отключения драйвера.
}
//**********************************************************
void MOTOR_DriverEnable(void){

	_motor_TimerEnable();
	DRV_EN_Low();	  //Включение драйвера.
}
//**********************************************************
//Управление моментом удержания ШД.
void MOTOR_TorqueControl(uint32_t state){

	torqueState = state;
	if(state == MOTOR_TORQUE_ON) DRV_EN_Low();  //Вкл. драйвера.
	else				   		 DRV_EN_High(); //Откл. драйвера.
}
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_CalcAndStartRotation(void){

	uint32_t uSteps        = uStepsPerTurn  * reducerRate; //кол-во микрошагов на оборот с учетом режима микрошагов и передаточного числа редуктора.
	uint32_t speedWithRate = targetVelocity * reducerRate; //скорость с учетом передаточного числа редуктора.

	degreeInOneStepX1e8  =  (uint32_t)((degree360x1e8 + uSteps/2) / uSteps); //кол-во градусов в одном шаге *1е8. Умножение на 1е8 нужно для увеличения точности расчетов.
	stepsInOneDegreeX1e6 = ((uint64_t)uSteps * 1000000 + 360000/2) / 360000; //кол-во шагов в одном градусе *1000000. Умножение на 1000000 нужно для увеличения точности расчетов.

	//кол-во шагов необходимое для перемещение на угол targetAngle, заданный в 0.001 градуса.
	int64_t stepsX1e6 = (int64_t)targetAngle * stepsInOneDegreeX1e6;
	int32_t steps     = (int32_t)((stepsX1e6 + 500000) / 1000000);

	//--------------------------
	//Обработка Паузы вращения/Возобновление вращения
	//Пауза во вращении.
	if(motorFlags.f_Pausa == FLAG_SET)
	{
		motorFlags.f_Pausa        = FLAG_CLEAR;
		motorFlags.f_PausaWaiting = FLAG_SET;
		steps = pausa_reverseSteps;		 //Шаги назад, что бы вернуться в момент паузы.
		_motor_Direction(MOTOR_DIR_INV); //Инверсия направления вращения.
	}
	//Снятие с паузы
	else if(motorFlags.f_Start == FLAG_SET)
	{
		motorFlags.f_Start = FLAG_CLEAR;
		//кол-во шагов, которое нужно сделать от положения паузы,
		//до целевого положения.
		steps = pausa_stepsToTarget;
		_motor_Direction(MOTOR_DIR_INV); //Инверсия направления вращения.
	}
	//--------------------------
	//Определение направления вращения.
	else if(steps < 0) _motor_Direction(MOTOR_DIR_CW);  //по часовой стрелке
	else 			   _motor_Direction(MOTOR_DIR_CCW); //против часовой стрелки

	//Преобразование скорости из об/мин в рад/сек (0.01*rad/sec)
	//uint32_t speed = (uint32_t)(targetVelocity * reducerRate * radSec);
	uint32_t speed = (uint32_t)((speedWithRate * _2PI * 100 + 30) / 60);//так точнее

	//Пересчет ускорения заданного в мс в рад. в сек. за сек. (0.01*rad/sec^2.)
	const float radSecSec = _2PI / 3600 * 100;       // in 0.01*rad/sec^2. 3600 - это 60 секунд в квадрате.
	float accelSec = (float)accelTime / 1000 / 60.0; //
	uint32_t accel = (uint32_t)(((float)speedWithRate * radSecSec + accelSec/2) / accelSec);//так точнее
	//uint32_t accel = (uint32_t)(((float)speedWithRate * 5 * radSecSec + accelSec/2) / accelSec);

	//Расчет значений и пуск вращения.
	_motor_SpeedCntrMove(steps, 	//param step   Number of steps to move
						 accel, 	//param accel  Accelration to use, in 0.01*rad/sec^2.
						(accel*2),  //accel, 	//param decel  Decelration to use, in 0.01*rad/sec^2.
						 speed);	//param speed  Max speed, in 0.01*rad/sec.
	//-----------------------------------------------------
}
//**********************************************************
void MOTOR_PausaRotation(void){

	if(MOTOR_GetSpeedRampState() != STOP) motorFlags.f_Pausa = FLAG_SET;
}
//**********************************************************
uint32_t MOTOR_GetPausaPosition(void){

	return (uint32_t)(((uint64_t)pausa_steps * degreeInOneStepX1e8 + 50000) / 100000);
}
//**********************************************************
void MOTOR_StartRotation(void){

	//Если ждем команду старт, то применяем ее.
	if(motorFlags.f_PausaWaiting == FLAG_SET)
	{
		motorFlags.f_Start        = FLAG_SET;
		motorFlags.f_PausaWaiting = FLAG_CLEAR;
		MOTOR_CalcAndStartRotation();
	}
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//Обработчик прерывания таймера. Тут производятся шаги для управления мотором.
void MOTOR_TimerITHandler(void){

	   	   volatile uint32_t x;
	   	   volatile  int32_t y;
		   volatile uint32_t newStepDelay;   	 // Holds next delay period.
	static volatile  int32_t lastAccelDelay = 0; // Remember the last step delay used when accelrating.
	static volatile uint32_t modulo = 0;   	   	 // остаток от деления. Keep track of remainder from new_step-delay calculation to incrase accurancy
	//-------------------
	DRV_STEP_High();				     	   //Делаем шаг.
	_motor_SetTimerCount(speedRamp.step_delay);//Обновляем значение таймера.
	stepCount++;					 	       //+1 к общему счетчику шагов
	DRV_STEP_Low();
	//Машина состояний рампы разгона/торможения мотора.
	switch(speedRamp.run_state){
		//-------------------
		case STOP:
			_motor_TimerDisable();							  //Откл. таймера.
			if(torqueState == MOTOR_TORQUE_OFF) DRV_EN_High();//Откл. драйвера мотора.
			lastAccelDelay = 0;								  //Сброс рабочих переменных.
			modulo     	   = 0;
			//Если была команда Пауза, то нужно после остановки вернуться
			//в положение, в котором был вал до прихода команды Пауза.
			if(motorFlags.f_Pausa == FLAG_SET)
			{
				//найдем на сколько шагов нужно переместиться назад,
				//чтобы попасть в место, в котором началось замедление.
				pausa_reverseSteps = pausa_steps - stepCount;

				//сохраним кол-во шагов, которое нужно сделать от положения паузы,
				//до целевого положения.
				pausa_stepsToTarget = targetSteps - pausa_steps;

				//тепепь нужно вернуться на нужное кол-во шагов назад.
				MOTOR_CalcAndStartRotation();
				return;
			}
			//сбрасывать внутренние таблицы драйвера не нужно!!!! Так точнее работает!!
			//DRV_RESET_Low();
		break;
		//-------------------
		case ACCEL:
			speedRamp.accel_count++;//+1 к счетчику шагов ускорения

			//Расчет времени до следующего шага.
			x = (2 * speedRamp.step_delay  + modulo);
			y = (4 * speedRamp.accel_count + 1);
			newStepDelay = speedRamp.step_delay - (x / y);
			modulo       = x % y;

			//Замедляемся
			if(stepCount >= speedRamp.decel_start)
			{
				//speedRamp.accel_count = speedRamp.decel_val;
				speedRamp.decel_val = -speedRamp.decel_val;
				speedRamp.run_state = DECEL;
			}
			//Достигли установленной скорости.
			else if(newStepDelay <= speedRamp.min_delay)
			{
				lastAccelDelay      = newStepDelay;
				newStepDelay        = speedRamp.min_delay;
				modulo 			    = 0;
				speedRamp.run_state = RUN;
			}
		break;
		//-------------------
		case RUN:
			newStepDelay = speedRamp.min_delay;//постоянная скорость вращения

			//Условия перехода к замедлению.
			if(stepCount >= speedRamp.decel_start || // Пора переходить к замедлению.
			   motorFlags.f_Pausa == FLAG_SET)       // Пришла команда "пауза вращения".
			{
				//сохраним кол-во пройденых шагов. Это для паузы.
				if(motorFlags.f_Pausa == FLAG_SET) pausa_steps = stepCount;

				//speedRamp.accel_count = speedRamp.decel_val;
				speedRamp.decel_val = -speedRamp.decel_val;

				//Начнем замедление с той же задержкой, на которой закончилось ускорение.
				newStepDelay = lastAccelDelay;
				speedRamp.run_state = DECEL;
			}
		break;
		//-------------------
		case DECEL:
			speedRamp.decel_val--;//-1 к счетчику шагов замедления

			//Расчет времени до следующего шага.
			x = (2 * speedRamp.step_delay + modulo);
			y = (4 * speedRamp.decel_val  + 1);
			newStepDelay = speedRamp.step_delay + (x / y);
			modulo       = x % y;

			//Если сделали последний шаг замедления то СТОП
			if(speedRamp.decel_val <= 0) speedRamp.run_state = STOP;
		break;
		//-------------------
	}
	//значене задержки между шагами. Загружается в счетчик таймера.
	speedRamp.step_delay = newStepDelay;
}
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************











