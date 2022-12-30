/*
 * Motor.h
 *
 *  Created on: 5 авг. 2022 г.
 *      Author: belyaev
 */

#ifndef MCUV7_MOTOR_H
#define MCUV7_MOTOR_H
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"
//#include "math.h"

//*******************************************************************************************
//*******************************************************************************************
#define MOTOR_TIMER					TIM1		  //базовый таймер для генерации шагов
#define MOTOR_TIMER_FREQ_Hz			(1000 * 1000U)//частота тактироваия базового таймера в Гц.

#define MOTOR_ONE_FULL_STEP_DEGREE	1.8f 										  //угол поворота мотора за один полный шаг.
#define MOTOR_FULL_STEPS_PER_TURN	(uint32_t)(360.0 / MOTOR_ONE_FULL_STEP_DEGREE)//Кол-во полных шагов ШД на оборот.
//**********************************
//Направление вращением
#define	MOTOR_DIR_CCW				0    	//против часовой стрелки
#define	MOTOR_DIR_CW				1	 	//по часовой стрелке
#define	MOTOR_DIR_INV				2		//инвертирование вращения
//**********************************
//Управление моментом.
#define	MOTOR_TORQUE_ON				1		//вкл. момента удержания
#define	MOTOR_TORQUE_OFF			0		//откл. момента удержания.
//**********************************
//Заводские настройки.
#define MOTOR_DEFAULT_MICROSTEP		STEP_32	//количество микрошагов
#define MOTOR_DEFAULT_REDUCER_RATE	6 //120		//передаточное чисо редуктора
#define MOTOR_DEFAULT_ACCEL_TIME_mS	1000	//время ускорения/замедлениия в мС
#define MOTOR_DEFAULT_RPM			5		//скорость в RPM
//**********************************
//максимальные и минимальные значения параметров.
#define MOTOR_REDUCER_RATE_MAX		120  	//передаточное число редуктора
#define MOTOR_REDUCER_RATE_MIN		1

#define MOTOR_ACCEL_TIME_mS_MAX		5000	//время ускорения/замедлениия в мС
#define MOTOR_ACCEL_TIME_mS_MIN		50

#define MOTOR_RPM_MAX				350		//скорость в RPM
#define MOTOR_RPM_MIN				1
//****************************************************
//****************************************************
// Микрошаги.
typedef enum uint32_t{
	STEP_ERR = 0,
	STEP_1   = 1,
	STEP_2   = 2,
	STEP_4   = 4,
	STEP_8   = 8,
	STEP_16  = 16,
	STEP_32  = 32,
}MotorStepMode_t;
//**********************************
// Speed ramp states
typedef enum {
	STOP = 0,
	ACCEL,
	RUN,
	DECEL,
}MotorState_t;
//**********************************
//Флаги питания
typedef struct{
//	uint32_t 	f_DirRotation  : 1; //Направление вращения: 0 - против часовой стрелки
								   //					   1 - по часовой стрелке
	uint32_t 	f_Pausa  	   : 1; //Пауза во вращении.
	uint32_t 	f_PausaWaiting : 1; //Отработали паузу и ждем продолжения вращения(ждем команду Старт)
	uint32_t 	f_Start  	   : 1; //Снятие с паузы (пришла команда Старт).
}MotorFlag_t;
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void);

void MOTOR_SetMicrostepMode(MotorStepMode_t uSteps);
void MOTOR_SetReducerRate(uint32_t rate);
void MOTOR_SetAccelerationTime(uint32_t accelTime_mS);
void MOTOR_SetVelocity(uint32_t vel);
void MOTOR_SetTargetPosition(int32_t angle);

MotorStepMode_t MOTOR_GetMicrostepMode(void);
uint32_t 		MOTOR_GetReducerRate(void);
uint32_t 		MOTOR_GetAccelerationTime(void);
uint32_t 		MOTOR_GetVelocity(void);
int32_t  		MOTOR_GetTargetPosition(void);
uint32_t 		MOTOR_GetMotorPosition(void);
MotorState_t    MOTOR_GetSpeedRampState(void);

void MOTOR_DriverReset(void);
void MOTOR_DriverDisable(void);
void MOTOR_DriverEnable(void);
void MOTOR_TorqueControl(uint32_t state);

void 	 MOTOR_CalcAndStartRotation(void);
void 	 MOTOR_PausaRotation(void);
uint32_t MOTOR_GetPausaPosition(void);
void 	 MOTOR_StartRotation(void);
//*******************************************************************************************
//*******************************************************************************************
//Реализация управления мотора по статье
//"AVR446 Линейное управление скоростью шагового двигателя"
//http://avrdoc.narod.ru/index/0-7
//https://embeddeddesign.org/stepper-motor-controller/ - вариант реализации

/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
	MotorState_t run_state;	  //! What part of the speed ramp we are in.
	uint32_t	 step_delay;  //! Peroid of next timer delay. At start this value set the accelration rate.
	uint32_t	 decel_start; //! What step_pos to start decelaration
	int32_t   	 decel_val;	  //! Sets deceleration rate.
	int32_t 	 min_delay;	  //! Minimum time delay (max speed)
	int32_t 	 accel_count; //! Counter used when accelerateing/decelerateing to calculate step_delay.
}SpeedRampData_t;
//****************************************************
/*! \Brief Frequency of timer1 in [Hz].
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 * Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1_FREQ 460750)
 */
#define T1_FREQ 	MOTOR_TIMER_FREQ_Hz		//460750

//*******************************************************************************************
//*******************************************************************************************
void MOTOR_TimerITHandler(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_MOTOR_H_ */
