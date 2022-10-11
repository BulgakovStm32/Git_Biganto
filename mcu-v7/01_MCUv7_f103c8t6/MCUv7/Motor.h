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
//****************************************************
// Speed ramp states
typedef enum {
//	NO_INIT = 0,
	STOP = 0,
	ACCEL,
	RUN,
	DECEL,
}MotorState_t;
//****************************************************
//частота тактироваия базового таймера в Гц.
#define MOTOR_T_FREQ_Hz 			1000000U
//на столько градусов повернетсвал мотора за один полный шаг.
#define MOTOR_ONE_FULL_STEP_DEGREE	1.8f
//Кол-во полных шагов ШД на оборот.
#define MOTOR_FULL_STEPS_PER_TURN	(uint32_t)(360.0 / MOTOR_ONE_FULL_STEP_DEGREE)

#define MOTOR_QUANT_TIME_mS			1
//-----------------------------------
//Управление моментом.
#define	MOTOR_TORQUE_ON		1
#define	MOTOR_TORQUE_OFF	0

//-----------------------------------
//Заводские настройки.
#define MOTOR_DEFAULT_MICROSTEP			STEP_32	//количество микрошагов
#define MOTOR_DEFAULT_REDUCER_RATE		6		//передаточное чисо редуктора
#define MOTOR_DEFAULT_ACCEL_TIME_mS		1500	//время ускорения/замедлениия в мС
#define MOTOR_DEFAULT_RPM				1		//скорость в RPM

//-----------------------------------
//максимальные и минимальные значения параметров.
#define MOTOR_REDUCER_RATE_MAX			60   	//передаточное чисо редуктора
#define MOTOR_REDUCER_RATE_MIN			1

#define MOTOR_ACCEL_TIME_mS_MAX			10000	//время ускорения/замедлениия в мС
#define MOTOR_ACCEL_TIME_mS_MIN			100

#define MOTOR_RPM_MAX					60		//скорость в RPM
#define MOTOR_RPM_MIN					1
//*******************************************************************************************
//*******************************************************************************************
void MOTOR_Init(void);

void MOTOR_SetMicrostepMode(MotorStepMode_t steps);
void MOTOR_SetReducerRate(uint32_t rate);
void MOTOR_SetAccelerationTime(uint32_t accelTime_mS);
void MOTOR_SetVelocity(uint32_t maxVel);
void MOTOR_SetPosition(int32_t angle);

MotorStepMode_t MOTOR_GetMicrostepMode(void);
uint32_t 		MOTOR_GetReducerRate(void);
uint32_t 		MOTOR_GetAccelerationTime(void);
uint32_t 		MOTOR_GetVelocity(void);
int32_t  		MOTOR_GetPosition(void);

void MOTOR_TorqueControl(uint32_t state);
void MOTOR_DriverReset(void);
void MOTOR_EmergencyStopRotation(void);



void 			MOTOR_Disable(void);
void            MOTOR_Enable(void);
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
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1_FREQ 460750)
#define T1_FREQ 	MOTOR_T_FREQ_Hz		//460750

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define _2PI		 (float)(2 * 3.1415927)
#define T1_FREQ_148 ((uint32_t)((T1_FREQ*0.676)/100))  // divided by 100 and scaled by 0.676


//! Number of (full)steps per round on stepper motor in use.
//#define MICROSTEPS	32									 //количество микрошагов
//#define SPR 		(MOTOR_FULL_STEPS_PER_TURN * MICROSTEPS) //кол-во микрошагов на оборот

//#define ALPHA 		 (_2PI / SPR)            	  // 2*pi/spr
//#define A_T_x100	((long)(ALPHA*T1_FREQ*100))   // (ALPHA / T1_FREQ)*100
//#define A_SQ 		 (long)(ALPHA*2*10000000000)  // ALPHA*2*10000000000
//#define A_x20000 	 (int)(ALPHA*20000)           // ALPHA*20000
//*******************************************************************************************
//*******************************************************************************************
void 	 	 MOTOR_SpeedCntrMove(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
MotorState_t MOTOR_GetSpeedRampState(void);
void 	 	 MOTOR_SetSpeedRampState(MotorState_t state);
void 	 	 MOTOR_TimerITHandler(void);

void 		 MOTOR_StartRotate(void);

//*************************************
void MOTOR_CalcAccelDecel(uint32_t step, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS);
void MOTOR_Move(int32_t angle, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS);
void MOTOR_TimerITHandler2(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_MOTOR_H_ */
