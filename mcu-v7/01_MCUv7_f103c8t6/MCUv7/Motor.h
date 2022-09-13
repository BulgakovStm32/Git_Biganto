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
typedef enum uint32_t{
	STEP_ERR = 0,
	STEP_1   = 1,
	STEP_2   = 2,
	STEP_4   = 4,
	STEP_8   = 8,
	STEP_16  = 16,
	STEP_32  = 32,
}MotorStepMode_t;
//-----------------------------------
typedef enum {
	MOTOR_NO_INIT = 0,
	MOTOR_STOP,
	MOTOR_ACCEL,
	MOTOR_RUN,
	MOTOR_DECEL,
}MotorState_t;
//-----------------------------------
#define MOTOR_FREQ_TIM_Hz 			1000000U //частота тактироваия базового таймера
#define MOTOR_QUANT_TIME_mS			1
#define MOTOR_ONE_FULL_STEP_DEGREE	1.8F	 //на столько градусов повернетсвал мотора за один шаг.
#define MOTOR_FULL_STEPS_PER_TURN	(uint32_t)(360.0 / MOTOR_ONE_FULL_STEP_DEGREE)
//*******************************************************************************************
//*******************************************************************************************
void 	 		MOTOR_Init(void);
void 	 		MOTOR_SetMicrostep(MotorStepMode_t steps);
MotorStepMode_t MOTOR_GetMicrostep(void);
void 			MOTOR_Disable(void);
void            MOTOR_Enable(void);


void MOTOR_ResetDrv(void);
void MOTOR_ResetPosition(void);
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
	uint32_t 	run_state : 3;//! What part of the speed ramp we are in.
	uint32_t 	dir : 1;	  //! Direction stepper motor should move.
	uint32_t	step_delay;	  //! Peroid of next timer delay. At start this value set the accelration rate.
	uint32_t	decel_start;  //! What step_pos to start decelaration
	int32_t   	decel_val;	  //! Sets deceleration rate.
	int32_t 	min_delay;	  //! Minimum time delay (max speed)
	int32_t 	accel_count;  //! Counter used when accelerateing/decelerateing to calculate step_delay.
}SpeedRampData_t;
//-----------------------------------

/*! \Brief Frequency of timer1 in [Hz].
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1_FREQ 460750)
#define T1_FREQ 	MOTOR_FREQ_TIM_Hz		//460750

//! Number of (full)steps per round on stepper motor in use.
#define FSPR 		200					//кол-во полных шагов мотора на один оборот
#define MICROSTEPS	32					//количество микрошагов
#define SPR 		(FSPR * MICROSTEPS) //кол-во микрошагов на оборот

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA 		 (2*3.14159/SPR)              // 2*pi/spr
#define A_T_x100	((long)(ALPHA*T1_FREQ*100))   // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100))  // divided by 100 and scaled by 0.676
#define A_SQ 		 (long)(ALPHA*2*10000000000)  // ALPHA*2*10000000000
#define A_x20000 	 (int)(ALPHA*20000)           // ALPHA*20000

#define _2PI		 (float)(2 * 3.14159)
// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

// Direction of stepper motor movement
#define CW  0
#define CCW 1

#define TRUE 1
#define FALSE 0

//*******************************************************************************************
//*******************************************************************************************
void 	 MOTOR_SpeedCntrMove(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);
uint32_t MOTOR_GetSpeedRampState(void);
void 	 MOTOR_TimerITHandler(void);

//*************************************
void MOTOR_CalcAccelDecel(uint32_t step, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS);
void MOTOR_Move(int32_t angle, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS);
void MOTOR_TimerITHandler2(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_MOTOR_H_ */
