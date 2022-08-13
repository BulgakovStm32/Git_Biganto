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

//*******************************************************************************************
//*******************************************************************************************
typedef enum {
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
	MOTOR_READY,
	MOTOR_CALC_OK,
	MOTOR_ACCEL,
	MOTOR_DESIRED_SPEED,
	MOTOR_DECEL,
	MOTOR_BUSY,
}MotorState_t;
//-----------------------------------
#define MOTOR_FREQ_TIM_Hz 		1000000U //частота тактироваия базового таймера
#define MOTOR_QUANT_TIME		1 		 //период в мс расчета значений скорости ускорения/замедлениия
#define MOTOR_STEPS_PER_TURN	200		 //количество шагов мотора на один оборот

#define MOTOR_ONE_STEP_DEGREE	1.8f	 //на столько градусов повернетсвал мотора за один шаг.

//*******************************************************************************************
//*******************************************************************************************
void 	 MOTOR_Init(void);
void 	 MOTOR_SetMicrostep(MotorStepMode_t steps);
uint32_t MOTOR_GetMicrostep(void);

uint32_t MOTOR_GetCurrentPosition(void);
uint32_t MOTOR_GetCurrentAcceleration(void);
uint32_t MOTOR_GetCurrentVelocity(void);

void MOTOR_SetTargetPosition(uint32_t position);
void MOTOR_SetMaxAcceleration(uint32_t acceleration);
void MOTOR_SetMaxVelocity(uint32_t vel_rpm);

void MOTOR_ResetDrv(void);
void MOTOR_ResetPosition(void);
//-----------------------------------
void MOTOR_CalculateAccelDecel(uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t timeAccel_ms);
void MOTOR_SpinStart(MotorState_t spinMode);
void MOTOR_AccelDecelLoop(void);


//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_MOTOR_H_ */
