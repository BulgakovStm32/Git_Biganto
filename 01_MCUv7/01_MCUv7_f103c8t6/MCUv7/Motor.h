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
	MOTOR_STOP,
	MOTOR_ACCEL,
	MOTOR_RUN,
	MOTOR_DECEL,
}MotorState_t;
//-----------------------------------
typedef struct{
	MotorState_t state;
	uint32_t 	 spsVstart;
	uint32_t 	 spsVmax;
	int32_t  	 allSteps;

	uint32_t 	 accelSteps;
	uint32_t 	 runSteps;
	uint32_t 	 decelSteps;

	uint32_t 	 stepCount;

}StepMotorRampData_t;
//-----------------------------------
#define MOTOR_FREQ_TIM_Hz 			1000000U //частота тактироваия базового таймера
#define MOTOR_QUANT_TIME_mS			1 		 //период в мС расчета значений скорости ускорения/замедлениия

#define MOTOR_ONE_FULL_STEP_DEGREE	1.8f	 //на столько градусов повернетсвал мотора за один шаг.
#define MOTOR_FULL_STEPS_PER_TURN	(uint32_t)(360.0 / MOTOR_ONE_FULL_STEP_DEGREE)
//#define MOTOR_FULL_STEPS_PER_TURN	200		 //количество шагов мотора на один оборот

//*******************************************************************************************
//*******************************************************************************************
void 	 MOTOR_Init(void);
void 	 MOTOR_SetMicrostep(MotorStepMode_t steps);
MotorStepMode_t MOTOR_GetMicrostep(void);

void MOTOR_ResetDrv(void);
void MOTOR_ResetPosition(void);
//-----------------------------------
void	 MOTOR_CalcAccelDecel(int32_t step, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS);
void     MOTOR_Move(int32_t angle, uint32_t Vstart_rpm, uint32_t Vmax_rpm, uint32_t accelTime_mS);
void 	 MOTOR_AccelDecelLoop(void);


//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_MOTOR_H_ */
