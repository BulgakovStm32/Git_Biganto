
#ifndef _Scheduler_H
#define _Scheduler_H
//-----------------------------------------------------------------------------

//#include "sys_tick_ST.h"
#include "main.h"

//-----------------------------------------------------------------------------
//Очередь задач.
#define TASK_QUEUE_SIZE		20	  //Макс. число задач в очереди задач.
typedef void(*TaskPTR_t)(void);   //Указатель-пустышка.
//Очередь таймеров.
#define TIMER_TASK_QUEUE_SIZE  TASK_QUEUE_SIZE //
typedef struct{
	//---------
	TaskPTR_t Task; //Указатель на задачу.
	uint32_t  Time; //Выдержка в мс
	//---------
}TaskWithTime_t;
//-----------------------------------------------------------------------------
void Scheduler_Init 	       (void);
void Scheduler_Start	       (void);
void Scheduler_Loop		       (void);//Это вызывается в основном цикле main()
void Scheduler_TimerServiceLoop(void);//Служба таймеров вызывается каждую 1мс в обработчике прерывания от таймера.

uint32_t Scheduler_GetTickCount(void);
void 	 Scheduler_SetTask     (TaskPTR_t task);
void     Scheduler_SetTimerTask(TaskPTR_t task, uint16_t newTime);
//--------------------------------
// Прототипы задач 
void IdleTask(void);
//-----------------------------------------------------------------------------
#endif /*_Scheduler_H*/


