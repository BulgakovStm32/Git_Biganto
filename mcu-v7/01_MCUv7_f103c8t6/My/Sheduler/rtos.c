/*
 *
 *
 *    Created on:
 *        Author:
 *  Редактировал: Беляев А.А. 08.02.2022
 *
 * За основу взят планировщик задач с сайта ChipEnable.ru
 * http://chipenable.ru/index.php/programming-avr/item/110-planirovschik.html
 */

#include "rtos.h"

//*******************************************************************************************
//*******************************************************************************************
static volatile task_t   taskArray[MAX_TASKS];// очередь задач
static volatile uint32_t arrayTail = 0;		  // "хвост" очереди
static volatile uint32_t tickCount = 0;		  //

//*******************************************************************************************
//*******************************************************************************************
//Глобальное запрещение прерываний.
__INLINE static void _disableInterrupt(void){

	//__disable_irq();
}
//**********************************************************
//Глобальное разрешение рерываний.
__INLINE static void _enableInterrupt(void){

	//__enable_irq();
}
//*******************************************************************************************
//*******************************************************************************************
/* Инициализация РТОС.
 *
 */
void RTOS_Init(void){

//   TCCR0        |= (1<<CS01)|(1<<CS00);         // прескалер - 64
//   TIFR         = (1<<TOV0);                   // очищаем флаг прерывания таймера Т0
//   TIMSK        |= (1<<TOIE0);                  // разрешаем прерывание по переполнению
//   TIMER_COUNTER = TIMER_START;                 // загружаем начальное зн. в счетный регистр

	arrayTail = 0;//"хвост" в 0
}
//**********************************************************
/* Добавление задачи в список
 *
 */
void RTOS_SetTask(void(*taskFunc)(void), uint32_t taskDelay, uint32_t taskPeriod){
   
	volatile task_t *task = &taskArray[0];
	//-----------------------------
	if(!taskFunc) return;
	//Поиск задачи в текущем списке
	for(uint32_t i = 0; i < arrayTail; i++)
	{
		if(task->pFunc == taskFunc)// если нашли, то обновляем переменные
		{
			_disableInterrupt();//Глобальное запрещение прерываний.
			task->delay  = taskDelay;
			task->period = taskPeriod;
			task->state  = TASK_STOP;
			_enableInterrupt();//Глобальное разрешение рерываний.
			return;
		}
		task++;
	}
	//Если такой задачи в списке нет и есть место, то добавляем в конец массива
	if(arrayTail < MAX_TASKS)
	{
		_disableInterrupt();//Глобальное запрещение прерываний.
		task 		 = &taskArray[arrayTail];
		task->pFunc  = taskFunc;
		task->delay  = taskDelay;
		task->period = taskPeriod;
		task->state  = TASK_STOP;
		arrayTail++;	   // увеличиваем "хвост"
		_enableInterrupt();//Глобальное разрешение рерываний.
	}
}
//**********************************************************
/* Удаление задачи из списка
 *
 */
void RTOS_DeleteTask(void(*taskFunc)(void)){

	for(uint32_t i=0; i < arrayTail; i++)  //проходим по списку задач
	{
		if(taskArray[i].pFunc == taskFunc) //если задача в списке найдена
		{
			_disableInterrupt();  //Глобальное запрещение прерываний.
			if(i != (arrayTail-1))//переносим последнюю задачу на место удаляемой
			{
				taskArray[i] = taskArray[arrayTail-1];
			}
			arrayTail--;	   //уменьшаем указатель "хвоста"
			_enableInterrupt();//Глобальное разрешение рерываний.
			return;
		}
	}
}
//**********************************************************
/* Диспетчер РТОС, вызывается в main
 *
 */
void RTOS_DispatchLoop(void){

	void(*function)(void);
	volatile task_t *task = &taskArray[0];
	//-----------------------------
	//проходим по списку задач
	for(uint32_t i=0; i< arrayTail; i++)
	{
		//если флаг на выполнение взведен,
		if(task->state == TASK_RUN)
		{
			_disableInterrupt();   //Глобальное запрещение прерываний.
			function = task->pFunc;// запоминаем задачу, т.к. во время выполнения может измениться индекс
			if(task->period == 0) RTOS_DeleteTask(task->pFunc);//если период = 0 - удаляем задачу.
			else
			{
				task->state = TASK_STOP;						  //иначе снимаем флаг запуска
				if(task->delay == 0) task->delay = task->period-1;//если задача не изменила задержку задаем ее
															      //задача для себя может сделать паузу   ????????? не понятно
			}
			_enableInterrupt();//Глобальное разрешение рерываний.
			//выполняем задачу
			(*function)();
		}
		task++;
	}
}
//**********************************************************
/* Таймерная служба РТОС (прерывание аппаратного таймера)
 *
 */
//ISR(RTOS_ISR)
//{
//   u08 i;
//
//   TIMER_COUNTER = TIMER_START;                       // задаем начальное значение таймера
//
//   for (i=0; i<arrayTail; i++)                        // проходим по списку задач
//   {
//      if  (TaskArray[i].delay == 0)                   // если время до выполнения истекло
//           TaskArray[i].run = 1;                      // взводим флаг запуска,
//      else TaskArray[i].delay--;                      // иначе уменьшаем время
//   }
//}

void RTOS_TimerServiceLoop(void){

	volatile task_t *task = &taskArray[0];
	//-----------------------------
	tickCount++;
	for(uint32_t i=0; i < arrayTail; i++)//проходим по списку задач
	{
	      if(task->delay == 0) task->state = TASK_RUN;//если время до выполнения истекло взводим флаг запуска,
	    else task->delay--;                           //иначе уменьшаем время
	    task++;

	    //Такая реализация занимает больше места.
//		if(TaskArray[i].delay == 0) TaskArray[i].state = TASK_RUN;
//		else TaskArray[i].delay--;
	}
}
//**********************************************************
/* Значение счетчика тиков RTOS.
 *
 */
uint32_t RTOS_GetTickCount(void){

	return tickCount;
}
//*******************************************************************************************
//*******************************************************************************************



