/*
 * main.h
 *
 *  Created on: Dec 18, 2020
 *      Author: belyaev
 */

#ifndef _MAIN_H_
#define _MAIN_H_
//*******************************************************************************************
//*******************************************************************************************
//Drivers.
#include "sys_ST.h"
#include "iwdg_ST.h"
#include "gpio_ST.h"
//#include "sys_tick_ST.h"
//#include "flash_ST.h"
//#include "spi_ST.h"
//#include "i2c_ST.h"
//#include "timers_ST.h"
//#include "dma_ST.h"
//#include "adc_ST.h"
//#include "uart_ST.h"
//-------------------------
//Application
#include "Delay.h"
#include "bootLoader.h"
#include "sharedMemory.h"

#include "GpioMCU.h"
//*******************************************************************************************
//*******************************************************************************************
#define FLAG_SET	1
#define FLAG_CLEAR	0

#define TRUE 		1
#define FALSE 		0
//*******************************************************************************************
//*******************************************************************************************
//extern void Reset_Handler(void);
//int  main(void);
//void SysTick_IT_Handler(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* CORE_INC_MAIN_H_ */











