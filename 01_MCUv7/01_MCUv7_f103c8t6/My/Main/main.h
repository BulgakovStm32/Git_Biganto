/*
 * main.h
 *
 *  Created on: Dec 18, 2020
 *      Author: belyaev
 */
//*******************************************************************************************
//*******************************************************************************************
#ifndef _MAIN_H_
#define _MAIN_H_
//-------------------------
//Hardware.
#include "stm32f103xb.h"

#include "math.h"
//-------------------------
//Drivers.
#include "sys_ST.h"
#include "gpio_ST.h"
#include "sys_tick_ST.h"
#include "spi_ST.h"
#include "i2c_ST.h"
#include "timers_ST.h"
#include "dma_ST.h"
#include "adc_ST.h"
#include "uart_ST.h"
//-------------------------
//Планировщик
//#include "Scheduler.h"
#include "rtos.h"
//-------------------------
//Middle level.
//#include "Encoder.h"
//-------------------------
//Application
#include "Delay.h"
#include "LcdGraphic.h"
//#include "Lcd_i2c_1602.h"
#include "Blink.h"
#include "DS18B20.h"
//#include "Filters.h"
//#include "AHT10.h"
#include "DS2782.h"
#include "Encoder.h"

#include "ssd1306.h"
#include "MyString.h"
//*******************************************************************************************
//*******************************************************************************************



//*******************************************************************************************
//*******************************************************************************************
#endif /* CORE_INC_MAIN_H_ */
