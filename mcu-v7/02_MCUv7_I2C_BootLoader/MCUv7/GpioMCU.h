/*
 * GpioMCU.h
 *
 *  Created on: 11 янв. 2023 г.
 *      Author: belyaev
 */

#ifndef MCUV7_GPIOMCU_H_
#define MCUV7_GPIOMCU_H_
//*******************************************************************************************
//*******************************************************************************************

//#include "main.h"
#include "gpio_ST.h"

//*******************************************************************************************
//*******************************************************************************************
// MCU_EN - PB2
#define MCU_EN_GPIO_PORT		GPIOB
#define MCU_EN_GPIO_PIN			2
#define MCU_POWER_ON()	   		GPIO_PIN_High(MCU_EN_GPIO_PORT, MCU_EN_GPIO_PIN)   //(MCU_EN_GPIO_PORT->BSRR = GPIO_BSRR_BS2)
#define MCU_POWER_OFF()    		GPIO_PIN_Low(MCU_EN_GPIO_PORT, MCU_EN_GPIO_PIN)    //(MCU_EN_GPIO_PORT->BSRR = GPIO_BSRR_BR2)
#define MCU_EN_Toggel() 		GPIO_PIN_Toggel(MCU_EN_GPIO_PORT, MCU_EN_GPIO_PIN) //(MCU_EN_GPIO_PORT->ODR ^= GPIO_ODR_ODR2)
// PWR_BTN_LED - PB9
#define PWR_BTN_LED_GPIO_PORT 	GPIOB
#define PWR_BTN_LED_GPIO_PIN  	9
#define PWR_BTN_LED_High()   	GPIO_PIN_High(PWR_BTN_LED_GPIO_PORT, PWR_BTN_LED_GPIO_PIN)  //(GPIOB->BSRR = GPIO_BSRR_BS9)
#define PWR_BTN_LED_Low()    	GPIO_PIN_Low(PWR_BTN_LED_GPIO_PORT, PWR_BTN_LED_GPIO_PIN)   //(GPIOB->BSRR = GPIO_BSRR_BR9)
#define PWR_BTN_LED_Toggel() 	GPIO_PIN_Toggel(PWR_BTN_LED_GPIO_PORT, PWR_BTN_LED_GPIO_PIN)//(GPIOB->ODR ^= GPIO_ODR_ODR9)
// MCU_PWR_PBN - PB7 - вход. Кнопка включения питания.
#define MCU_PWR_BTN_GPIO_PORT	GPIOB
#define MCU_PWR_BTN_GPIO_PIN	7








// LED_ACT - PA15
#define LED_ACT_GPIO_PORT		GPIOA
#define LED_ACT_GPIO_PIN		15
#define LED_ACT_High()   		GPIO_PIN_High(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN)  //(LED_ACT_GPIO_PORT->BSRR = GPIO_BSRR_BS15)
#define LED_ACT_Low()    		GPIO_PIN_Low(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN)   //(LED_ACT_GPIO_PORT->BSRR = GPIO_BSRR_BR15)
#define LED_ACT_Toggel() 		GPIO_PIN_Toggel(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN)//(LED_ACT_GPIO_PORT->ODR ^= GPIO_ODR_ODR15)
// GPS_EN - PB13
#define GPS_EN_GPIO_PORT		GPIOB
#define GPS_EN_GPIO_PIN			13
#define GPS_EN_High()   		GPIO_PIN_High(GPS_EN_GPIO_PORT, GPS_EN_GPIO_PIN)  //(GPIOB->BSRR = GPIO_BSRR_BS13)
#define GPS_EN_Low()    		GPIO_PIN_Low(GPS_EN_GPIO_PORT, GPS_EN_GPIO_PIN)   //(GPIOB->BSRR = GPIO_BSRR_BR13)
#define GPS_EN_Toggel() 		GPIO_PIN_Toggel(GPS_EN_GPIO_PORT, GPS_EN_GPIO_PIN)//(GPIOB->ODR ^= GPIO_ODR_ODR13)
// LIDAR_EN - PB5
#define LIDAR_EN_GPIO_PORT		GPIOB
#define LIDAR_EN_GPIO_PIN		5
#define LIDAR_EN_High()   		GPIO_PIN_High(LIDAR_EN_GPIO_PORT, LIDAR_EN_GPIO_PIN)  //(GPIOB->BSRR = GPIO_BSRR_BS5)
#define LIDAR_EN_Low()    		GPIO_PIN_Low(LIDAR_EN_GPIO_PORT, LIDAR_EN_GPIO_PIN)   //(GPIOB->BSRR = GPIO_BSRR_BR5)
#define LIDAR_EN_Toggel() 		GPIO_PIN_Toggel(LIDAR_EN_GPIO_PORT, LIDAR_EN_GPIO_PIN)//(GPIOB->ODR ^= GPIO_ODR_ODR5)
// BB_PWR_OK - PA12
#define BB_PWR_OK_GPIO_PORT		GPIOA
#define BB_PWR_OK_GPIO_PIN		12
// BB_PWR_BTN - PA8
#define BB_PWR_BTN_GPIO_PORT 	GPIOA
#define BB_PWR_BTN_GPIO_PIN	 	8
#define BB_PWR_BTN_High()   	GPIO_PIN_High(BB_PWR_BTN_GPIO_PORT, BB_PWR_BTN_GPIO_PIN)  //(GPIOA->BSRR = GPIO_BSRR_BS8)
#define BB_PWR_BTN_Low()    	GPIO_PIN_Low(BB_PWR_BTN_GPIO_PORT, BB_PWR_BTN_GPIO_PIN)   //(GPIOA->BSRR = GPIO_BSRR_BR8)
#define BB_PWR_BTN_Toggel() 	GPIO_PIN_Toggel(BB_PWR_BTN_GPIO_PORT, BB_PWR_BTN_GPIO_PIN)//(GPIOA->ODR ^= GPIO_ODR_ODR8)
// BB_PWR_EN - PA11
#define BB_PWR_EN_GPIO_PORT		GPIOA
#define BB_PWR_EN_GPIO_PIN		11
#define BB_PWR_EN_High()   		GPIO_PIN_High(BB_PWR_EN_GPIO_PORT, BB_PWR_EN_GPIO_PIN)  //(GPIOA->BSRR = GPIO_BSRR_BS8)
#define BB_PWR_EN_Low()    		GPIO_PIN_Low(BB_PWR_EN_GPIO_PORT, BB_PWR_EN_GPIO_PIN)   //(GPIOA->BSRR = GPIO_BSRR_BR8)
#define BB_PWR_EN_Toggel() 		GPIO_PIN_Toggel(BB_PWR_EN_GPIO_PORT, BB_PWR_EN_GPIO_PIN)//(GPIOA->ODR ^= GPIO_ODR_ODR8)
// FAN_EN - PB12
#define FAN_EN_GPIO_PORT		GPIOB
#define FAN_EN_GPIO_PIN	 		12
#define FAN_EN_High()   		GPIO_PIN_High(FAN_EN_GPIO_PORT, FAN_EN_GPIO_PIN)  //(GPIOB->BSRR = GPIO_BSRR_BS12)
#define FAN_EN_Low()    		GPIO_PIN_Low(FAN_EN_GPIO_PORT, FAN_EN_GPIO_PIN)   //(GPIOB->BSRR = GPIO_BSRR_BR12)
#define FAN_EN_Toggel() 		GPIO_PIN_Toggel(FAN_EN_GPIO_PORT, FAN_EN_GPIO_PIN)//(GPIOB->ODR ^= GPIO_ODR_ODR12)
// LAMP_PWM - PB1
#define LAMP_PWM_GPIO_PORT 		GPIOB
#define LAMP_PWM_GPIO_PIN  		1
#define LAMP_PWM_High()   		GPIO_PIN_High(LAMP_PWM_GPIO_PORT, LAMP_PWM_GPIO_PIN)  //(GPIOB->BSRR = GPIO_BSRR_BS1)
#define LAMP_PWM_Low()     		GPIO_PIN_Low(LAMP_PWM_GPIO_PORT, LAMP_PWM_GPIO_PIN)   //(GPIOB->BSRR = GPIO_BSRR_BR1)
#define LAMP_PWM_Toggel() 		GPIO_PIN_Toggel(LAMP_PWM_GPIO_PORT, LAMP_PWM_GPIO_PIN)//(GPIOB->ODR ^= GPIO_ODR_ODR1)
//************************************
// Линия упраяления драйвером мотора DRV8825
// DRV_MODE2 - PC13
#define DRV_MODE2_GPIO_PORT 	GPIOC
#define DRV_MODE2_GPIO_PIN  	13
#define DRV_MODE2_High()   		GPIO_PIN_High(DRV_MODE2_GPIO_PORT, DRV_MODE2_GPIO_PIN)  //(GPIOC->BSRR = GPIO_BSRR_BS13)
#define DRV_MODE2_Low()    		GPIO_PIN_Low(DRV_MODE2_GPIO_PORT, DRV_MODE2_GPIO_PIN)   //(GPIOC->BSRR = GPIO_BSRR_BR13)
#define DRV_MODE2_Toggel() 		GPIO_PIN_Toggel(DRV_MODE2_GPIO_PORT, DRV_MODE2_GPIO_PIN)//(GPIOC->ODR ^= GPIO_ODR_ODR13)
// DRV_MODE1 - PC14
#define DRV_MODE1_GPIO_PORT 	GPIOC
#define DRV_MODE1_GPIO_PIN  	14
#define DRV_MODE1_High()   		GPIO_PIN_High(DRV_MODE1_GPIO_PORT, DRV_MODE1_GPIO_PIN)  //(GPIOC->BSRR = GPIO_BSRR_BS14)
#define DRV_MODE1_Low()    		GPIO_PIN_Low(DRV_MODE1_GPIO_PORT, DRV_MODE1_GPIO_PIN)   //(GPIOC->BSRR = GPIO_BSRR_BR14)
#define DRV_MODE1_Toggel() 		GPIO_PIN_Toggel(DRV_MODE1_GPIO_PORT, DRV_MODE1_GPIO_PIN)//(GPIOC->ODR ^= GPIO_ODR_ODR14)
// DRV_MODE0 - PC15
#define DRV_MODE0_GPIO_PORT 	GPIOC
#define DRV_MODE0_GPIO_PIN  	15
#define DRV_MODE0_High()   		GPIO_PIN_High(DRV_MODE0_GPIO_PORT, DRV_MODE0_GPIO_PIN)  //(GPIOC->BSRR = GPIO_BSRR_BS15)
#define DRV_MODE0_Low()    		GPIO_PIN_Low(DRV_MODE0_GPIO_PORT, DRV_MODE0_GPIO_PIN)   //(GPIOC->BSRR = GPIO_BSRR_BR15)
#define DRV_MODE0_Toggel() 		GPIO_PIN_Toggel(DRV_MODE0_GPIO_PORT, DRV_MODE0_GPIO_PIN)//(GPIOC->ODR ^= GPIO_ODR_ODR15)
// DRV_STEP - PA0
#define DRV_STEP_GPIO_PORT		GPIOA
#define DRV_STEP_GPIO_PIN		0
#define DRV_STEP_High()   		GPIO_PIN_High(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN)  //(GPIOA->BSRR = GPIO_BSRR_BS0)
#define DRV_STEP_Low()    		GPIO_PIN_Low(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN)   //(GPIOA->BSRR = GPIO_BSRR_BR0)
#define DRV_STEP_Toggel() 		GPIO_PIN_Toggel(DRV_STEP_GPIO_PORT, DRV_STEP_GPIO_PIN)//(GPIOA->ODR ^= GPIO_ODR_ODR0)
// /DRV_EN - PA1
#define DRV_EN_GPIO_PORT		GPIOA
#define DRV_EN_GPIO_PIN			1
#define DRV_EN_High()   		GPIO_PIN_High(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN)  //(GPIOA->BSRR = GPIO_BSRR_BS1)
#define DRV_EN_Low()    		GPIO_PIN_Low(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN)   //(GPIOA->BSRR = GPIO_BSRR_BR1)
#define DRV_EN_Toggel() 		GPIO_PIN_Toggel(DRV_EN_GPIO_PORT, DRV_EN_GPIO_PIN)//(GPIOA->ODR ^= GPIO_ODR_ODR1)
// DRV_DIR - PA4
#define DRV_DIR_GPIO_PORT		GPIOA
#define DRV_DIR_GPIO_PIN		4
#define DRV_DIR_High()   		GPIO_PIN_High(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN)  //(GPIOA->BSRR = GPIO_BSRR_BS4)
#define DRV_DIR_Low()    		GPIO_PIN_Low(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN)   //(GPIOA->BSRR = GPIO_BSRR_BR4)
#define DRV_DIR_Toggel() 		GPIO_PIN_Toggel(DRV_DIR_GPIO_PORT, DRV_DIR_GPIO_PIN)//(GPIOA->ODR ^= GPIO_ODR_ODR4)
// /DRV_RESET - PA7
#define DRV_RESET_GPIO_PORT		GPIOA
#define DRV_RESET_GPIO_PIN		7
#define DRV_RESET_High()   		GPIO_PIN_High(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN)  //(GPIOA->BSRR = GPIO_BSRR_BS7)
#define DRV_RESET_Low()    		GPIO_PIN_Low(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN)   //(GPIOA->BSRR = GPIO_BSRR_BR7)
#define DRV_RESET_Toggel() 		GPIO_PIN_Toggel(DRV_RESET_GPIO_PORT, DRV_RESET_GPIO_PIN)//(GPIOA->ODR ^= GPIO_ODR_ODR7)

//*******************************************************************************************
//*******************************************************************************************
void 	 GPIO_MCU_Init(void);
uint32_t GPIO_MCU_GetPwrButton(void);

//*******************************************************************************************
//*******************************************************************************************
#endif /* MCUV7_GPIOMCU_H_ */













