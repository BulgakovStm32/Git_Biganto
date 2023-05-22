
#ifndef _gpio_ST_H
#define _gpio_ST_H
//*******************************************************************************************
//*******************************************************************************************

#include "stm32f103xb.h"

//*******************************************************************************************
//*******************************************************************************************
#define GPIO_POLLING_DELAY  10//50//10

#define PIN_HIGH	1
#define PIN_LOW		0
//**********************************************************
// Макросы
#define GPIO_PIN_High(gpio, pin) 	(gpio->BSRR =  (1 << pin))
#define GPIO_PIN_Low(gpio, pin) 	(gpio->BSRR = ((1 << pin) << 16))
#define GPIO_PIN_Toggel(gpio, pin)	(gpio->ODR ^=  (1 << pin))

//**********************************************************
// LED_PC13 - PC13
//#define LED_PC13_GPIO 	  GPIOC
//#define LED_PC13_PIN	  13
//#define LED_PC13_On()     (LED_PC13_GPIO->BSRR = GPIO_BSRR_BR13)
//#define LED_PC13_Off()    (LED_PC13_GPIO->BSRR = GPIO_BSRR_BS13)
//#define LED_PC13_Toggel() (LED_PC13_GPIO->ODR ^= (1<<LED_PC13_PIN))
//*******************************************************************************************
//*******************************************************************************************
void GPIO_InitForOutputPushPull(GPIO_TypeDef *port, uint32_t pin);
void GPIO_InitForOutputOpenDrain(GPIO_TypeDef *port, uint32_t pin);
void GPIO_InitForInputPullUp(GPIO_TypeDef *port, uint32_t pin);
void GPIO_InitForInputPullDown(GPIO_TypeDef *port, uint32_t pin);
void GPIO_InitForFloatingInput(GPIO_TypeDef *port, uint32_t pin);

void     GPIO_Init     (void);
void     GPIO_CheckLoop(void);
uint32_t GPIO_GetPortState(GPIO_TypeDef *port);
uint32_t GPIO_GetPinState(GPIO_TypeDef *port, uint32_t pin);

//*******************************************************************************************
//*******************************************************************************************
#endif /*_gpio_ST_H*/


