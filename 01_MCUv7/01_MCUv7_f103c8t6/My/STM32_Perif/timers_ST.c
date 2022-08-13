/*
 * timers_ST.c
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: Zver
 */
//*******************************************************************************************
//*******************************************************************************************

#include "timers_ST.h"

//*******************************************************************************************
//*******************************************************************************************
void TIM3_InitForPWM(void){

	//Включение тактирования таймера.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	//Выбор источника тактирования.

	//Прескаллер.
	//APB1_CLK = 36MHz, TIM3_CLK = APB1_CLK * 2 = 72MHz.
	TIM3->PSC = (4 - 1);//таймер будет тактироваться с частотой 72МГц/PSC.
	//Auto reload register. - это значение, до которого будет считать таймер.
	TIM3->ARR = (100 - 1);
	//Задаем режим работы - PWM mode on OC1
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | //OC1M:  Output compare 1 mode - 110: PWM mode 1.
				   TIM_CCMR1_OC1PE;						 //OC1PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.
	//Enable CC1 - включение первого канала
	TIM3->CCER |= TIM_CCER_CC1E;

	//Настройка ножки микроконтроллера.
	//Используется порт PA6(TIM3_CH1)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRL |= GPIO_CRL_CNF6_1;//PA6(TIM3_CH1) - выход, альтернативный режим push-pull.
	GPIOA->CRL |= GPIO_CRL_MODE6; //PA6(TIM3_CH1) - тактирование 50МГц.

	//Включение DMA для работы с таймером.
	//TIM3->DIER |= TIM_DIER_CC1DE;
	//Включение таймера
	TIM3->CR1 |= TIM_CR1_CEN;
}
//*******************************************************************************************
//*******************************************************************************************
void TIM1_Init(void){

	//Включение тактирования таймера.
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	//Прескаллер.
	//APB2_CLK = 72MHz, TIM1_CLK = APB2_CLK * 1 = 72MHz.
	TIM1->PSC = 18-1;//таймер будет тактироваться с частотой 72МГц/(PSC - 1).
	//Auto reload register. - это значение, до которого будет считать таймер.
	TIM1->ARR = 3;

	TIM1->DIER |= TIM_DIER_UIE; //Update interrupt enable
	TIM1->CR1   = TIM_CR1_ARPE ;//Auto-reload preload enable
	//Разрешение прерывания от TIM4.
	NVIC_SetPriority(TIM1_UP_IRQn, 5);
	NVIC_EnableIRQ(TIM1_UP_IRQn);

//	//Задаем режим работы - PWM mode 1 on OC1
//	TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | //OC1M: Output compare 1 mode - 110: PWM mode 1.
//				   TIM_CCMR1_OC2PE;						 //OC1PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.
//	//Enable CC1 - включение первого канала
//	TIM1->CCER |= TIM_CCER_CC2E;
//	//Main output enable.
//	TIM1->BDTR |= TIM_BDTR_AOE;
//
//	//Настройка ножки микроконтроллера.
//	//Используется порт PA9(TIM1_CH2)
//	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
//
//	GPIOA->CRH |= GPIO_CRH_CNF9_1;//PA9(TIM1_CH2) - выход, альтернативный режим push-pull.
//	GPIOA->CRH |= GPIO_CRH_MODE9; //PA9(TIM1_CH2) - тактирование 50МГц.

	//Включение таймера
	TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
}
//*******************************************************************************************
//*******************************************************************************************
void TIM2_Init(void){

	//Включение тактирования таймера.
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//Прескаллер. APB1_CLK = 36MHz, TIM2_CLK = APB1_CLK * 2 = 72MHz.
	TIM2->PSC = (72 - 1); //таймер будет тактироваться с частотой 72МГц/(PSC-1).
	//TIM2->ARR = (100 - 1);//Auto reload register. - это значение, до которого будет считать таймер.

	TIM2->CR1 |= TIM_CR1_ARPE ;//Auto-reload preload enable

	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S); //00: CC1 channel is configured as output.
	//Задаем режим работы - PWM mode on OC1
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | //OC1M:  Output compare 1 mode - 110: PWM mode 1.
				   TIM_CCMR1_OC1PE;//  |					 					//OC1PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.
				   //TIM_CCMR1_OC1FE;						 					//OC1FE: Output Compare 1 fast enable.

	//Enable CC1 - включение первого канала
	TIM2->CCER |= TIM_CCER_CC1E;

	//Режимы работы ШИМ. CMS[1:0]: Center-aligned mode selection
	//TIM2->CR1 &= ~(TIM_CR1_CMS_0 | TIM_CR1_CMS_1);

	//Настройка ножки микроконтроллера.
	//Используется порт PA0(TIM2_CH1)
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOA->CRL |= GPIO_CRL_CNF0_1;//PA0(TIM2_CH1) - выход, альтернативный режим push-pull.
	GPIOA->CRL |= GPIO_CRL_MODE0; //PA0(TIM2_CH1) - тактирование 50МГц.

	//Включение DMA для работы с таймером.
	//TIM3->DIER |= TIM_DIER_CC1DE;

	//TIM2->CCR1 = 50;

	//Включение таймера
	//TIM2->CR1 |= TIM_CR1_CEN;
}
//*******************************************************************************************
//*******************************************************************************************




















