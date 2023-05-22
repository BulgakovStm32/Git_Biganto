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
void TIM3_InitForPWM(uint32_t freq_Hz){

	//APB1_CLK = 36MHz, TIM3_CLK = APB1_CLK * 2 = 72MHz.
	//таймер будет тактироваться с частотой TIM3_CLK/(PSC - 1).
	uint32_t psc = ((72000000 / freq_Hz + 50)/100) - 1;
	//--------------------------
	//Включение тактирования таймера.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	//Прескаллер.
	TIM3->PSC = psc; //(72 - 1);  //таймер будет тактироваться с частотой 72МГц/PSC.
	TIM3->ARR = (100 - 1);//Auto reload register. - это значение, до которого будет считать таймер.

	//Задаем режим работы - PWM mode on OC1
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | //OC1M:  Output compare 1 mode - 110: PWM mode 1.
				   TIM_CCMR1_OC1PE;						 //OC1PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.

//	//Задаем режим работы - PWM mode on OC2
//	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | //OC2M: Output compare 1 mode - 110: PWM mode 1.
//				   TIM_CCMR1_OC2PE;						 //OC2PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.
//
//	//Задаем режим работы - PWM mode on OC3
//	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | //OC3M: Output compare 1 mode - 110: PWM mode 1.
//				   TIM_CCMR2_OC3PE;						 //OC3PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.
//
//	//Задаем режим работы - PWM mode on OC4
//	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | //OC4M: Output compare 1 mode - 110: PWM mode 1.
//				   TIM_CCMR2_OC4PE;						 //OC4PE: Output compare 1 preload enable. 1: Preload register on TIMx_CCR1 enabled.

	TIM3->CCER |= TIM_CCER_CC1E;//Enable CC1 - включение первого канала
//	TIM3->CCER |= TIM_CCER_CC2E;//Enable CC2 - включение второго канала
//	TIM3->CCER |= TIM_CCER_CC3E;//Enable CC3 - включение третьего канала.
//	TIM3->CCER |= TIM_CCER_CC4E;//Enable CC4 - включение четвертого канала.

	//Настройка ножки микроконтроллера.
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRL |= GPIO_CRL_CNF6_1;//PA6(TIM3_CH1) - выход, альтернативный режим push-pull.
	GPIOA->CRL |= GPIO_CRL_MODE6; //PA6(TIM3_CH1) - тактирование 50МГц.

//	GPIOA->CRL |= GPIO_CRL_CNF7_1;//PA7(TIM3_CH2) - выход, альтернативный режим push-pull.
//	GPIOA->CRL |= GPIO_CRL_MODE7; //PA7(TIM3_CH2) - тактирование 50МГц.

//	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

//	GPIOB->CRL |= GPIO_CRL_CNF0_1;//PB0(TIM3_CH3) - выход, альтернативный режим push-pull.
//	GPIOB->CRL |= GPIO_CRL_MODE0; //PB0(TIM3_CH3) - тактирование 50МГц.

//	GPIOB->CRL |= GPIO_CRL_CNF1_1 | //PB1(TIM3_CH4) - выход, альтернативный режим push-pull.
//				  GPIO_CRL_MODE1;   //PB1(TIM3_CH4) - тактирование 50МГц.

	//Включение DMA для работы с таймером.
//	TIM3->DIER |= TIM_DIER_CC1DE;

	//Установка длительности цикла ШИМ.
	TIM3->CCR1 = 80;
//	TIM3->CCR4 = 10;

	//Включение таймера
	TIM3->CR1 |= TIM_CR1_CEN;
}
//*******************************************************************************************
//*******************************************************************************************
void TIM1_Init(uint32_t freq_Hz){

	//таймер будет тактироваться с частотой 72МГц/(PSC - 1).
	uint32_t psc = (72000000 / freq_Hz) - 1;
	//--------------------------
	//Включение тактирования таймера.
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	//Прескаллер. APB2_CLK = 72MHz, TIM1_CLK = APB2_CLK * 1 = 72MHz.
	//таймер будет тактироваться с частотой 72МГц/(PSC - 1).
	TIM1->PSC = psc;
	//TIM1->PSC = 72-1;		  //таймер будет тактироваться с частотой 1MHz.
	//TIM1->PSC = 144-1;	  //таймер будет тактироваться с частотой 500kHz.
	//TIM1->PSC = 32-1;		  //таймер будет тактироваться с частотой 2MHz.
	//TIM1->ARR  = 100-1; 	  //Auto reload register. - это значение, до которого будет считать таймер.
	//TIM1->CR1 |= TIM_CR1_ARPE;//Auto-reload preload enable
	//Настройка прерываний.
	TIM1->CR1  |= TIM_CR1_URS; //Update Request Source
	TIM1->EGR  |= TIM_EGR_UG;  //Update generation
	TIM1->DIER |= TIM_DIER_UIE;//Update interrupt enable
	//Разрешение прерывания от TIM1.

	//Пример установки приоритетов прерываний.
//		/* 2 bits for pre-emption priority and 2 bits for subpriority */
//		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//
//		/* Set USART1 interrupt preemption priority to 1 */
//		NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
//
//		/* Set SysTick interrupt preemption priority to 3 */
//		NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));

	/* Set TIM1_UP_IRQn interrupt preemption priority to 1 */
//	NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
//	NVIC_EnableIRQ(TIM1_UP_IRQn);
	//Включение таймера
	//TIM1->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
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

//**********************************************************
void TIMx_Enable(TIM_TypeDef *tim){

	tim->CR1 |= TIM_CR1_CEN;//CEN: Counter enable
}
//**********************************************************
void TIMx_Disable(TIM_TypeDef *tim){

	tim->CR1 &= ~TIM_CR1_CEN;//CEN: Counter disable
}
//*******************************************************************************************
//*******************************************************************************************














