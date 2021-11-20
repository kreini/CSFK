/*****************************************************/
/***  PWM with TIM2 using 3 channel *** STM32F072  ***/
/*****************************************************/
/****************  Gabor Kreinicker  *****************/
/*****************************************************/


// includes, defines and global variables

#define		F_CPU			48000000
#define		_GNU_SOURCE
#define		stm32_gpio_alternate(GPIOx,n,f)		\
			do {						\
				stm32_gpio_mode((GPIOx),(n),2);	\
				stm32_gpio_af((GPIOx),(n),(f));	\
			} while(0)
#define		delta			0.006289475
#define		t_pi			6.283185307

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include "math.h"
#include "delay_basic.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "config.h"

float order_1[999];
float order_2[999];
float order_3[999];


// "delay" function

static inline void msleep (int millisec) {
	while (0 < millisec) {
	_delay_loop(F_CPU/8000L);
	millisec--;
	};
}


// "write" function via UART

ssize_t usart_write(void *cookie, const char *buff, size_t size) {
	USART_TypeDef *USART = cookie;
	ssize_t ssize;
	ssize = 0;
	while (size > 0) {
		while (!(USART->ISR & USART_ISR_TXE));
		USART->TDR = *buff;
		buff++;
		size--;
		ssize++;
	};
	return(ssize);
}


// pulse signal

void pulse(void) {
	for (int i = 0; i < 999; i++) {
		TIM2->CCR4 = order_3[i];			// set dutycycle for CH4
		TIM2->CCR3 = order_2[i];			// set dutycycle for CH3
		TIM2->CCR2 = order_1[i];			// set dutycycle for CH2
		msleep(2);
	}
}


// PWM initialization and application

void init_PWM(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// GPIOB enable
	stm32_gpio_alternate(GPIOB,11,2);			// GPIOB 11 alternate
	stm32_gpio_alternate(GPIOB,10,2);			// GPIOB 10 alternate
	stm32_gpio_alternate(GPIOB,3,2);			// GPIOB 3 alternate
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			// TIM2 enable
	TIM2->PSC = 48-1;					// set prescaler
	TIM2->ARR = 1000-1;					// set autoreload
	// TIM2->CCR4 = 100;					// set dutycycle
}

void init_TIM2_4(void) {
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S;			// set CC4 channel output
	TIM2->CCER &= ~TIM_CCER_CC4P;				// output -> high
	TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;			// output Compare 4 -> PWM Mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;	// output Compare 4 -> PWM Mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;			// enable preload register
	TIM2->CCER |= TIM_CCER_CC4E;				// CC4 enable
}

void init_TIM2_3(void) {
	TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;
	TIM2->CCER &= ~TIM_CCER_CC3P;
	TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
	TIM2->CCER |= TIM_CCER_CC3E;
}

void init_TIM2_2(void) {
	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM2->CCER &= ~TIM_CCER_CC2P;
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM2->CCER |= TIM_CCER_CC2E;
}

void start_CNT(void) {
	TIM2->EGR |= TIM_EGR_UG;				// init registers
	TIM2->BDTR |= TIM_BDTR_MOE;				// enable TIM2 output
	TIM2->CR1 |= TIM_CR1_CEN;				// start counting
}


// shift array

void shift(int amount, float from[], float to[]) {
	int index_max = 0;
	for (int i = 0; i < 999; i++) {
		if (i + amount < 999) {
			to[i] = from[i + amount];
			if (i > index_max)
				index_max = i;
		}
		else
			break;
	}
	for (int j = 0; j < amount; j++) {
		to[index_max + 1] = from[j];
		index_max++;
	}
}


// upload elements for an array

void upload(float to[]) {
	int index = 0;
	for (float i = 0; i < t_pi; i = i + delta) {
		to[index] = 500 + (sin(i) * 500);
		index++;
	}
}


// "main" function

int main(void)	{

	int i = 1;
	
	cookie_io_functions_t cookie_uart;
	stm32_rcc_init();
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR=417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	
	init_PWM();						// PWM init
	init_TIM2_4();						// TIM2 CH4 init
	init_TIM2_3();						// TIM2 CH3 init
	init_TIM2_2();						// TIM2 CH2 init
	start_CNT();						// start counting

	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);
	
	upload(order_1);
	shift(333, order_1, order_2);
	shift(666, order_1, order_3);

	while (1) {
		pulse();
		printf("\t[%i]\n", i);
		i++;
	}
	return(0);
}
