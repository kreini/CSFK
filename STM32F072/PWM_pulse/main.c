/*************************************/
/***  PWM with TIM2 *** STM32F072  ***/
/*************************************/
/********  Gabor Kreinicker  *********/
/*************************************/


// include-ok, define-ok  és globális változók

#define		F_CPU			48000000
#define		_GNU_SOURCE
#define		stm32_gpio_alternate(GPIOx,n,f)		\
			do {						\
				stm32_gpio_mode((GPIOx),(n),2);	\
				stm32_gpio_af((GPIOx),(n),(f));	\
			} while(0)

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <math.h>
#include "delay_basic.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "config.h"



int ledstate = 0;


// "delay" function

static inline void msleep (int millisec) {
	while (0 < millisec) {
	_delay_loop(F_CPU/8000L);
	millisec--;
	};
}


// led function

void led(int state) {
	if (state == 1)
		GPIOA->ODR |=  (1<<5);
	if (state == 0)
		GPIOA->ODR &= ~(1<<5);
	else
		GPIOA->ODR ^= (1<<5);
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
	for (int i = 0; i < 1000; i++) {
		TIM2->CCR4 = i;
		printf("%i\n", i);
		msleep(2);
	}
	for (int j = 1000; j > 0; j--) {
		TIM2->CCR4 = j;
		printf("%i\n", j);
		msleep(2);
	}
}


// PWM init GPIOB

void init_GPIOB(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// GPIOB enable
	stm32_gpio_alternate(GPIOB,11,2);
}


// PWM init TIM2

void init_TIM2(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			// TIM2 enable
	TIM2->PSC = 48-1;					// set prescaler
	TIM2->ARR = 1000-1;					// set autoreload
	// TIM2->CCR4 = 100;				// set dutycycle
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S;			// set CC4 channel output
	TIM2->CCER &= ~TIM_CCER_CC4P;				// output -> high
	TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;			// output Compare 4 -> PWM Mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;	// output Compare 4 -> PWM Mode 1
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;			// enable preload register
	TIM2->CCER |= TIM_CCER_CC4E;				// CC4 enable
	TIM2->EGR |= TIM_EGR_UG;				// init registers
	TIM2->BDTR |= TIM_BDTR_MOE;				// enable TIM2 output
	TIM2->CR1 |= TIM_CR1_CEN;				// start counting
}


// "main" function

int main(void)	{	
	cookie_io_functions_t cookie_uart;
	stm32_rcc_init();
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	USART2->BRR=417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	
	init_GPIOB();						// GPIOB init
	init_TIM2();						// TIM2 init

	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);

	while (1) {
		pulse();
	}
	return(0);
}
