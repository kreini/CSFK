/*************************************/
/****** PWM ******** STM32F072  ******/
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


// PWM function in GPIOB 11

void PWM(int T_ms, int duty) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// GPIOB enable
	stm32_gpio_alternate(GPIOB,11,2);			// set GPIOB 11
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			// TIM2 enable
	TIM2->PSC = 48000-1;					// set prescaler
	TIM2->ARR = T_ms - 1;					// set autoreload
	TIM2->CCR4 = duty;					// set dutycycle
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
	
	PWM(20, 2);						// 20 ms period, 2 ms duty cicle

	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);

	while (1) {
		printf("[%li]\t<- CNT\n", TIM2->CNT);
		msleep(100);
	}
	return(0);
}
