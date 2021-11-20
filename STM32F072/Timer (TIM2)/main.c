/*************************************/
/**  timer with TIM2 *** STM32F072  **/
/*************************************/
/********  Gabor Kreinicker  *********/
/*************************************/


// include-ok, define-ok  és globális változók

#define		F_CPU			48000000
#define		_GNU_SOURCE

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include "delay_basic.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "config.h"

int ledstate = 0;


// "delay" function

static inline void alvas (int millisec) {
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


// "main" function

int main(void)	{

	int j = 1;
	
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

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 1000-1;
  	TIM2->ARR = 48000-1;
  	TIM2->CR1  |= TIM_CR1_CEN;


	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);

	while (1) {
		if (TIM2->SR & TIM_SR_UIF) {
			printf("[%i] tele\n", j);
	       	TIM2->SR &= ~TIM_SR_UIF;
	       	j++;
	       	led(2);
		}
	}
	return(0);
}
