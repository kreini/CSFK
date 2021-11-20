/**********************************************************/
/**  timer with two registers (TIM6 TIM7) *** STM32F072  **/
/**********************************************************/
/*******************  Gabor Kreinicker  *******************/
/**********************************************************/


// includes defines and global variables

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

static inline void msleep (int millisec) {
	while (0 < millisec) {
	_delay_loop(F_CPU/8000L);
	millisec--;
	};
}


// led function

int led(int state) {
	if (state == 1) {
		GPIOA->ODR |=  (1<<5);
	}
	if (state == 0) {
		GPIOA->ODR &= ~(1<<5);
	}
	if (state == 2) {
		if (ledstate == 0) {
			GPIOA->ODR &= ~(1<<5);
			ledstate = 1;
			return(1);
		}
		if (ledstate == 1) {
			GPIOA->ODR |=  (1<<5);
			ledstate = 0;
			return(1);
		}
	}
	return(1);
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

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;	// TIM6 turn in
	TIM6->PSC = 480 - 1;			// PSC setup
  	TIM6->ARR = 50000 - 1;			// ARR setup
  	TIM6->CR1  |= TIM_CR1_CEN;		// TIM6 start
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;	// TIM7 turn in
	TIM7->PSC = 480 - 1;			// PSC setup
  	TIM7->ARR = 20000 - 1;			// ARR setup
  	TIM7->CR1  |= TIM_CR1_CEN;		// TIM7 start


	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);

	while (1) {
		if (TIM6->SR & TIM_SR_UIF || TIM7->SR & TIM_SR_UIF) {
			if (TIM6->SR & TIM_SR_UIF) {
				printf("[%i TIM6] tele\n", j);
				TIM6->SR &= ~TIM_SR_UIF;
			}
	       	if (TIM7->SR & TIM_SR_UIF) {
	       		printf("[%i TIM7] tele\n", j);
				TIM7->SR &= ~TIM_SR_UIF;
			}			
	       	j++;
	       	led(2);
		}
	}
	return(0);
}
