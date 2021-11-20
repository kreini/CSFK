/*****************************************************/
/*** math.h sin() function testing  *** STM32F072  ***/
/*****************************************************/
/****************  Gabor Kreinicker  *****************/
/*****************************************************/


// includes, defines and global variables

#define		F_CPU			48000000
#define		_GNU_SOURCE

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


// "main" function

int main(void)	{
	
	cookie_io_functions_t cookie_uart;
	stm32_rcc_init();
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR=417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;

	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);

	double counted;
	double increment = 0;
	
	while (1) {
		for (int i = 0; i < 10000000; i++) {	// ez a for ciklusos
			counted = sin(increment);
			increment += 0.1;
		}
		/*counted = sin(increment);		// ez meg a sima
		increment += 0.1;*/
		printf("\n");
	}
	return(0);
}
