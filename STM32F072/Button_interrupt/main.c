/******************************************/
/**  interrupt  ************  STM32F072  **/
/******************************************/
/***********  Kreinicker Gábor  ***********/
/******************************************/


// includes, defines  és global variables

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

void led(int state) {
	if (state == 1)
		GPIOA->ODR |=  (1<<5);
	if (state == 0)
		GPIOA->ODR &= ~(1<<5);
	else
		GPIOA->ODR ^= (1<<5);
}


// handler function for EXTI interrupt

void EXTI4_15_IRQHandler (void) {
	if (EXTI->PR & (1<<13)) {
		msleep(50);
		EXTI->PR |= (1<<13);
		led(2);
		printf("button\n");
	}
}


// handler function for TIM6 interrupt

void TIM6_DAC_IRQHandler (void) {
	TIM6->SR &= ~TIM_SR_UIF;
	led(2);
	printf("timer\n");
}


// vector table for interrupt

uint32_t* _VECTOR_IRQ[32] __attribute__ ((section(".vectors.irq"))) = {
	[EXTI4_15_IRQn] (uint32_t *) EXTI4_15_IRQHandler,
	[TIM6_DAC_IRQn] (uint32_t *) TIM6_DAC_IRQHandler
};


// system reboot

int system_reboot (void) {
	NVIC_SystemReset();
	return(0);
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

int main(void) {
	int i = 0;

	cookie_io_functions_t	cookie_uart;

	stm32_rcc_init();
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	//RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	stm32_gpio_alternate(GPIOA,2,1);      /* STM32F072_AF_PA2_USART2 */
	stm32_gpio_alternate(GPIOA,3,1);      /* STM32F072_AF_PA3_USART2 */
	
	USART2->BRR=417;                       /* baud: 115200 @48MHz  */
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	
	cookie_uart.read=NULL;
	cookie_uart.write=usart_write;
	cookie_uart.seek=NULL;
	cookie_uart.close=NULL;

	stdout=fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = 48000-1;
	TIM6->ARR = 1000-1;
	TIM6->DIER |= TIM_DIER_UIE;
	TIM6->CR1  |= TIM_CR1_CEN;	
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_SetPriority(TIM6_DAC_IRQn, 0);
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	stm32_gpio_input(GPIOC,13);
	SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~(15<<4)) | (2<<4);
	EXTI->IMR |= (1<<13);
	EXTI->RTSR |= (1<<13);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	NVIC_SetPriority(EXTI4_15_IRQn, 0);

	while ( 1 ) {
		printf("[%i] main\n", i);
		msleep(200);
		i++;
	}

	return(0);
}
