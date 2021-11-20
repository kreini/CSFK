/******************************************/
/**  FreeRTOS queue  *******  STM32F072  **/
/******************************************/
/***********  Kreinicker Gábor  ***********/
/******************************************/


// includes, defines, and global variables

#define	F_CPU			48000000
#define	_GNU_SOURCE

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <math.h>

#include "delay_basic.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "config.h"

#include "rtos/FreeRTOS.h"	// for FreeRTOS
#include "rtos/task.h"
#include "rtos/queue.h"
#include "rtos/timers.h"
#include "rtos/semphr.h"


// handlers and vector tables

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t* RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));


// setup FreeRTOS

void vApplicationTickHook(void) {
	static uint32_t ulCount = 0;
	ulCount++;
	if (500UL <= ulCount)
		ulCount = 0UL;
}

void vApplicationMallocFailedHook(void) {
	for(;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName) {
	(void) pcTaskName;
	(void) xTask;
	for(;;);
}

void vApplicationIdleHook(void){};


// busy wait function

void msleep(int t) {
	vTaskDelay(t / portTICK_RATE_MS);
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


// queue functions

QueueHandle_t myQueue = NULL;		// queue handler
int size = 16;				// size of the queue
char rx;

static void USART2_IRQHandler(void) {
  	if (USART2->ISR & USART_ISR_RXNE) {
		char cIn;
		BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		do {
			cIn = USART2->RDR;
			xQueueSendFromISR(myQueue, (void*)&cIn, &xHigherPriorityTaskWoken);
		} while (USART2->ISR & USART_ISR_RXNE);
		
		if (xHigherPriorityTaskWoken) {
			taskYIELD();	// a taskYIELD_FROM_ISR()-t nem ismeri
		}
	}
}

void receive_task(void *p) {
	while(1) {
		if(myQueue != 0) {			// data -> receive and print 
			if (xQueueReceive(myQueue, (void*)&rx, (TickType_t) 10)) {
				if (rx != '\n')
					printf("Received:\t%c\n", rx);
				else
					printf("Received:\t\'\\n\'\n\n", rx);
			}
		}
		
	}
}


// main function

int main(void) {
	myQueue = xQueueCreate(size, sizeof(char));

	cookie_io_functions_t cookie_uart;
	stm32_rcc_init();
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	RAM_VECTORS[16 + SVC_IRQn] = (uint32_t*)SVC_Handler;
	RAM_VECTORS[16 + PendSV_IRQn] = (uint32_t*)PendSV_Handler;
	RAM_VECTORS[16 + SysTick_IRQn] = (uint32_t*)SysTick_Handler;
	RAM_VECTORS[16 + USART2_IRQn]=(uint32_t *)USART2_IRQHandler;
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1 | SYSCFG_CFGR1_MEM_MODE_0;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_mode(GPIOA, 5, 1);
	stm32_gpio_otype(GPIOA, 5, 0);
	stm32_gpio_ospeed(GPIOA, 5, 3);
	stm32_gpio_pupd(GPIOA, 5, 0);
	
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	stm32_gpio_alternate(GPIOA, 2, 1);
	stm32_gpio_alternate(GPIOA, 3, 1);
	USART2->BRR = 417;
	USART2->CR1 |= USART_CR1_RE | USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);
	USART2->CR1 |= USART_CR1_RXNEIE;
	
	cookie_uart.read = NULL;
	cookie_uart.write = usart_write;
	cookie_uart.seek = NULL;
	cookie_uart.close = NULL;

	stdout = fopencookie(USART2,"w",cookie_uart);
	setlinebuf(stdout);
		
	xTaskCreate(receive_task,
		(signed char*)"upper",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		NULL);
	
	vTaskStartScheduler();
	for(;;);
}
