/*****************************************************/
/**********  RTOS terminal  *** STM32F072  ***********/
/*****************************************************/
/****************  Gabor Kreinicker  *****************/
/*****************************************************/


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stm32f0xx.h>

#include "rtos/FreeRTOS.h"	/* Must come first. */
#include "rtos/task.h"		/* RTOS task related API prototypes. */
#include "rtos/queue.h"	/* RTOS queue related API prototypes. */
#include "rtos/timers.h"	/* Software timer related API prototypes. */
#include "rtos/semphr.h"	/* Semaphore related API prototypes. */

#include "delay_basic.h"

#include "stm32_rcc.h"
#include "stm32_gpio.h"

#define	F_CPU			48000000
// #define	_GNU_SOURCE

int ci = 0;

/*****************************************************************************/

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t * RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));


/*****************************************************************************/

static volatile int idlehookcnt;
static volatile int tickhookcnt;

static QueueHandle_t queue_usart_tx;
static QueueHandle_t queue_usart_rx;

void vApplicationTickHook(void) {
	tickhookcnt++;
}

void vApplicationMallocFailedHook( void ) {
	for(;;);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ) {
	(void) pcTaskName;
	(void) xTask;
	for (;;);
}


void vApplicationIdleHook(void) {
	idlehookcnt++;
	__DSB();
	__WFI();
	__ISB();
}

/*****************************************************************************/

uint32_t data_lost_counter;

void USART2_Handler(void) {
	BaseType_t xTaskWokenByReceive = pdFALSE;
	char c;

	if (USART2->ISR & USART_ISR_TXE) {
 		BaseType_t ret;
		ret = xQueueReceiveFromISR(queue_usart_tx,&c,&xTaskWokenByReceive);
		if (ret)
			USART2->TDR = (uint16_t)c;
		else
			USART2->CR1 &= ~USART_CR1_TXEIE;
	}

	if (USART2->ISR & USART_ISR_RXNE) {
 		BaseType_t	ret;
		c = (uint8_t)USART2->RDR;
		ret = xQueueSendFromISR(queue_usart_rx, &c, &xTaskWokenByReceive);
		if (ret == errQUEUE_FULL)
			data_lost_counter++;
	}

	if (xTaskWokenByReceive != pdFALSE)
		taskYIELD();
}

/*****************************************************************************/

ssize_t usart_write_queue(void *cookie, const char *buff, size_t size) {
	USART_TypeDef *USART=cookie;
	ssize_t ssize;

	ssize=0;
	while (0 < size) {
		xQueueSend(queue_usart_tx,buff,portMAX_DELAY);
		USART->CR1 |= USART_CR1_TXEIE;
		buff++;
		size--;
		ssize++;
	}
	return(ssize);
}

ssize_t usart_read_queue(void *cookie, char *buff, size_t size) {
	ssize_t ssize;
	ssize=0;
	while (0 < size) {
		BaseType_t	ret;

		ret=xQueueReceive(queue_usart_rx,buff,0<ssize?0:portMAX_DELAY);

		buff++;
		size--;
		if (ret)
			ssize++;
		else
			return(ssize);	
	}
	return(ssize);
}


ssize_t usart_write(void *cookie, const char *buff, size_t size) {
	USART_TypeDef *USART=cookie;
	ssize_t ssize;
 
	ssize=0;
	while (0 < size) {
		while (!(USART->ISR & USART_ISR_TXE)) {
        		USART->TDR=*buff;
			buff++;
			size--;
			ssize++;
		}
	};

 return(ssize);
}

/*****************************************************************************/

void msleep(int t) {
	vTaskDelay( t / portTICK_RATE_MS );  
}


void T_led(char* buff) {
	char cmd[16];
	for (int i = 4; i < strlen(buff); i++)
		cmd[i-4] = buff[i];
	stm32_gpio_output_toggle(GPIOA,5);
	msleep(atoi(cmd));
	stm32_gpio_output_toggle(GPIOA,5);	
}

void T_run(char* buff) {
	char cmd[32];
	sprintf(cmd, "A futas soran eltelt %i s\n", ci);
	fwrite(cmd, 1, strlen(cmd), stdout);
}

void T_info(char* buff) {
	char* answ1 = "\nJelenleg hasznalhato parancsok:\n";
	char* answ2 = "\thalo\t - koszones\n";
	char* answ3 = "\truntime\t - futas kozben eltelt ido [s]\n";
	char* answ4 = "\tled\t - megvaltoztatja a led allapotat\n";
	char* answ5 = "\tled:x\t - x idore megvaltoztatja a led allapotat\n";
	char* answ6 = "\treset\t - ujrainditas\n";
	char* answ7 = "\tinfo\t - parancs lista\n";
	
	fwrite(answ1, 1, strlen(answ1), stdout);
	fwrite(answ2, 1, strlen(answ2), stdout);
	fwrite(answ3, 1, strlen(answ3), stdout);
	fwrite(answ4, 1, strlen(answ4), stdout);
	fwrite(answ5, 1, strlen(answ5), stdout);
	fwrite(answ6, 1, strlen(answ6), stdout);
	fwrite(answ7, 1, strlen(answ7), stdout);
}

void count_task(void * parameters) {
	while (1) {
		// stm32_gpio_output_toggle(GPIOA,5);
		ci++;
		msleep(1000);
		
	}
}

int T_reset(void) {
	NVIC_SystemReset();
	return(0);
}

void terminal_task(void * parameters) {
	
	char buff[32];
	char* answ = "szia\n";
	char* err = "E01 - ismeretlen parancs\n";

	while (1) {
		while (fgets(buff,32,stdin) != NULL) {
			if (strcmp(buff, "halo\n") == 0) {
				fwrite(answ, 1, strlen(answ), stdout);
				break;
			}
			if (strcmp(buff, "led\n") == 0) {
				stm32_gpio_output_toggle(GPIOA,5);
				break;
			}
			if (buff[0] == 'l' && buff[1] == 'e' && buff[2] == 'd' && buff[3] == ':') {	
				T_led(buff);
				break;
			}
			if (strcmp(buff, "runtime\n") == 0) {
				T_run(buff);
				break;
			}
			if (strcmp(buff, "info\n") == 0) {
				T_info(buff);
				break;
			}
			if (strcmp(buff, "reset\n") == 0) {
				T_reset();
				break;
			}
			else
				fwrite(err, 1, strlen(err), stdout);
	 	}
	}
}
 

/* https://www.freertos.org/Hardware-independent-RTOS-example.html */

int main(void) {
	cookie_io_functions_t cookie_uart[2];
	stm32_rcc_init();
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
	RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
	RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;
	RAM_VECTORS[16+USART2_IRQn	]=(uint32_t *)USART2_Handler;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	stm32_gpio_mode(GPIOA,5,1);
	stm32_gpio_otype(GPIOA,5,0);
	stm32_gpio_ospeed(GPIOA,5,3);
	stm32_gpio_pupd(GPIOA,5,0);

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	USART2->BRR=417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);
	USART2->CR1 |= USART_CR1_RXNEIE;
	
	while ( 0 ) {
		__DSB();
		__WFI();
		__ISB();
		stm32_gpio_output_toggle(GPIOA,5);
	}

	cookie_uart[0].read=usart_read_queue;
	cookie_uart[0].write=NULL;
	cookie_uart[0].seek=NULL;
	cookie_uart[0].close=NULL;
	
	stdin=fopencookie(USART2,"r",cookie_uart[0]);

	cookie_uart[1].read=NULL;
	cookie_uart[1].write=usart_write_queue;
	cookie_uart[1].seek=NULL;
	cookie_uart[1].close=NULL;

	stdout=fopencookie(USART2,"w",cookie_uart[1]);

	setlinebuf(stdout);

	xTaskCreate(count_task,
		"count",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		NULL );

	queue_usart_tx=xQueueCreate(32,1);
	queue_usart_rx=xQueueCreate(32,1);

	xTaskCreate(terminal_task,
		(signed char *) "terminal",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		NULL );
	 
	vTaskStartScheduler();

	for( ;; );
}

