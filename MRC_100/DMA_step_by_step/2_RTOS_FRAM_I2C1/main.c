/* 2nd step: RTOS and I2C */
/* STM32F072 */
/* Gabor Kreinicker, Andras Pal */


/* basic libraries */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stm32f0xx.h>

/* freertos libraries */
#include "rtos/FreeRTOS.h"	/* Must come first. */
#include "rtos/task.h"		/* RTOS task related API prototypes. */
#include "rtos/queue.h"	/* RTOS queue related API prototypes. */
#include "rtos/timers.h"	/* Software timer related API prototypes. */
#include "rtos/semphr.h"	/* Semaphore related API prototypes. */

/* delay_basic library */
#include "delay_basic.h"

/* stm32 libraries */
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"


/* CPU clock frequency */
#define         F_CPU                   48000000


/*****************************************************************************/
/* handlers */

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t * RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));


/*****************************************************************************/
/* freeRTOS setup */


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
/* UART handler, write and read functions */


uint32_t data_lost_counter;

void USART2_Handler(void) {
	
 	BaseType_t xTaskWokenByReceive = pdFALSE;
 	char c;

 	if (USART2->ISR & USART_ISR_TXE) {
 		BaseType_t ret;
		ret = xQueueReceiveFromISR(queue_usart_tx, &c, &xTaskWokenByReceive);
		if (ret)
			USART2->TDR = (uint16_t)c;
		else
			USART2->CR1 &= ~USART_CR1_TXEIE;
  	}
 	if (USART2->ISR & USART_ISR_RXNE) {
 		BaseType_t ret;
		c = (uint8_t)USART2->RDR;
		ret = xQueueSendFromISR(queue_usart_rx, &c, &xTaskWokenByReceive);
		if (ret == errQUEUE_FULL)
			data_lost_counter++;
  	}
 	if (xTaskWokenByReceive != pdFALSE)
		taskYIELD();

}

ssize_t usart_write_queue(void *cookie, const char *buff, size_t size) {

 	USART_TypeDef *USART = cookie;
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

ssize_t usart_write_native(void *cookie, const char *buff, size_t size) {

 	USART_TypeDef *USART = cookie;
 	ssize_t ssize;
 	ssize=0;
 	
 	while (0 < size) {	
  		while (!(USART->ISR & USART_ISR_TXE));
		USART->TDR= *buff;
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
 		BaseType_t ret;
		ret = xQueueReceive(queue_usart_rx, buff, !ssize?portMAX_DELAY:0);
		buff++;
		size--;
		if (ret)
			ssize++;
		else
			return(ssize);	
  	}
 	return(ssize);
}


/*****************************************************************************/
/* busy wait function */


void msleep(int t) {
	vTaskDelay(t / portTICK_RATE_MS);  
}


/*****************************************************************************/
/* blink task */


void blink_task(void* parameters) {
	while (1) {
		stm32_gpio_output_toggle(GPIOA, 5);
		msleep(1000);
	}
}

/* FRAM task */

void fram_task(void* parameters) {
	char buff[64];
	uint8_t array[16];
	int ncycle;
	int number_of_bytes = 8;
	ncycle = 0;

	while (1) {
		int n;
		int16_t temp_1;

		msleep(1000);

		array[0] = 0x55;
		array[1] = 0xAA;
		array[2] = 0xCC;
		array[3] = 0x33;
		array[4] = 0x55;
		array[5] = 0xAA;
		array[6] = 0xCC;
		array[7] = 0x33;
		array[8] = 0x55;
		array[9] = 0xAA;
		array[10] = 0xCC;
		array[11] = 0x33;
		array[12] = 0x55;
		array[13] = 0xAA;
		array[14] = 0xCC;
		array[15] = 0x33;

		
		stm32_i2c_write_read(I2C1, 0x50, "\x3E\xA2", 2, array, number_of_bytes);
		
		for (int i = 0; i < number_of_bytes; i++) {
			n = sprintf(buff, "%.2x ", array[i]);
			usart_write_queue(USART2, buff, n);
		}
		usart_write_queue(USART2, "\n", 2);
		
		ncycle++;
	}
}


/*****************************************************************************/
/* main function */


int main(void) {
	int x;
	cookie_io_functions_t cookie_uart[2];
	stm32_rcc_init();
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
	RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
	RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;
	RAM_VECTORS[16+USART2_IRQn	]=(uint32_t *)USART2_Handler;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	stm32_gpio_mode(GPIOA,5,1);
	stm32_gpio_otype(GPIOA,5,0);
	stm32_gpio_ospeed(GPIOA,5,3);
	stm32_gpio_pupd(GPIOA,5,0);
	

	/* stm32_gpio_output_toggle(GPIOB,3); */

	/* USART2 configuration: */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	USART2->BRR = 417;
	USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
	USART2->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,0);
	USART2->CR1 |= USART_CR1_RXNEIE; 

	/* I2C1 configuration: */
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	stm32_gpio_alternate(GPIOB,8,1);
	stm32_gpio_alternate(GPIOB,9,1);
	stm32_i2c_configure(I2C1);
	I2C1->CR1 |= I2C_CR1_PE;
	
	/* FRAM write data */
	if (0) {
		uint8_t buff[20];
		int i;
		for (i = 0; i < 32768 ; i += 16) {
			int j;
			buff[0] = i>>8;
			buff[1] = i&0xFF;
			for (j = 0; j < 16; j += 2) {
				buff[2+j+0] = (i+j)>>8;
				buff[2+j+1] = (i+j)&0xFF;
		        }
			stm32_i2c_write(I2C1, 0x50, buff, 2+16);
		}
	}

	while (0) {
			__DSB();
			__WFI();
			__ISB();
			stm32_gpio_output_toggle(GPIOA, 5);
	}

	cookie_uart[0].read = usart_read_queue;
	cookie_uart[0].write = NULL;
	cookie_uart[0].seek = NULL;
	cookie_uart[0].close = NULL;

	stdin = fopencookie(USART2, "r", cookie_uart[0]);

	cookie_uart[1].read = NULL;
	cookie_uart[1].write = usart_write_queue;
	cookie_uart[1].seek = NULL;
	cookie_uart[1].close = NULL;

	stdout = fopencookie(USART2, "w" ,cookie_uart[1]);
	setlinebuf(stdout);
	
	
	/* Tasks */
	xTaskCreate(blink_task,
		"blink",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 2,
       	 NULL);
	
	queue_usart_tx=xQueueCreate(32,1);
	queue_usart_rx=xQueueCreate(32,1);
 
	xTaskCreate(fram_task,
		( signed char * ) "upper",
		256,
		NULL,
		tskIDLE_PRIORITY + 1,
        	NULL);
 
	vTaskStartScheduler();

	for( ;; );
}
