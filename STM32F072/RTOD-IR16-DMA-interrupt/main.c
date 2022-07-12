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
#include "stm32_i2c_dma.h"

#define         F_CPU                   48000000

/*****************************************************************************/

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t * RAM_VECTORS[48] __attribute__ ((section(".ramvectors")));


/*****************************************************************************/

static int			led_delay;
static int16_t 	ir_page[2][12*16];  //valuse of IR sensors

static QueueHandle_t	ir_queue;  //holds the value of the subpage when a subpage is read
static QueueHandle_t	dma_queue; //holds any value when the dma is finished

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

void DMA1_Chanel2_3_Handler(void) {
	while (!(DMA1-> ISR & DMA_ISR_TCIF3));  //While previous DMA3 transfer not complite
 	DMA1->IFCR = DMA_IFCR_CTCIF3;           //While DMA3 transfer complite
 	while (!(I2C1->ISR & I2C_ISR_STOPF));   
 	I2C1->CR1 &= ~I2C_CR1_PE;
   
   // send something to dma_queue (this shows that the DMA transfer is complite
 	int tmp;
 	if( xQueueSend(dma_queue, (void*)&tmp, 10) == pdFALSE)
   {
       led_delay = 8096; //this should not happen
   }
} 

/*****************************************************************************/

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
/*
const uint8_t color_map[4*32] = {
	0,0,0,0,
	0,0,51,0,
	0,0,102,0,
	0,0,153,0,
	0,0,204,0,
	0,0,255,0,
	0,51,255,0,
	0,102,255,0,
	0,153,255,0,
	0,204,255,0,
	0,255,255,0,
	0,255,204,0,
	0,255,153,0,
	0,255,102,0,
	0,255,51,0,
	0,255,0,0,
	51,255,0,0,
	102,255,0,0,
	153,255,0,0,
	204,255,0,0,
	255,255,0,0,
	255,204,0,0,
	255,153,0,0,
	255,102,0,0,
	255,51,0,0,
	255,0,0,0,
	255,51,51,0,
	255,102,102,0,
	255,153,153,0,
	255,204,204,0,
	255,255,255,0,
	255,255,255,0
}; */

/*****************************************************************************/
/*
int color_key(int16_t in) {
	if (in < -768) in = -768;
	else if(in > 256) in = 256;
	return ((in+768)/32*4);
}*/

static inline uint16_t ntohs(uint16_t x)
{
	uint16_t y;
	y  = (x>>8)&0x00FF;
	y |= (x<<8)&0xFF00;
	return(y);
}

void msleep(int t) {
	vTaskDelay( t / portTICK_RATE_MS );  
}

/********************************** TASKS *****************************************/

void blink_task(void * parameters) {
	while (1) {	
		msleep(led_delay);
		stm32_gpio_output_toggle(GPIOA,5);
	}
}

void ir_read_task(void *parameters) 
{
	static unsigned char	array[64];
	uint8_t		subpage; 
	uint8_t 		address[4];
	int				tmp;
 
	//only neede for debug
	int 					ret, n;
	static char	buff[32];


	while ( 1 ) 
	{	
		// Check for a new subpage
		address[0]=0x80;
		address[1]=0x00;  
		address[2]=0x00;  
		address[3]=0x00;  
		stm32_i2c_write(I2C1,0x33,"\x80\x00",4);  //reset register at first run
		uint8_t counter =0;
		while(1)
		{
			array[0] = 0xBE;
			array[1] = 0xEF; 
  	 	
			//Test that dma_queue is emty
			if (xQueueReceive(dma_queue, (void*)&tmp, 0) == pdTRUE)
			{
				led_delay = 8096; //this should not happen
			}	
  	 	
			// Check for a new subpage (useing I2C and DMA)
			stm32_i2c_dma_write_read(I2C1,0x33,address,2,array,2);
			//ret = stm32_i2c_dma_write_read(I2C1,0x33,address,2,array,2); //use this row in case of debugging
			// Only go after dma_queue can be read from //dma_queue is written by: DMA1_Chanel2_3_Handler(void)
			if (xQueueReceive(dma_queue, (void*)&tmp, portMAX_DELAY) == pdFALSE)
			{
				led_delay = 8096; //this should not happen
			}	

			if ((array[1]&8) == 8)  //found new page
			{  		
				subpage = array[1]&0x01;  //get subpage
				address[3] = array[1]&0xF7;
				stm32_i2c_write(I2C1,0x33,address,4);  //reset register
				if (counter == 0) led_delay /= 2;   //4-10 is ideal		
				break; //skip to "reading the data from subpage"
			} 
    	
			counter++;
			msleep(5); //wait a litle before the next try
			//set to 5 int case 8 fps or faster, 10 in case of 4 fps or slower
		}	
    
		//Reading the data from subpage
		for (uint8_t i=0; i<6; i++)
		{
			address[0]=0x04+i/4;  //0x04 for the first 4 times 05 for the last 2
			address[1]=(i%4)<<6|subpage<<5;  //0x00, 0x40, 0x80 ...

			// Read 2 rows at once (useing I2C and DMA)
			stm32_i2c_dma_write_read(I2C1,0x33,address,2,&ir_page[subpage][16*2*i],64);
			//ret = stm32_i2c_dma_write_read(I2C1,0x33,address,2,&ir_page[subpage][16*2*i],64)); //use this row in case of debugging
			// Only go after dma_queue can be read from //dma_queue is written by: DMA1_Chanel2_3_Handler(void)
			if (xQueueReceive(dma_queue, (void*)&tmp, portMAX_DELAY) == pdFALSE)
			{
				led_delay = 8096; //this should not happen
			}	
		}
   
		// send subpage to ir_queue (this shows that the the data from the sensor has been read from the sensor)
		if (xQueueSend(ir_queue, (void*)&subpage, 10) == pdFALSE) 
		{
			led_delay = 8096; //this should not happen
		}
		//wait some time before trying to read the next subpage
		msleep(0); //set to: 0 in case 16fps, 60 in case 8fps, 170 in case 4fps, 400 in case 2fps
	}
}

void ir_print_task(void *parameters) 
{
	static char		buff[32];

	while ( 1 ) 
	{	
		uint8_t subpage;
		//Wait until a subpage has been read from the sensor
		if(xQueueReceive(ir_queue, (void*)&subpage, portMAX_DELAY) == pdFALSE)
		{
			int n;
			n=sprintf(buff,"ir_queue read was NOT sucsessfull \n");
   		usart_write_queue(USART2,buff,n);
		}
		else
		{
			int n;
    		
			/*//clear terminal - use in case of colour display
			n=sprintf(buff,"\033[H");
			usart_write_queue(USART2,buff,n);*/

			for (uint8_t i = 0; i<12; i++)
			{
				/*//colour display
				for (uint8_t j=0; j<16; j++)
				{ 
					n = color_key(((ir_page[subpage][i*16+j]&0xff00)>>8)+((ir_page[subpage][i*16+j]&0x00ff)<<8));
					n=sprintf(buff,"\033[48;2;%i;%i;%im  ",n, color_map[n+1], color_map[n+2]);
					usart_write_queue(USART2,buff,n);
				}*/
				//hex display
				for (uint8_t j=0; j<4; j++)
				{
					n=sprintf(buff,"%.4x %.4x %.4x %.4x ", 
					ntohs(ir_page[subpage][i*16+4*j+0]),
					ntohs(ir_page[subpage][i*16+4*j+1]),
					ntohs(ir_page[subpage][i*16+4*j+2]),
					ntohs(ir_page[subpage][i*16+4*j+3]));
					usart_write_queue(USART2,buff,n);
				}
				usart_write_queue(USART2,"\n",1);
			}
			n=sprintf(buff,"---end---\n");
			usart_write_queue(USART2,buff,n);
		}
	}
}

/********************************** MAIN *****************************************/

int main(void) {
	cookie_io_functions_t cookie_uart[2];
	stm32_rcc_init();
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
	RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
	RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;
	RAM_VECTORS[16+USART2_IRQn	]=(uint32_t *)USART2_Handler;
	RAM_VECTORS[16+DMA1_Channel2_3_IRQn	]=(uint32_t *)DMA1_Chanel2_3_Handler;

	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	stm32_gpio_mode(GPIOA,5,1);
	stm32_gpio_otype(GPIOA,5,0);
	stm32_gpio_ospeed(GPIOA,5,3);
	stm32_gpio_pupd(GPIOA,5,0);

/* USART2 configuration: */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER &= ~(3<<(2*5));
	GPIOA->MODER |=  (1<<(2*5));
	stm32_gpio_alternate(GPIOA,2,1);
	stm32_gpio_alternate(GPIOA,3,1);
	USART2->BRR = 52;
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
 
/* DMA1-CH3 configuration */
	RCC->AHBENR  |= RCC_AHBENR_DMA1EN;  // enable pheropherial clock on DMA
	DMA1_Channel3->CCR |= DMA_CCR_MINC;  //memory increment
	
/* Configure NVIC for DMA */
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn); 
	NVIC_SetPriority(DMA1_Channel2_3_IRQn,2);  // 
 	
/* Crete queues */
	queue_usart_tx=xQueueCreate(32,1);
	queue_usart_rx=xQueueCreate(32,1);
 	ir_queue = xQueueCreate(2,1);
 	dma_queue = xQueueCreate(2,1); 	
 	
 	led_delay = 1024;
 	
/* Controll register: 0x800D
 	 2 fps: 0000 0001 0000 0001 = 0x0101
 	 4 fps: 0000 0001 1000 0001 = 0x0181
 	 8 fps: 0000 0010 0000 0001 = 0x0201
 	16 fps: 0000 0010 1000 0001 = 0x0281
 	32 fps: 0000 0011 0000 0001 = 0x0301*/
/* Set framerate to 16 fps */
	uint8_t address[4];
	address[0]=0x80;
	address[1]=0x0D;  
	address[2]=0x02;  
	address[3]=0x81; 
	stm32_i2c_write(I2C1,0x33,address,4);  //reset register


	while (0) 
	{
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
	


	xTaskCreate(blink_task,
		"blink",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL );
 
	xTaskCreate(ir_read_task,
		"ir_read_task",
		256,
		NULL,
		tskIDLE_PRIORITY+1,
		NULL );
  
	xTaskCreate(ir_print_task,
		"ir_print_task",
		384,
		NULL,
		tskIDLE_PRIORITY+2,
		NULL );

	vTaskStartScheduler();
	for( ;; );
}
