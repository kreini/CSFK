#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stm32f0xx.h>

#include "rtos/FreeRTOS.h"	/* Must come first. */
#include "rtos/task.h"		/* RTOS task related API prototypes. */
#include "rtos/queue.h"		/* RTOS queue related API prototypes. */
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

static int	led_delay;
static int ir_page_0[12][16];  //valuse of IR sensors subpage page 0 (16bit)
static SemaphoreHandle_t ir_mutex;

static volatile int	idlehookcnt;
static volatile int	tickhookcnt;

static QueueHandle_t	queue_usart_tx;
static QueueHandle_t	queue_usart_rx;

void vApplicationTickHook( void )
{
 tickhookcnt++;
}

void vApplicationMallocFailedHook( void )
{
 for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
 (void) pcTaskName;
 (void) xTask;

 for (;;);
}


void vApplicationIdleHook( void )
{
 idlehookcnt++;

 __DSB();
 __WFI();
 __ISB();
}

/*****************************************************************************/

uint32_t	data_lost_counter;

void USART2_Handler(void)
{
	//stm32_gpio_output_toggle(GPIOB,3);
	
 	BaseType_t xTaskWokenByReceive = pdFALSE;
 	char	c;

 	if ( USART2->ISR & USART_ISR_TXE )
  	{	BaseType_t	ret;
		ret=xQueueReceiveFromISR(queue_usart_tx,&c,&xTaskWokenByReceive);
		if ( ret )
			USART2->TDR = (uint16_t)c;
		else
			USART2->CR1 &= ~USART_CR1_TXEIE;
  	}

 	if ( USART2->ISR & USART_ISR_RXNE )
  	{	BaseType_t	ret;

		c=(uint8_t)USART2->RDR;
		ret=xQueueSendFromISR(queue_usart_rx,&c,&xTaskWokenByReceive);
		if ( ret == errQUEUE_FULL )
			data_lost_counter++;

	
  	}

 	if ( xTaskWokenByReceive != pdFALSE )
		taskYIELD();

}

/*****************************************************************************/

ssize_t usart_write_queue(void *cookie, const char *buff, size_t size)
{
 	USART_TypeDef *USART=cookie;
 	ssize_t        ssize;

 	ssize=0;
 	while ( 0<size )
  	{	xQueueSend(queue_usart_tx,buff,portMAX_DELAY);
		USART->CR1 |= USART_CR1_TXEIE;
		buff++;
		size--;
		ssize++;
  	}
 	return(ssize);
}

ssize_t usart_write_native(void *cookie, const char *buff, size_t size)
{
 	USART_TypeDef *USART=cookie;
 	ssize_t        ssize;

 	ssize=0;
 	while ( 0<size )
  	{	
  		while ( ! (USART->ISR & USART_ISR_TXE) );
		USART->TDR= *buff;
		
		buff++;
		size--;
		ssize++;
  	}
 	return(ssize);
}

ssize_t usart_read_queue(void *cookie, char *buff, size_t size)
{
 	ssize_t        ssize;

 	ssize=0;
 	while ( 0<size )
  	{	BaseType_t	ret;

		ret=xQueueReceive(queue_usart_rx,buff,!ssize?portMAX_DELAY:0);

		buff++;
		size--;
		if ( ret )
			ssize++;
		else
			return(ssize);	
  	}
 	return(ssize);
}

/* not used
ssize_t usart_write(void *cookie, const char *buff, size_t size)
{
 	USART_TypeDef *USART=cookie;
 	ssize_t        ssize;
 
 	ssize=0;
 	while ( 0<size )
	 {	while ( ! (USART->ISR & USART_ISR_TXE) );
		USART->TDR=*buff;
        	buff++;
        	size--;
        	ssize++;
  	 };

 	return(ssize);
}*/
/*****************************************************************************/
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
};

/*****************************************************************************/
int color_key(int16_t in)
{
  if (in < -768) in = -768;
  else if(in > 124) in = 124;
  return ((in+768)/32*4);
}

void msleep(int t)
{
 vTaskDelay( t / portTICK_RATE_MS );  
}

/********************************** TASKS *****************************************/

void blink_task(void * parameters) 
{
 while ( 1 ) 
  {	
	msleep(led_delay);
	stm32_gpio_output_toggle(GPIOB,3);
  }
}

void ir_read_task(void *parameters) 

{
 unsigned char	array[64];
 
  //char		buff[32];  ////////

 while ( 1 ) 
  {	
    // Take ir_mutex
    if (xSemaphoreTake(ir_mutex, 0) == pdTRUE) {
  	//int	n;  //kiiratáshoz
  	uint16_t i, j;	  	
  	//int k, n;	 //////////////////
	for (i=0; i<6; i++)
	{
	  uint8_t address[2];
	  address[0]=0x04+i/4;  //0x04 az első 4 alkalommal utána 05
	  //subpage 0:
	  address[1]=(i%4)<<6;  //0x00, 0x40, 0x80 ...
	  //subpage 1:
	  //address[1]=(i%4)<<6|1<<5;  //0x20, 0x60, 0xA0 ...
	  for (j=0; j<32; j++)
  	    {
  	      array[2*j] = 0xBE;
  	      array[2*j+1] = 0xEF; 
  	    }
	
	  stm32_i2c_dma_write_read(I2C1,0x33,address,2,array,64);
	  //k = stm32_i2c_dma_write_read(I2C1,0x33,address,2,array,64);
	  //n=sprintf(buff,"%i \n", k); ////////////////
	  //usart_write_queue(USART2,buff,n);  ///////////////////
	  
	  for (j=0; j<16; j++)
	  {
	    ir_page_0[10-2*i+1][15-j] = (array[2*j]<<8)|array[1+2*j];
	    ir_page_0[10-2*i][15-j] = (array[32+2*j]<<8)|array[33+2*j];
	  }
	}
	// Give back ir_mutex
        xSemaphoreGive(ir_mutex);
	msleep(500);
    } else {
        //if ir_mutex is taken wait
        msleep(50);
    }
  }
}

void ir_print_task(void *parameters) 
{
 char		buff[32];

 while ( 1 ) 
  {	
    // Take ir_mutex
    if (xSemaphoreTake(ir_mutex, 0) == pdTRUE) {
  	int	n;
  	uint16_t i, j;	
	
	
	n=sprintf(buff,"\n Memory: \n");
	usart_write_queue(USART2,buff,n);
	for (i = 0; i<12; i++)
	{
	  for (j=0; j<16; j++)
	  { 
	    n=sprintf(buff," %.4x",ir_page_0[i][j]&0xFFFF);
	    usart_write_queue(USART2,buff,n);
	  }
	  n=sprintf(buff," \n");
	  usart_write_queue(USART2,buff,n);
	}
	n=sprintf(buff,"\n Memory: \n");
	usart_write_queue(USART2,buff,n);
	uint16_t fire, human;
	fire= 0xFFFF;
	human= 0xFED0;
	for (i = 0; i<12; i++)
	{
	  for (j=0; j<16; j++)
	  { 
	    /*
	    if(ir_page_0[i][j] > fire) {n=sprintf(buff,".###");}
	    else if(ir_page_0[i][j] > human) {n=sprintf(buff,".---");}
	    else {n=sprintf(buff,".   ");}
	    //n=sprintf(buff," 0x%.4x",ir_page_0[i][j]&0xFFFF);
	    usart_write_queue(USART2,buff,n);
	    */
	    n = color_key(ir_page_0[i][j]);
	    n=sprintf(buff,"\033[48;2;%i;%i;%im  ",color_map[n], color_map[n+1], color_map[n+2]);
	    //n=sprintf(buff," %i; %i; %i  ",n, color_map[n+1], color_map[n+2]);
	    usart_write_queue(USART2,buff,n);

	    
	  }
	  n=sprintf(buff,"\033[0m\n");
	  usart_write_queue(USART2,buff,n);

	}
	n=sprintf(buff," \n");
	usart_write_queue(USART2,buff,n);
	// Give back ir_mutex
        xSemaphoreGive(ir_mutex);
        msleep(5000);
    } else {
        //if ir_mutex is taken wait
        msleep(50);
    }
  }
}
 


/* https://www.freertos.org/Hardware-independent-RTOS-example.html */

/********************************** MAIN *****************************************/

int main(void)
{
 cookie_io_functions_t  cookie_uart[2];

 stm32_rcc_init();

 RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

 RAM_VECTORS[16+SVC_IRQn	]=(uint32_t *)SVC_Handler;
 RAM_VECTORS[16+PendSV_IRQn	]=(uint32_t *)PendSV_Handler;
 RAM_VECTORS[16+SysTick_IRQn	]=(uint32_t *)SysTick_Handler;
 RAM_VECTORS[16+USART2_IRQn	]=(uint32_t *)USART2_Handler;

 SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_1|SYSCFG_CFGR1_MEM_MODE_0;

 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

 stm32_gpio_mode(GPIOB,3,1);
 stm32_gpio_otype(GPIOB,3,0);
 stm32_gpio_ospeed(GPIOB,3,3);
 stm32_gpio_pupd(GPIOB,3,0);

/* stm32_gpio_output_toggle(GPIOB,3); */

 /* USART2 configuration: */
 RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
 RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
 stm32_gpio_alternate(GPIOA,2,1);
 stm32_gpio_alternate(GPIOA,15,1);
 USART2->BRR=417;
 USART2->CR1 |= USART_CR1_RE|USART_CR1_TE;
 USART2->CR1 |= USART_CR1_UE;
 NVIC_EnableIRQ(USART2_IRQn);
 NVIC_SetPriority(USART2_IRQn,0);
 USART2->CR1 |= USART_CR1_RXNEIE; 

 /* I2C1 configuration: */
 RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
 RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
 stm32_gpio_alternate(GPIOA,9,4);
 stm32_gpio_alternate(GPIOA,10,4);
 stm32_i2c_configure(I2C1);
 I2C1->CR1 |= I2C_CR1_PE;
 
 /* DMA1 configuration */
 RCC->AHBENR  |= RCC_AHBENR_DMA1EN;  // enable pheropherial clock on DMA ??? másolt nem ellenőrzött
 DMA1_Channel3->CCR |= DMA_CCR_MINC;  //memory increment
 
 // Create ir_mutex
  ir_mutex = xSemaphoreCreateMutex();
 
 led_delay = 1000;

//????
 while ( 0 ) 
  {	__DSB();
	__WFI();
	__ISB();
	stm32_gpio_output_toggle(GPIOB,3);
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
/* setlinebuf(stdout); */

 queue_usart_tx=xQueueCreate(32,1);
 queue_usart_rx=xQueueCreate(32,1);


 xTaskCreate(blink_task,
	"blink",
	configMINIMAL_STACK_SIZE,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );
 
 xTaskCreate(ir_read_task,
	"ir_read_task",
	128,
	NULL,
	tskIDLE_PRIORITY + 1,
        NULL );
  
 xTaskCreate(ir_print_task,
	"ir_print_task",
	192,
	NULL,
	tskIDLE_PRIORITY,
        NULL );
        
 
 vTaskStartScheduler();

 for( ;; );
}

