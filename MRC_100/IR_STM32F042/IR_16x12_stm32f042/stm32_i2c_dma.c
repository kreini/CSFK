#include <stdint.h>
#include <stm32f0xx.h>

#include "delay_basic.h"
#include "stm32_i2c_dma.h"

void stm32_i2c_configure(I2C_TypeDef *I2Cx)
{
 I2Cx->TIMINGR=0xB0420F13;		/* 100 kHz @48MHz clock */
 //I2Cx->TIMINGR=0x50330309;		/* 400 kHz @48MHz clock */
 //I2Cx->TIMINGR=0x50100103;		/*   1 MHz @48MHz clock */
}

void stm32_i2c_slave(I2C_TypeDef *I2Cx,uint8_t address)
{
 I2Cx->OAR1=((address&0x7F)<<1)|I2C_OAR1_OA1EN;
}


static void stm32_i2c_wait(I2C_TypeDef *I2Cx)
{
 while ( ! ( I2Cx->ISR & (I2C_ISR_NACKF|I2C_ISR_STOPF|I2C_ISR_TXIS|I2C_ISR_ARLO|I2C_ISR_BERR) ) );
}

#define	stm32_i2c_transfer_fail(I2Cx) (I2Cx->ISR & (I2C_ISR_NACKF|I2C_ISR_STOPF|I2C_ISR_ARLO|I2C_ISR_BERR))

static void stm32_i2c_recover(I2C_TypeDef *I2Cx)
{
 if ( I2Cx->ISR & I2C_ISR_STOPF )	I2Cx->ICR |= I2C_ICR_STOPCF;
 if ( I2Cx->ISR & I2C_ISR_NACKF )	I2Cx->ICR |= I2C_ICR_NACKCF;
 if ( I2Cx->ISR & I2C_ISR_ARLO  )	I2Cx->ICR |= I2C_ICR_ARLOCF;
 if ( I2Cx->ISR & I2C_ISR_BERR  )	I2Cx->ICR |= I2C_ICR_BERRCF;
}

int stm32_i2c_detect(I2C_TypeDef *I2Cx,uint8_t address)
{
 uint32_t	pe;
 int		r;

 I2Cx->CR2 = ((1&0xFF)<<16) | ((address&0x7F)<<1);

 pe = I2Cx->CR1 & I2C_CR1_PE;

 I2Cx->CR1 |= I2C_CR1_PE;
 I2Cx->CR2 |= I2C_CR2_START;
 
 stm32_i2c_wait(I2Cx);

 if ( I2Cx->ISR & I2C_ISR_NACKF )
	r=0;
 else
	r=1;

 stm32_i2c_recover(I2Cx);

 if ( ! pe )	I2Cx->CR1 &= ~I2C_CR1_PE;

 return(r);
}


int stm32_i2c_write(I2C_TypeDef *I2Cx,uint8_t address,void *vdata,int length)
{
 uint32_t	pe;
 uint8_t	*data=vdata;
 int		r;

 /* only small writes are supported at this moment - but this is fine: */
 if ( 255<length )
	return(0);
 else if ( length<=0 )
	return(0);

 while ( stm32_i2c_is_busy(I2Cx) && ! (I2Cx->ISR & (I2C_ISR_ADDR|I2C_ISR_RXNE)) );

 if ( I2Cx->ISR & (I2C_ISR_ADDR|I2C_ISR_RXNE) )
	return(0);

 I2Cx->CR2 = I2C_CR2_AUTOEND | ((length&0xFF)<<16) | ((address&0x7F)<<1);

 pe = I2Cx->CR1 & I2C_CR1_PE;

 I2Cx->CR1 |= I2C_CR1_PE;
 I2Cx->CR2 |= I2C_CR2_START;
 stm32_i2c_wait(I2Cx);

 if ( stm32_i2c_transfer_fail(I2Cx) )
  {	uint32_t	isr;

	isr=I2Cx->ISR;		

	stm32_i2c_recover(I2Cx);

 	if ( ! pe )
		I2Cx->CR1 &= ~I2C_CR1_PE;

	if ( isr & I2C_ISR_ARLO )
		return(0);
	else
		return(-1);
  }

 r=1;

 while ( 1<length )
  {	I2Cx->TXDR=*data;
	data++;
	stm32_i2c_wait(I2Cx);

	if ( stm32_i2c_transfer_fail(I2Cx) )
	 {	uint32_t	isr;

		isr=I2Cx->ISR;

		stm32_i2c_recover(I2Cx);

	 	if ( ! pe )
			I2Cx->CR1 &= ~I2C_CR1_PE;

		if ( isr & I2C_ISR_ARLO )
			return(r);
		else
	        	return(-1);
	 }

	length--;
	r++;
  }

 I2Cx->TXDR=*data;
 r++;

 while ( ! (I2Cx->ISR & I2C_ISR_STOPF) );
 I2Cx->ICR |= I2C_ICR_STOPCF;

 if ( ! pe )
	I2Cx->CR1 &= ~I2C_CR1_PE;

 return(r);
}

//DMA
int stm32_i2c_dma_write_read(I2C_TypeDef *I2Cx,uint8_t address,void *vdata,int length,void *vout,int olength)
{
 uint8_t *data = vdata;
 uint8_t *out  = vout;

 if ( I2Cx->ISR & I2C_ISR_STOPF )
	I2Cx->ICR |= I2C_ICR_STOPCF;

 // only small writes are supported at this moment - but this is fine: 
 if ( 255<length )
	return(1);
 else if ( length<=0 )
	return(2);

 I2Cx->CR1 &= ~I2C_CR1_RXDMAEN; //DMA disabled
 I2Cx->CR1 |= I2C_CR1_PE; // pheripheril enabled
 I2Cx->CR2 = ((length&0xFF)<<16) | ((address&0x7F)<<1) | I2C_CR2_START;
 

 stm32_i2c_wait(I2Cx);  

 if ( stm32_i2c_transfer_fail(I2Cx) )
  {	  
  	stm32_i2c_recover(I2Cx);
	I2Cx->CR1 &= ~I2C_CR1_PE; 
        return(3);
  }


 while ( 1<length )
  {	I2Cx->TXDR=*data;
	data++;
	stm32_i2c_wait(I2Cx);
	if ( stm32_i2c_transfer_fail(I2Cx) )
	 {	stm32_i2c_recover(I2Cx);
		//I2Cx->CR1 &= ~I2C_CR1_PE; 
	        return(4);
	 }
	length--;
  }
 I2Cx->TXDR=*data;
 while ( ! (I2Cx->ISR & I2C_ISR_TC) );

 //DMA:
 DMA1_Channel3->CCR &= ~DMA_CCR_EN;  //DMA disable
 DMA1_Channel3->CPAR = (uint32_t) (&(I2Cx->RXDR));  //Peripherial data register address
 DMA1_Channel3->CMAR = (uint32_t) (out);  //Memory address
 DMA1_Channel3->CNDTR = olength;  //length in Bytes
 DMA1_Channel3->CCR |= DMA_CCR_PL_0 | DMA_CCR_EN;  //prio: 1 , error DMA enable


 I2Cx->CR1 |= I2C_CR1_RXDMAEN | I2C_CR1_PE; //DMA reception requests and pheripheril enabled after DMA is set and enabled
 I2Cx->CR2 = I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | ((olength&0xFF)<<16) | ((address&0x7F)<<1) | I2C_CR2_START;
 
 
 while (!(DMA1-> ISR & DMA_ISR_TCIF3));  //While DMA3 transfer not complite
 DMA1->IFCR = DMA_IFCR_CTCIF3;           //While DMA3 transfer complite
 

 return(0);
}




