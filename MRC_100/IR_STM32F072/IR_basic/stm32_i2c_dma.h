/*****************************************************************************/
/* stm32_i2c_dma.h								     */
/*****************************************************************************/

#ifndef	__STM32_I2C_H_INCLUDED
#define	__STM32_I2C_H_INCLUDED	1

/*****************************************************************************/

#define	stm32_i2c_is_busy(I2Cx)		((I2Cx)->ISR & I2C_ISR_BUSY)

void	stm32_i2c_configure(I2C_TypeDef *I2Cx);
void	stm32_i2c_slave(I2C_TypeDef *I2Cx,uint8_t address);
int	stm32_i2c_detect(I2C_TypeDef *I2Cx,uint8_t address);
int	stm32_i2c_write(I2C_TypeDef *I2Cx,uint8_t address,void *data,int length);
int	stm32_i2c_dma_write_read(I2C_TypeDef *I2Cx,uint8_t address,void *vdata,int length,void *vout,int olength);

/*****************************************************************************/

#endif

/*****************************************************************************/

