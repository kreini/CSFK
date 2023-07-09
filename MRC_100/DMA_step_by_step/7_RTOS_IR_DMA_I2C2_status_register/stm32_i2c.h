/*****************************************************************************/
/* stm32_i2c.h								     */
/*****************************************************************************/

#ifndef	__STM32_I2C_H_INCLUDED
#define	__STM32_I2C_H_INCLUDED	1

/*****************************************************************************/

#define	stm32_i2c_is_busy(I2Cx)		((I2Cx)->ISR & I2C_ISR_BUSY)
#define stm32_i2c_transfer_fail(I2Cx)	(I2Cx->ISR & (I2C_ISR_NACKF|I2C_ISR_STOPF|I2C_ISR_ARLO|I2C_ISR_BERR))

void	stm32_i2c_configure(I2C_TypeDef *I2Cx);
void	stm32_i2c_slave(I2C_TypeDef *I2Cx,uint8_t address);
int	stm32_i2c_detect(I2C_TypeDef *I2Cx,uint8_t address);
int	stm32_i2c_write(I2C_TypeDef *I2Cx,uint8_t address,void *data,int length);
int	stm32_i2c_write_read(I2C_TypeDef *I2Cx,uint8_t address,void *data,int length,void *out,int olength);

void	stm32_i2c_wait(I2C_TypeDef *I2Cx);
void	stm32_i2c_recover(I2C_TypeDef *I2Cx);

/*****************************************************************************/

#endif

/*****************************************************************************/

