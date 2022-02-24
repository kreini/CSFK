/*****************************************************************************/
/* stm32_i2c.h								     */
/*****************************************************************************/

#ifndef	__STM32_I2C_H_INCLUDED
#define	__STM32_I2C_H_INCLUDED	1

/*****************************************************************************/

void	stm32_i2c_configure(I2C_TypeDef *I2Cx);
void	stm32_i2c_slave(I2C_TypeDef *I2Cx,uint8_t address);
int	stm32_i2c_detect(I2C_TypeDef *I2Cx,uint8_t address);
int	stm32_i2c_write(I2C_TypeDef *I2Cx,uint8_t address,void *data,int length);
int	stm32_i2c_write_multiple(I2C_TypeDef *I2Cx,uint8_t address,void *vdata1,int length1,void *vdata2,int length2);
int	stm32_i2c_write_read(I2C_TypeDef *I2Cx,uint8_t address,void *data,int length,void *out,int olength);

/*****************************************************************************/

#endif

/*****************************************************************************/

