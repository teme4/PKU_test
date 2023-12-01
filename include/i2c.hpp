#include "stm32f407xx.h"


class i2c
{
private: 
uint8_t _rxdata[16]={0,},_tmp;
public:
uint8_t rxdata[16]={0,};
uint8_t I2C_Read(I2C_TypeDef *I2Cx,uint8_t reg_addr,uint8_t bytes_count,uint8_t i2c_adr);
void I2C_Write(I2C_TypeDef *I2Cx,uint8_t reg_addr, uint8_t *data,uint8_t count,uint8_t i2c_addr);
uint8_t CMSIS_I2C_Scan(I2C_TypeDef *I2Cx,uint8_t addr);
void i2c_init(I2C_TypeDef *I2Cx); 
};



