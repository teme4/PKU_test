#include "stm32f407xx.h"
#include "hardware_config.hpp"
#include "i2c.hpp"
#include "gpio.hpp"
#include "lcd.hpp"



void i2c:: i2c_init(I2C_TypeDef *I2Cx)
{
RCC->APB1ENR|=RCC_APB1ENR_I2C3EN; 
//указываем частоту тактирования модуля
I2C2->CR2 &= ~I2C_CR2_FREQ;
I2Cx->CR2|=42;
  //задаем частоту работы модуля SCL по формуле 10 000nS/(2* TPCLK1) 
I2Cx->CCR|=208;//10 000ns/48ns = 208
//Standart_Mode = 1000nS, Fast_Mode = 300nS, 1/42MHz = 24nS
I2Cx->TRISE=42+1;//(1000nS/24nS)+1
//настраиваем модуль в режим I2C
I2Cx->CR1&= ~I2C_CR1_SMBUS;
//конфигурируем I2C, standart mode, 100 KHz duty cycle 1/2	
I2Cx->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY);
I2Cx->CR1 |= I2C_CR1_ACK; 
 //включаем модуль
I2Cx->CR1 |= I2C_CR1_PE;
}

volatile uint8_t adr[30]={0,},k=0;
uint8_t i2c::CMSIS_I2C_Scan(I2C_TypeDef *I2Cx,uint8_t addr) 
{
    
	I2Cx->CR1&=~(I2C_CR1_POS);//Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    I2Cx->CR1|= I2C_CR1_START;
   	while ((I2Cx->SR1 & I2C_SR1_SB) == 0) {
       }		//Ожидаем до момента, пока не сработает Start condition generated
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью данных в регистр DR или когда PE=0*/
	I2Cx->SR1;
	I2Cx->DR = (addr); //Адрес + Write	
	while (!(I2Cx->SR1 & I2C_SR1_AF) && (I2Cx->SR1 & I2C_SR1_ADDR)) {
    }//Ждем, пока адрес отзовется
	if (!(I2Cx->SR1 & I2C_SR1_ADDR)) 
        {//Если устройство отозвалось
        I2Cx->CR1|= I2C_CR1_STOP;//Отправляем сигнал STOP
        delay_ms(1);
    	I2Cx->SR1;
		I2Cx->SR2;		
         delay_ms(2);
        return true;
		}
		else
          {           
          adr[k]=addr;
          k++;      
          return false; 
          }
        
          }

void  i2c::I2C_Write(I2C_TypeDef *I2Cx,uint8_t reg_addr, uint8_t *data,uint8_t count,uint8_t i2c_addr)
{
        //стартуем
    I2Cx->CR1 |= I2C_CR1_START;		
	while(!(I2Cx->SR1 & I2C_SR1_SB)){};
	(void) I2Cx->SR1;
		
        //передаем адрес устройства
	I2Cx->DR = i2c_addr;
	while(!(I2Cx->SR1 & I2C_SR1_ADDR)){};
	(void) I2Cx->SR1;
	(void) I2Cx->SR2;
		
        //передаем адрес регистра
	I2Cx->DR = reg_addr;	
	while(!(I2Cx->SR1 & I2C_SR1_TXE)){};	
			
        //пишем данные	
        for(int n=0;n<count;n++)
        {
    I2Cx->DR = data[n];	
	while(!(I2Cx->SR1 & I2C_SR1_BTF)){};
        }
		
	I2Cx->CR1 |= I2C_CR1_STOP;		
}

uint8_t  i2c::I2C_Read(I2C_TypeDef *I2Cx,uint8_t reg_addr,uint8_t bytes_count,uint8_t i2c_adr)
{

 /*--- Start-bit ------------------- */
I2Cx->CR1 |= I2C_CR1_START;                  // Start bit
while(!(I2Cx->SR1 & I2C_SR1_SB)) { }         // ожидается готовность
/*----------------------------------*/

/*--- Передача адреса устройства ---*/
I2Cx->DR =i2c_adr;   // передается адрес

while((I2Cx->SR1& I2C_SR1_ADDR) == 0) { } // ожидается готовность
_tmp = I2Cx->SR2;
/*-----------------------------------*/

/*--- Передача адреса регистра ------*/
I2Cx->DR = reg_addr;                        // регистр 0xFF
//while((I2Cx->SR1& I2C_SR1_TXE) == 0) { } // ожидается прием ACK
/*-----------------------------------*/

/*--- Ristort-bit ------------------ */
I2Cx->CR1 |= I2C_CR1_START;                  // Ristort bit
while((I2Cx->SR1& I2C_SR1_SB) == 0) { }   // ожидается готовность
/*----------------------------------*/

/*--- Бит подтверждения приема -----*/
I2Cx->CR1 |= I2C_CR1_ACK;
/*----------------------------------*/

/*--- Передача адреса устройства ---*/
I2Cx->DR = i2c_adr|0x01;//READ
while((I2Cx->SR1& I2C_SR1_ADDR) == 0) { } // ожидается готовность
_tmp = I2Cx->SR2;
/*-----------------------------------*/

/*=== ПРИНИМАЮТСЯ БАЙТЫ =============*/
for(int i=0;i<bytes_count;i++)
{
if(i==(bytes_count-1))  
{
  I2Cx->CR1 &= ~I2C_CR1_ACK;  
}  
while((I2Cx->SR1& I2C_SR1_RXNE) == 0)   // ожидается прием
{ }
rxdata[i] = I2Cx->DR; 
}

/*--- Stop-bit -------------------- */
I2Cx->CR1 |= I2C_CR1_STOP;                  // Stop bit
/*----------------------------------*/   
return *rxdata;
}