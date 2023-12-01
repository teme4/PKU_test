#include "stm32f407xx.h"

#include "hardware_config.hpp"
#include "gpio.hpp"
#include "usart.hpp"
#include "lcd.hpp"
#include "i2c.hpp"
#include "muxs.hpp"

#include "stdio.h"
#include <string.h>

/* Wait till HSE is ready */
#define HSE_STARTUP_TIMEOUT ((uint32_t)100) /*!< Time out for HSE start up, in ms */
#define HSE_TIMEOUT_VALUE HSE_STARTUP_TIMEOUT

#define CONFIG 0x00 //'Config' register address
#define EN_AA 0x01 //'Enable Auto Acknowledgment' register address
#define EN_RXADDR 0x02 //'Enabled RX addresses' register address
#define SETUP_AW 0x03 //'Setup address width' register address
#define SETUP_RETR 0x04 //'Setup Auto. Retrans' register address
#define RF_CH 0x05 //'RF channel' register address
#define RF_SETUP 0x06 //'RF setup' register address
#define STATUS 0x07 //'Status' register address
#define RX_ADDR_P0 0x0A //'RX address pipe0' register address
#define RX_ADDR_P1 0x0B //'RX address pipe1' register address
#define TX_ADDR 0x10 //'TX address' register address
#define RX_PW_P0 0x11 //'RX payload width, pipe0' register address
#define RX_PW_P1 0x12 //'RX payload width, pipe1' register address
#define FIFO_STATUS 0x17 //'FIFO Status Register' register address
#define DYNPD 0x1C
#define FEATURE 0x1D

#define led_H GPIOA->ODR |= (1<<2);
#define led_L GPIOA->ODR &= ~ (1<<2);
  


#define CE_RESET STM32F407.set_pin_state(GPIOA,gpio_spi.ce,0)
#define CE_SET STM32F407.set_pin_state(GPIOA,gpio_spi.ce,1)
#define cs_l STM32F407.set_pin_state(GPIOC,gpio_spi.cs,0)
#define cs_h STM32F407.set_pin_state(GPIOC,gpio_spi.cs,1)


#define TX_ADR_WIDTH 3
#define TX_PLOAD_WIDTH 5
uint16_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb3,0xb4,0x01};
uint16_t RX_BUF[TX_PLOAD_WIDTH] = {0};
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2




gpio STM32F407;
gpio_mux_led leds;
gpio_lcd gpio_lcds;
gpio_mux_ETH eth;
gpio_mux_SW sw;
gpio_usart gpio_usart1;
gpio_i2c i2_gpio;
gpio_SPI gpio_spi;
gpio_swithes gpio_sw_a;



i2c ina260;
lcd oled;
usart usart1;
muxs_ muxs;

 char str[80];
uint16_t tx_data[16]={0,};
uint16_t rx_data[16]={0,};
volatile uint16_t NRF_data=0;

void gpio_init()
{     
//MUX_LEDS
STM32F407.gpio_init(GPIOC,leds.gpio_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);    
STM32F407.gpio_init(GPIOC,leds.gpio_2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOC,leds.gpio_3,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOC,leds.gpio_4,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOC,leds.gpio_5,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOC,leds.gpio_6,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOC,leds.gpio_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOC,leds.gpio_8,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOA,0,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOE,14,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 

STM32F407.gpio_init(GPIOD,5,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.set_pin_state(GPIOD,5,0);
//LCD
STM32F407.gpio_init(GPIOD,gpio_lcds.RS,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOD,gpio_lcds.RW,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOD,gpio_lcds.EN,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOD,gpio_lcds.data_4,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOD,gpio_lcds.data_5,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOD,gpio_lcds.data_6,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOD,gpio_lcds.data_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
//MUX_ETH
STM32F407.gpio_init(GPIOD,eth.gpio_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOD,eth.gpio_2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOD,eth.gpio_3,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOD,eth.gpio_4,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOA,eth.en,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOA,eth.in_out,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
//IN_ETH
STM32F407.gpio_init(GPIOE,eth.in_0,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOE,eth.in_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOE,eth.in_2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOE,eth.in_3,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOE,eth.in_4,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOE,eth.in_5,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOE,eth.in_6,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOE,eth.in_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high); 
//MUX_SW
STM32F407.gpio_init(GPIOB,sw.s0,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOB,sw.s1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);   
STM32F407.gpio_init(GPIOD,sw.s2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
STM32F407.gpio_init(GPIOD,sw.s3,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOA,sw.en,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOA,sw.state,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high); 
//USART1
STM32F407.gpio_init(GPIOA,gpio_usart1.RX,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high); 
STM32F407.gpio_init(GPIOA,gpio_usart1.TX,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_very_high); 
STM32F407.config_af(GPIOA,gpio_usart1.RX,0x7);
STM32F407.config_af(GPIOA,gpio_usart1.TX,0x7);
//I2C_2
STM32F407.gpio_init(GPIOA,i2_gpio.SCL,gpio_type::gpio_type_od,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOC,i2_gpio.SDA,gpio_type::gpio_type_od,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_high); 
STM32F407.config_af(GPIOA,i2_gpio.SCL,0x4);
STM32F407.config_af(GPIOC,i2_gpio.SDA,0x4);
//SPI3
STM32F407.gpio_init(GPIOC,gpio_spi.miso,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOC,gpio_spi.mosi,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOC,gpio_spi.cs,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOC,gpio_spi.sck,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_alternate,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOA,gpio_spi.ce,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.config_af(GPIOC,gpio_spi.miso,0x6);
STM32F407.config_af(GPIOC,gpio_spi.mosi,0x6);
STM32F407.config_af(GPIOC,gpio_spi.sck,0x6);
//GPIOC->PUPDR|=(1)<<13;
//SWITCHES
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_1,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_2,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_3,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_4,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_5,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_6,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
STM32F407.gpio_init(GPIOB,gpio_sw_a.EN_8,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_high); 
}





void init_RCC()
{
RCC->CR |= RCC_CR_HSEON;
uint32_t stortup_counter = 0, hse_status;
do 
{
hse_status = RCC->CR & RCC_CR_HSERDY;
++stortup_counter;
} 
while((!hse_status) && (stortup_counter != HSE_STARTUP_TIMEOUT));
if (hse_status) 
{
//Отчищаем значиния множителей из регистра PLLCFGR
RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP);
//Выстанавливаем множители
//С PLLP ничего не делаем - нули дают значение делителя 2
//В PLLN записываем 336
//В PLLM записываем 8
RCC->PLLCFGR |= (168 << RCC_PLLCFGR_PLLN_Pos)|RCC_PLLCFGR_PLLM_3;
//Отчищаем значиния множителей из регистра CFGR
RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
//Выставляем множители
RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
//Включаем PLL
RCC->CR |= RCC_CR_PLLON;
//Ждем ита готовности PLL
while (!(RCC->CR & RCC_CR_PLLRDY)) {
}
//Отчищаем SW (System clock switch) в регистре CFGR
RCC->CFGR &= !RCC_CFGR_SW;
//Выставляем PLL в качестве источника частоты
RCC->CFGR |= RCC_CFGR_SW_PLL;
//Ждем пока статус не переидет в PLL
while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
}
}
}

void PKU_stat()
{
volatile double volt=0,current=0,power=0;
ina260.I2C_Read(I2C3,0x1,2,64);
current=((((ina260.rxdata[0]<<8)|ina260.rxdata[1])*1.25)/1000);
ina260.I2C_Read(I2C3,0x2,2,64);
volt=((((ina260.rxdata[0]<<8)|ina260.rxdata[1])*1.25)/1000);
power=current*volt;

oled.fake_ClearLCD();
oled.Cursor(0,0); 
oled.PrintStr("Vdd: ");
sprintf(str, "%d.%dV", (int)volt, (int)(volt * 100) % 100);
oled.PrintStr(str);
oled.Cursor(1,0); 
oled.PrintStr("I:   ");
sprintf(str, "%d.%dA", (int)current, (int)(current * 100) % 100);
oled.PrintStr(str);
delay_ms(1000);
}


uint16_t spi_transmit_1byte(uint16_t data)
{
    cs_l;   
      SPI3->DR=data;
      while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
      while(!(SPI3->SR & SPI_SR_RXNE)); // wait until receive complete
      while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore    
    cs_h;
 return SPI3->DR; // return received data from SPI data register
}

uint16_t NRF_write_reg(uint16_t reg, uint16_t data)
{
    reg|=0x20;

 cs_l;   
      SPI3->DR=reg;
      while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
      while(!(SPI3->SR & SPI_SR_RXNE)); // wait until receive complete
      while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore    
       SPI3->DR=data;
      while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
      while(!(SPI3->SR & SPI_SR_RXNE)); // wait until receive complete
      while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore  
    cs_h;     
 }

uint16_t spi_transmit(uint16_t *data, int32_t len)
{
    cs_l;
    for(int i=0; i<len;i++)
    {
      SPI3->DR=data[i];
      while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
      while(!(SPI3->SR & SPI_SR_RXNE)); // wait until receive complete
      while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore    
    }   
     cs_h;
 return SPI3->DR; // return received data from SPI data register
}



uint16_t spi_receive_1byte(uint16_t reg)
{
  cs_l;
  SPI3->DR=reg;
  while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
  SPI3->DR = 0; //запускаем обмен  
  while(!(SPI3->SR & SPI_SR_RXNE));  
  while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore 
  cs_h;
  return SPI3->DR;
}
uint16_t spi_receive(uint16_t *data,uint16_t reg,uint16_t len)
{
    cs_l;
    SPI3->DR=reg;
    while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
    while(!(SPI3->SR & SPI_SR_RXNE)); // wait until receive complete
    while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore  
   data[15]=SPI3->DR;  
     
  for(int i=0; i<len;i++)
    {
   SPI3->DR = 0; //запускаем обмен  
   while(!(SPI3->SR & SPI_SR_TXE)); // wait until transmit complete
   while(!(SPI3->SR & SPI_SR_RXNE));  
    while(SPI3->SR & SPI_SR_BSY); // wait until SPI is not busy anymore  
   data[i]=SPI3->DR;
     }
     cs_h;
  return *data;
}

static void TIM2_Init(void)
{
  uint32_t tmpcr1, tmpcr2, tmpccer, tmpccmr2;
  //SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
  RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
  tmpcr1 = TIM2->CR1;
 // &=~
  TIM2->CR1 &=~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
  //CLEAR_BIT(tmpcr1, TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);
  TIM2->CR1|=tmpcr1;
  //Set the auto-reload value
  TIM2->ARR= 65535;
  //Set the prescaler value
 TIM2->PSC= 0;
  //Generate an update event to reload the Prescaler
  //and the repetition counter value (if applicable) immediately
  TIM2->EGR|=TIM_EGR_UG;
  //Disable auto-reload
  TIM2->CR1&=~TIM_CR1_ARPE;
  //Set clock source internal
  TIM2->SMCR&=~( TIM_SMCR_SMS | TIM_SMCR_ECE);
  //CH3 AND CH4 Enable Preload
  TIM2->CCMR2|=(TIM_CCMR2_OC4PE | TIM_CCMR2_OC3PE);
  //Disable the Channel 3 and 4: Reset the CC3E and CC4E Bits
  TIM2->CCER&=~(TIM_CCER_CC4E | TIM_CCER_CC3E);
  //Get the TIM2 CCER register value
  tmpccer = TIM2->CCER;
  //Get the TIM2 CR2 register value
  tmpcr2 = TIM2->CR2;
  //Get the TIM2 CCMR2 register value
  tmpccmr2 = TIM2->CCMR2;
  //Reset Capture/Compare selection Bits
  TIM2->CCMR2&=~(TIM_CCMR2_CC4S | TIM_CCMR2_CC3S);
  //Select the Output Compare Mode
  TIM2->CCMR2|= (TIM_CCMR2_OC4M | TIM_CCMR2_OC3M|TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 |TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
  //Set the Output Compare Polarity
  TIM2->CCER&=~(TIM_CCER_CC4P | TIM_CCER_CC3P);
  //Set the Output State
   TIM2->CCER&=~(TIM_CCER_CC4E | TIM_CCER_CC3E);
  //Write to TIM2 CR2
  TIM2->CR2=tmpcr2;
  //Write to TIM2 CCMR2
  TIM2->CCMR2=tmpccmr2;
  //Set the Capture Compare Registers value
  TIM2->CCR3= 0;
  TIM2->CCR4=0;
  //Write to TIM2 CCER
  TIM2->CCER=tmpccer;
  //TIM2 OC Disable Fast
  TIM2->CCMR2&=~(TIM_CCMR2_OC4FE | TIM_CCMR2_OC3FE);
  //Disable Master Mode Selection
  TIM2->CR2&=~ TIM_CR2_MMS;
  //Disable Master/Slave mode
  TIM2->SMCR&=~TIM_SMCR_MSM;
}

void spi_nit()
{
RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
     
     SPI3->CR1 = 0<<SPI_CR1_DFF_Pos  //Размер кадра 8 бит
    | 0<<SPI_CR1_LSBFIRST_Pos     //MSB first
    | 0<<SPI_CR1_SSM_Pos          //Программное управление SS
    | 1<<SPI_CR1_SSI_Pos          //SS в высоком состоянии
    | 0x07<<SPI_CR1_BR_Pos       //Скорость передачи: F_PCLK/32
    | 1<<SPI_CR1_MSTR_Pos        //Режим Master (ведущий) /////////////////1   
    | 0<<SPI_CR1_CPHA_Pos      
    | 0<<SPI_CR1_CPOL_Pos;    
    SPI3->CR2 =1<<SPI_CR2_SSOE_Pos;    
    SPI3->CR1 |= 1<<SPI_CR1_SPE_Pos; //Включаем SPI
}
void NRF24_FlushRX(void)
{
  uint16_t dt= FLUSH_RX;
   spi_receive_1byte(dt);
 // HAL_SPI_Transmit(&hspi1,dt,1,1000);
  delay_us(10);
  
}
//------------------------------------------------
void NRF24_FlushTX(void)
{
  uint16_t dt = FLUSH_TX;
  spi_receive_1byte(dt);
  delay_us(10);

}

void NRF_Read_Buf(uint16_t addr,uint16_t *pBuf,uint16_t bytes)
{
   cs_l; 
   //spi_transmit_1byte(addr);
   spi_receive(pBuf,addr,bytes);
   //HAL_SPI_Receive(&hspi1,pBuf,bytes,1000);//îòïðàâèì äàííûå â áóôåð
   cs_h;
}
//------------------------------------------------
void NRF_Write_Buf(uint16_t addr,uint16_t *pBuf,uint16_t bytes)
{
  addr |= 0x20;//âêëþ÷èì áèò çàïèñè â àäðåñ
  cs_l; 
  //spi_transmit(pBuf,bytes+1); 
  spi_receive(pBuf,addr,bytes+1);
  cs_h;
}

void NRF_ToggleFeatures(void)
{
  uint16_t dt= 0x55;
  cs_l; 
  spi_receive_1byte(dt);
  delay_us(5);
  dt = 0x73;
  spi_receive_1byte(dt);
  cs_h; 
}

void NRF24_ini(void)
{
	CE_SET;
  delay_ms(120);
 	NRF_write_reg(CONFIG, 0x0a); // Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
  delay_ms(5);
	NRF_write_reg(EN_AA,0x01); // Enable Pipe1
  delay_ms(1);
	NRF_write_reg(EN_RXADDR, 0x01); // Enable Pipe1
   delay_ms(1);
	NRF_write_reg(SETUP_AW, 0x01); // Setup address width=3 bytes
   delay_ms(1);
	NRF_write_reg(SETUP_RETR, 0x5F); // // 1500us, 15 retrans
   delay_ms(1);
	NRF_ToggleFeatures();
	NRF_write_reg(FEATURE, 0x00);
   delay_ms(1);
	NRF_write_reg(DYNPD, 0x00);
   delay_ms(1);
	NRF_write_reg(STATUS, 0x70); //Reset flags for IRQ
   delay_ms(1);
	NRF_write_reg(RF_CH, 0x4C); // ������� 2476 MHz
   delay_ms(1);
	NRF_write_reg(RF_SETUP, 0x06); //TX_PWR:0dBm, Datarate:1Mbps
   delay_ms(1);
	NRF_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
   delay_ms(1);
	NRF_Write_Buf(RX_ADDR_P1, TX_ADDRESS, TX_ADR_WIDTH);
   delay_ms(1);
	NRF_write_reg(RX_PW_P1, TX_PLOAD_WIDTH); //Number of bytes in RX payload in data pipe 1
 //���� ������ � ����� ��������
}

uint8_t NRF24_ReadReg(uint16_t reg)
{
      uint16_t dt=0, cmd;
     spi_transmit_1byte(reg);
      delay_us(10);
    spi_receive_1byte(dt);
   if (reg!=STATUS)//åñëè àäðåñ ðàâåí àäðåñ ðåãèñòðà ñòàòóñ òî è âîçâàðùàåì åãî ñîñòîÿíèå
  {
    cmd=0xFF;
    spi_transmit_1byte(cmd);
    delay_us(10);
    spi_receive_1byte(dt); 
  }  
  return dt;
}
/*
void usart_nrf_reg(char a[80])
{
sprintf(str, "\033[1;38;5;68m"," \033[0m\r\n");
uart_tx_data(str);
sprintf(str, 
  );
uart_tx_data(str);
sprintf(str, "0x","\r\n");
uart_tx_data(str);
sprintf(str,"%02X",rx_data[0]);
uart_tx_data(str);
sprintf(str, "\r\n");
uart_tx_data(str);
}
*/

char _str[80];
uint8_t _temp,_n;

void BME280_init()
{          
tx_data[0]=0xE0;
tx_data[1]=0xB6; 
spi_transmit(tx_data,2);

tx_data[0]=0xF2;
tx_data[1]=0x01; 
spi_transmit(tx_data,2);

tx_data[0]=0xF4;
tx_data[1]=0x97; 
spi_transmit(tx_data,2);

tx_data[0]=0xF5;
tx_data[1]=0x70; 
spi_transmit(tx_data,2);

  /*
uint8_t temp[16];
//uint16_t reg_temp,var1,var2,t_fine,t;
temp[0]=0x01;
//ina260.I2C_Write(I2C3, 0xF2,temp, 1,0xEC); 
temp[0]=((0x05 << 5) | (0x05 << 2) | 0x03);
//ina260.I2C_Write(I2C3, 0xF4,temp , 1,0xEC);    
temp[0]=((0x03 << 5) | (0x04 << 2));     
ina260.I2C_Write(I2C3, 0xF5,temp , 1,0xEC);    
*/                        			                      //
}

void BME280_reg()
{
//ina260.I2C_Read(I2C3,0xFA,1,0xEC);
spi_receive(rx_data,0xFA,2);
uart_tx_data("0xFA:");
sprintf(str, "%02X",rx_data[0]);
uart_tx_data(str);
uart_tx_data("\r\n");

//ina260.I2C_Read(I2C3,0xFB,1,0xEC);
spi_receive(rx_data,0xFB,2);
uart_tx_data("0xFB:");
sprintf(str, "%02X",rx_data[0]);
uart_tx_data(str);
uart_tx_data("\r\n");

//ina260.I2C_Read(I2C3,0xFC,1,0xEC);
spi_receive(rx_data,0xFC,2);
uart_tx_data("0xFC:");
sprintf(str, "%02X",rx_data[0]);
uart_tx_data(str);
uart_tx_data("\r\n");

//ina260.I2C_Read(I2C3,0x88,1,0xEC);
spi_receive_1byte(0x88);
uart_tx_data("0x88:");
sprintf(str, "%02X",ina260.rxdata[0]);
uart_tx_data(str);
uart_tx_data("\r\n");

//ina260.I2C_Read(I2C3,0x8A,1,0xEC);
spi_receive_1byte(0x8A);
uart_tx_data("0x8A:");
sprintf(str, "%02X",ina260.rxdata[0]);
uart_tx_data(str);
uart_tx_data("\r\n");

//ina260.I2C_Read(I2C3,0x8C,1,0xEC);
spi_receive_1byte(0x8C);
uart_tx_data("0x8C:");
sprintf(str, "%02X",ina260.rxdata[0]);
uart_tx_data(str);
uart_tx_data("\r\n");
uart_tx_data("****************************************************************************");
uart_tx_data("\r\n");
}
void led_1()
{
led_H;
delay_ns(600);
led_L;
delay_ns(650);
}
void led_w()
{
  for (int i=0;i<24;i++)
  {
  led_1();
  }
delay_us(50);
}

int main(void)
{

init_RCC();
gpio_init();
usart1.usart_init();
/*
ina260.i2c_init(I2C3);

spi_nit();
//TIM2_Init();

uint8_t sw_state=0;

STM32F407.set_pin_state(GPIOA,leds.EN1,0);
STM32F407.set_pin_state(GPIOE,leds.EN2,0);
for(int i=1;i<33;i++)
{
muxs.mux_leds(i);
delay_ms(25);
}
oled.InitializeLCD();
oled.InitializeLCD();
oled.InitializeLCD();
oled.InitializeLCD(); //Инициализация дисплея
oled.ClearLCDScreen();
oled.PrintStr("Start");
*/
/*
for(int i=1;i<33;i++)
{
mux_leds(i);
}*/
/*
STM32F407.set_pin_state(GPIOA,leds.EN1,0);
STM32F407.set_pin_state(GPIOE,leds.EN2,0);*/

/*
\033[37;1;41m
\033[38;2;⟨r⟩;⟨g⟩;⟨b⟩m — цвет текста
\033[48;2;⟨r⟩;⟨g⟩;⟨b⟩m — цвет фона
\033[01;38;05;226m-желтый
\033[01;38;05;68m - голубой
*/

/*
NRF24_ini();
delay_ms(100);

sprintf(str,"\033[01;38;05;226m Значение регистров NRF2401: \033[0m");
uart_tx_data(str);
sprintf(str,"\r\n");
uart_tx_data(str);   


spi_receive(rx_data,CONFIG,1);
usart_nrf_reg("CONFIG: ");
spi_receive(rx_data,EN_AA,1);
usart_nrf_reg("EN_AA: ");
spi_receive(rx_data,EN_RXADDR,1);
usart_nrf_reg("EN_RXADDR: ");
spi_receive(rx_data,SETUP_AW,1);
usart_nrf_reg("SETUP_AW: ");
spi_receive(rx_data,SETUP_RETR,1);
usart_nrf_reg("SETUP_RETR: ");
spi_receive(rx_data,FEATURE,1);
usart_nrf_reg("FEATURE: ");
spi_receive(rx_data,DYNPD,1);
usart_nrf_reg("DYNPD: ");
spi_receive(rx_data,STATUS,1);
usart_nrf_reg("STATUS: ");
spi_receive(rx_data,RF_CH,1);
usart_nrf_reg("RF_CH: ");
spi_receive(rx_data,RF_SETUP,1);
usart_nrf_reg("RF_SETUP: ");
spi_receive(rx_data,TX_ADDR,3);
usart_nrf_reg("TX_ADDR: ");
spi_receive(rx_data,RX_ADDR_P0,3);
usart_nrf_reg("RX_ADDR_P0: ");
*/

/*
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_1,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_2,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_3,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_4,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_5,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_6,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_7,1);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_8,1);

STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_1,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_2,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_3,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_4,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_5,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_6,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_7,0);
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_8,0);
*/
//led_w();
//PKU_stat();
/*
volatile uint16_t result=77;
int en_state[8]={1,1,1,1,1,1,1,1};*/
while(1)
{
//PKU_stat();
/*result=muxs.mux_sw();
if(result==0)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_1,1);
en_state[0]=1;
}
if(result==1)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_2,1);
en_state[1]=1;
}
if(result==2)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_3,1);
en_state[2]=1;
}
if(result==3)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_4,1);
en_state[3]=1;
}
if(result==4)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_5,1);
en_state[4]=1;
}
if(result==5)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_6,1);
en_state[5]=1;
}
if(result==6)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_7,1);
en_state[6]=1;
}
if(result==7)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_8,1);
en_state[7]=1;
}

if(result==8)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_1,0);
en_state[0]=0;
}
if(result==9)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_2,0);
en_state[1]=0;
}
if(result==10)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_3,0);
en_state[2]=0;
}
if(result==11)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_4,0);
en_state[3]=0;
}
if(result==12)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_5,0);
en_state[4]=0;
}
if(result==13)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_6,0);
en_state[5]=0;
}
if(result==14)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_7,0);
en_state[6]=0;
}
if(result==15)
{
STM32F407.set_pin_state(GPIOB,gpio_sw_a.EN_8,0);
en_state[7]=0;
}
result=77;
//oled.fake_ClearLCD();
oled.Cursor(0,0);
oled.PrintStr("EN 12345678");
oled.Cursor(1,3);
for(int i=0;i<8;i++)
{
sprintf(str,"%d", en_state[i]);
oled.PrintStr(str);
}*/

//led_w();

//mux_eth();
//PKU_stat();

sprintf(str, "Значение регистров NRF2401:");
uart_tx_data(str);
delay_ms(1000);
}
}

extern "C"
{
void HardFault_Handler(void)
{
  int k=0;
  while(1)
  {
    k++;
  }
}
}
