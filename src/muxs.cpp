#include "stm32f407xx.h"
#include "hardware_config.hpp"
#include "muxs.hpp"
#include "gpio.hpp"
#include "lcd.hpp"

extern gpio STM32F407;
extern lcd oled;
extern gpio_mux_ETH eth;
extern gpio_mux_SW sw;
extern gpio_mux_led leds;

void muxs_:: mux_eth()
{

STM32F407.set_pin_state(GPIOA,eth.en,0); 
STM32F407.set_pin_state(GPIOA,eth.in_out,0); 

STM32F407.set_pin_state(GPIOD,eth.gpio_1,0);
STM32F407.set_pin_state(GPIOD,eth.gpio_2,0);
STM32F407.set_pin_state(GPIOD,eth.gpio_3,0);
STM32F407.set_pin_state(GPIOD,eth.gpio_4,0);
oled.ClearLCDScreen();
oled.Cursor(0,3); //Установка курсора, 0-ая строка, 2-ой столбец
oled.PrintStr("TEST ETH");
delay_ms(300);
oled.ClearLCDScreen();
oled.Cursor(0,3); //Установка курсора, 0-ая строка, 2-ой столбец
oled.PrintStr("12345678");
oled.Cursor(1,3);
for(int i=8;i<16;i++)
{
GPIOD->ODR&=~0x3C;    
GPIOD->ODR|=i<<2;
while(1)
{
   delay_ms(150);
   _temp=GPIOE->IDR;//&0xFF;
   _temp=_temp&0xFF; 
   _temp=_temp&(1<<_n);
    if(_temp==0)  
    {
        _temp=_n+1;
       if(_temp==9)
       {
        oled.Cursor(1,3+i-8);    
        char r='x';   
        oled.PrintStr(&r); 
        _n=0;
        break;
       }
        sprintf(_str,"%i",_temp);
        oled.Cursor(1,3+i-8);       
        oled.PrintStr(_str);
        _n=0;
        break;
    }  
    _n++;
}
delay_ms(500);
}
delay_ms(2500);
oled.ClearLCDScreen();
oled.Cursor(0,3); //Установка курсора, 0-ая строка, 2-ой столбец
oled.PrintStr("TEST END");
STM32F407.set_pin_state(GPIOA,eth.en,0); 
delay_ms(500);
oled.ClearLCDScreen();
}

uint16_t muxs_:: mux_sw()
{
for(int k=0;k<16;k++)
{
STM32F407.set_pin_state(GPIOA,sw.en,1);
STM32F407.set_pin_state(GPIOD,sw.s3,(bool(k&(1<<3))));
STM32F407.set_pin_state(GPIOD,sw.s2,(bool((k&(1<<1)))));
STM32F407.set_pin_state(GPIOB,sw.s1,(bool((k&(1<<2)))));
STM32F407.set_pin_state(GPIOB,sw.s0,(bool(k&(1<<0))));
STM32F407.set_pin_state(GPIOA,sw.en,0);
_n=k;
if(!STM32F407.get_state_pin(GPIOA,sw.state))
{
STM32F407.set_pin_state(GPIOA,sw.en,1);
return _n;
}
}
return 77;
}

void muxs_:: mux_leds(uint8_t num_led)
{
STM32F407.set_pin_state(GPIOA,leds.EN1,1);
STM32F407.set_pin_state(GPIOE,leds.EN2,1);
if(num_led<9)
{
GPIOC->ODR &= ~0x0F;
GPIOC->ODR|=8-num_led;
STM32F407.set_pin_state(GPIOA,leds.EN1,0);
}
if(num_led>8)
{
GPIOC->ODR &= ~0x0F;
GPIOC->ODR|=24-num_led;
STM32F407.set_pin_state(GPIOA,leds.EN1,0);
}
if(num_led>16)
{
STM32F407.set_pin_state(GPIOA,leds.EN1,1);
GPIOC->ODR &= ~0xFF;
GPIOC->ODR|=(24-num_led)*16;
STM32F407.set_pin_state(GPIOE,leds.EN2,0);
}
}