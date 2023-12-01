#include "stm32f407xx.h"
#include "stdio.h"

class muxs_
{
private: 

char _str[80];
uint8_t _temp,_n;

public:
void mux_eth();
uint16_t mux_sw();
void mux_leds(uint8_t num_led);
};



