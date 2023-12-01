#include "stm32f407xx.h"

class usart
{
private: 
public:
   uint16_t GetBBRsettings(uint32_t ClokFreq,float baurd_rate);
   void usart_init();
};
void uart_tx_byte(unsigned char data);         
void uart_tx_data(const char *data);


