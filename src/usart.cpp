#include "usart.hpp"
#include "stm32f407xx.h"
#include <cmath>
#include "stdio.h"
#include <string.h>

uint8_t len=0;


    /**********************************************************************************
     * @brief Конфигураяция Baud rate
     *********************************************************************************/
      uint16_t usart::GetBBRsettings(uint32_t ClokFreq,float baurd_rate)
      {
        double UsarDiv=static_cast<double>(ClokFreq)/(baurd_rate);
        return roundf(UsarDiv);
      }

     /**********************************************************************************
     * @brief Конфигураяция USART
     *********************************************************************************/
      void usart::usart_init()
      {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 
        USART1->CR3  = 0;
        USART1->CR2 =  0;
        USART1->CR1  = 0;
        USART1->BRR =  GetBBRsettings(SystemCoreClock,115200);
        USART1->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE |USART_CR1_TE; 
        //USART1->CR1 |=USART_CR1_TCIE;     
        NVIC_EnableIRQ(USART1_IRQn);
        __enable_irq();
      }
     /**********************************************************************************
     * @brief Отправка одного байта или массива байт
     *********************************************************************************/
      void uart_tx_byte(uint8_t  data)
      {
      while ((USART1->SR & USART_SR_TXE) == 0)  {}//USART_SR_TXE
      USART1->DR = data;
      } 
      
      void uart_tx_data(const char *data)
      {
      len = strlen(data); 
      while(len--)
      {
      uart_tx_byte(*data++);
      }
      }
    /**********************************************************************************
     * @brief Прием данных
     *********************************************************************************/
      extern "C" void USART1_IRQHandler()
      {
      if(USART1->CR1&USART_SR_RXNE)
      {
      volatile uint8_t data=USART1->DR;
      char text='r';
      uart_tx_data(&text);
      //USART1->CR1&=~USART_SR_RXNE;
      }
      }