/******************************************************************************/
/*     	 Name:         spi_soft_my.h                                          */
/*       Author:       Vladimir Pecherskih                                    */
/*       Date:         07.09.2020                                             */
/******************************************************************************/
#ifndef	__SPI_SOFT_MY_H
#define	__SPI_SOFT_MY_H

#include "stm32f1xx_hal.h"
/******************************************************************************/
/*              Constant declarations :                                       */
/******************************************************************************/
#define SPI_RX_LEN				    45

/******************************************************************************/
/*              Variable declarations :                                       */
/******************************************************************************/
extern  	uint8_t	spi_rx[SPI_RX_LEN];

/******************************************************************************/
/*      	    Functions declarations :                                      */
/******************************************************************************/
extern  	void      SPI_Delay(uint16_t cnt);
extern  	void      SPI_Unselect(void);
extern  	void      SPI_Select(void);

extern  	void      SPI_SoftInit(void);
extern  	uint8_t   SPI_TxRxByte(uint8_t tx_rx_byte);
extern  	uint8_t   SPI_ReadStatus(void);
extern  	uint8_t   SPI_ReadByte(uint8_t reg);
extern  	uint8_t   SPI_ReadBuffer(uint8_t reg, uint8_t len);
extern  	uint8_t   SPI_WriteCommand(uint8_t reg);
extern  	uint8_t   SPI_WriteByte(uint8_t reg, uint8_t value);
extern  	uint8_t   SPI_WriteBuffer(uint8_t reg, const uint8_t* buf, uint8_t len);

extern  	uint8_t   SPI_WriteCommand(uint8_t reg);
/******************************************************************************/
/*		                Init pins                                             */
/******************************************************************************/
#define LED_PORT            GPIOC
#define LED_PIN             GPIO_PIN_13
#define LED_HIGH            LED_PORT->BSRR = LED_PIN
#define LED_LOW             LED_PORT->BSRR = (uint32_t)LED_PIN << 16U

#define SPI_CE_PORT		    GPIOB
#define SPI_CE_PIN		    GPIO_PIN_4
#define SPI_CE_HIGH         SPI_CE_PORT->BSRR = SPI_CE_PIN
#define SPI_CE_LOW          SPI_CE_PORT->BSRR = (uint32_t)SPI_CE_PIN << 16U

#define SPI_IRQ_PORT		GPIOB
#define SPI_IRQ_PIN		    GPIO_PIN_5
#define SPI_IRQ_HIGH        SPI_IRQ_PORT->BSRR = SPI_IRQ_PORT
#define SPI_IRQ_LOW         SPI_IRQ_PORT->BSRR = (uint32_t)SPI_IRQ_PORT << 16U

#define SPI_MISO_PORT		GPIOB
#define SPI_MISO_PIN		GPIO_PIN_6
#define SPI_MISO_HIGH       SPI_MISO_PORT->BSRR = SPI_MISO_PIN
#define SPI_MISO_LOW        SPI_MISO_PORT->BSRR = (uint32_t)SPI_MISO_PIN << 16U

#define SPI_CLK_PORT		GPIOB
#define SPI_CLK_PIN		    GPIO_PIN_7
#define SPI_CLK_HIGH        SPI_CLK_PORT->BSRR = SPI_CLK_PIN
#define SPI_CLK_LOW         SPI_CLK_PORT->BSRR = (uint32_t)SPI_CLK_PIN << 16U

#define SPI_MOSI_PORT		GPIOB
#define SPI_MOSI_PIN		GPIO_PIN_8
#define SPI_MOSI_HIGH       SPI_MOSI_PORT->BSRR = SPI_MOSI_PIN
#define SPI_MOSI_LOW        SPI_MOSI_PORT->BSRR = (uint32_t)SPI_MOSI_PIN << 16U

#define SPI_CSN_PORT		GPIOB
#define SPI_CSN_PIN		    GPIO_PIN_9
#define SPI_CSN_HIGH        SPI_CSN_PORT->BSRR = SPI_CSN_PIN;
#define SPI_CSN_LOW         SPI_CSN_PORT->BSRR = (uint32_t)SPI_CSN_PIN << 16U;

/******************************************************************************/
/******************************************************************************/
#endif	/* __SPI_SOFT_MY_H */

