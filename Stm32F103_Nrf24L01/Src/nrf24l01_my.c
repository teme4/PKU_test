/******************************************************************************/
/*     	 Name:         nrf24l01_my.c                                          */
/*       Author:       Vladimir Pecherskih                                    */
/*       Date:         07.09.2020                                             */
/******************************************************************************/
#include "main.h"
#include "spi_soft_my.h"
#include "nrf24l01_my.h"
#include "usbd_cdc_if.h"
/******************************************************************************/
/*              Constant declarations :                                       */
/******************************************************************************/


/******************************************************************************/
/*              Variable declarations :                                       */
/******************************************************************************/   
const uint8_t chRf[] = {2, 26,80};
const uint8_t chLe[] = {37,38,39};

uint8_t   temp;
uint8_t   L, i;
uint8_t   buf[32];
uint8_t   ch = 0;       // channel 37 - 2402 MHz   
/******************************************************************************/
/*      	    Functions declarations :                                      */
/******************************************************************************/
void   nrf24L01_Init(void);
void   EncodePacket(void);
void   DataTx_Init(void);
void   Nrf24_TxData(void);
void   Nrf24_RxData(void);

void    DelayMy(uint32_t nCount);
void    ProcessingData(void);
uint8_t swapbits(uint8_t a);
void    btLeCrc(const uint8_t* data, uint8_t len, uint8_t* dst);
void    btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff);

static inline uint8_t btLeWhitenStart(uint8_t chan);
void   btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan);
void   RePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan);
void   nrf_simplebyte(uint8_t cmd);
void   nrf_manybytes(uint8_t* data, uint8_t len);
/******************************************************************************/
/******************************************************************************/





/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*  						I N I T                                           */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*          	            Init BLE                                          */
/*                                                                            */
/******************************************************************************/
void    nrf24L01_Init(void)
{
    DelayMy(60000);               // 135 ms зависит от кварца ???????????
    //--------------------------------------------------------------------------
    SPI_WriteByte(0x20, 0x12);    // on, no crc, int on RX/TX done
    // SPI_WriteByte(0x20, 0x1E); // on, crc16-2byte, int on RX/TX done       
    
    SPI_WriteByte(0x21, 0x00);    // no auto-acknowledge
    SPI_WriteByte(0x22, 0x00);    // no RX       
    
    SPI_WriteByte(0x23, 0x02);    // 4-byte address	
    //SPI_WriteByte(0x23, 0x03);  // 5-byte address !!!!!!!!!!!!!!!!	
    
    SPI_WriteByte(0x24, 0x00);    // no auto-retransmit
    SPI_WriteByte(0x26, 0x06);    // 1MBps at 0dBm
    SPI_WriteByte(0x27, 0x3E);    // clear various flags  
    
    //SPI_WriteByte(0x3C, 0x00);  // no dynamic payloads
    SPI_WriteByte(0x3C, 0xFF);    // no  dynamic payloads  !!!!!!!!!!!!!!!!!
    SPI_WriteByte(0x3D, 0x00);    // no  dynamic payloads  !!!!!!!!!!!!!!!!!               
    
    SPI_WriteByte(0x3D, 0x00);    // no features     
    SPI_WriteByte(0x31, 32);      // always RX 32 bytes  
    SPI_WriteByte(0x22, 0x01);    // RX on pipe 0 

    //uint8_t tx_address[5] = { 0xC0, 0x01, 0x23, 0x45, 0x67 };
    buf[0] =          0x30;	      // set addresses
    buf[1] = swapbits(0x8E);      // adr 0x10 txaddr
    buf[2] = swapbits(0x89);      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    buf[3] = swapbits(0xBE);
    buf[4] = swapbits(0xD6);
    nrf_manybytes(buf, 5);        // Tx               
    buf[0] = 0x2A;                // adr 0x0A rxaddr
    nrf_manybytes(buf, 5);
}
/******************************************************************************/
/*                                                                            */
/*          	        Init Data Out                                         */
/*                                                                            */
/******************************************************************************/
void DataTx_Init(void)
{
    L = 0;        
    //buf[L++] = 0x42;	// PDU type, given address is random
    buf[L++] = 0x02;	// PDU type, given address is public
    //buf[L++] = 25;
    buf[L++] = 9;

    buf[L++] = MY_MAC_0;
    buf[L++] = MY_MAC_1;
    buf[L++] = MY_MAC_2;
    buf[L++] = MY_MAC_3;
    buf[L++] = MY_MAC_4;
    buf[L++] = MY_MAC_5;
    
    buf[L++] = 2;	//flags (LE-only, limited discovery mode)
    buf[L++] = 0x01;
    buf[L++] = 0x05;
  
    buf[L++] = 9;
    buf[L++] = 0x08;
    buf[L++] = 'n';
    buf[L++] = 'R';
    buf[L++] = 'F';
    buf[L++] = '2';
    buf[L++] = '4';
    buf[L++] = 'L';    
    buf[L++] = '0';
    buf[L++] = '1';        
    buf[1]  +=  10;    
    
    buf[L++] = 0x55;	//CRC start value: 0x555555
    buf[L++] = 0x55;
    buf[L++] = 0x55;                  
}
/******************************************************************************/
/*                                                                            */
/*          	      Encode  Packet                                          */
/*                                                                            */
/******************************************************************************/
void    EncodePacket(void)
{
  btLePacketEncode(buf, L, chLe[ch]);   
}
/******************************************************************************/
/*                                                                            */
/*          	      Nrf24_TxData                                            */
/*                                                                            */
/******************************************************************************/
void Nrf24_TxData(void)
{
   SPI_CE_LOW;
   //if(++ch == sizeof(chRf)) ch = 0;       // iaiyai eaiae               
    SPI_WriteByte(0x25, chRf[ch]);          // channel 37 - 2402 MHz (2, 26, 80)               
    SPI_WriteByte(0x27, 0x6E);	            // clear flags                   
    nrf_simplebyte(0xE2);                   // Clear RX Fifo
    nrf_simplebyte(0xE1);                   // Clear TX Fifo
    
    SPI_Select();
    SPI_TxRxByte(0xA0);                     // caieoeaaai iieao ia ia?aaa?o
    for(i = 0 ; i < L ; i++) SPI_TxRxByte(buf[i]);
    SPI_Unselect();
    
    SPI_WriteByte(0x20, 0x12);              // tx on   
    SPI_CE_HIGH;
    
    DelayMy(6000);                            // 13.5 ms caaeneo io eaa?oa  
    SPI_CE_LOW;
    
    //GPIO_SetBits(LED_PORT, LED_PIN);        // led on               
}
/******************************************************************************/
/*                                                                            */
/*          	      Nrf24_RxData                                            */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
void Nrf24_RxData(void)
{
    SPI_WriteByte(0x20, 0x13);                // on, no crc, int rx mode  
    SPI_WriteByte(0x22, 0x3F);                // All channel     
    SPI_CE_HIGH;
 
    if(HAL_GPIO_ReadPin(SPI_IRQ_PORT, SPI_IRQ_PIN) == GPIO_PIN_RESET)       
    {    
      DelayMy(300000);                      // 200000 - min !!!!!!!!
      temp = SPI_ReadBuffer(0x61, 32);      // spi_rx[]                          
      RePacketEncode(spi_rx, 32, chLe[ch]); // channel 37 - 2402 MHz      
      ProcessingData();
      
      SPI_WriteByte(0x27, 0x6E);	        // clear flags         
      nrf_simplebyte(0xE2);                 // Clear RX Fifo
      nrf_simplebyte(0xE1);                 // Clear TX Fifo        
    }
}
/******************************************************************************/
/******************************************************************************/





/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*  				    N R F 2 4 L 0 1                                       */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                  	Programs Data                                         */
/*                                                                            */
/******************************************************************************/
void ProcessingData(void)
{ 
  if((spi_rx[13] == 'n')&&(spi_rx[14] == 'R')&&(spi_rx[15] == 'F')&&
     (spi_rx[16] == '5')&&(spi_rx[17] == '2')&&(spi_rx[18] == '8'))
  {
      //GPIO_TOGGLE(LED_PORT, LED_PIN); 
      //USART_SendData8(USART1, spi_rx[25]);
      
      CDC_Transmit_FS(&spi_rx[25], 1);
  }
}
/******************************************************************************/
/*                                                                            */
/*                  	    DelayMy                                           */
/*                                                                            */
/******************************************************************************/
void DelayMy(uint32_t nCount)
{
  while (nCount != 0)
  {
    nCount--;
  }
}
/******************************************************************************/
/*                                                                            */
/*                  	  CRC                                                 */
/*                                                                            */
/******************************************************************************/
void btLeCrc(const uint8_t* data, uint8_t len, uint8_t* dst)
{
	uint8_t v, t, d;

	while(len--)
	{
		d = *data++;
		for(v = 0; v < 8; v++, d >>= 1)
		{
			t = dst[0] >> 7;
			
			dst[0] <<= 1;
			if(dst[1] & 0x80) dst[0] |= 1;
			dst[1] <<= 1;
			if(dst[2] & 0x80) dst[1] |= 1;
			dst[2] <<= 1;
			
			if(t != (d & 1))
			{
				dst[2] ^= 0x5B;
				dst[1] ^= 0x06;
			}
		}	
	}
}
/******************************************************************************/
/*                                                                            */
/*                      Transposition bits                                    */
/*                                                                            */
/******************************************************************************/
uint8_t swapbits(uint8_t a)
{
	uint8_t v = 0;
	
	if(a & 0x80) v |= 0x01;
	if(a & 0x40) v |= 0x02;
	if(a & 0x20) v |= 0x04;
	if(a & 0x10) v |= 0x08;
	if(a & 0x08) v |= 0x10;
	if(a & 0x04) v |= 0x20;
	if(a & 0x02) v |= 0x40;
	if(a & 0x01) v |= 0x80;

	return v;
}
/******************************************************************************/
/*                                                                            */
/*                  	    Whiten                                            */
/*                                                                            */
/******************************************************************************/
void btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff)
{
	uint8_t  m;
	
	while(len--)
	{
		for(m = 1; m; m <<= 1)
		{
			if(whitenCoeff & 0x80)
			{
				whitenCoeff ^= 0x11;
				(*data) ^= m;
			}
			whitenCoeff <<= 1;
		}
		data++;
	}
}
/******************************************************************************/
/*                                                                            */
/*                  	Progs                                                 */
/*                                                                            */
/******************************************************************************/
static inline uint8_t btLeWhitenStart(uint8_t chan)
{
    //the value we actually use is what BT'd use left shifted one...makes our life easier
    return swapbits(chan) | 2;	
}
/******************************************************************************/
/*                                                                            */
/*                  	Progs                                                 */
/*                                                                            */
/******************************************************************************/
void btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan)
{
    //length is of packet, including crc. pre-populate crc in packet with initial crc value!
    uint8_t i, dataLen = len - 3;
    btLeCrc(packet, dataLen, packet + dataLen);
    for(i = 0; i < 3; i++, dataLen++) packet[dataLen] = swapbits(packet[dataLen]);
    btLeWhiten(packet, len, btLeWhitenStart(chan));
    for(i = 0; i < len; i++) packet[i] = swapbits(packet[i]);                  
}
/******************************************************************************/
/*                                                                            */
/*                   Re Packet Encode My                                      */
/*                                                                            */
/******************************************************************************/
void RePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan)
{
    uint8_t i, dataLen = len - 3;
    for(i = 0; i < len; i++) packet[i] = swapbits(packet[i]);
    btLeWhiten(packet, len, btLeWhitenStart(chan));
    for(i = 0; i < 3; i++, dataLen++) packet[dataLen] = swapbits(packet[dataLen]);  
}
/******************************************************************************/
/*                                                                            */
/*                  	Progs                                                 */
/*                                                                            */
/******************************************************************************/
void nrf_simplebyte(uint8_t cmd)
{
    SPI_Select();
    SPI_TxRxByte(cmd);
    SPI_Unselect();
}
/******************************************************************************/
/*                                                                            */
/*                  	Progs                                                 */
/*                                                                            */
/******************************************************************************/
void nrf_manybytes(uint8_t* data, uint8_t len)
{
    SPI_Select();
    do 
    {
      SPI_TxRxByte(*data++);
    } while (--len);
    SPI_Unselect();
}
/******************************************************************************/
/******************************************************************************/


