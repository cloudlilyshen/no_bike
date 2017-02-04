/**
  ******************************************************************************
  * @file    util.h 
  * @author  Alex Chen
  * @version V3.3.0
  * @date    04/16/2010
  * @brief    utilities for project: led, lock, etc.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * 
  */ 
  
#ifndef __UTIL_H
#define __UTIL_H

#ifdef __cplusplus
 extern "C" {
#endif 

bool Util_Buffer_cmp(uint8_t* src, uint8_t* dst, uint16_t buffer_len);
void Int2FourBCD(uint32_t src,uint8_t* dst);
#define BCDTODEC(bcd) (((bcd)&15) + ((bcd)>>4)*10)
#define DECTOBCD(dcb) ((((dcb)/10)<<4) + (((dcb)%10) & 15))  //dbc <=60
void PrintRecieveData(uint8_t *dat,uint16_t len);
void delay_seconds(uint8_t second);
void delay_ms(uint16_t ms);
void delay_us(uint32_t us);
uint16_t get_crc16(uint8_t *data, uint32_t len);
bool isEqualcrc16(uint16_t crc16,uint8_t crc_low,uint8_t crc_high);
uint32_t convert_second(UTCTime_t *tm);
uint8_t get_chk_bcc(uint8_t *dat, uint32_t len);
uint8_t  ChartoHex(char dat);
uint8_t HEX_BCD(uint8_t hex);
uint8_t BCD_HEX(uint8_t bcd);






















#ifdef __cplusplus
}
#endif

#endif

