/******************** (C) COPYRIGHT 2010 www.armjishu.com ********************
* File Name          : spi_flash.h
* Author             : www.armjishu.com
* Version            : V1.0
* Library            : Using STM32F10X_STDPERIPH_VERSION V3.3.0
* Date               : 10/16/2010
* Description        : Header for spi_flash.c file.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"


#define  sFLASH_ID       0xC22015    //MX25L1608D  ID






#define FLASH_Bike_SZStauts_Base_Address      (0x03EC000)
#define FLASH_Bike_Validation_Base_Address      (0x03ED000)
#define FLASH_Bike_Header_Base_Address      (0x03EE000)
#define FLASH_Bike_Tailer_Base_Address      (0x03EF000)
#define FLASH_Bike_Recorder_Data_START      (0x003F0000)


#define FLASH_Bike_SZStauts_OFFSET      (0)
#define FLASH_Bike_SZStauts_LEN            (4)
#define FLASH_Bike_SZStauts_START        (FLASH_Bike_SZStauts_Base_Address + FLASH_Bike_SZStauts_OFFSET)/* from 0x03EC000 */


#define FLASH_Bike_Validation_OFFSET      (0)
#define FLASH_Bike_Validation_LEN            (4)
#define FLASH_Bike_Validation_START        (FLASH_Bike_Validation_Base_Address + FLASH_Bike_Validation_OFFSET)/* from 0x03ED000 */

#define FLASH_Bike_Header_OFFSET      (0)
#define FLASH_Bike_Header_LEN            (2)
#define FLASH_Bike_Header_START        (FLASH_Bike_Header_Base_Address + FLASH_Bike_Header_OFFSET)/* from 0x03EE000 */

#define FLASH_Bike_Tailer_OFFSET      (0)
#define FLASH_Bike_Tailer_LEN            (2)
#define FLASH_Bike_Tailer_START        (FLASH_Bike_Tailer_Base_Address + FLASH_Bike_Tailer_OFFSET)/* from 0x03EF000 */


#define FLASH_Block_Size      (0x00010000)
#define FLASH_Sector_Size      (0x0001000)
#define FLASH_Sector_Size_Per_1024      (4)



/*
block size:0x10000
block number:8

total size: 0x10000*8

total number of recorder: 0x10000*0x08/0x20=64*1024*8/32=0x0004000


block size:0x10000
block number:1

16 sectors each block

sector size:0x1000

total number of sector: 0x1000*0x01/0x20=4*1024*1/32=0x00080=128 recorder

total size: 0x10000*1

total number of recorder: 0x10000*0x01/0x20=64*1024*1/32=0x000800=2048 recorder
*/
#define Available_Block_Number  1
#define Bike_Recorder_Toltal_Number 0x000800

#define Bike_Recorder_PerBlock_Number 0x000800  //the number of bike recorder each block
#define Bike_Recorder_Double_Block_Number 0x001000

#define Bike_Recorder_PerSector_Number 0x00080//the number of bike recorder each sector
#define Bike_Recorder_Double_Secotor_Number 0x00100
/*

block size:0x10000
block number:1

16 sectors each block

sector size:0x1000

total number of sector: 0x1000*0x01/0x40=4*1024*1/64=0x00040=64 recorder

total size: 0x10000*1

total number of recorder: 0x10000*0x01/0x40=64*1024*1/64=0x000400=1024 recorder
*/
//#define Available_Block_Number  1
#define CpuCardFinal_Recorder_Toltal_Number 0x000400

#define CpuCardFinal_Recorder_PerBlock_Number 0x000400  //the number of CpuCardFinal recorder each block
#define CpuCardFinal_Recorder_Double_Block_Number 0x000800

#define CpuCardFinal_Recorder_PerSector_Number 0x00040//the number of CpuCardFinal recorder each sector
#define CpuCardFinal_Recorder_Double_Secotor_Number 0x00080








void flash_spi_init(void);
uint32_t  flash_spi_readID(void);
void flash_spi_writeEnable(void);
void flash_spi_waitForWriteEnd(void);
void flash_spi_sectorErase(uint32_t SectorAddr);
void flash_spi_block64K_Erase(uint32_t BlockAddr);
void flash_spi_buldErase(void);
void flash_spi_pageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void flash_spi_bufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void flash_spi_bufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void flash_spi_startReadSequence(uint32_t ReadAddr);
void flash_spi_powerDown(void);
void flash_spi_wakeUp(void);
bool flash_spi_bufferWrite_try(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite,uint8_t trytimes);


#endif /* __SPI_FLASH_H */

/******************* (C) COPYRIGHT 2010 www.armjishu.com *****END OF FILE****/
