/******************** (C) COPYRIGHT 2010 www.armjishu.com ********************
* File Name          : spi_flash.c
* Author             : www.armjishu.com
* Version            : V1.0
* Library            : Using STM32F10X_STDPERIPH_VERSION V3.3.0
* Date               : 10/16/2010
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and SPI W25X16 FLASH.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi_flash.h"
#include "util.h"

#define COMPID "spi_flash"


extern SPI_HandleTypeDef hspi1;




/* Private typedef -----------------------------------------------------------*/
//#define SPI_FLASH_PageSize      4096
#define SPI_FLASH_PageSize      256
#define SPI_FLASH_PerWritePageSize      256

/* Private define ------------------------------------------------------------*/
/////////////////////////////////////////////////////MX25L16
#define MX25L_WriteEnable		0x06 
#define MX25L_WriteDisable		0x04 
#define MX25L_ReadStatusReg		0x05 
#define MX25L_WriteStatusReg	        0x01 
#define MX25L_ReadData			0x03 
#define MX25L_FastReadData		0x0B 
#define MX25L_FastReadDual		0xBB 
#define MX25L_PageProgram		0x02 
#define MX25L_BlockErase		0xD8 
#define MX25L_SectorErase		0x20 
#define MX25L_ChipErase			0xC7 
#define MX25L_PowerDown			0xB9 
#define MX25L_ReleasePowerDown	        0xAB 
#define MX25L_ManufactDeviceID	        0x9F 

#define MX25L_WIP_Flag          0x01  /* Write In Progress (WIP) flag */

#define MX25L_Dummy_Byte        0xA5



uint8_t spi_flash_sendbyte(uint8_t byte)
{
    uint8_t val;
    HAL_SPI_TransmitReceive(&hspi1,&byte,&val,1,SPI_TIMEOUT);  
    return val;
}

/*******************************************************************************
* Function Name  : flash_spi_readID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
uint32_t flash_spi_readID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  MX_SPI1_Init();                
  

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  spi_flash_sendbyte(0x9F);

  /* Read a byte from the FLASH */
  Temp0 = spi_flash_sendbyte(MX25L_Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp1 = spi_flash_sendbyte(MX25L_Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp2 = spi_flash_sendbyte(MX25L_Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  HAL_SPI_DeInit(&hspi1);
  

  return Temp;
}

/*******************************************************************************
* Function Name  : flash_spi_writeEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_writeEnable(void)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */
  spi_flash_sendbyte(MX25L_WriteEnable);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : flash_spi_waitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


void flash_spi_waitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  spi_flash_sendbyte(MX25L_ReadStatusReg);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = spi_flash_sendbyte(MX25L_Dummy_Byte);

  } while ((FLASH_Status & MX25L_WIP_Flag) == SET); /* Write in progress */

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : flash_spi_sectorErase
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_sectorErase(uint32_t SectorAddr)
{
  /* Send write enable instruction */
  MX_SPI1_Init();                
  
  flash_spi_writeEnable();

  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  spi_flash_sendbyte(MX25L_SectorErase);
  /* Send SectorAddr high nibble address byte */
  spi_flash_sendbyte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  spi_flash_sendbyte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  spi_flash_sendbyte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  flash_spi_waitForWriteEnd();

  HAL_SPI_DeInit(&hspi1);
  
}

/*******************************************************************************
* Function Name  : flash_spi_block64K_Erase
* Description    : Erases the specified FLASH 64K block.
* Input          : BlockAddr: address of the block to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_block64K_Erase(uint32_t BlockAddr)
{
  MX_SPI1_Init();                
    
  /* Send write enable instruction */
  flash_spi_writeEnable();

  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  spi_flash_sendbyte(MX25L_BlockErase);
  /* Send SectorAddr high nibble address byte */
  spi_flash_sendbyte((BlockAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  spi_flash_sendbyte((BlockAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  spi_flash_sendbyte(BlockAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  flash_spi_waitForWriteEnd();

  HAL_SPI_DeInit(&hspi1);
  
}
/*******************************************************************************
* Function Name  : flash_spi_buldErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_buldErase(void)
{
  MX_SPI1_Init();                
    
  /* Send write enable instruction */
  flash_spi_writeEnable();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */
  spi_flash_sendbyte(MX25L_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  flash_spi_waitForWriteEnd();

  HAL_SPI_DeInit(&hspi1);
  
}

/*******************************************************************************
* Function Name  : flash_spi_pageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_pageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    
  /* Enable the write access to the FLASH */
  flash_spi_writeEnable();

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  spi_flash_sendbyte(MX25L_PageProgram);
  /* Send WriteAddr high nibble address byte to write to */
  spi_flash_sendbyte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  spi_flash_sendbyte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  spi_flash_sendbyte(WriteAddr & 0xFF);

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
  }

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    spi_flash_sendbyte(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  flash_spi_waitForWriteEnd();
}

/*******************************************************************************
* Function Name  : flash_spi_bufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_bufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  MX_SPI1_Init();                

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      flash_spi_pageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        flash_spi_pageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      flash_spi_pageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
        temp = NumOfSingle - count;

        flash_spi_pageWrite(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        flash_spi_pageWrite(pBuffer, WriteAddr, temp);
      }
      else
      {
        flash_spi_pageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      flash_spi_pageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        flash_spi_pageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        flash_spi_pageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }

  HAL_SPI_DeInit(&hspi1);
  

}

/*******************************************************************************
* Function Name  : flash_spi_bufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_bufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  MX_SPI1_Init();                
    
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  spi_flash_sendbyte(MX25L_ReadData);

  /* Send ReadAddr high nibble address byte to read from */
  spi_flash_sendbyte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  spi_flash_sendbyte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  spi_flash_sendbyte(ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = spi_flash_sendbyte(MX25L_Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  HAL_SPI_DeInit(&hspi1);
  
}

/*******************************************************************************
* Function Name  : flash_spi_startReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_startReadSequence(uint32_t ReadAddr)
{
  MX_SPI1_Init();                
    
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  spi_flash_sendbyte(MX25L_ReadData);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  spi_flash_sendbyte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  spi_flash_sendbyte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  spi_flash_sendbyte(ReadAddr & 0xFF);

  HAL_SPI_DeInit(&hspi1);
  
}

/*******************************************************************************
* Function Name  : flash_spi_powerDown
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_powerDown(void)
{ 
  MX_SPI1_Init();                
    
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  spi_flash_sendbyte(MX25L_PowerDown);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  HAL_SPI_DeInit(&hspi1);
  
}   

/*******************************************************************************
* Function Name  : flash_spi_wakeUp
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void flash_spi_wakeUp(void)
{
  MX_SPI1_Init();                
    
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  spi_flash_sendbyte(MX25L_ReleasePowerDown);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();                             //µÈ´ýTRES1

  HAL_SPI_DeInit(&hspi1);
  
} 


bool flash_spi_bufferWrite_try(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite,uint8_t trytimes)
{
    uint8_t pBuffer_back[64];
    uint8_t pBuffer_back_liu[64];
    memcpy(pBuffer_back_liu,pBuffer,NumByteToWrite);
    memset(pBuffer_back,0,64);
    
    flash_spi_bufferRead(pBuffer_back, WriteAddr,NumByteToWrite);
    for(uint8_t i=0;i<NumByteToWrite;i++)
    {
        if(pBuffer_back[i]!=0xff)
        {
//            LOG("pBuffer_back is not 0xff!");
            LOGA("pBuffer_back of oxff: %d",pBuffer_back[i]);
        }
    }
 
    while(trytimes--)
    {
        delay_ms(1);
        flash_spi_bufferWrite(pBuffer, WriteAddr,NumByteToWrite);
        flash_spi_bufferRead(pBuffer_back, WriteAddr,NumByteToWrite);
        if(Util_Buffer_cmp(pBuffer_back, pBuffer, NumByteToWrite))
        {
            return TRUE;
        }
        LOG("flash_spi_bufferWrite failed!");          
    }      
    
    return FALSE;
    
}

