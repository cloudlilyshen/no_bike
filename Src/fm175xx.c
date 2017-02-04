#include "main.h"
#include "fm175xx.h"
#define COMPID "fm1755xx"

unsigned char FM175XX_IRQ;

#define MI_NOTAGERR 0xEE
#define MAXRLEN 64


#define     SSEL_EN         (0 << 16)
#define     SSEL_DIS        (1 << 16)
#define     EOT_EN          (1 << 20)
#define     EOT_DIS         (0 << 20)
#define     EOF_EN          (1 << 21)
#define     EOF_DIS         (0 << 21)
#define     RXIGNORE_EN     (1 << 22)
#define     RXIGNORE_DIS    (0 << 22)
#define     FLEN(n)         (((n) - 1) << 24)
extern SPI_HandleTypeDef hspi1;


static FM17550_Data_t FM17550Data;

/*
** 调教变量列表
*/ 
uint8_t  T3ClkDivK ;
uint8_t  LpcdBiasCurrent;                                               /* 3bit 由样片测试指标决定，
                                                                            在config文件中设定          */
uint8_t  LpcdGainReduce;                                                /* 2bit                         */
uint8_t  LpcdGainAmplify;                                               /* 3bit                         */
uint8_t  LpcdADCRefernce;                                               /* 7bit                         */

uint8_t  Timer1Cfg;                                                     /* 4bit                         */
uint8_t  Timer2Cfg;                                                     /* 5bit                         */
uint8_t  Timer3Cfg;                                                     /* 5bit                         */

uint8_t  ADCResultFullScale;                                            /* T3下满幅ADCResult信息        */
uint8_t  ADCResultThreshold;                                            /* 检测幅度，设置成相对值       */
uint8_t  LpcdThreshold_L;                                               /* LPCD幅度低阈值               */
uint8_t  LpcdThreshold_H;                                               /* LPCD幅度高阈值               */
uint8_t  ADCResultCenter;                                               /* LPCD幅度中心点               */
uint8_t  LpcdADCResult[10];                                             /* Lpcd幅度信息，用于误触发判断 */



/*********************************************************************************************************
** Function name:       spi_SetReg
** Descriptions:        SPI写读卡芯片寄存器函数
** input parameters:    ucRegAddr：寄存器地址
**                      ucRegVal：要写入的值
** output parameters:   无
** Returned value:      TRUE
*********************************************************************************************************/
uint8_t spi_SetReg(uint8_t ucRegAddr, uint8_t ucRegVal)
{
    uint8_t val=0;    
    ucRegAddr <<= 1;
    ucRegAddr &= 0x7F;
    
    SPI_FM17550_CS_LOW ();
    if(HAL_SPI_TransmitReceive(&hspi1,&ucRegAddr,&val,1,SPI_TIMEOUT)!=HAL_OK)
    LOG("HAL_SPI_TransmitReceive is false");
    if(HAL_SPI_TransmitReceive(&hspi1,&ucRegVal,&val,1,SPI_TIMEOUT)!=HAL_OK)
    LOG("HAL_SPI_TransmitReceive is false");
    SPI_FM17550_CS_HIGH ();
    return TRUE;
}

/*********************************************************************************************************
** Function name:       spi_GetReg
** Descriptions:        SPI读读卡芯片寄存器函数
** input parameters:    ucRegAddr：寄存器地址
** output parameters:   无
** Returned value:      目标寄存器的值
*********************************************************************************************************/
uint8_t spi_GetReg(uint8_t ucRegAddr)
{
    uint8_t val;    
    uint8_t ucRegVal;
    ucRegAddr <<= 1;
    ucRegAddr |= 0x80;
    
    SPI_FM17550_CS_LOW();
    if(HAL_SPI_TransmitReceive(&hspi1,&ucRegAddr,&val,1,SPI_TIMEOUT)!=HAL_OK)     
    LOG("HAL_SPI_TransmitReceive is false");
        
    val=0;
    if(HAL_SPI_TransmitReceive(&hspi1,&val,&ucRegVal,1,SPI_TIMEOUT)!=HAL_OK)                      
    LOG("HAL_SPI_TransmitReceive is false");
    SPI_FM17550_CS_HIGH();
    return ucRegVal;
}


/*********************************************************************************************************
** Function name:       spi_GetReg
** Descriptions:        SPI读读卡芯片寄存器函数
** input parameters:    ucRegAddr：寄存器地址
** output parameters:   无
** Returned value:      目标寄存器的值
*********************************************************************************************************/
uint8_t spi_GetReg2(uint8_t ucRegAddr,uint8_t *p)
{
    uint8_t val;   

    ucRegAddr <<= 1;
    ucRegAddr |= 0x80;
    
    
    SPI_FM17550_CS_LOW();
    if(HAL_SPI_TransmitReceive(&hspi1,&ucRegAddr,&val,1,SPI_TIMEOUT)!= HAL_OK)     
    LOG("HAL_SPI_TransmitReceive is false");

    val=0;
    if(HAL_SPI_TransmitReceive(&hspi1,&val,p,1,SPI_TIMEOUT)!=HAL_OK)                      
    LOG("HAL_SPI_TransmitReceive is false");
    SPI_FM17550_CS_HIGH();

    return TRUE;
}



/*********************************************************************************************************
** Function name:       SPIRead_Sequence
** Descriptions:        SPI读FIFO寄存器的值
** input parameters:    sequence_length 数据长度 ucRegAddr：寄存器地址  *reg_value 数据指针
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SPIRead_Sequence(unsigned char sequence_length,unsigned char ucRegAddr,unsigned char *reg_value)    
{
    uint8_t i;
    if (sequence_length==0)
    return;
    SPI_FM17550_CS_LOW();

    for(i=0;i<sequence_length;i++) {
        *(reg_value+i) = spi_GetReg(ucRegAddr);
    }
    SPI_FM17550_CS_HIGH();

    return;

}

/*********************************************************************************************************
** Function name:       SPIWrite_Sequence
** Descriptions:        SPI写FIFO的值
** input parameters:    sequence_length 数据长度 
**                      ucRegAddr：寄存器地址  
**                      *reg_value 数据指针
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SPIWrite_Sequence(unsigned char sequence_length,unsigned char ucRegAddr,unsigned char *reg_value)
{
    
    uint8_t i;
    if(sequence_length==0)
        return;
    for(i=0;i<sequence_length;i++) {
        spi_SetReg(ucRegAddr, *(reg_value+i));
    }
    return ;    
}





/*********************************************************************************************************
** Function name:       pcd_Init
** Descriptions:        MCU初始化函数、包括SPI和UART的初始化
** input parameters:    N/A
** output parameters:   N/A
** Returned value:      
*********************************************************************************************************/
void pcd_Init(void)
{
    FM17550_GPIO_Init();
    MX_SPI1_Init();
}

/*********************************************************************************************************
** Function name:       Read_Reg
** Descriptions:        读取寄存器                
** input parameters:    reg_add:寄存器数值
** output parameters:   N/A
** Returned value:      寄存器数值
*********************************************************************************************************/
unsigned char Read_Reg(unsigned char reg_add)
{
    unsigned char  reg_value;       
    reg_value=spi_GetReg(reg_add);
    return reg_value;
}

/*********************************************************************************************************
** Function name:       Read_Reg_All
** Descriptions:        读取全部寄存器                
** input parameters:    reg_value:寄存器数值
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Read_Reg_All(unsigned char *reg_value)
{
    for (uint8_t i=0;i<64;i++)       
        *(reg_value+i) = spi_GetReg(i);
    return TRUE;
}


/*********************************************************************************************************
** Function name:       Write_Reg
** Descriptions:        写寄存器操作                
** input parameters:    reg_add:寄存器地址
**                      reg_value:寄存器数值
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Write_Reg(unsigned char reg_add,unsigned char reg_value)
{
    spi_SetReg(reg_add,reg_value);
    return TRUE;
}


/*********************************************************************************************************
** Function name:       Read_FIFO
** Descriptions:        读出FIFO的数据         
** input parameters:    length:读取数据长度
**                      *fifo_data:数据存放指针
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
void Read_FIFO(unsigned char length,unsigned char *fifo_data)
{     
    SPIRead_Sequence(length,FIFODataReg,fifo_data);
    return;
}


/*********************************************************************************************************
** Function name:       Write_FIFO
** Descriptions:        写入FIFO         
** input parameters:    length:读取数据长度
**                      *fifo_data:数据存放指针
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
void Write_FIFO(unsigned char length,unsigned char *fifo_data)
{
    SPIWrite_Sequence(length,FIFODataReg,fifo_data);
    return;
}

/*********************************************************************************************************
** Function name:       Set_BitMask
** Descriptions:        置位寄存器操作    
** input parameters:    reg_add，寄存器地址
**                      mask，寄存器清除位
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Set_BitMask(unsigned char reg_add,unsigned char mask)
{
    uint8_t result;
    result=spi_SetReg(reg_add,Read_Reg(reg_add) | mask);                /* set bit mask                 */
    return result;
}


/*********************************************************************************************************
** Function name:       Clear_FIFO
** Descriptions:        清空FIFO              
** input parameters:   
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Clear_FIFO(void)
{
    Set_BitMask(FIFOLevelReg,0x80);                                     /* 清除FIFO缓冲                 */
    if ( spi_GetReg(FIFOLevelReg) == 0 )
        return TRUE;
    else
        return FALSE;
}



/*********************************************************************************************************
** Function name:       Clear_BitMask
** Descriptions:        清除位寄存器操作
** input parameters:    reg_add，寄存器地址
**                      mask，寄存器清除位
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Clear_BitMask(unsigned char reg_add,unsigned char mask)
{
    uint8_t result;
    result=Write_Reg(reg_add,Read_Reg(reg_add) & ~mask);                /* clear bit mask               */
    return result;
}


/*********************************************************************************************************
** Function name:       Set_RF
** Descriptions:        设置射频输出
** input parameters:    mode，射频输出模式
**                      0，关闭输出
**                      1，仅打开TX1输出
**                      2，仅打开TX2输出
**                      3，TX1，TX2打开输出，TX2为反向输出
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Set_Rf(unsigned char mode)
{
    unsigned char result;
    if( (Read_Reg(TxControlReg)&0x03) == mode )
        return TRUE;
    if( mode == 0 )
        result = Clear_BitMask(TxControlReg,0x03);                      /* 关闭TX1，TX2输出             */
    if( mode== 1 )
        result = Clear_BitMask(TxControlReg,0x01);                      /* 仅打开TX1输出                */
    if( mode == 2)
        result = Clear_BitMask(TxControlReg,0x02);                      /* 仅打开TX2输出                */
    if (mode==3)
        result=Set_BitMask(TxControlReg,0x03);                          /* 打开TX1，TX2输出             */
    delay_us(10000);
    return result;
}
 
/*********************************************************************************************************
** Function name:       Pcd_Comm
** Descriptions:        读卡器通信 不利用IRQ管脚的情况
** input parameters:    Command:通信操作命令
**                      pInData:发送数据数组
**                      InLenByte:发送数据数组字节长度
**                      pOutData:接收数据数组
**                      pOutLenBit:接收数据的位长度
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Pcd_Comm(    unsigned char Command, 
                         unsigned char *pInData, 
                         unsigned char InLenByte,
                         unsigned char *pOutData, 
                         unsigned int *pOutLenBit)
{
    uint8_t status  = FALSE;
    uint8_t irqEn   = 0x00;                                             /* 使能的中断                   */
    uint8_t waitFor = 0x00;                                             /* 等待的中断                   */
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;
    Write_Reg(ComIrqReg, 0x7F);                                         /* 清楚IRQ标记                  */
    Write_Reg(TModeReg,0x80);                                           /* 设置TIMER自动启动            */
    switch (Command) {
    case MFAuthent:                                                     /* Mifare认证                   */
        irqEn   = 0x12;
        waitFor = 0x10;
        break;
    case Transceive:                                       /* 发送FIFO中的数据到天线，传输后激活接收电路*/
        irqEn   = 0x77;
        waitFor = 0x30;
        break;
    default:
        break;
    }
   
    Write_Reg(ComIEnReg, irqEn | 0x80);
   // Clear_BitMask(ComIrqReg, 0x80);
    Write_Reg(CommandReg, Idle);
    Set_BitMask(FIFOLevelReg, 0x80);
    
    for (i=0; i < InLenByte; i++) {
        Write_Reg(FIFODataReg, pInData[i]);
    }
    Write_Reg(CommandReg, Command);

    if (Command == Transceive) {
        Set_BitMask(BitFramingReg, 0x80);
    }

    i = 3000;                                              /* 根据时钟频率调整，操作M1卡最大等待时间25ms*/

    do {
        n = Read_Reg(ComIrqReg);
        i--;                                                            /* i==0表示延时到了             */
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));            /* n&0x01!=1表示PCDsettimer时间未到 */
                                                                        /* n&waitFor!=1表示指令执行完成 */
    Clear_BitMask(BitFramingReg, 0x80);
    if (i != 0) {
        if(!(Read_Reg(ErrorReg) & 0x1B)) {
            status = TRUE;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }
            if (Command == Transceive) {
                n = Read_Reg(FIFOLevelReg);
                lastBits = Read_Reg(ControlReg) & 0x07;
                if (lastBits) {
                    *pOutLenBit = (n - 1) * 8 + lastBits;
                } else {
                    *pOutLenBit = n * 8;
                }
                if (n == 0) {
                    n = 1;
                }
                if (n > MAXRLEN) {
                    n = MAXRLEN;
                }
                for (i = 0; i < n; i++) {
                    pOutData[i] = Read_Reg(FIFODataReg);
                }
            }
        } else {
            status = FALSE;
        }
    }
       Clear_BitMask(BitFramingReg,0x80);//关闭发送
    return status;
}


/*********************************************************************************************************
** Function name:       Pcd_SetTimer
** Descriptions:        设置接收延时
** input parameters:    delaytime，延时时间（单位为毫秒）  
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Pcd_SetTimer(unsigned long delaytime)
{
    unsigned long  TimeReload;
    unsigned int  Prescaler;

    Prescaler=0;
    TimeReload=0;
    while(Prescaler<0xfff) {
        TimeReload = ((delaytime*(long)13560)-1)/(Prescaler*2+1);
        if( TimeReload<0xffff)
            break;
        Prescaler++;
    }
    TimeReload=TimeReload&0xFFFF;
    Set_BitMask(TModeReg,Prescaler>>8);
    Write_Reg(TPrescalerReg,Prescaler&0xFF);                    
    Write_Reg(TReloadMSBReg,TimeReload>>8);
    Write_Reg(TReloadLSBReg,TimeReload&0xFF);
    return TRUE;
}

/*********************************************************************************************************
** Function name:       Pcd_ConfigISOType
** Descriptions:        配置ISO14443A/B协议
** input parameters:    type = 0：ISO14443A协议；
**                                 type = 1，ISO14443B协议；   
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Pcd_ConfigISOType(unsigned char type)
{
    if (type == 0)   {                                                  /* 配置为ISO14443_A             */
        Set_BitMask(ControlReg, 0x10);                                /* ControlReg 0x0C 设置reader模式 */
        Set_BitMask(TxAutoReg, 0x40);                                  /* TxASKReg 0x15 设置100%ASK有效 */
        Write_Reg(TxModeReg, 0x00);                 /* TxModeReg 0x12 设置TX CRC无效，TX FRAMING =TYPE A */
        Write_Reg(RxModeReg, 0x00);                 /* RxModeReg 0x13 设置RX CRC无效，RX FRAMING =TYPE A */
    }
    if (type == 1)   {                                                  /* 配置为ISO14443_B           */
        Write_Reg(ControlReg,0x10);
        Write_Reg(TxModeReg,0x83);                                      /* BIT1~0 = 2'b11:ISO/IEC 14443B */
        Write_Reg(RxModeReg,0x83);                                      /* BIT1~0 = 2'b11:ISO/IEC 14443B */
        Write_Reg(TxAutoReg,0x00);
        Write_Reg(RxThresholdReg,0x55);
        Write_Reg(RFCfgReg,0x48);
        Write_Reg(TxBitPhaseReg,0x87);                                  /* 默认值                         */
        Write_Reg(GsNReg,0x83);    
        Write_Reg(CWGsPReg,0x30);
        Write_Reg(GsNOffReg,0x38);
        Write_Reg(ModGsPReg,0x20);
    
    }
    delay_us(3000);
    return TRUE;
}


/*********************************************************************************************************
** Function name:       FM175X_SoftReset
** Descriptions:        FM175xx软件复位
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char  FM175X_SoftReset(void)
{    
    Write_Reg(CommandReg,SoftReset);
    return    Set_BitMask(ControlReg,0x10);                               /* 17520初始值配置              */
}

/*********************************************************************************************************
** Function name:       FM175X_HardReset
** Descriptions:        FM175xx硬件复位
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char FM175X_HardReset(void)
{    
    SPI_FM17550_RST_LOW();    
    delay_us(100);
    SPI_FM17550_RST_HIGH();    
    delay_us(100);
    return TRUE;
}

    
/*********************************************************************************************************
** Function name:       FM175X_SoftPowerdown
** Descriptions:        硬件低功耗操作
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char FM175X_SoftPowerdown(void)
{
    if(Read_Reg(CommandReg)&0x10) {
       Clear_BitMask(CommandReg,0x10);                                 /* 退出低功耗模式               */
       return FALSE;
    }
    else
        Set_BitMask(CommandReg,0x10);                                       /* 进入低功耗模式               */
    return TRUE;
}

/*********************************************************************************************************
** Function name:       FM175X_HardPowerdown
** Descriptions:        硬件低功耗操作
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char FM175X_HardPowerdown(void)
{    
    //NPD=~NPD;
    //if(NPD==1)                                                          /* 进入低功耗模式               */
    return TRUE;                                
//    else
        //return FALSE;                                                     /* 退出低功耗模式               */
}

/*********************************************************************************************************
** Function name:       Read_Ext_Reg
** Descriptions:        读取扩展寄存器
** input parameters:    reg_add，寄存器地址；         
** output parameters:   reg_value，寄存器数值
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Read_Ext_Reg(unsigned char reg_add)
{
     Write_Reg(0x0F,0x80+reg_add);
     return Read_Reg(0x0F);
}


/*********************************************************************************************************
** Function name:       GetReg_Ext(uint8_t  ExtRegAddr,uint8_t * ExtRegData)
** Descriptions:        读取扩展寄存器值
** input parameters:    ExtRegAddr:扩展寄存器地址   
** output parameters:   ExtRegData:读取的值
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  GetReg_Ext(uint8_t  ExtRegAddr,uint8_t * ExtRegData)
{
    uint8_t  res;
    uint8_t  addr,regdata;

    addr = JREG_EXT_REG_ENTRANCE;                                       /* 扩展寄存器0x0f地址           */
    regdata = JBIT_EXT_REG_RD_ADDR + ExtRegAddr;                    /* JBIT_EXT_REG_RD_DATA写入二级地址 */
                                                                        /* ExtRegAddr 扩展寄存器地址    */
    res = spi_SetReg(addr,regdata);                                     /* 写入扩展寄存器地址           */
    if (res == FALSE) 
        return FALSE;

    addr = JREG_EXT_REG_ENTRANCE;                                       /* 扩展寄存器0x0f地址           */
    res = spi_GetReg2(addr,&regdata);                                   /* 读出扩展寄存器数据           */
    if (res == FALSE) 
        return FALSE;
    *ExtRegData = regdata;

    return TRUE;    
}

/*********************************************************************************************************
** Function name:       Write_Ext_Reg
** Descriptions:        写入扩展寄存器
** input parameters:    reg_add，寄存器地址；
**                      reg_value，寄存器数值
** output parameters:   
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char Write_Ext_Reg(unsigned char reg_add,unsigned char reg_value)
{
    Write_Reg(0x0F,0x40+reg_add);
    return (Write_Reg(0x0F,0xC0+reg_value));
}




/*********************************************************************************************************
** Function name:       SetReg_Ext(uint8_t  ExtRegAddr,uint8_t * ExtRegData)
** Descriptions:        写扩展寄存器
** input parameters:    ExtRegAddr:扩展寄存器地址   
** output parameters:   ExtRegData:要写入的值
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  SetReg_Ext(uint8_t  ExtRegAddr,uint8_t  ExtRegData)
{
    uint8_t  res;
    uint8_t  addr,regdata;

    addr = JREG_EXT_REG_ENTRANCE;                                       /* 扩展寄存器入口地址0x0f       */
    regdata = JBIT_EXT_REG_WR_ADDR + ExtRegAddr;                        /* 写入扩展寄存器地址           */
    res = spi_SetReg(addr,regdata);
    if (res == FALSE) 
        return FALSE;

    addr = JREG_EXT_REG_ENTRANCE;
    regdata = JBIT_EXT_REG_WR_DATA + ExtRegData;                        /* 读出扩展寄存器数据           */
    res = spi_SetReg(addr,regdata);
    if (res == FALSE) 
        return FALSE;
        
    return TRUE;    
}


/*********************************************************************************************************
** Function name:       modifyReg_Ext(uint8_t  ExtRegAddr,uint8_t * mask,uint8_t  set)
** Descriptions:        寄存器位操作
** input parameters:    ExtRegAddr:目标寄存器地址   mask:要改变的位  
**                      set:  0:标志的位清零   其它:标志的位置起
** output parameters:  
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  ModifyReg_Ext(uint8_t  ExtRegAddr,uint8_t  mask,uint8_t  set)
{
    uint8_t  status;
    uint8_t  regdata;
    
    status = GetReg_Ext(ExtRegAddr,&regdata);
    if(status == TRUE) {
        if(set) {
            regdata |= mask;
            }
        else {
            regdata &= ~(mask);
        }
    }
    else
        return FALSE;

    status = SetReg_Ext(ExtRegAddr,regdata);
    return status;
}



uint8_t MCU_TO_PCD_TEST(void)
{
    volatile uint8_t ucRegVal;
    ucRegVal = spi_GetReg(ControlReg);
    spi_SetReg(ControlReg, 0x10);                                   
    ucRegVal = spi_GetReg(ControlReg);
    spi_SetReg(GsNReg, 0xF0 | 0x04);                               
    ucRegVal = spi_GetReg(GsNReg);
    if(ucRegVal != 0xF4)                                         
        return FALSE;
    return TRUE;
}
/*********************************************************************************************************
** Function name:       TypeA_Request        
** Descriptions:        TypeA_Request卡片寻卡    
** input parameters:    N/A
** output parameters:   pTagType[0],pTagType[1] =ATQA 
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char TypeA_Request(unsigned char *pTagType)
{
    unsigned char  result,send_buff[1],rece_buff[2];
    unsigned int  rece_bitlen;  
    Clear_BitMask(TxModeReg,0x80);                                      /* 关闭TX CRC                   */
    Clear_BitMask(RxModeReg,0x80);                                      /* 关闭RX CRC                   */
    Set_BitMask(RxModeReg, 0x08);                                       /* 关闭位接收                   */
    Clear_BitMask(Status2Reg,0x08);                                     /* 清除加密标志，使用明文通信   */
    Write_Reg(BitFramingReg,0x07);                                      /* 最后一字节发送7位            */
    send_buff[0] = 0x26;                                                /* 寻卡命令 0x26                */                  
    Pcd_SetTimer(1);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,1,rece_buff,&rece_bitlen);
    if ((result == TRUE) && (rece_bitlen == 2*8)) {                     /* 正常接收到2字节返回信息      */
        *pTagType     = rece_buff[0];
        *(pTagType+1) = rece_buff[1];
    }
    return result;
}

/*********************************************************************************************************
** Function name:       TypeA_WakeUp        
** Descriptions:        卡片唤醒
** input parameters:    N/A
** output parameters:   pTagType[0],pTagType[1] =ATQA 
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char TypeA_WakeUp(unsigned char *pTagType)
{
    unsigned char   result,send_buff[1],rece_buff[2];
    unsigned int   rece_bitlen;  
    Clear_BitMask(TxModeReg,0x80);                                      /* 关闭TX CRC                   */
    Clear_BitMask(RxModeReg,0x80);                                      /* 关闭RX CRC                   */
    Set_BitMask(RxModeReg, 0x08);                                       /* 不接收小于4bit的数据         */
    Clear_BitMask(Status2Reg,0x08);                                     /* 清除加密标志，使用明文通信   */
    Write_Reg(BitFramingReg,0x07);                                      /* 最后一字节发送7位            */
    send_buff[0] = 0x52;
    Pcd_SetTimer(1);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,1,rece_buff,&rece_bitlen);
    if ((result == TRUE) && (rece_bitlen == 2*8)) {
        *pTagType     = rece_buff[0];
        *(pTagType+1) = rece_buff[1];
    }
    return result;
}

/*********************************************************************************************************
** Function name:       Save_UID        
** Descriptions:        该函数实现保存卡片收到的序列号 
** input parameters:    Bytes: 产生冲突的字节    
**                      Bits: 产生冲突的位    
**                      length: 接収到的UID数据长度 
**                      buff:读回来的UID信息
** output parameters:   picc_uid：需要保存的UID信息
** Returned value:      void
*********************************************************************************************************/
void TypeA_Save_UID(unsigned char Bytes,unsigned char Bits,unsigned char length,unsigned char *buff,unsigned char *picc_uid)
{
    unsigned char i;
    unsigned char temp1;
    unsigned char temp2;
      
    temp2=buff[0];                                                      /* 后一次接收到的第一个有效UID  */
    temp1=picc_uid[Bytes-1];                                            /* 前一次接收到的最后一个有效UID*/
    switch (Bits) {                                                     /* 更具前一次的冲突位置         */
                                                                        /* 决定如何合并temp1和tmep2     */
        case 0:
            temp1=0x00;
            Bytes=Bytes+1;
            break;
        case 1:
            temp2=temp2 & 0xFE;
            temp1=temp1 & 0x01;
            break;            
        case 2:
            temp2=temp2 & 0xFC;
            temp1=temp1 & 0x03;
            break;
        case 3:
            temp2=temp2 & 0xF8;
            temp1=temp1 & 0x07;
            break;
        case 4:
            temp2=temp2 & 0xF0;
            temp1=temp1 & 0x0F;
            break;
        case 5:
            temp2=temp2 & 0xE0;
            temp1=temp1 & 0x1F;
            break;
        case 6:
            temp2=temp2 & 0xC0;
            temp1=temp1 & 0x3F;
            break;
        case 7:
            temp2=temp2 & 0x80;
            temp1=temp1 & 0x7F;
            break;
        default:
            break;
        }
        picc_uid[Bytes-1]=temp1 | temp2;                                /* 结合前后两部分               */
        for(i=1;i<length;i++) {
            picc_uid[Bytes-1+i]=buff[i];                                /* 将后续字节都拷贝到UID中      */
    }

}

/*********************************************************************************************************
** Function name:       Set_BitFraming        
** Descriptions:        该函数实现对收到的卡片的序列号的判断
** input parameters:    bytes: 已知的UID字节数  
**                      bits: 额外的已知UIDbits        
**                      length: 接収到的UID数据长度 
** output parameters:   NVB:防冲突种类
** Returned value:      void
*********************************************************************************************************/
void TypeA_Set_BitFraming(unsigned char  bytes,unsigned char  bits,unsigned char *NVB)
{    
    switch(bytes) {
        case 0: 
            *NVB = 0x20;
            break;
        case 1:
            *NVB = 0x30;
            break;
        case 2:
            *NVB = 0x40;
            break;
        case 3:
            *NVB = 0x50;
            break;
        case 4:
            *NVB = 0x60;
            break;
        default:
            break;
    }    
    switch(bits) {
        case 0:
            Write_Reg(BitFramingReg,0x00);
            break;
        case 1:
            Write_Reg(BitFramingReg,0x11);
            *NVB = (*NVB | 0x01);
            break;
        case 2:
            Write_Reg(BitFramingReg,0x22);
            *NVB = (*NVB | 0x02);
            break;
        case 3:
            Write_Reg(BitFramingReg,0x33);
            *NVB = (*NVB | 0x03);
            break;
        case 4:
            Write_Reg(BitFramingReg,0x44);
            *NVB = (*NVB | 0x04);
            break;
        case 5:
            Write_Reg(BitFramingReg,0x55);
            *NVB = (*NVB | 0x05);
            break;
        case 6:
            Write_Reg(BitFramingReg,0x66);
            *NVB = (*NVB | 0x06);
            break;
        case 7:
            Write_Reg(BitFramingReg,0x77);
            *NVB = (*NVB | 0x07);
            break;
        default:
            break;
    }
}

/*********************************************************************************************************
** Function name:       TypeA_Anticollision        
** Descriptions:        卡片防冲突    
** input parameters:    selcode：卡片选择编码 0x93，0x95，0x97    
**                      picc_uid：卡片UID号
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char TypeA_Anticollision(unsigned char selcode,unsigned char *picc_uid)
{
    unsigned char result,i;
    unsigned char send_buff[10];
    unsigned char rece_buff[10];
    unsigned int  rece_bitlen,bitCnt;                                   /* 可以使用 volatile            */
    unsigned char nBytes,Bits,Pre_nBytes,Pre_Bits;
    nBytes = 0;                                                         /* 当前防冲突指令接收字节数     */
    Bits = 0;                                                           /* 当前防冲突指令接收位数       */
    Pre_nBytes = 0;                                                     /* 防冲突指令已经接收字节数     */
    Pre_Bits = 0;                                                       /* 防冲突指令已经接收位数       */
    rece_bitlen = 0;                                                    /* 指令执行返回位数             */
    bitCnt=0;
    Clear_BitMask(TxModeReg,0x80);                                      /* 关闭TX CRC                   */
    Clear_BitMask(RxModeReg,0x80);                                      /* 关闭RX CRC                   */
    Clear_BitMask(Status2Reg,0x08);                                     /* 清除验证标志，使用明文通信   */
    Write_Reg(BitFramingReg,0x00);                                      /* 最后一字节发送8位            */
    Clear_BitMask(CollReg,0x80);                                        /* 清除防冲突位                 */
  
    send_buff[0] = selcode;                                             /* 防冲突命令                   */
    send_buff[1] = 0x20;                                                /* NVB 冲突位                   */
    Pcd_SetTimer(5);
    result = Pcd_Comm(Transceive,send_buff,2,rece_buff,&rece_bitlen);
    if(result == TRUE) {
        for (i=0; i<5; i++) { 
            *(picc_uid+i)  = rece_buff[i];           
        }
    }
    bitCnt = rece_bitlen;                                               /* 接收到的数据长度             */
        
    if(result==Anticollision){                                          /* 产生冲突                     */
        nBytes = bitCnt/8;                                              /* 接收到的字节数               */
        Bits  =  bitCnt%8;                                              /* 接收到的位数                 */
        if(bitCnt%8) nBytes++;                                          /* 算出第一次返回的数据字节数   */
            memcpy(picc_uid,rece_buff,nBytes);                          /* 保存接收到的部分UID          */
        Pre_nBytes += nBytes;                                           /* 保存上一次有效字节数         */
        Pre_Bits = Bits;                                                /* 保存上一次有效位             */                 

        while(result==Anticollision){                               /* 防冲突过程，只要有冲突，一直运行 */
            send_buff[0] = selcode;                                     /* 选择码                       */
            TypeA_Set_BitFraming(Pre_nBytes-1,Pre_Bits,&send_buff[1]);  /* 设置NVB发送                  */
            memcpy(&send_buff[2],picc_uid,nBytes);                      /* 将有效字节拷贝到发送buff     */
                                                      /* 设置最后字节发送的位，和接收的数据存放的起始位 */

        Pcd_SetTimer(5);
        Clear_FIFO();
        result = Pcd_Comm(Transceive,send_buff,2+nBytes,rece_buff,&rece_bitlen);    
        nBytes = rece_bitlen/8;
        Bits   = rece_bitlen%8;   
        if(Bits)
            nBytes+=1;
        TypeA_Save_UID(Pre_nBytes,Pre_Bits,nBytes,rece_buff,picc_uid);
        
        bitCnt += rece_bitlen;
        Pre_nBytes =  bitCnt/8;                                         /* 保存上一次有效字节数         */
        Pre_Bits = bitCnt%8;
        if(bitCnt%8) Pre_nBytes++; 
        }
    }
    if(picc_uid[4] != (picc_uid[0]^picc_uid[1]^picc_uid[2]^picc_uid[3])) /* 异或UID校验                 */
        result = FALSE;    
    return result;
}

/*********************************************************************************************************
** Function name:       TypeA_Select        
** Descriptions:        选择卡片
** input parameters:    selcode：卡片选择编码 0x93，0x95，0x97    
**                      pSnr：卡片UID号
**                      pSak：卡片选择应答
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char TypeA_Select(unsigned char selcode,unsigned char *pSnr,unsigned char *pSak)
{
    unsigned char   result,i,send_buff[7],rece_buff[5];
    unsigned int   rece_bitlen;
    Write_Reg(BitFramingReg,0x00);
    Set_BitMask(TxModeReg,0x80);                                        /* 打开TX CRC                   */
    Set_BitMask(RxModeReg,0x80);                                        /* 打开接收RX 的CRC校验         */
    Clear_BitMask(Status2Reg,0x08);                                     /* 清除验证标志位               */
    
    send_buff[0] = selcode;                                             /* select命令                   */
    send_buff[1] = 0x70;                                                /* NVB                          */
    for (i=0; i<5; i++) {
        send_buff[i+2] = *(pSnr+i);
    }
    send_buff[6] = pSnr[0]^pSnr[1]^pSnr[2]^pSnr[3];
    Pcd_SetTimer(1);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,7,rece_buff,&rece_bitlen);
    if (result == TRUE) {
        *pSak=rece_buff[0]; 
    }
    return result;
}

/*********************************************************************************************************
** Function name:       TypeA_Halt        
** Descriptions:        卡片睡眠
** input parameters:    AnticollisionFlag 休眠验证标志 
**                      AnticollisionFlag = 0 没有验证，使用明文通信，需要清除验证标志
**                      AnticollisionFlag = 1 密码验证函数通过，使用密文通信，不需要清除验证标志
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char TypeA_Halt(unsigned char AnticollisionFlag)
{
    unsigned char   result,send_buff[2],rece_buff[1];
    unsigned int   rece_bitlen;
    send_buff[0] = 0x50;
    send_buff[1] = 0x00;

    Write_Reg(BitFramingReg,0x00);                                      /* 最后发送一字节8位            */
    Set_BitMask(TxModeReg,0x80);                                        /* 打开TX CRC                   */
    Set_BitMask(RxModeReg,0x80);                                        /* 打开RX CRC                   */
    if( !AnticollisionFlag ) {
        Clear_BitMask(Status2Reg,0x08);
    }
    Pcd_SetTimer(1);
    Clear_FIFO();
    result = Pcd_Comm(Transmit,send_buff,2,rece_buff,&rece_bitlen);
    return result;
}

/*********************************************************************************************************
** Function name:       TypeA_CardActive        
** Descriptions:        函数实现寻卡防冲突和选择卡
** input parameters:    pTagType: 卡片类型 ATQA
**                      pSnr: 卡片UID
**                      pSak: 卡片应答数据 SAK
** output parameters:   N/A
** Returned value:      TRUE：操作成功 ERROR：操作失败    
*********************************************************************************************************/
unsigned char TypeA_CardActive(unsigned char *pTagType,unsigned char *pSnr,unsigned char *pSak)
{
    unsigned char   result;
    result=TypeA_Request(pTagType);                                     /* 寻卡 Standard                */
        if (result==FALSE) {
        return FALSE;
    }

    if( (pTagType[0]&0xC0) == 0x00 ) {                                  /* M1卡,ID号只有4位             */
        result=TypeA_Anticollision(0x93,pSnr);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x93,pSnr,pSak);                            /* 选择UID                      */
        if (result == FALSE) {
            return FALSE;
        }    
    }
        
    if( (pTagType[0]&0xC0) == 0x40 )  {                                 /* ID号有7位                    */
        result=TypeA_Anticollision(0x93,pSnr);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x93,pSnr,pSak);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Anticollision(0x95,pSnr+5);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x95,pSnr+5,pSak+1);
        if (result==FALSE) {
            return FALSE;
        }
    }
		
    if( (pTagType[0]&0xC0) == 0x80 )  {                                 /* ID号有10位                   */
        result=TypeA_Anticollision(0x93,pSnr);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x93,pSnr,pSak);
        if (result==FALSE) {
             return FALSE;
        }
        result=TypeA_Anticollision(0x95,pSnr+5);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x95,pSnr+5,pSak+1);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Anticollision(0x97,pSnr+10);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x97,pSnr+10,pSak+2);
        if (result==FALSE) {
            return FALSE;
        }
    }
    return result;
}

uint8_t  FM175XX_Initial(void)
{
    uint8_t  regdata,res;
//    uint8_t liu;
    
    regdata = 0x15;         //WaterLevel，收到一半数据时候，起中断
    res = spi_SetReg(WaterLevelReg,regdata);
    if(res != TRUE)
    {
         LOG("FM175XX_Initial is fail.\r\n"); 
        
    return FALSE;
    }
         LOG("FM175XX_Initial is OK.\r\n"); 



//liu= spi_GetReg(WaterLevelReg);

//        LOGA("liu = %d \r\n",liu);   
         
    
    return TRUE;
}

/*********************************************************************************************************
** Function name:       LpcdSet_DetectSensitive()
** Descriptions:        设置检测灵敏度
** input parameters:    loat Sensitive：灵敏度
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t   LpcdSet_DetectSensitive(float Sensitive)
{
    uint8_t  ret,Threshold;
    Threshold =ADCResultCenter*Sensitive; 
    /* 微调阈值 */
    /****LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;*/
    /****LpcdThreshold_L= ADCResultCenter - ADCResultThreshold;*/
    LpcdThreshold_H = ADCResultCenter + Threshold;
    LpcdThreshold_L = ADCResultCenter - Threshold;
    /*
    ** LpcdThreshold_L1寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_L,(LpcdThreshold_L& 0x3F));
    IF_ERR_THEN_RETURN;
    /*
    ** LpcdThreshold_L2寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_H,(LpcdThreshold_L>>6));
    IF_ERR_THEN_RETURN;
    /*
    ** LpcdThreshold_H1寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_L,(LpcdThreshold_H& 0x3F));
    IF_ERR_THEN_RETURN;
    /*
    ** LpcdThreshold_H2寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_H,(LpcdThreshold_H>>6));
    IF_ERR_THEN_RETURN;
//    uartPrintf("LpcdThreshold_H = %d \r\n",LpcdThreshold_H);        
//    uartPrintf("LpcdThreshold_L = %d \r\n",LpcdThreshold_L);        
//    uartPrintf("ADCResultCenter = %d \r\n",ADCResultCenter);    
    return TRUE;    
}

/*********************************************************************************************************
** Function name:       LpcdParamInit()
** Descriptions:        LPCD参数初始化
** input parameters:    
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/

uint8_t  LpcdParamInit()
{
    LpcdBiasCurrent = LPCD_BIAS_CURRENT ;                               /* 由样片测试指标决定           */
                                                                        /* 在config文件中设定           */
    LpcdGainReduce = 0x3;                                               /* 1x                           */
    LpcdGainAmplify = 0x0;                                              /* 1x                           */
    LpcdADCRefernce = 0;
    Timer1Cfg = TIMER1_CFG;                                             /* LPCD检测休眠阶段时间配置     */
    Timer2Cfg = TIMER2_CFG;                                             /* LPCD准备检测卡片时间配置     */
    Timer3Cfg = TIMER3_CFG;                                             /* LPCD检测阶段时间配置         */

    if (Timer3Cfg > 0xF)  {                                             /* Timer3Cfg用到5bit  16分频    */
        T3ClkDivK = 2;                                                  /* 16分频                       */
        ADCResultFullScale =  ((Timer3Cfg - 1)<<3);
        ADCResultCenter = (ADCResultFullScale >>1);
        ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);
    }
    else if(Timer3Cfg > 0x7) {                                          /* Timer3Cfg用到4bit  8分频     */
        T3ClkDivK = 1;                                                  /* 8分频                        */
        ADCResultFullScale =  ((Timer3Cfg - 1)<<4);                     /* 160                          */
        ADCResultCenter = (ADCResultFullScale >>1);                     /* 80                           */
        ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);  /* 10                       */
    }
    else  {
        T3ClkDivK = 0;                                                   /* 4分频                        */
        ADCResultFullScale =  ((Timer3Cfg - 1)<<5);
        ADCResultCenter = (ADCResultFullScale >>1);
        ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);
    }

    LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;               /* 高触发门槛 80+10=90          */
    LpcdThreshold_L= ADCResultCenter - ADCResultThreshold;               /* 低触发门槛 80-10=70          */

        LOGA("LpcdThreshold_H = %d \r\n",LpcdThreshold_H);        
        LOGA("LpcdThreshold_L = %d \r\n",LpcdThreshold_L);        
        LOGA("ADCResultCenter = %d \r\n",ADCResultCenter);        
        LOGA("ADCResultThreshold = %d \r\n",ADCResultThreshold);        
        LOGA("ADCResultFullScale = %d \r\n",ADCResultFullScale);        

        LOGA("Timer1Cfg = %d \r\n",Timer1Cfg);        
        LOGA("Timer2Cfg = %d \r\n",Timer2Cfg);        
        LOGA("Timer3Cfg = %d \r\n",Timer3Cfg);        
    
    return TRUE;
}

/*********************************************************************************************************
** Function name:       TyteA_Test()
** Descriptions:        ISO14443A协议测试
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
bool TyteA_Test (void)
{
    uint8_t statues = FALSE;
    uint8_t num=0;
    uint8_t picc_atqa[2];                                               /* 储存寻卡返回卡片类型信息     */
    static uint8_t picc_uid[15];                                        /* 储存卡片UID信息              */
    uint8_t picc_sak[3];                                                /* 储存卡片应答信息             */
    FM175X_SoftReset( );                                                /* FM175xx软件复位              */
    Set_Rf( 3 );                                                        /* 打开双天线                   */
    Pcd_ConfigISOType( 0 );                                             /* ISO14443寄存器初始化         */

        statues = TypeA_CardActive( picc_atqa,picc_uid,picc_sak );      /* 激活卡片                     */
        if ( statues == TRUE ) {
            num = 0;
            TypeA_Halt(0);                                              /* 睡眠卡片                     */     
            memset(picc_uid,0x00,15);   
        }
        else
        {
statues = FALSE;

        }

        return statues;

}
/*********************************************************************************************************
** Function name:       ReadLpcdADCResult()
** Descriptions:        读取LPCD幅度
** input parameters:    uchar *ADCResult
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  ReadLpcdADCResult(uint8_t  *ADCResult)
{
    uint8_t  ExtRegData;
    uint8_t  ret;
    uint8_t  ADCResultTemp=0;

    *ADCResult = 0;
    
    ret = GetReg_Ext(JREG_LPCD_ADC_RESULT_H,&ExtRegData);               /* 读取宽度信息                 */
    IF_ERR_THEN_RETURN;
    ADCResultTemp = (ExtRegData & 0x3) << 6;
    
    ret = GetReg_Ext(JREG_LPCD_ADC_RESULT_L,&ExtRegData);
    IF_ERR_THEN_RETURN;
    
    ADCResultTemp += (ExtRegData&0x3F) ;
    *ADCResult = ADCResultTemp;

    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN); /* 复位LPCD寄存器               */
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN); /* 复位放开LPCD寄存器           */
    IF_ERR_THEN_RETURN;
        
    LOGA("ADCResultTemp = %d \r\n",ADCResultTemp);        
    return TRUE;
}


/*********************************************************************************************************
** Function name:       LpcdRegisterInit()
** Descriptions:        LPCD寄存器初始化
** input parameters:    
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  LpcdRegisterInit(void)
{
    uint8_t  ret;
    uint8_t  regdata;
    uint8_t  read;
    uint8_t  write;
    
    regdata = COMMIEN_DEF;                                              /* 中断设置，设置中断引脚IRQ输出*/
    ret = spi_SetReg(ComIEnReg,regdata);                                /* 中断IRQ输出低电平            */
    IF_ERR_THEN_RETURN;


    read=spi_GetReg(ComIEnReg) ;
 

    LOGA("ComIEnReg = %d \r\n",read);     


    
    /* 配置IRQ引脚作为标准CMOS输出  */
    ret = spi_SetReg(DivIEnReg,regdata);                                /* STatus1Reg的IRQ位相反        */


    
    IF_ERR_THEN_RETURN;       


    read=spi_GetReg(DivIEnReg);    
 

    LOGA("DivIEnReg = %d \r\n",read);     

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read); 

    GetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,&read);
    LOGA("JREG_LPCD_AUTO_WUP_CFG = %d \r\n",read);        
    
    
    
    /*
    ** LpcdCtrl1寄存器
    */ 
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN); /* 复位LPCD寄存器               */
    IF_ERR_THEN_RETURN;

   write=JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        





    
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN); /* 复位放开LPCD寄存器           */
    IF_ERR_THEN_RETURN;


   write=JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN;
    LOGA("write = %d \r\n",write);        


    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        
    
    
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_EN);   /* 使能LPCD功能                 */
    IF_ERR_THEN_RETURN;


   write=JBIT_BIT_CTRL_SET+JBIT_LPCD_EN;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        
    
    
    ret = SetReg_Ext(JREG_LPCD_CTRL1,(LPCD_IE<<5)+JBIT_LPCD_IE);        /* 配置LPCD中断寄存器状态       */
    IF_ERR_THEN_RETURN;   


   write=(LPCD_IE<<5)+JBIT_LPCD_IE;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        
    /* 反映到IRQ引脚                */
    
                                                                        /* 配置进场检测次数             */
    ret = SetReg_Ext(JREG_LPCD_CTRL1,(LPCD_IE<<5)+JBIT_LPCD_CMP_1);     /* 一次检测到卡有效             */
    //ret = SetReg_Ext(JREG_LPCD_CTRL1,(LPCD_DS<<5)+JBIT_LPCD_CMP_3);   /* 3次检测到卡有效              */
    IF_ERR_THEN_RETURN;


   write=(LPCD_IE<<5)+JBIT_LPCD_CMP_1;
    LOGA("write = %d \r\n",write);        
    
    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        

    
    /*
    ** LpcdCtrl2寄存器
    */        
    ret = SetReg_Ext(JREG_LPCD_CTRL2,((LPCD_TX2RFEN<<4)+(LPCD_CWN<<3)+LPCD_CWP)); /* P管驱动能力从0到7  */
    IF_ERR_THEN_RETURN;       



   write=((LPCD_TX2RFEN<<4)+(LPCD_CWN<<3)+LPCD_CWP);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL2,&read);
    LOGA("JREG_LPCD_CTRL2 = %d \r\n",read);        
    /* 一次增大，这里选了3          */
        
    /*
    ** LpcdCtrl3寄存器
    */   
    ret = SetReg_Ext(JREG_LPCD_CTRL3,LPCD_MODE<<3);                     /* 没看出实际意义               */
    IF_ERR_THEN_RETURN;


   write=LPCD_MODE<<3;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL3,&read);
    LOGA("JREG_LPCD_CTRL3 = %d \r\n",read);        

    /*
    ** Timer1Cfg寄存器
    */ 
    ret = SetReg_Ext(JREG_LPCD_T1CFG,(T3ClkDivK<<4)+Timer1Cfg);         /* 设置LPCD睡眠时间             */
    IF_ERR_THEN_RETURN;



   write=(T3ClkDivK<<4)+Timer1Cfg;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_T1CFG,&read);
    LOGA("JREG_LPCD_T1CFG = %d \r\n",read);        
 
    /*
    ** Timer2Cfg寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_T2CFG,Timer2Cfg);                        /* 设置LPCD准备检测时间         */ 
    IF_ERR_THEN_RETURN;


   write=Timer2Cfg;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_T2CFG,&read);
    LOGA("JREG_LPCD_T2CFG = %d \r\n",read);        
    
        
    /*
    ** Timer3Cfg寄存器
    */ 
    ret = SetReg_Ext(JREG_LPCD_T3CFG,Timer3Cfg);                        /* 设置LPCD检测卡片时间         */
    IF_ERR_THEN_RETURN;


   write=Timer3Cfg;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_T3CFG,&read);
    LOGA("JREG_LPCD_T3CFG = %d \r\n",read);        

    /*
    ** VmidBdCfg寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_VMIDBD_CFG,VMID_BG_CFG);                 /* 不建议用户修改               */
    IF_ERR_THEN_RETURN;



   write=VMID_BG_CFG;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_VMIDBD_CFG,&read);
    LOGA("JREG_LPCD_VMIDBD_CFG = %d \r\n",read);        
        
    /*
    ** AutoWupCfg寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,(AUTO_WUP_EN<<3)+AUTO_WUP_CFG);  /* 设置自动唤醒时间        */
    IF_ERR_THEN_RETURN;



   write=(AUTO_WUP_EN<<3)+AUTO_WUP_CFG;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,&read);
    LOGA("JREG_LPCD_AUTO_WUP_CFG = %d \r\n",read);        
        
    /*
    ** LpcdThreshold_L1寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_L,(LpcdThreshold_L & 0x3F));  /* 设置卡检测下阀值          */
    IF_ERR_THEN_RETURN;



   write=(LpcdThreshold_L & 0x3F);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MIN_L,&read);
    LOGA("JREG_LPCD_THRESHOLD_MIN_L = %d \r\n",read);        
        
    /*
    ** LpcdThreshold_L2寄存器
    */ 
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_H,(LpcdThreshold_L>>6));   
    IF_ERR_THEN_RETURN;



   write=(LpcdThreshold_L>>6);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MIN_H,&read);
    LOGA("JREG_LPCD_THRESHOLD_MIN_H = %d \r\n",read);        
        
    /*
    ** LpcdThreshold_H1寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_L,(LpcdThreshold_H& 0x3F));  /* 设置卡检测上阀值           */
    IF_ERR_THEN_RETURN;


   write=(LpcdThreshold_H& 0x3F);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MAX_L,&read);
    LOGA("JREG_LPCD_THRESHOLD_MAX_L = %d \r\n",read);        
    
    /*
    ** LpcdThreshold_H2寄存器
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_H,(LpcdThreshold_H>>6));   
    IF_ERR_THEN_RETURN;


   write=(LpcdThreshold_H>>6);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MAX_H,&read);
    LOGA("JREG_LPCD_THRESHOLD_MAX_H = %d \r\n",read);        
    
    
    /*
    ** Auto_Wup_Cfg寄存器
    */
    ret=SetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,(AUTO_WUP_EN<<3) + AUTO_WUP_CFG );  /* 再次设置自动唤醒时间   */
    IF_ERR_THEN_RETURN;



   write=(AUTO_WUP_EN<<3) + AUTO_WUP_CFG ;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,&read);
    LOGA("JREG_LPCD_AUTO_WUP_CFG = %d \r\n",read);        
    

            LOG(" LpcdRegisterInit  OK.\r\n");             /* 表示LPCD调校成功             */
        
    return TRUE;
}



/*********************************************************************************************************
** Function name:       WaitForLpcdIrq()
** Descriptions:        等待LPCD中断
** input parameters:    IrqType
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  WaitForLpcdIrq(uint8_t  IrqType)
{
    uint8_t  ExtRegData;
    uint8_t  ret;
    uint8_t  TimeOutCount;
    
    TimeOutCount = 0;
    ret = GetReg_Ext(JREG_LPCD_IRQ,&ExtRegData);
    //debug
    if (ret == 0)
    {
        ret = GetReg_Ext(JREG_LPCD_IRQ,&ExtRegData);
        ret = GetReg_Ext(JREG_LPCD_IRQ,&ExtRegData);
        ret = GetReg_Ext(JREG_LPCD_IRQ,&ExtRegData);
        ret =1;
    }
    IF_ERR_THEN_RETURN;
    while ((ExtRegData & IrqType) != IrqType)
    {
        ret = GetReg_Ext(JREG_LPCD_IRQ,&ExtRegData);
        //debug
        if (ret == 0)
        {
            ret =1;
        }
        IF_ERR_THEN_RETURN;
        //超时退出
        delay_us(1000);                                                /* 延时10ms                     */
        TimeOutCount++;
        if (TimeOutCount > IRQ_TIMEOUT)  
        {
            return FALSE;                                               /* 150ms 超时退出               */
        }
    }
                LOG(" WaitForLpcdIrq  is true.\r\n");          /* LPCD调校失败                 */
    return TRUE; 
}


/*********************************************************************************************************
** Function name:       CalibraReadADCResult()
** Descriptions:        调教并读取LPCD幅度
** input parameters:    uchar *ADCResult
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t  CalibraReadADCResult(uint8_t  *ADCResult)
{
    //使能调教模式
    uint8_t  ret;
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN);
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_CALIBRA_EN);
    IF_ERR_THEN_RETURN;
    delay_us(10000);

    ret = WaitForLpcdIrq(JBIT_CALIB_IRQ);                               /* 等待调教结束中断             */
    //debug
    if (ret == 0)
    {
        ret =1;
    }
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN); /* 关闭调教模式           */
    IF_ERR_THEN_RETURN;
        
    ret = ReadLpcdADCResult(ADCResult);                                 /* 读取幅度信息                 */
    IF_ERR_THEN_RETURN;

            LOG(" CalibraReadADCResult  is true.\r\n");          /* LPCD调校失败                 */
    
    return TRUE;
}

/*********************************************************************************************************
** Function name:       LpcdSet_ADCRefvoltage()
** Descriptions:        设置合适的ADC参考电压
** input parameters:    uint8_t  *CalibraFlag, uint8_t  *ADCResult   幅度检测结果保存地址
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t   LpcdSet_ADCRefvoltage(uint8_t  *CalibraFlag, uint8_t  *ADCResult)
{
    uint8_t  ret;
    uint8_t  ADCResult_Pre;                                             /* 幅度信息的前一个值           */
    
    //扫描充电电容
    for(LpcdADCRefernce = ADC_REFERNCE_MIN;LpcdADCRefernce < ADC_REFERNCE_MAX;LpcdADCRefernce++)
    //通过此循环可以确定最终的参考电压和ADC中心比较值
    {
            //配置参考电压值
        ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
        IF_ERR_THEN_RETURN;
        ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
        IF_ERR_THEN_RETURN;
        //备份幅度信息
        ADCResult_Pre = *ADCResult;
        //调教读取当前幅度信息
        ret = CalibraReadADCResult(ADCResult);
        IF_ERR_THEN_RETURN;
        /*
              ** 算法一
              */
        // 如果幅度开始比中心值小，因为开始比中心值大
        if (*ADCResult < ADCResultCenter) {
            //调教成功
            (*CalibraFlag) = TRUE;
            //与前一个调教参考电压，判断哪个幅度更接近中心点
            if((ADCResultCenter - *ADCResult) < (ADCResult_Pre-ADCResultCenter))
            {
                //直接用当前值作为中心点
                ADCResultCenter = *ADCResult;
            }
            else
            {
                //直接用当前值作为中心点
                ADCResultCenter = ADCResult_Pre;
                //参考电压采用之前的参考电压
                LpcdADCRefernce--;
                //重新配置参考电压值
                ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
                IF_ERR_THEN_RETURN;
                ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
                IF_ERR_THEN_RETURN;
            }
            break;
        }
    }
    if(LpcdADCRefernce < ADC_REFERNCE_MAX)
        return TRUE;
    else
        return FALSE;
}



/*********************************************************************************************************
** Function name:       LpcdSet_PGA()
** Descriptions:        设置合适的PGA参数
** input parameters:    uint8_t  *CalibraFlag, uint8_t  *ADCResult   幅度检测结果保存地址
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t   LpcdSet_PGA(uint8_t  *GainCalibraFlag, uint8_t  *ADCResult)
{
    uint8_t  ret;
    //参考电压值配置最小
    LpcdADCRefernce = ADC_REFERNCE_MIN;
    ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)>>1)+LpcdBiasCurrent&0x7);
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
    IF_ERR_THEN_RETURN;

    //调教读取当前幅度信息
    ret = CalibraReadADCResult(ADCResult);
    IF_ERR_THEN_RETURN;
    //缺省增益不需要调教
    *GainCalibraFlag = TRUE;
   
    //判断是否幅度太窄，如果太窄lpcd_gain衰减
    if  (*ADCResult < ADCResultCenter)
    {
            LOG("ADCResult<ADCResultCenter 00.\r\n");          /* LPCD调校失败                 */
        
        //增益需要调教
        *GainCalibraFlag = FALSE;
        //*GainCalibraFlag = LpcdSetPGA_GainReduce(ADCResult);   //设置PGA增益衰减
        while(1)
        {
            LOG("ADCResult<ADCResultCenter 01.\r\n");          /* LPCD调校失败                 */
            
                //如果当前已经是最小增益，调教失败
            if (LpcdGainReduce == 0)
            {
                *GainCalibraFlag = FALSE;
                break;
            }
            //继续衰减
            LpcdGainReduce --; 
            // uartPrintf("LpcdSet_PGA  LpcdGainReduce=%d !\r\n",LpcdGainReduce);
            //配置增益
            ret = SetReg_Ext(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));
            IF_ERR_THEN_RETURN;

            //调教读取当前幅度信息
            ret = CalibraReadADCResult(ADCResult);
            //uartPrintf("LpcdSet_PGA  ADCResult=%d !\r\n",*ADCResult);
            IF_ERR_THEN_RETURN;
            //调教成功，把中心点移到中心点右侧
            if (*ADCResult >ADCResultCenter)
            {
                *GainCalibraFlag = TRUE;
                break;
            }
         }    
    }
    else
    {
            LOG("LpcdADCRefernce = ADC_REFERNCE_MAX.\r\n");          /* LPCD调校失败                 */
        
        //参考电压值配置最大
        LpcdADCRefernce = ADC_REFERNCE_MAX;
        ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
        IF_ERR_THEN_RETURN;
        ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
        IF_ERR_THEN_RETURN;

        //调教读取当前幅度信息
        ret = CalibraReadADCResult(ADCResult);
        //uartPrintf("LpcdSet_PGA  ADCResult=%d !\r\n",*ADCResult);
        IF_ERR_THEN_RETURN;

        //调教成功标志初始化
        *GainCalibraFlag = TRUE;
        
        //判断是否幅度太小，如果太小lpcd_gain放大
        if (*ADCResult > ADCResultCenter)
        {
            LOG("ADCResult>ADCResultCenter 00.\r\n");          /* LPCD调校失败                 */
            
            //增益需要调教
            *GainCalibraFlag = FALSE;
            while(1)
            {
            LOG("ADCResult>ADCResultCenter 01.\r\n");          /* LPCD调校失败                 */
                
                //如果当前已经是最大增益，调教失败
                if (LpcdGainAmplify == 0x7)
                {
                    *GainCalibraFlag = FALSE;
                    break;
                }
                else//继续放大
                {
                    LpcdGainAmplify++;  
                }
                        //配置增益
                ret = SetReg_Ext(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));
                IF_ERR_THEN_RETURN;

                //调教读取当前幅度信息
                  ret = CalibraReadADCResult(ADCResult);
                IF_ERR_THEN_RETURN;
                
                //调教成功，把中心点移到中心点左侧
                if (*ADCResult < ADCResultCenter)
                {
                    *GainCalibraFlag = TRUE;
                     break;
                }
            }
        }
    }
    return TRUE;
}

/*********************************************************************************************************
** Function name:       LpcdInitCalibra()
** Descriptions:        初始化调教
** input parameters:    uint8_t  *CalibraFlag 调教标志，是否需要调教
** output parameters:   
** Returned value:      uint8_t   TRUE：读取成功   FALSE:失败
*********************************************************************************************************/
uint8_t   LpcdInitCalibra(uint8_t  *CalibraFlag)
{
    uint8_t  ret;
    uint8_t  ADCResult;                                                 /* LPCD幅度信息                 */
    uint8_t  GainCalibraFlag;                                           /* 增益调教结果                 */
                                                                        /* 配置增益                     */
    ret = SetReg_Ext(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));
    IF_ERR_THEN_RETURN;
                                                                        /* 配置偏置电流和参考电压       */
    ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)>>1)+LpcdBiasCurrent&0x7);
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN);            /* 开启调校模式中Vmind使能      */
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_T1CFG,(T3ClkDivK<<4)+Timer1Cfg);         /* Timer1Cfg配置                */
    IF_ERR_THEN_RETURN;


    ret = SetReg_Ext(JREG_LPCD_T2CFG,Timer2Cfg);                        /* Timer2Cfg配置                */
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_T3CFG,Timer3Cfg);                        /* Timer3Cfg配置                */
    IF_ERR_THEN_RETURN;

            LOG("LpcdInitCalibra enter  LpcdSet_PGA.\r\n");
    
                                                                          
    ret = LpcdSet_PGA(&GainCalibraFlag,&ADCResult);                     /* PGA参数设置                  */
    IF_ERR_THEN_RETURN;                                              /* 调校过程PGA的增益和衰减参数设定 */



    if (GainCalibraFlag == FALSE)                                       /* 如果增益调教失败，则失败     */
    {        
          (*CalibraFlag) = FALSE;
        return ADCResult;                                               /* 调教失败返回幅度             */
    }
    //扫描参考电压值，找到合适的穿越中心点的配置
    (*CalibraFlag) = FALSE;
    GainCalibraFlag = LpcdSet_ADCRefvoltage(CalibraFlag,&ADCResult);    /* 设置合适的ADC参考电压        */
    ret = LpcdSet_DetectSensitive(LPCD_DetectSensitive);                /* 调校过程检测灵敏度设定       */
    IF_ERR_THEN_RETURN;
    
    if (GainCalibraFlag == FALSE)
    {        
        (*CalibraFlag) = FALSE;
        ret = ModifyReg_Ext(JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN,0);   /* 增加 调教结束关闭CalibVmidEn */
        IF_ERR_THEN_RETURN;
        return ADCResult;                                               /* 调教失败返回幅度             */
    }
    
    ret = ModifyReg_Ext(JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN,0);       /* 调教结束关闭CalibVmidEn      */
    IF_ERR_THEN_RETURN;
    return TRUE;
}

void FM17550MgrHandle(NoBikeTaskData_t *app)
{
    uint8_t CalibraFlag;
    
    switch(FM17550Data.FM17550SeqData)
    {
        case FM17550Seq_Idle:
            FM17550Data.FM17550SeqData=FM17550Seq_RstStart;

        break;
        case FM17550Seq_RstStart:
            pcd_Init();
            SPI_FM17550_RST_HIGH();
            app->Time_Delay.FM17550_TIMEOUT_counts_1ms=0;
            FM17550Data.FM17550SeqData=FM17550Seq_RstLow;
        break;    

        case FM17550Seq_RstLow:
            if(app->Time_Delay.FM17550_TIMEOUT_counts_1ms>delay_2ms)
            {
                SPI_FM17550_RST_LOW();
                app->Time_Delay.FM17550_TIMEOUT_counts_1ms=0;
                FM17550Data.FM17550SeqData=FM17550Seq_RstFinish;
            }

        break;
        
        case FM17550Seq_RstFinish:
            if(app->Time_Delay.FM17550_TIMEOUT_counts_1ms>delay_2ms)
            {
                SPI_FM17550_RST_HIGH();
                app->Time_Delay.FM17550_TIMEOUT_counts_1ms=0;
                FM17550Data.FM17550SeqData=FM17550Seq_VerifyStart;
            }

        break;
        
        case FM17550Seq_VerifyStart:
            if(MCU_TO_PCD_TEST())
            {
                FM17550Data.FM17550SeqData=FM17550Seq_Config;
         LOG("MCU_TO_PCD_TEST is OK.\r\n"); 
                
            }
            else
            {
                LOG("FM17550 init false");

            }
        break;


        case FM17550Seq_Config:
            if(app->Time_Delay.FM17550_TIMEOUT_counts_1ms>delay_100ms)
            {
                #if 1
                spi_SetReg(SerialSpeedReg,0x3A);                              
                delay_us(10000);
          
                FM175XX_Initial();
                LpcdParamInit();                                                /* LPCD参数初始化               */
                LpcdRegisterInit();                                             /* LPCD寄存器初始化             */
                //LpcdAuxSelect(OFF);                                           /* 开启AUX观测通道              */
                LpcdInitCalibra(&CalibraFlag);                                  /* LPCD初始化调教               */


                if (CalibraFlag == TRUE)    
                { 

                    LOG("CalibraFlag is ture");
                    
            delay_us(10000);
                    SPI_FM17550_RST_LOW();      
            
            LOG("FM17550 sleep.\r\n");
                FM17550Data.FM17550SeqData=FM17550Seq_WaitData;
            





                    
                } 
                else 
                {
                                         
                }
          
#endif

                
//                if(TyteA_Test())
//                FM17550Data.FM17550SeqData=FM17550Seq_OpenLock;
//                else
//                {
//                    app->Time_Delay.FM17550_TIMEOUT_counts_1ms=0;
//                }
            }
        break;


        case FM17550Seq_WaitData:


        break;


        case FM17550Seq_OpenLock:
                                                    
            OpenUnLockStart();
            LOG("OpenUnLockStart");

             app->Time_Delay.FM17550_TIMEOUT_counts_1ms=0;
            

            FM17550Data.FM17550SeqData=FM17550Seq_OpenLock_Delay;


        break;

        case FM17550Seq_OpenLock_Delay:
            if(app->Time_Delay.FM17550_TIMEOUT_counts_1ms>delay_1s)
            FM17550Data.FM17550SeqData=FM17550Seq_Config;
                


        break;
        case FM17550Seq_End:
        break;
    }
}

