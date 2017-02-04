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
** ���̱����б�
*/ 
uint8_t  T3ClkDivK ;
uint8_t  LpcdBiasCurrent;                                               /* 3bit ����Ƭ����ָ�������
                                                                            ��config�ļ����趨          */
uint8_t  LpcdGainReduce;                                                /* 2bit                         */
uint8_t  LpcdGainAmplify;                                               /* 3bit                         */
uint8_t  LpcdADCRefernce;                                               /* 7bit                         */

uint8_t  Timer1Cfg;                                                     /* 4bit                         */
uint8_t  Timer2Cfg;                                                     /* 5bit                         */
uint8_t  Timer3Cfg;                                                     /* 5bit                         */

uint8_t  ADCResultFullScale;                                            /* T3������ADCResult��Ϣ        */
uint8_t  ADCResultThreshold;                                            /* �����ȣ����ó����ֵ       */
uint8_t  LpcdThreshold_L;                                               /* LPCD���ȵ���ֵ               */
uint8_t  LpcdThreshold_H;                                               /* LPCD���ȸ���ֵ               */
uint8_t  ADCResultCenter;                                               /* LPCD�������ĵ�               */
uint8_t  LpcdADCResult[10];                                             /* Lpcd������Ϣ�������󴥷��ж� */



/*********************************************************************************************************
** Function name:       spi_SetReg
** Descriptions:        SPIд����оƬ�Ĵ�������
** input parameters:    ucRegAddr���Ĵ�����ַ
**                      ucRegVal��Ҫд���ֵ
** output parameters:   ��
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
** Descriptions:        SPI������оƬ�Ĵ�������
** input parameters:    ucRegAddr���Ĵ�����ַ
** output parameters:   ��
** Returned value:      Ŀ��Ĵ�����ֵ
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
** Descriptions:        SPI������оƬ�Ĵ�������
** input parameters:    ucRegAddr���Ĵ�����ַ
** output parameters:   ��
** Returned value:      Ŀ��Ĵ�����ֵ
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
** Descriptions:        SPI��FIFO�Ĵ�����ֵ
** input parameters:    sequence_length ���ݳ��� ucRegAddr���Ĵ�����ַ  *reg_value ����ָ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        SPIдFIFO��ֵ
** input parameters:    sequence_length ���ݳ��� 
**                      ucRegAddr���Ĵ�����ַ  
**                      *reg_value ����ָ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        MCU��ʼ������������SPI��UART�ĳ�ʼ��
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
** Descriptions:        ��ȡ�Ĵ���                
** input parameters:    reg_add:�Ĵ�����ֵ
** output parameters:   N/A
** Returned value:      �Ĵ�����ֵ
*********************************************************************************************************/
unsigned char Read_Reg(unsigned char reg_add)
{
    unsigned char  reg_value;       
    reg_value=spi_GetReg(reg_add);
    return reg_value;
}

/*********************************************************************************************************
** Function name:       Read_Reg_All
** Descriptions:        ��ȡȫ���Ĵ���                
** input parameters:    reg_value:�Ĵ�����ֵ
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Read_Reg_All(unsigned char *reg_value)
{
    for (uint8_t i=0;i<64;i++)       
        *(reg_value+i) = spi_GetReg(i);
    return TRUE;
}


/*********************************************************************************************************
** Function name:       Write_Reg
** Descriptions:        д�Ĵ�������                
** input parameters:    reg_add:�Ĵ�����ַ
**                      reg_value:�Ĵ�����ֵ
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Write_Reg(unsigned char reg_add,unsigned char reg_value)
{
    spi_SetReg(reg_add,reg_value);
    return TRUE;
}


/*********************************************************************************************************
** Function name:       Read_FIFO
** Descriptions:        ����FIFO������         
** input parameters:    length:��ȡ���ݳ���
**                      *fifo_data:���ݴ��ָ��
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
void Read_FIFO(unsigned char length,unsigned char *fifo_data)
{     
    SPIRead_Sequence(length,FIFODataReg,fifo_data);
    return;
}


/*********************************************************************************************************
** Function name:       Write_FIFO
** Descriptions:        д��FIFO         
** input parameters:    length:��ȡ���ݳ���
**                      *fifo_data:���ݴ��ָ��
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
void Write_FIFO(unsigned char length,unsigned char *fifo_data)
{
    SPIWrite_Sequence(length,FIFODataReg,fifo_data);
    return;
}

/*********************************************************************************************************
** Function name:       Set_BitMask
** Descriptions:        ��λ�Ĵ�������    
** input parameters:    reg_add���Ĵ�����ַ
**                      mask���Ĵ������λ
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Set_BitMask(unsigned char reg_add,unsigned char mask)
{
    uint8_t result;
    result=spi_SetReg(reg_add,Read_Reg(reg_add) | mask);                /* set bit mask                 */
    return result;
}


/*********************************************************************************************************
** Function name:       Clear_FIFO
** Descriptions:        ���FIFO              
** input parameters:   
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Clear_FIFO(void)
{
    Set_BitMask(FIFOLevelReg,0x80);                                     /* ���FIFO����                 */
    if ( spi_GetReg(FIFOLevelReg) == 0 )
        return TRUE;
    else
        return FALSE;
}



/*********************************************************************************************************
** Function name:       Clear_BitMask
** Descriptions:        ���λ�Ĵ�������
** input parameters:    reg_add���Ĵ�����ַ
**                      mask���Ĵ������λ
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Clear_BitMask(unsigned char reg_add,unsigned char mask)
{
    uint8_t result;
    result=Write_Reg(reg_add,Read_Reg(reg_add) & ~mask);                /* clear bit mask               */
    return result;
}


/*********************************************************************************************************
** Function name:       Set_RF
** Descriptions:        ������Ƶ���
** input parameters:    mode����Ƶ���ģʽ
**                      0���ر����
**                      1������TX1���
**                      2������TX2���
**                      3��TX1��TX2�������TX2Ϊ�������
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Set_Rf(unsigned char mode)
{
    unsigned char result;
    if( (Read_Reg(TxControlReg)&0x03) == mode )
        return TRUE;
    if( mode == 0 )
        result = Clear_BitMask(TxControlReg,0x03);                      /* �ر�TX1��TX2���             */
    if( mode== 1 )
        result = Clear_BitMask(TxControlReg,0x01);                      /* ����TX1���                */
    if( mode == 2)
        result = Clear_BitMask(TxControlReg,0x02);                      /* ����TX2���                */
    if (mode==3)
        result=Set_BitMask(TxControlReg,0x03);                          /* ��TX1��TX2���             */
    delay_us(10000);
    return result;
}
 
/*********************************************************************************************************
** Function name:       Pcd_Comm
** Descriptions:        ������ͨ�� ������IRQ�ܽŵ����
** input parameters:    Command:ͨ�Ų�������
**                      pInData:������������
**                      InLenByte:�������������ֽڳ���
**                      pOutData:������������
**                      pOutLenBit:�������ݵ�λ����
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Pcd_Comm(    unsigned char Command, 
                         unsigned char *pInData, 
                         unsigned char InLenByte,
                         unsigned char *pOutData, 
                         unsigned int *pOutLenBit)
{
    uint8_t status  = FALSE;
    uint8_t irqEn   = 0x00;                                             /* ʹ�ܵ��ж�                   */
    uint8_t waitFor = 0x00;                                             /* �ȴ����ж�                   */
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;
    Write_Reg(ComIrqReg, 0x7F);                                         /* ���IRQ���                  */
    Write_Reg(TModeReg,0x80);                                           /* ����TIMER�Զ�����            */
    switch (Command) {
    case MFAuthent:                                                     /* Mifare��֤                   */
        irqEn   = 0x12;
        waitFor = 0x10;
        break;
    case Transceive:                                       /* ����FIFO�е����ݵ����ߣ�����󼤻���յ�·*/
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

    i = 3000;                                              /* ����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms*/

    do {
        n = Read_Reg(ComIrqReg);
        i--;                                                            /* i==0��ʾ��ʱ����             */
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));            /* n&0x01!=1��ʾPCDsettimerʱ��δ�� */
                                                                        /* n&waitFor!=1��ʾָ��ִ����� */
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
       Clear_BitMask(BitFramingReg,0x80);//�رշ���
    return status;
}


/*********************************************************************************************************
** Function name:       Pcd_SetTimer
** Descriptions:        ���ý�����ʱ
** input parameters:    delaytime����ʱʱ�䣨��λΪ���룩  
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
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
** Descriptions:        ����ISO14443A/BЭ��
** input parameters:    type = 0��ISO14443AЭ�飻
**                                 type = 1��ISO14443BЭ�飻   
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Pcd_ConfigISOType(unsigned char type)
{
    if (type == 0)   {                                                  /* ����ΪISO14443_A             */
        Set_BitMask(ControlReg, 0x10);                                /* ControlReg 0x0C ����readerģʽ */
        Set_BitMask(TxAutoReg, 0x40);                                  /* TxASKReg 0x15 ����100%ASK��Ч */
        Write_Reg(TxModeReg, 0x00);                 /* TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE A */
        Write_Reg(RxModeReg, 0x00);                 /* RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE A */
    }
    if (type == 1)   {                                                  /* ����ΪISO14443_B           */
        Write_Reg(ControlReg,0x10);
        Write_Reg(TxModeReg,0x83);                                      /* BIT1~0 = 2'b11:ISO/IEC 14443B */
        Write_Reg(RxModeReg,0x83);                                      /* BIT1~0 = 2'b11:ISO/IEC 14443B */
        Write_Reg(TxAutoReg,0x00);
        Write_Reg(RxThresholdReg,0x55);
        Write_Reg(RFCfgReg,0x48);
        Write_Reg(TxBitPhaseReg,0x87);                                  /* Ĭ��ֵ                         */
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
** Descriptions:        FM175xx�����λ
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char  FM175X_SoftReset(void)
{    
    Write_Reg(CommandReg,SoftReset);
    return    Set_BitMask(ControlReg,0x10);                               /* 17520��ʼֵ����              */
}

/*********************************************************************************************************
** Function name:       FM175X_HardReset
** Descriptions:        FM175xxӲ����λ
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
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
** Descriptions:        Ӳ���͹��Ĳ���
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char FM175X_SoftPowerdown(void)
{
    if(Read_Reg(CommandReg)&0x10) {
       Clear_BitMask(CommandReg,0x10);                                 /* �˳��͹���ģʽ               */
       return FALSE;
    }
    else
        Set_BitMask(CommandReg,0x10);                                       /* ����͹���ģʽ               */
    return TRUE;
}

/*********************************************************************************************************
** Function name:       FM175X_HardPowerdown
** Descriptions:        Ӳ���͹��Ĳ���
** input parameters:    N/A       
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char FM175X_HardPowerdown(void)
{    
    //NPD=~NPD;
    //if(NPD==1)                                                          /* ����͹���ģʽ               */
    return TRUE;                                
//    else
        //return FALSE;                                                     /* �˳��͹���ģʽ               */
}

/*********************************************************************************************************
** Function name:       Read_Ext_Reg
** Descriptions:        ��ȡ��չ�Ĵ���
** input parameters:    reg_add���Ĵ�����ַ��         
** output parameters:   reg_value���Ĵ�����ֵ
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Read_Ext_Reg(unsigned char reg_add)
{
     Write_Reg(0x0F,0x80+reg_add);
     return Read_Reg(0x0F);
}


/*********************************************************************************************************
** Function name:       GetReg_Ext(uint8_t  ExtRegAddr,uint8_t * ExtRegData)
** Descriptions:        ��ȡ��չ�Ĵ���ֵ
** input parameters:    ExtRegAddr:��չ�Ĵ�����ַ   
** output parameters:   ExtRegData:��ȡ��ֵ
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t  GetReg_Ext(uint8_t  ExtRegAddr,uint8_t * ExtRegData)
{
    uint8_t  res;
    uint8_t  addr,regdata;

    addr = JREG_EXT_REG_ENTRANCE;                                       /* ��չ�Ĵ���0x0f��ַ           */
    regdata = JBIT_EXT_REG_RD_ADDR + ExtRegAddr;                    /* JBIT_EXT_REG_RD_DATAд�������ַ */
                                                                        /* ExtRegAddr ��չ�Ĵ�����ַ    */
    res = spi_SetReg(addr,regdata);                                     /* д����չ�Ĵ�����ַ           */
    if (res == FALSE) 
        return FALSE;

    addr = JREG_EXT_REG_ENTRANCE;                                       /* ��չ�Ĵ���0x0f��ַ           */
    res = spi_GetReg2(addr,&regdata);                                   /* ������չ�Ĵ�������           */
    if (res == FALSE) 
        return FALSE;
    *ExtRegData = regdata;

    return TRUE;    
}

/*********************************************************************************************************
** Function name:       Write_Ext_Reg
** Descriptions:        д����չ�Ĵ���
** input parameters:    reg_add���Ĵ�����ַ��
**                      reg_value���Ĵ�����ֵ
** output parameters:   
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char Write_Ext_Reg(unsigned char reg_add,unsigned char reg_value)
{
    Write_Reg(0x0F,0x40+reg_add);
    return (Write_Reg(0x0F,0xC0+reg_value));
}




/*********************************************************************************************************
** Function name:       SetReg_Ext(uint8_t  ExtRegAddr,uint8_t * ExtRegData)
** Descriptions:        д��չ�Ĵ���
** input parameters:    ExtRegAddr:��չ�Ĵ�����ַ   
** output parameters:   ExtRegData:Ҫд���ֵ
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t  SetReg_Ext(uint8_t  ExtRegAddr,uint8_t  ExtRegData)
{
    uint8_t  res;
    uint8_t  addr,regdata;

    addr = JREG_EXT_REG_ENTRANCE;                                       /* ��չ�Ĵ�����ڵ�ַ0x0f       */
    regdata = JBIT_EXT_REG_WR_ADDR + ExtRegAddr;                        /* д����չ�Ĵ�����ַ           */
    res = spi_SetReg(addr,regdata);
    if (res == FALSE) 
        return FALSE;

    addr = JREG_EXT_REG_ENTRANCE;
    regdata = JBIT_EXT_REG_WR_DATA + ExtRegData;                        /* ������չ�Ĵ�������           */
    res = spi_SetReg(addr,regdata);
    if (res == FALSE) 
        return FALSE;
        
    return TRUE;    
}


/*********************************************************************************************************
** Function name:       modifyReg_Ext(uint8_t  ExtRegAddr,uint8_t * mask,uint8_t  set)
** Descriptions:        �Ĵ���λ����
** input parameters:    ExtRegAddr:Ŀ��Ĵ�����ַ   mask:Ҫ�ı��λ  
**                      set:  0:��־��λ����   ����:��־��λ����
** output parameters:  
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
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
** Descriptions:        TypeA_Request��ƬѰ��    
** input parameters:    N/A
** output parameters:   pTagType[0],pTagType[1] =ATQA 
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char TypeA_Request(unsigned char *pTagType)
{
    unsigned char  result,send_buff[1],rece_buff[2];
    unsigned int  rece_bitlen;  
    Clear_BitMask(TxModeReg,0x80);                                      /* �ر�TX CRC                   */
    Clear_BitMask(RxModeReg,0x80);                                      /* �ر�RX CRC                   */
    Set_BitMask(RxModeReg, 0x08);                                       /* �ر�λ����                   */
    Clear_BitMask(Status2Reg,0x08);                                     /* ������ܱ�־��ʹ������ͨ��   */
    Write_Reg(BitFramingReg,0x07);                                      /* ���һ�ֽڷ���7λ            */
    send_buff[0] = 0x26;                                                /* Ѱ������ 0x26                */                  
    Pcd_SetTimer(1);
    Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,1,rece_buff,&rece_bitlen);
    if ((result == TRUE) && (rece_bitlen == 2*8)) {                     /* �������յ�2�ֽڷ�����Ϣ      */
        *pTagType     = rece_buff[0];
        *(pTagType+1) = rece_buff[1];
    }
    return result;
}

/*********************************************************************************************************
** Function name:       TypeA_WakeUp        
** Descriptions:        ��Ƭ����
** input parameters:    N/A
** output parameters:   pTagType[0],pTagType[1] =ATQA 
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char TypeA_WakeUp(unsigned char *pTagType)
{
    unsigned char   result,send_buff[1],rece_buff[2];
    unsigned int   rece_bitlen;  
    Clear_BitMask(TxModeReg,0x80);                                      /* �ر�TX CRC                   */
    Clear_BitMask(RxModeReg,0x80);                                      /* �ر�RX CRC                   */
    Set_BitMask(RxModeReg, 0x08);                                       /* ������С��4bit������         */
    Clear_BitMask(Status2Reg,0x08);                                     /* ������ܱ�־��ʹ������ͨ��   */
    Write_Reg(BitFramingReg,0x07);                                      /* ���һ�ֽڷ���7λ            */
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
** Descriptions:        �ú���ʵ�ֱ��濨Ƭ�յ������к� 
** input parameters:    Bytes: ������ͻ���ֽ�    
**                      Bits: ������ͻ��λ    
**                      length: �Ӆ�����UID���ݳ��� 
**                      buff:��������UID��Ϣ
** output parameters:   picc_uid����Ҫ�����UID��Ϣ
** Returned value:      void
*********************************************************************************************************/
void TypeA_Save_UID(unsigned char Bytes,unsigned char Bits,unsigned char length,unsigned char *buff,unsigned char *picc_uid)
{
    unsigned char i;
    unsigned char temp1;
    unsigned char temp2;
      
    temp2=buff[0];                                                      /* ��һ�ν��յ��ĵ�һ����ЧUID  */
    temp1=picc_uid[Bytes-1];                                            /* ǰһ�ν��յ������һ����ЧUID*/
    switch (Bits) {                                                     /* ����ǰһ�εĳ�ͻλ��         */
                                                                        /* ������κϲ�temp1��tmep2     */
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
        picc_uid[Bytes-1]=temp1 | temp2;                                /* ���ǰ��������               */
        for(i=1;i<length;i++) {
            picc_uid[Bytes-1+i]=buff[i];                                /* �������ֽڶ�������UID��      */
    }

}

/*********************************************************************************************************
** Function name:       Set_BitFraming        
** Descriptions:        �ú���ʵ�ֶ��յ��Ŀ�Ƭ�����кŵ��ж�
** input parameters:    bytes: ��֪��UID�ֽ���  
**                      bits: �������֪UIDbits        
**                      length: �Ӆ�����UID���ݳ��� 
** output parameters:   NVB:����ͻ����
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
** Descriptions:        ��Ƭ����ͻ    
** input parameters:    selcode����Ƭѡ����� 0x93��0x95��0x97    
**                      picc_uid����ƬUID��
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char TypeA_Anticollision(unsigned char selcode,unsigned char *picc_uid)
{
    unsigned char result,i;
    unsigned char send_buff[10];
    unsigned char rece_buff[10];
    unsigned int  rece_bitlen,bitCnt;                                   /* ����ʹ�� volatile            */
    unsigned char nBytes,Bits,Pre_nBytes,Pre_Bits;
    nBytes = 0;                                                         /* ��ǰ����ͻָ������ֽ���     */
    Bits = 0;                                                           /* ��ǰ����ͻָ�����λ��       */
    Pre_nBytes = 0;                                                     /* ����ͻָ���Ѿ������ֽ���     */
    Pre_Bits = 0;                                                       /* ����ͻָ���Ѿ�����λ��       */
    rece_bitlen = 0;                                                    /* ָ��ִ�з���λ��             */
    bitCnt=0;
    Clear_BitMask(TxModeReg,0x80);                                      /* �ر�TX CRC                   */
    Clear_BitMask(RxModeReg,0x80);                                      /* �ر�RX CRC                   */
    Clear_BitMask(Status2Reg,0x08);                                     /* �����֤��־��ʹ������ͨ��   */
    Write_Reg(BitFramingReg,0x00);                                      /* ���һ�ֽڷ���8λ            */
    Clear_BitMask(CollReg,0x80);                                        /* �������ͻλ                 */
  
    send_buff[0] = selcode;                                             /* ����ͻ����                   */
    send_buff[1] = 0x20;                                                /* NVB ��ͻλ                   */
    Pcd_SetTimer(5);
    result = Pcd_Comm(Transceive,send_buff,2,rece_buff,&rece_bitlen);
    if(result == TRUE) {
        for (i=0; i<5; i++) { 
            *(picc_uid+i)  = rece_buff[i];           
        }
    }
    bitCnt = rece_bitlen;                                               /* ���յ������ݳ���             */
        
    if(result==Anticollision){                                          /* ������ͻ                     */
        nBytes = bitCnt/8;                                              /* ���յ����ֽ���               */
        Bits  =  bitCnt%8;                                              /* ���յ���λ��                 */
        if(bitCnt%8) nBytes++;                                          /* �����һ�η��ص������ֽ���   */
            memcpy(picc_uid,rece_buff,nBytes);                          /* ������յ��Ĳ���UID          */
        Pre_nBytes += nBytes;                                           /* ������һ����Ч�ֽ���         */
        Pre_Bits = Bits;                                                /* ������һ����Чλ             */                 

        while(result==Anticollision){                               /* ����ͻ���̣�ֻҪ�г�ͻ��һֱ���� */
            send_buff[0] = selcode;                                     /* ѡ����                       */
            TypeA_Set_BitFraming(Pre_nBytes-1,Pre_Bits,&send_buff[1]);  /* ����NVB����                  */
            memcpy(&send_buff[2],picc_uid,nBytes);                      /* ����Ч�ֽڿ���������buff     */
                                                      /* ��������ֽڷ��͵�λ���ͽ��յ����ݴ�ŵ���ʼλ */

        Pcd_SetTimer(5);
        Clear_FIFO();
        result = Pcd_Comm(Transceive,send_buff,2+nBytes,rece_buff,&rece_bitlen);    
        nBytes = rece_bitlen/8;
        Bits   = rece_bitlen%8;   
        if(Bits)
            nBytes+=1;
        TypeA_Save_UID(Pre_nBytes,Pre_Bits,nBytes,rece_buff,picc_uid);
        
        bitCnt += rece_bitlen;
        Pre_nBytes =  bitCnt/8;                                         /* ������һ����Ч�ֽ���         */
        Pre_Bits = bitCnt%8;
        if(bitCnt%8) Pre_nBytes++; 
        }
    }
    if(picc_uid[4] != (picc_uid[0]^picc_uid[1]^picc_uid[2]^picc_uid[3])) /* ���UIDУ��                 */
        result = FALSE;    
    return result;
}

/*********************************************************************************************************
** Function name:       TypeA_Select        
** Descriptions:        ѡ��Ƭ
** input parameters:    selcode����Ƭѡ����� 0x93��0x95��0x97    
**                      pSnr����ƬUID��
**                      pSak����Ƭѡ��Ӧ��
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char TypeA_Select(unsigned char selcode,unsigned char *pSnr,unsigned char *pSak)
{
    unsigned char   result,i,send_buff[7],rece_buff[5];
    unsigned int   rece_bitlen;
    Write_Reg(BitFramingReg,0x00);
    Set_BitMask(TxModeReg,0x80);                                        /* ��TX CRC                   */
    Set_BitMask(RxModeReg,0x80);                                        /* �򿪽���RX ��CRCУ��         */
    Clear_BitMask(Status2Reg,0x08);                                     /* �����֤��־λ               */
    
    send_buff[0] = selcode;                                             /* select����                   */
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
** Descriptions:        ��Ƭ˯��
** input parameters:    AnticollisionFlag ������֤��־ 
**                      AnticollisionFlag = 0 û����֤��ʹ������ͨ�ţ���Ҫ�����֤��־
**                      AnticollisionFlag = 1 ������֤����ͨ����ʹ������ͨ�ţ�����Ҫ�����֤��־
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char TypeA_Halt(unsigned char AnticollisionFlag)
{
    unsigned char   result,send_buff[2],rece_buff[1];
    unsigned int   rece_bitlen;
    send_buff[0] = 0x50;
    send_buff[1] = 0x00;

    Write_Reg(BitFramingReg,0x00);                                      /* �����һ�ֽ�8λ            */
    Set_BitMask(TxModeReg,0x80);                                        /* ��TX CRC                   */
    Set_BitMask(RxModeReg,0x80);                                        /* ��RX CRC                   */
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
** Descriptions:        ����ʵ��Ѱ������ͻ��ѡ��
** input parameters:    pTagType: ��Ƭ���� ATQA
**                      pSnr: ��ƬUID
**                      pSak: ��ƬӦ������ SAK
** output parameters:   N/A
** Returned value:      TRUE�������ɹ� ERROR������ʧ��    
*********************************************************************************************************/
unsigned char TypeA_CardActive(unsigned char *pTagType,unsigned char *pSnr,unsigned char *pSak)
{
    unsigned char   result;
    result=TypeA_Request(pTagType);                                     /* Ѱ�� Standard                */
        if (result==FALSE) {
        return FALSE;
    }

    if( (pTagType[0]&0xC0) == 0x00 ) {                                  /* M1��,ID��ֻ��4λ             */
        result=TypeA_Anticollision(0x93,pSnr);
        if (result==FALSE) {
            return FALSE;
        }
        result=TypeA_Select(0x93,pSnr,pSak);                            /* ѡ��UID                      */
        if (result == FALSE) {
            return FALSE;
        }    
    }
        
    if( (pTagType[0]&0xC0) == 0x40 )  {                                 /* ID����7λ                    */
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
		
    if( (pTagType[0]&0xC0) == 0x80 )  {                                 /* ID����10λ                   */
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
    
    regdata = 0x15;         //WaterLevel���յ�һ������ʱ�����ж�
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
** Descriptions:        ���ü��������
** input parameters:    loat Sensitive��������
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t   LpcdSet_DetectSensitive(float Sensitive)
{
    uint8_t  ret,Threshold;
    Threshold =ADCResultCenter*Sensitive; 
    /* ΢����ֵ */
    /****LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;*/
    /****LpcdThreshold_L= ADCResultCenter - ADCResultThreshold;*/
    LpcdThreshold_H = ADCResultCenter + Threshold;
    LpcdThreshold_L = ADCResultCenter - Threshold;
    /*
    ** LpcdThreshold_L1�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_L,(LpcdThreshold_L& 0x3F));
    IF_ERR_THEN_RETURN;
    /*
    ** LpcdThreshold_L2�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_H,(LpcdThreshold_L>>6));
    IF_ERR_THEN_RETURN;
    /*
    ** LpcdThreshold_H1�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_L,(LpcdThreshold_H& 0x3F));
    IF_ERR_THEN_RETURN;
    /*
    ** LpcdThreshold_H2�Ĵ���
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
** Descriptions:        LPCD������ʼ��
** input parameters:    
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/

uint8_t  LpcdParamInit()
{
    LpcdBiasCurrent = LPCD_BIAS_CURRENT ;                               /* ����Ƭ����ָ�����           */
                                                                        /* ��config�ļ����趨           */
    LpcdGainReduce = 0x3;                                               /* 1x                           */
    LpcdGainAmplify = 0x0;                                              /* 1x                           */
    LpcdADCRefernce = 0;
    Timer1Cfg = TIMER1_CFG;                                             /* LPCD������߽׶�ʱ������     */
    Timer2Cfg = TIMER2_CFG;                                             /* LPCD׼����⿨Ƭʱ������     */
    Timer3Cfg = TIMER3_CFG;                                             /* LPCD���׶�ʱ������         */

    if (Timer3Cfg > 0xF)  {                                             /* Timer3Cfg�õ�5bit  16��Ƶ    */
        T3ClkDivK = 2;                                                  /* 16��Ƶ                       */
        ADCResultFullScale =  ((Timer3Cfg - 1)<<3);
        ADCResultCenter = (ADCResultFullScale >>1);
        ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);
    }
    else if(Timer3Cfg > 0x7) {                                          /* Timer3Cfg�õ�4bit  8��Ƶ     */
        T3ClkDivK = 1;                                                  /* 8��Ƶ                        */
        ADCResultFullScale =  ((Timer3Cfg - 1)<<4);                     /* 160                          */
        ADCResultCenter = (ADCResultFullScale >>1);                     /* 80                           */
        ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);  /* 10                       */
    }
    else  {
        T3ClkDivK = 0;                                                   /* 4��Ƶ                        */
        ADCResultFullScale =  ((Timer3Cfg - 1)<<5);
        ADCResultCenter = (ADCResultFullScale >>1);
        ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);
    }

    LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;               /* �ߴ����ż� 80+10=90          */
    LpcdThreshold_L= ADCResultCenter - ADCResultThreshold;               /* �ʹ����ż� 80-10=70          */

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
** Descriptions:        ISO14443AЭ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
bool TyteA_Test (void)
{
    uint8_t statues = FALSE;
    uint8_t num=0;
    uint8_t picc_atqa[2];                                               /* ����Ѱ�����ؿ�Ƭ������Ϣ     */
    static uint8_t picc_uid[15];                                        /* ���濨ƬUID��Ϣ              */
    uint8_t picc_sak[3];                                                /* ���濨ƬӦ����Ϣ             */
    FM175X_SoftReset( );                                                /* FM175xx�����λ              */
    Set_Rf( 3 );                                                        /* ��˫����                   */
    Pcd_ConfigISOType( 0 );                                             /* ISO14443�Ĵ�����ʼ��         */

        statues = TypeA_CardActive( picc_atqa,picc_uid,picc_sak );      /* ���Ƭ                     */
        if ( statues == TRUE ) {
            num = 0;
            TypeA_Halt(0);                                              /* ˯�߿�Ƭ                     */     
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
** Descriptions:        ��ȡLPCD����
** input parameters:    uchar *ADCResult
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t  ReadLpcdADCResult(uint8_t  *ADCResult)
{
    uint8_t  ExtRegData;
    uint8_t  ret;
    uint8_t  ADCResultTemp=0;

    *ADCResult = 0;
    
    ret = GetReg_Ext(JREG_LPCD_ADC_RESULT_H,&ExtRegData);               /* ��ȡ�����Ϣ                 */
    IF_ERR_THEN_RETURN;
    ADCResultTemp = (ExtRegData & 0x3) << 6;
    
    ret = GetReg_Ext(JREG_LPCD_ADC_RESULT_L,&ExtRegData);
    IF_ERR_THEN_RETURN;
    
    ADCResultTemp += (ExtRegData&0x3F) ;
    *ADCResult = ADCResultTemp;

    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN); /* ��λLPCD�Ĵ���               */
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN); /* ��λ�ſ�LPCD�Ĵ���           */
    IF_ERR_THEN_RETURN;
        
    LOGA("ADCResultTemp = %d \r\n",ADCResultTemp);        
    return TRUE;
}


/*********************************************************************************************************
** Function name:       LpcdRegisterInit()
** Descriptions:        LPCD�Ĵ�����ʼ��
** input parameters:    
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t  LpcdRegisterInit(void)
{
    uint8_t  ret;
    uint8_t  regdata;
    uint8_t  read;
    uint8_t  write;
    
    regdata = COMMIEN_DEF;                                              /* �ж����ã������ж�����IRQ���*/
    ret = spi_SetReg(ComIEnReg,regdata);                                /* �ж�IRQ����͵�ƽ            */
    IF_ERR_THEN_RETURN;


    read=spi_GetReg(ComIEnReg) ;
 

    LOGA("ComIEnReg = %d \r\n",read);     


    
    /* ����IRQ������Ϊ��׼CMOS���  */
    ret = spi_SetReg(DivIEnReg,regdata);                                /* STatus1Reg��IRQλ�෴        */


    
    IF_ERR_THEN_RETURN;       


    read=spi_GetReg(DivIEnReg);    
 

    LOGA("DivIEnReg = %d \r\n",read);     

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read); 

    GetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,&read);
    LOGA("JREG_LPCD_AUTO_WUP_CFG = %d \r\n",read);        
    
    
    
    /*
    ** LpcdCtrl1�Ĵ���
    */ 
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN); /* ��λLPCD�Ĵ���               */
    IF_ERR_THEN_RETURN;

   write=JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        





    
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN); /* ��λ�ſ�LPCD�Ĵ���           */
    IF_ERR_THEN_RETURN;


   write=JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN;
    LOGA("write = %d \r\n",write);        


    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        
    
    
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_EN);   /* ʹ��LPCD����                 */
    IF_ERR_THEN_RETURN;


   write=JBIT_BIT_CTRL_SET+JBIT_LPCD_EN;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        
    
    
    ret = SetReg_Ext(JREG_LPCD_CTRL1,(LPCD_IE<<5)+JBIT_LPCD_IE);        /* ����LPCD�жϼĴ���״̬       */
    IF_ERR_THEN_RETURN;   


   write=(LPCD_IE<<5)+JBIT_LPCD_IE;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        
    /* ��ӳ��IRQ����                */
    
                                                                        /* ���ý���������             */
    ret = SetReg_Ext(JREG_LPCD_CTRL1,(LPCD_IE<<5)+JBIT_LPCD_CMP_1);     /* һ�μ�⵽����Ч             */
    //ret = SetReg_Ext(JREG_LPCD_CTRL1,(LPCD_DS<<5)+JBIT_LPCD_CMP_3);   /* 3�μ�⵽����Ч              */
    IF_ERR_THEN_RETURN;


   write=(LPCD_IE<<5)+JBIT_LPCD_CMP_1;
    LOGA("write = %d \r\n",write);        
    
    GetReg_Ext(JREG_LPCD_CTRL1,&read);
    LOGA("JREG_LPCD_CTRL1 = %d \r\n",read);        

    
    /*
    ** LpcdCtrl2�Ĵ���
    */        
    ret = SetReg_Ext(JREG_LPCD_CTRL2,((LPCD_TX2RFEN<<4)+(LPCD_CWN<<3)+LPCD_CWP)); /* P������������0��7  */
    IF_ERR_THEN_RETURN;       



   write=((LPCD_TX2RFEN<<4)+(LPCD_CWN<<3)+LPCD_CWP);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL2,&read);
    LOGA("JREG_LPCD_CTRL2 = %d \r\n",read);        
    /* һ����������ѡ��3          */
        
    /*
    ** LpcdCtrl3�Ĵ���
    */   
    ret = SetReg_Ext(JREG_LPCD_CTRL3,LPCD_MODE<<3);                     /* û����ʵ������               */
    IF_ERR_THEN_RETURN;


   write=LPCD_MODE<<3;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_CTRL3,&read);
    LOGA("JREG_LPCD_CTRL3 = %d \r\n",read);        

    /*
    ** Timer1Cfg�Ĵ���
    */ 
    ret = SetReg_Ext(JREG_LPCD_T1CFG,(T3ClkDivK<<4)+Timer1Cfg);         /* ����LPCD˯��ʱ��             */
    IF_ERR_THEN_RETURN;



   write=(T3ClkDivK<<4)+Timer1Cfg;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_T1CFG,&read);
    LOGA("JREG_LPCD_T1CFG = %d \r\n",read);        
 
    /*
    ** Timer2Cfg�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_T2CFG,Timer2Cfg);                        /* ����LPCD׼�����ʱ��         */ 
    IF_ERR_THEN_RETURN;


   write=Timer2Cfg;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_T2CFG,&read);
    LOGA("JREG_LPCD_T2CFG = %d \r\n",read);        
    
        
    /*
    ** Timer3Cfg�Ĵ���
    */ 
    ret = SetReg_Ext(JREG_LPCD_T3CFG,Timer3Cfg);                        /* ����LPCD��⿨Ƭʱ��         */
    IF_ERR_THEN_RETURN;


   write=Timer3Cfg;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_T3CFG,&read);
    LOGA("JREG_LPCD_T3CFG = %d \r\n",read);        

    /*
    ** VmidBdCfg�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_VMIDBD_CFG,VMID_BG_CFG);                 /* �������û��޸�               */
    IF_ERR_THEN_RETURN;



   write=VMID_BG_CFG;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_VMIDBD_CFG,&read);
    LOGA("JREG_LPCD_VMIDBD_CFG = %d \r\n",read);        
        
    /*
    ** AutoWupCfg�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,(AUTO_WUP_EN<<3)+AUTO_WUP_CFG);  /* �����Զ�����ʱ��        */
    IF_ERR_THEN_RETURN;



   write=(AUTO_WUP_EN<<3)+AUTO_WUP_CFG;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,&read);
    LOGA("JREG_LPCD_AUTO_WUP_CFG = %d \r\n",read);        
        
    /*
    ** LpcdThreshold_L1�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_L,(LpcdThreshold_L & 0x3F));  /* ���ÿ�����·�ֵ          */
    IF_ERR_THEN_RETURN;



   write=(LpcdThreshold_L & 0x3F);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MIN_L,&read);
    LOGA("JREG_LPCD_THRESHOLD_MIN_L = %d \r\n",read);        
        
    /*
    ** LpcdThreshold_L2�Ĵ���
    */ 
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MIN_H,(LpcdThreshold_L>>6));   
    IF_ERR_THEN_RETURN;



   write=(LpcdThreshold_L>>6);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MIN_H,&read);
    LOGA("JREG_LPCD_THRESHOLD_MIN_H = %d \r\n",read);        
        
    /*
    ** LpcdThreshold_H1�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_L,(LpcdThreshold_H& 0x3F));  /* ���ÿ�����Ϸ�ֵ           */
    IF_ERR_THEN_RETURN;


   write=(LpcdThreshold_H& 0x3F);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MAX_L,&read);
    LOGA("JREG_LPCD_THRESHOLD_MAX_L = %d \r\n",read);        
    
    /*
    ** LpcdThreshold_H2�Ĵ���
    */
    ret = SetReg_Ext(JREG_LPCD_THRESHOLD_MAX_H,(LpcdThreshold_H>>6));   
    IF_ERR_THEN_RETURN;


   write=(LpcdThreshold_H>>6);
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_THRESHOLD_MAX_H,&read);
    LOGA("JREG_LPCD_THRESHOLD_MAX_H = %d \r\n",read);        
    
    
    /*
    ** Auto_Wup_Cfg�Ĵ���
    */
    ret=SetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,(AUTO_WUP_EN<<3) + AUTO_WUP_CFG );  /* �ٴ������Զ�����ʱ��   */
    IF_ERR_THEN_RETURN;



   write=(AUTO_WUP_EN<<3) + AUTO_WUP_CFG ;
    LOGA("write = %d \r\n",write);        

    GetReg_Ext(JREG_LPCD_AUTO_WUP_CFG,&read);
    LOGA("JREG_LPCD_AUTO_WUP_CFG = %d \r\n",read);        
    

            LOG(" LpcdRegisterInit  OK.\r\n");             /* ��ʾLPCD��У�ɹ�             */
        
    return TRUE;
}



/*********************************************************************************************************
** Function name:       WaitForLpcdIrq()
** Descriptions:        �ȴ�LPCD�ж�
** input parameters:    IrqType
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
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
        //��ʱ�˳�
        delay_us(1000);                                                /* ��ʱ10ms                     */
        TimeOutCount++;
        if (TimeOutCount > IRQ_TIMEOUT)  
        {
            return FALSE;                                               /* 150ms ��ʱ�˳�               */
        }
    }
                LOG(" WaitForLpcdIrq  is true.\r\n");          /* LPCD��Уʧ��                 */
    return TRUE; 
}


/*********************************************************************************************************
** Function name:       CalibraReadADCResult()
** Descriptions:        ���̲���ȡLPCD����
** input parameters:    uchar *ADCResult
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t  CalibraReadADCResult(uint8_t  *ADCResult)
{
    //ʹ�ܵ���ģʽ
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

    ret = WaitForLpcdIrq(JBIT_CALIB_IRQ);                               /* �ȴ����̽����ж�             */
    //debug
    if (ret == 0)
    {
        ret =1;
    }
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN); /* �رյ���ģʽ           */
    IF_ERR_THEN_RETURN;
        
    ret = ReadLpcdADCResult(ADCResult);                                 /* ��ȡ������Ϣ                 */
    IF_ERR_THEN_RETURN;

            LOG(" CalibraReadADCResult  is true.\r\n");          /* LPCD��Уʧ��                 */
    
    return TRUE;
}

/*********************************************************************************************************
** Function name:       LpcdSet_ADCRefvoltage()
** Descriptions:        ���ú��ʵ�ADC�ο���ѹ
** input parameters:    uint8_t  *CalibraFlag, uint8_t  *ADCResult   ���ȼ���������ַ
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t   LpcdSet_ADCRefvoltage(uint8_t  *CalibraFlag, uint8_t  *ADCResult)
{
    uint8_t  ret;
    uint8_t  ADCResult_Pre;                                             /* ������Ϣ��ǰһ��ֵ           */
    
    //ɨ�������
    for(LpcdADCRefernce = ADC_REFERNCE_MIN;LpcdADCRefernce < ADC_REFERNCE_MAX;LpcdADCRefernce++)
    //ͨ����ѭ������ȷ�����յĲο���ѹ��ADC���ıȽ�ֵ
    {
            //���òο���ѹֵ
        ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
        IF_ERR_THEN_RETURN;
        ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
        IF_ERR_THEN_RETURN;
        //���ݷ�����Ϣ
        ADCResult_Pre = *ADCResult;
        //���̶�ȡ��ǰ������Ϣ
        ret = CalibraReadADCResult(ADCResult);
        IF_ERR_THEN_RETURN;
        /*
              ** �㷨һ
              */
        // ������ȿ�ʼ������ֵС����Ϊ��ʼ������ֵ��
        if (*ADCResult < ADCResultCenter) {
            //���̳ɹ�
            (*CalibraFlag) = TRUE;
            //��ǰһ�����̲ο���ѹ���ж��ĸ����ȸ��ӽ����ĵ�
            if((ADCResultCenter - *ADCResult) < (ADCResult_Pre-ADCResultCenter))
            {
                //ֱ���õ�ǰֵ��Ϊ���ĵ�
                ADCResultCenter = *ADCResult;
            }
            else
            {
                //ֱ���õ�ǰֵ��Ϊ���ĵ�
                ADCResultCenter = ADCResult_Pre;
                //�ο���ѹ����֮ǰ�Ĳο���ѹ
                LpcdADCRefernce--;
                //�������òο���ѹֵ
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
** Descriptions:        ���ú��ʵ�PGA����
** input parameters:    uint8_t  *CalibraFlag, uint8_t  *ADCResult   ���ȼ���������ַ
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t   LpcdSet_PGA(uint8_t  *GainCalibraFlag, uint8_t  *ADCResult)
{
    uint8_t  ret;
    //�ο���ѹֵ������С
    LpcdADCRefernce = ADC_REFERNCE_MIN;
    ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)>>1)+LpcdBiasCurrent&0x7);
    IF_ERR_THEN_RETURN;
    ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
    IF_ERR_THEN_RETURN;

    //���̶�ȡ��ǰ������Ϣ
    ret = CalibraReadADCResult(ADCResult);
    IF_ERR_THEN_RETURN;
    //ȱʡ���治��Ҫ����
    *GainCalibraFlag = TRUE;
   
    //�ж��Ƿ����̫խ�����̫խlpcd_gain˥��
    if  (*ADCResult < ADCResultCenter)
    {
            LOG("ADCResult<ADCResultCenter 00.\r\n");          /* LPCD��Уʧ��                 */
        
        //������Ҫ����
        *GainCalibraFlag = FALSE;
        //*GainCalibraFlag = LpcdSetPGA_GainReduce(ADCResult);   //����PGA����˥��
        while(1)
        {
            LOG("ADCResult<ADCResultCenter 01.\r\n");          /* LPCD��Уʧ��                 */
            
                //�����ǰ�Ѿ�����С���棬����ʧ��
            if (LpcdGainReduce == 0)
            {
                *GainCalibraFlag = FALSE;
                break;
            }
            //����˥��
            LpcdGainReduce --; 
            // uartPrintf("LpcdSet_PGA  LpcdGainReduce=%d !\r\n",LpcdGainReduce);
            //��������
            ret = SetReg_Ext(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));
            IF_ERR_THEN_RETURN;

            //���̶�ȡ��ǰ������Ϣ
            ret = CalibraReadADCResult(ADCResult);
            //uartPrintf("LpcdSet_PGA  ADCResult=%d !\r\n",*ADCResult);
            IF_ERR_THEN_RETURN;
            //���̳ɹ��������ĵ��Ƶ����ĵ��Ҳ�
            if (*ADCResult >ADCResultCenter)
            {
                *GainCalibraFlag = TRUE;
                break;
            }
         }    
    }
    else
    {
            LOG("LpcdADCRefernce = ADC_REFERNCE_MAX.\r\n");          /* LPCD��Уʧ��                 */
        
        //�ο���ѹֵ�������
        LpcdADCRefernce = ADC_REFERNCE_MAX;
        ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
        IF_ERR_THEN_RETURN;
        ret = SetReg_Ext(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);
        IF_ERR_THEN_RETURN;

        //���̶�ȡ��ǰ������Ϣ
        ret = CalibraReadADCResult(ADCResult);
        //uartPrintf("LpcdSet_PGA  ADCResult=%d !\r\n",*ADCResult);
        IF_ERR_THEN_RETURN;

        //���̳ɹ���־��ʼ��
        *GainCalibraFlag = TRUE;
        
        //�ж��Ƿ����̫С�����̫Сlpcd_gain�Ŵ�
        if (*ADCResult > ADCResultCenter)
        {
            LOG("ADCResult>ADCResultCenter 00.\r\n");          /* LPCD��Уʧ��                 */
            
            //������Ҫ����
            *GainCalibraFlag = FALSE;
            while(1)
            {
            LOG("ADCResult>ADCResultCenter 01.\r\n");          /* LPCD��Уʧ��                 */
                
                //�����ǰ�Ѿ���������棬����ʧ��
                if (LpcdGainAmplify == 0x7)
                {
                    *GainCalibraFlag = FALSE;
                    break;
                }
                else//�����Ŵ�
                {
                    LpcdGainAmplify++;  
                }
                        //��������
                ret = SetReg_Ext(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));
                IF_ERR_THEN_RETURN;

                //���̶�ȡ��ǰ������Ϣ
                  ret = CalibraReadADCResult(ADCResult);
                IF_ERR_THEN_RETURN;
                
                //���̳ɹ��������ĵ��Ƶ����ĵ����
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
** Descriptions:        ��ʼ������
** input parameters:    uint8_t  *CalibraFlag ���̱�־���Ƿ���Ҫ����
** output parameters:   
** Returned value:      uint8_t   TRUE����ȡ�ɹ�   FALSE:ʧ��
*********************************************************************************************************/
uint8_t   LpcdInitCalibra(uint8_t  *CalibraFlag)
{
    uint8_t  ret;
    uint8_t  ADCResult;                                                 /* LPCD������Ϣ                 */
    uint8_t  GainCalibraFlag;                                           /* ������̽��                 */
                                                                        /* ��������                     */
    ret = SetReg_Ext(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));
    IF_ERR_THEN_RETURN;
                                                                        /* ����ƫ�õ����Ͳο���ѹ       */
    ret = SetReg_Ext(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)>>1)+LpcdBiasCurrent&0x7);
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN);            /* ������Уģʽ��Vmindʹ��      */
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_T1CFG,(T3ClkDivK<<4)+Timer1Cfg);         /* Timer1Cfg����                */
    IF_ERR_THEN_RETURN;


    ret = SetReg_Ext(JREG_LPCD_T2CFG,Timer2Cfg);                        /* Timer2Cfg����                */
    IF_ERR_THEN_RETURN;

    ret = SetReg_Ext(JREG_LPCD_T3CFG,Timer3Cfg);                        /* Timer3Cfg����                */
    IF_ERR_THEN_RETURN;

            LOG("LpcdInitCalibra enter  LpcdSet_PGA.\r\n");
    
                                                                          
    ret = LpcdSet_PGA(&GainCalibraFlag,&ADCResult);                     /* PGA��������                  */
    IF_ERR_THEN_RETURN;                                              /* ��У����PGA�������˥�������趨 */



    if (GainCalibraFlag == FALSE)                                       /* ����������ʧ�ܣ���ʧ��     */
    {        
          (*CalibraFlag) = FALSE;
        return ADCResult;                                               /* ����ʧ�ܷ��ط���             */
    }
    //ɨ��ο���ѹֵ���ҵ����ʵĴ�Խ���ĵ������
    (*CalibraFlag) = FALSE;
    GainCalibraFlag = LpcdSet_ADCRefvoltage(CalibraFlag,&ADCResult);    /* ���ú��ʵ�ADC�ο���ѹ        */
    ret = LpcdSet_DetectSensitive(LPCD_DetectSensitive);                /* ��У���̼���������趨       */
    IF_ERR_THEN_RETURN;
    
    if (GainCalibraFlag == FALSE)
    {        
        (*CalibraFlag) = FALSE;
        ret = ModifyReg_Ext(JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN,0);   /* ���� ���̽����ر�CalibVmidEn */
        IF_ERR_THEN_RETURN;
        return ADCResult;                                               /* ����ʧ�ܷ��ط���             */
    }
    
    ret = ModifyReg_Ext(JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN,0);       /* ���̽����ر�CalibVmidEn      */
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
                LpcdParamInit();                                                /* LPCD������ʼ��               */
                LpcdRegisterInit();                                             /* LPCD�Ĵ�����ʼ��             */
                //LpcdAuxSelect(OFF);                                           /* ����AUX�۲�ͨ��              */
                LpcdInitCalibra(&CalibraFlag);                                  /* LPCD��ʼ������               */


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

