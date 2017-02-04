/* Includes ------------------------------------------------------------------*/
#define COMPID "gps"

#include "main.h"
#include "gps.h"
#include "util.h"
#include "spi_flash.h"

extern RTC_HandleTypeDef hrtc;

extern UART_HandleTypeDef huart1;

static GPS_Data_t GPSData;
extern CRC_HandleTypeDef hcrc;
extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern SPI_HandleTypeDef hspi1;

Bike_Record_t gBikeRecord;

extern char  GPRS_SendMYNETSRV[];

bool Flash_Bike_Recorder_Response_Verification_Write(uint16_t val,uint32_t WriteAddr)
{
    
    uint16_t val_temp=val;
    if(!flash_spi_bufferWrite_try((uint8_t *)&val_temp, WriteAddr,sizeof(uint16_t),flash_spi_bufferWrite_try_maxtimes))
    {
        LOG("Flash_Bike_Recorder_Response_Verification_Write is failed!");
        
        return FALSE;
    }
    return TRUE;
}


bool Flash_Bike_Recorder_HeaderTailer_Read(uint8_t HeaderOrTailer)
{
    uint16_t crc16;
    uint8_t trytimes=3;
    pNoBikeTaskData_t app=getApp();  
    uint8_t *dat;
    uint8_t len;
    uint32_t add;
    switch(HeaderOrTailer)
    {
        case tailer:
            dat=(uint8_t *)&app->Flash_Bike_Recorder_Tailer;
            len=sizeof(Bike_Record_tailer_t);
            add=FLASH_Bike_Tailer_START;
        break;
        case header:
            dat=(uint8_t *)&app->Flash_Bike_Recorder_Header;
            len=sizeof(Bike_Record_header_t);
            add=FLASH_Bike_Header_START;
        break;
    }
    while(trytimes--)
    {
        flash_spi_bufferRead(dat, add,len);
        crc16=get_crc16(dat,len-2);
        if(isEqualcrc16(crc16,dat[len-2],dat[len-1]))
        {
             return TRUE;
        }
        delay_ms(250);
    }

    dat[0]=0;
    dat[1]=0;

    LOG("Flash_Bike_Recorder_Header_Read failed!");  
    return FALSE;
}

bool Flash_Bike_Recoder_Init(void)
{
    uint32_t val,addr;
    uint8_t temp[4]; 
    flash_spi_sectorErase(FLASH_Bike_SZStauts_START);
    flash_spi_sectorErase(FLASH_Bike_Validation_START);
    flash_spi_sectorErase(FLASH_Bike_Header_START);
    flash_spi_sectorErase(FLASH_Bike_Tailer_START);

    val=0xdeadbeef;
    temp[0]=val&0xff;
    temp[1]=val>>8&0xff;
    temp[2]=val>>16&0xff;
    temp[3]=val>>24&0xff;

    addr=FLASH_Bike_Validation_START;
    if(!flash_spi_bufferWrite_try(temp, FLASH_Bike_Validation_START,FLASH_Bike_Validation_LEN,flash_spi_bufferWrite_try_maxtimes))
    {
        LOG("Flash_Bike_Recoder_Init write is failed!");
        return FALSE;
    }
    addr=FLASH_Bike_Recorder_Data_START;
    for(uint8_t i=0;i<Available_Block_Number;i++)
    {
        flash_spi_block64K_Erase(addr);
        addr +=FLASH_Block_Size;
    }
    delay_ms(500);
    if(!Flash_Bike_Recorder_HeaderTailer_Write(0,tailer))
        
        LOG("Flash_Bike_Recorder_Tailer_Write is failed!");
    if(!Flash_Bike_Recorder_HeaderTailer_Write(0,header))
        LOG("Flash_Bike_Recorder_Header_Writestart is failed!");
    return TRUE;
}




/*******************************************************************************
* Function Name  : flash_spi_test
* Description    : test spi interface nand flash
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool FlashInit(void)
{
    pNoBikeTaskData_t app=getApp();  
    
      /* Perform a write in the Flash followed by a read of the written data */
    /* Erase SPI FLASH Sector to write on */

    uint32_t temp0;
    uint8_t temp[4];   
    if(!flash_spi_readID())
    LOG("flash_spi_readID failed!");

    temp0=FLASH_Bike_Validation_START;

    flash_spi_bufferRead(temp, temp0,FLASH_Bike_Validation_LEN);
    temp0=temp[0]+(temp[1]<<8)+(temp[2]<<16)+(temp[3]<<24);
    if(temp0!=0xdeadbeef)/*need to erase */
    {
        Flash_Bike_Recoder_Init();
        LOG("0xdeadbeef!++++++++++++++++++++++++++++");
        LOGA("sFlash_Bike_Recoder_Init is:%08x",temp0);
        
    }
    LOGA("sFlash_Bike_Recoder_Init is:%08x",temp0);
    
    
    if((!Flash_Bike_Recorder_HeaderTailer_Read(tailer)||!Flash_Bike_Recorder_HeaderTailer_Read(header)))
    {
        LOG("Flash_Bike_Recorder_Header_Read_Flash_Bike_Recorder_Tailer_Read failed+++++++++++++!");
        Flash_Bike_Recoder_Init();
    }
    else
    {
                LOGA("app->Flash_Bike_Recorder_Header.header:0x%08x",app->Flash_Bike_Recorder_Header.header);
                LOGA("app->Flash_Bike_Recorder_Tailer.tailer:0x%08x",app->Flash_Bike_Recorder_Tailer.tailer);



    }
   return TRUE;
} 


bool Flash_Bike_Recorder_HeaderTailer_Write(uint16_t val,uint8_t HeaderOrTailer)
{
    pNoBikeTaskData_t app = getApp();
    uint32_t add;
    uint8_t *dat;

    uint8_t liu[4];
    
    switch(HeaderOrTailer)
    {
        case tailer:
            app->Flash_Bike_Recorder_Tailer.tailer=val;
            app->Flash_Bike_Recorder_Tailer.crc16=get_crc16((uint8_t *)&app->Flash_Bike_Recorder_Tailer,sizeof(Bike_Record_tailer_t)-2);
            dat=(uint8_t *)&app->Flash_Bike_Recorder_Tailer;

            add=FLASH_Bike_Tailer_START;
        break;
        case header:
            app->Flash_Bike_Recorder_Header.header=val;
            app->Flash_Bike_Recorder_Header.crc16=get_crc16((uint8_t *)&app->Flash_Bike_Recorder_Header,sizeof(Bike_Record_header_t)-2);
            dat=(uint8_t *)&app->Flash_Bike_Recorder_Header;

            add=FLASH_Bike_Header_START;
        break;
    }

                LOGA("HeaderOrTailer:0x%08x",HeaderOrTailer);
                LOGA("targetaddress:0x%08x",add);


    flash_spi_sectorErase(add);

            flash_spi_bufferRead(liu, add,4);
                LOGA("liu00:0x%08x",liu[0]);
                LOGA("liu00:0x%08x",liu[1]);
                LOGA("liu00:0x%08x",liu[2]);
                LOGA("liu00:0x%08x",liu[3]);



    
    if(!flash_spi_bufferWrite_try(dat, add,sizeof(Bike_Record_header_t),flash_spi_bufferWrite_try_maxtimes)) 
    return FALSE;
    return TRUE;
}

/*****************************************************************************
*Prototype: addRecord2Flash
*Description:addRecord2Flash
*Input: Bike_Record_t* record  
*Output: None
*Return Value: 
*Calls: 
*Called By: 
*****************************************************************************/
void addRecord2Flash(uint8_t *dat)
{
    pNoBikeTaskData_t app = getApp();
    uint32_t target_addr;
    uint16_t tailer_temp,header_temp;
    uint16_t per_sector_num;
    uint32_t start_add,target_add;
    target_addr=(uint32_t)FLASH_Bike_Recorder_Data_START + app->Flash_Bike_Recorder_Tailer.tailer* sizeof(Bike_Record_t);
    if(flash_spi_bufferWrite_try((uint8_t*)dat, target_addr,sizeof(Bike_Record_t),flash_spi_bufferWrite_try_maxtimes))
    {

        app->Flash_Bike_Recorder_Tailer.tailer=(app->Flash_Bike_Recorder_Tailer.tailer+1)%Bike_Recorder_Toltal_Number;

        tailer_temp=app->Flash_Bike_Recorder_Tailer.tailer;
        header_temp= app->Flash_Bike_Recorder_Header.header;
        per_sector_num=Bike_Recorder_PerSector_Number;
        LOGA("app->Flash_Bike_Recorder_Header.header:0x%08x",app->Flash_Bike_Recorder_Header.header);
        LOGA("app->Flash_Bike_Recorder_Tailer.tailer:0x%08x",app->Flash_Bike_Recorder_Tailer.tailer);
        
        if(tailer_temp == header_temp)
        {
            //app->SystemErrData.BITS.Bike_Recorder_Over_Flow_Err = TRUE;
        }

        Flash_Bike_Recorder_HeaderTailer_Write(tailer_temp,tailer);


        if(tailer_temp%per_sector_num==0)/*enter into the next sector*/
        {
            uint16_t sector=(uint16_t)(tailer_temp/per_sector_num); /*which setor will be erase*/   
            target_add=start_add+FLASH_Sector_Size*sector;
            LOGA("flash_spi_sectorErase:0x%08x",target_add);
            
            flash_spi_sectorErase(target_add);
            
            delay_ms(3);    
            LOG("enter into the next sector!");
        }
    }

    
}

void produce_record(Bike_Record_t* record,Bike_Action_t bike_action)
{
    pNoBikeTaskData_t app = getApp();

    static uint16_t seq_no=0;

    uint32_t utctime_int;

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    UTCTime_t UTCTime;
    uint16_t crc16;


    HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);  
    HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN); 


    HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);  
    HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN); 


//    LOGA("system time is:%04d/%02d/%02d %02d:%02d:%02d",
//    BCDTODEC(sDate.Year),BCDTODEC(sDate.Month), BCDTODEC(sDate.Date),
//    BCDTODEC(sTime.Hours), BCDTODEC(sTime.Minutes),BCDTODEC(sTime.Seconds));

    UTCTime.year=2000+sDate.Year;
    UTCTime.month=sDate.Month;
    UTCTime.date=sDate.Date;
    UTCTime.hours=sTime.Hours;
    UTCTime.minutes=sTime.Minutes;
    UTCTime.seconds=sTime.Seconds;

    utctime_int=convert_second(&UTCTime);
    record->Time[0]=utctime_int&0xff;
    record->Time[1]=utctime_int>>8&0xff;
    record->Time[2]=utctime_int>>16&0xff;
    record->Time[3]=utctime_int>>24&0xff;
    
    record->seq_no[0]=seq_no&0xff;
    record->seq_no[1]=seq_no>>8&0xff;
    seq_no++;


    
    record->bike_area_code[0]=app->GPRSConfigData.BikeArea[0];
    record->bike_area_code[1]=app->GPRSConfigData.BikeArea[1];
    record->bike_serial[0]=app->GPRSConfigData.BikeSerial[0];
    record->bike_serial[1]=app->GPRSConfigData.BikeSerial[1];
    record->bike_serial[2]=app->GPRSConfigData.BikeSerial[2];

    record->RentType=bike_action;
    

    app->GPRSCommHeadData.seq[0]=app->GPRSMsg_Seq&0xff;
    app->GPRSCommHeadData.seq[1]=app->GPRSMsg_Seq>>8;

//    app->GPRSRemoteRentData.Card[0]=0x80;
//    app->GPRSRemoteRentData.Card[1]=0xf0;
//    app->GPRSRemoteRentData.Card[2]=0xfa;
//    app->GPRSRemoteRentData.Card[3]=0x02;
    

    memcpy(record->Card_ID,(uint8_t *)&app->GPRSRemoteRentData,sizeof(app->GPRSRemoteRentData));

    memset(record->map,0xff,sizeof(record->map));
    memset((uint8_t *)&record->response,0xff,sizeof(record->response));
    
    crc16=get_crc16((uint8_t *)record,sizeof(Bike_Record_t)-sizeof(record->map)-sizeof(record->response)-2); 
    record->crc16_high=crc16>>8;
    record->crc16_low=crc16&0xff;
}
        

void GPSStart(void)
{

    pNoBikeTaskData_t app = getApp();
    GPS_GPIO_Init();
    GPS_PWR_ON();
    MX_USART1_UART_Init();
    GPSData.GPSSeqData=GPSSeq_DatParseInit;
    app->GpsIsConnected=FALSE;
}
bool GpsIsEnd(void)
{
    if(GPSData.GPSSeqData==GPSSeq_End)
    return TRUE;
    return FALSE;
}




void GPSMgrHandle(NoBikeTaskData_t *app)
{
    uint16_t header_pre;
    static uint16_t pointer_start,pointer_end;
    uint16_t len,p_len;
    uint8_t *dat_h,*dat_t;
    static uint16_t dat_len;
    UTCTime_t UTCTime;
    uint32_t utctime_int,utctime_int_gps;
    
    
    
    
    switch(GPSData.GPSSeqData)
    {
        case GPSSeq_Idle:
            GPSData.GPSSeqData=GPSSeq_End;
        break;

        case GPSSeq_DatParseInit:
            GPSData.GPSSeqData=GPSSeq_DatParse;
            pointer_start=GPSData.GpsReceiveData.header;
            app->Time_Delay.GPS_counts_1ms=0;
            
            
        break;
        case GPSSeq_DatParse:
            if(GPSData.GpsReceiveData.tailer!=GPSData.GpsReceiveData.header)
            {

                if(GPSData.GpsReceiveData.header==0)
                header_pre=Gps_Number_of_Receive-1;
                else
                header_pre=GPSData.GpsReceiveData.header-1;
                    
                
                if((GPSData.GpsReceiveData.dat[GPSData.GpsReceiveData.header]=='\n')&&(GPSData.GpsReceiveData.dat[header_pre]=='\r'))
                {
                        pointer_end = (GPSData.GpsReceiveData.header+1)%Gps_Number_of_Receive;
                        if(pointer_end>=pointer_start)
                        len=pointer_end-pointer_start;
                        else
                        len=Gps_Number_of_Receive- pointer_start + pointer_end;
                        
                        for(uint16_t i=0;i<len;i++)
                        {
                            GPSData.MsgParts[i]=GPSData.GpsReceiveData.dat[pointer_start];  
                            pointer_start=(pointer_start+1)%Gps_Number_of_Receive;
                        }
                                
                        if(strstr((const char *)GPSData.MsgParts,"$GPRMC")!=NULL)
                        {

//                        char *test="$GPRMC,055733.000,A,3119.2966,N,12042.3364,E,0.78,73.56,280916,,,A*53\r\n";

//                        len=strlen(test);
//                        memcpy(GPSData.MsgParts,test,strlen(test));

                            
                            pointer_start = pointer_end;
//                            GPSData.MsgParts[18]='A';
                            /*$GPRMC,000007.021,V,                 ,   ,                  ,  ,0.00,0.00,060180,,,N*46*/
                            /*$GPRMC,055733.000,A,3119.2966,N,12042.3364,E,0.78,73.56,280916,,,A*53*/

                            if(((ChartoHex(GPSData.MsgParts[len-4])<<4)|ChartoHex(GPSData.MsgParts[len-3]))==get_chk_bcc(&GPSData.MsgParts[1],len-6))
                                {
                                    char delims[] = ",";
                                    char *result = NULL;
                                    GPSGPRMC_UTC_t GPSGPRMC_UTCData;
                                    GPSGPRMC_LAT_t GPSGPRMC_LATData;
                                    GPSGPRMC_LON_t GPSGPRMC_LONData;
                                    GPSGPRMC_DATE_t GPSGPRMC_DATEData;
                                    RTC_TimeTypeDef sTime;
                                    RTC_DateTypeDef sDate;
                                    uint32_t temp=0,temp1=0;

                                    result = strtok(GPSData.MsgParts,delims);/*type*/
                                    if(result==NULL)
                                    goto Discard_CurrenDat;
                                    result = strtok( NULL, delims ); /*utc*/
                                    if(result==NULL||strlen(result)!=sizeof(GPSGPRMC_UTCData))
                                    goto Discard_CurrenDat;
                                    memcpy((uint8_t *)&GPSGPRMC_UTCData,result,sizeof(GPSGPRMC_UTCData));
                                    
                                    result = strtok( NULL, delims ); /*status*/
                                    if(result==NULL||strlen(result)!=1||*result!='A')
                                    goto Discard_CurrenDat;
                                    result = strtok( NULL, delims ); /*Lat*/
                                    if(result==NULL||strlen(result)!=sizeof(GPSGPRMC_LATData))
                                    goto Discard_CurrenDat;
                                    memcpy((uint8_t *)&GPSGPRMC_LATData,result,sizeof(GPSGPRMC_LATData));
                                    result = strtok( NULL, delims ); /*Lat_NS*/
                                    if(result==NULL||strlen(result)!=1)
                                    goto Discard_CurrenDat;

                                    if(*result=='N')
                                    {
                                        LOG("lat is N");
                                    }
                                    else if(*result=='S')
                                    {
                                        LOG("lat is S");
                                    }
                                    else
                                    {
                                        LOG("lat is valid");
                                    }

                                    result = strtok( NULL, delims ); /*Lon*/
                                    if(result==NULL||strlen(result)!=sizeof(GPSGPRMC_LONData))
                                    goto Discard_CurrenDat;
                                    memcpy((uint8_t *)&GPSGPRMC_LONData,result,sizeof(GPSGPRMC_LONData));


                                    result = strtok( NULL, delims ); /*Lon_EW*/
                                    if(result==NULL||strlen(result)!=1)
                                    goto Discard_CurrenDat;

                                    if(*result=='E')
                                    {
                                        LOG("lon is E");
                                    }
                                    else if(*result=='W')
                                    {
                                        LOG("lon is W");
                                    }
                                    else
                                    {
                                        LOG("lon is valid");
                                    }

                                    result = strtok( NULL, delims ); /*spd*/
                                    if(result==NULL)
                                    goto Discard_CurrenDat;
                                    result = strtok( NULL, delims ); /*cog*/
                                    if(result==NULL)
                                    goto Discard_CurrenDat;
                                    result = strtok( NULL, delims ); /*date*/
                                    if(result==NULL||strlen(result)!=sizeof(GPSGPRMC_DATEData))
                                    goto Discard_CurrenDat;
                                    memcpy((uint8_t *)&GPSGPRMC_DATEData,result,sizeof(GPSGPRMC_DATEData));

                                    
                                    LOG("GPS is OK!!!!!!!!!!!!");
                            
                                    app->GPS_Lat=0;
                                    for(uint8_t i=0;i<sizeof(GPSGPRMC_LATData.lat);i++)
                                    app->GPS_Lat +=ChartoHex(GPSGPRMC_LATData.lat[i])*(uint32_t)pow(10,7-i);  

                                    for(uint8_t i=0;i<sizeof(GPSGPRMC_LATData.lat);i++)
                                    app->GPS_Lat +=ChartoHex(GPSGPRMC_LATData.lat_decimal[i])*(uint32_t)pow(10,3-i);  


                            
                                    app->GPS_Lng=0;
                                    for(uint8_t i=0;i<sizeof(GPSGPRMC_LONData.lon);i++)
                                    app->GPS_Lng +=ChartoHex(GPSGPRMC_LONData.lon[i])*(uint32_t)pow(10,8-i);  

                                    for(uint8_t i=0;i<sizeof(GPSGPRMC_LONData.lon);i++)
                                    app->GPS_Lng +=ChartoHex(GPSGPRMC_LONData.lon_decimal[i])*(uint32_t)pow(10,3-i);  

                                    
                                    LOGA("GPS_Lat:%10d",app->GPS_Lat);
                                    LOGA("GPS_Lng:%10d",app->GPS_Lng);
                                    


//                                    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
//                                    LOGA(" sDate.Year is :%08x", sDate.Year);



                                    HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);  
                                    HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN); 


                                    HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);  
                                    HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN); 




                                    LOGA("system time is:%04d/%02d/%02d %02d:%02d:%02d",
                                    sDate.Year,sDate.Month, sDate.Date,
                                    sTime.Hours, sTime.Minutes,sTime.Seconds);

                                    UTCTime.year=2000+sDate.Year;
                                    UTCTime.month=sDate.Month;
                                    UTCTime.date=sDate.Date;
                                    UTCTime.hours=sTime.Hours;
                                    UTCTime.minutes=sTime.Minutes;
                                    UTCTime.seconds=sTime.Seconds;

                                    utctime_int=convert_second(&UTCTime);


                                    UTCTime.year=2000+BCD_HEX(ChartoHex(GPSGPRMC_DATEData.year[0])<<4|ChartoHex(GPSGPRMC_DATEData.year[1]));
                                    sDate.Year=BCD_HEX(ChartoHex(GPSGPRMC_DATEData.year[0])<<4|ChartoHex(GPSGPRMC_DATEData.year[1]));
                                    UTCTime.month =sDate.Month=BCD_HEX(ChartoHex(GPSGPRMC_DATEData.month[0])<<4|ChartoHex(GPSGPRMC_DATEData.month[1]));
                                    UTCTime.date = sDate.Date=BCD_HEX(ChartoHex(GPSGPRMC_DATEData.date[0])<<4|ChartoHex(GPSGPRMC_DATEData.date[1]));


                                    UTCTime.hours = sTime.Hours=BCD_HEX(HEX_BCD((ChartoHex(GPSGPRMC_UTCData.hour[0])*10+ChartoHex(GPSGPRMC_UTCData.hour[1])+8)%24));
                                    UTCTime.minutes =  sTime.Minutes=BCD_HEX(ChartoHex(GPSGPRMC_UTCData.minute[0])<<4|ChartoHex(GPSGPRMC_UTCData.minute[1]));
                                    UTCTime.seconds = sTime.Seconds=BCD_HEX(ChartoHex(GPSGPRMC_UTCData.second[0])<<4|ChartoHex(GPSGPRMC_UTCData.second[1]));

                                    utctime_int_gps=convert_second(&UTCTime);
                                    if(abs(utctime_int_gps-utctime_int)>30)
                                    {
                                        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
                                        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
                                        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
                                        {
                                            Error_Handler();
                                        }


                                    LOG("set time !!!!!!!!!!!!!!!!!!!!");


//                                        LOGA(" sDate.Year is :%08x", sDate.Year);

                                        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
                                        {
                                            Error_Handler();
                                        }

                                    }
                                    GPSData.GPSSeqData=GPSSeq_Standby;
                                    app->GpsIsConnected=TRUE;
                                }
                        }
Discard_CurrenDat:                        
                        PrintRecieveData(GPSData.MsgParts,len);   
                        memset(GPSData.MsgParts,0,sizeof(GPSData.MsgParts));
                        
                }
                GPSData.GpsReceiveData.header = (GPSData.GpsReceiveData.header+1)%Gps_Number_of_Receive;
            }
            if(app->Time_Delay.GPS_counts_1ms>delay_10s)
            {
                GPSData.GPSSeqData=GPSSeq_Standby;
                                    LOG("GPSSeq_Standby!!!!!!!!!!!!!!!!!!!!");
                
            }
        break;


        case GPSSeq_Standby:
            GPS_GPIO_DeInit();
            if (HAL_UART_DeInit(&huart1) != HAL_OK)
            {
                Error_Handler();
            }
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
            GPSData.GPSSeqData=GPSSeq_End;
        break;
       case GPSSeq_End:


        break;
    }
}


void ADConverStart(void)
{
    GPSData.ADSeqData=ADSeq_Init;
}

void ADConverStop(void)
{
    GPSData.ADSeqData=ADSeq_Deinit;
}

bool ADConverIsEnd(void)
{
    if(GPSData.ADSeqData==ADSeq_End)
    return TRUE;
    return FALSE;
}



void ADMgrHandle(NoBikeTaskData_t *app)
{
    uint32_t val;
    uint32_t temp0;
    uint8_t temp[4];   
//    float temp;
    
    switch(GPSData.ADSeqData)
    {

        case ADSeq_Idle:
//            ADConverStart();    
            GPSData.ADSeqData=ADSeq_End;

        break;

        case ADSeq_Init:
            MX_DMA_Init(); 
            MX_ADC_Init();
            app->Time_Delay.AD_counts_1ms=0;
            GPSData.ADSeqData=ADSeq_ConStart;
            
            
        break;
        case ADSeq_ConStart:
            if(app->Time_Delay.AD_counts_1ms>delay_200ms)
            {
                  if (HAL_ADC_Start_DMA(&hadc, app->uwADCxConvertedValue, 3) != HAL_OK)
                  {
                    Error_Handler();
                  }
                  app->ADIsFinished=FALSE;
                  GPSData.ADSeqData=ADSeq_CalValue;
                  
            }
        break;

        case ADSeq_CalValue:
            if(app->ADIsFinished)
            {
                LOGA("ADC_CHANNEL_0 is:%08d",
                app->uwADCxConvertedValue[0]);

                LOGA("ADC_CHANNEL_1 is:%08d",
                app->uwADCxConvertedValue[1]);


                LOGA("ADC_CHANNEL_2 is:%08d",
                app->uwADCxConvertedValue[2]);

//                temp=app->uwADCxConvertedValue[0]*10/app->uwADCxConvertedValue[1];
                app->bat_voltage = 2*VREFINT * (app->uwADCxConvertedValue[0]*1000/app->uwADCxConvertedValue[1])+VREF_COMP;
                LOGA("bat_voltage is:%08d",
                app->bat_voltage);
                ADConverStop();
                }
        break;
            
        case ADSeq_Deinit:
            {
            
            MX_ADC_DeInit();
            app->Time_Delay.AD_counts_1ms=0;
            GPSData.ADSeqData=ADSeq_End;
            }
        break;

       case ADSeq_End:
//            if(app->Time_Delay.AD_counts_1ms>delay_10s)
//                ADConverStart();
//        
        break;
    }
}

static bool MMA8652_WriteReg(uint8_t add,uint8_t dat)
{
    uint8_t State;
    uint16_t time_out=0;
    do{
        State=HAL_I2C_Mem_Write(&hi2c1, (uint16_t)I2C_ADDRESS,add, I2C_MEMADD_SIZE_8BIT,&dat,1,I2C_TIME_OUT);
        time_out++;
        
    }while((State!=HAL_OK)&&(time_out<MMA8652_TIME_OUT));
    if(State==HAL_OK)
    return TRUE;
    return FALSE;
}


bool MMA8652_ReadReg(uint8_t add,uint8_t* dat)
{
    uint8_t state;
    uint16_t time_out=0;
    do{
        state=HAL_I2C_Mem_Read(&hi2c1, (uint16_t)I2C_ADDRESS,add, I2C_MEMADD_SIZE_8BIT,dat, 1,I2C_TIME_OUT);
        time_out++;
        
    }while((state!=HAL_OK)&&(time_out<MMA8652_TIME_OUT));

    if(state==HAL_OK)
    return TRUE;
    return FALSE;
}

void SpeedMgrHandle(NoBikeTaskData_t *app)
{

    uint8_t val;

    
    
    switch(GPSData.SpeedSeqData)
    {
        case SpeedSeq_Idle:
//            MMA8652_WriteReg(CTRL_REG1,0x18);
//            MMA8652_WriteReg(TRANSIENT_CFG,0x16);
//            MMA8652_WriteReg(TRANSIENT_THS,0x08);
//            MMA8652_WriteReg(TRANSIENT_COUNT,0x0A);
//            MMA8652_WriteReg(CTRL_REG4,0x20);
//            MMA8652_WriteReg(CTRL_REG5,0x00);//route to INT2    
//            MMA8652_ReadReg(CTRL_REG5,&val);
//            
//            MMA8652_ReadReg(CTRL_REG1,&val);
//            val |=0x01;    
//            MMA8652_WriteReg(CTRL_REG1,val);
//            MMA8652_ReadReg(CTRL_REG1,&val);

//            GPSData.SpeedSeqData=SpeedSeq_End;
        break;
        
       case SpeedSeq_End:
        break;
    }
}

void OpenUnLockStart(void)
{
    GPSData.LockSeqData=LockSeq_Open;
}

static bool LockIsClose(void)
{
    Switch_GPIO_Init();       
    if(HAL_GPIO_ReadPin(MOTO_IS_OPEN_GPIO_Port,MOTO_IS_CLOSE_Pin)==GPIO_PIN_SET)
    {
        Switch_GPIO_DeInit();    
        return TRUE;
    }
    Switch_GPIO_DeInit();    
    return FALSE;
}

static bool LockIsOpen(void)
{
    Switch_GPIO_Init();       
    if(HAL_GPIO_ReadPin(MOTO_IS_OPEN_GPIO_Port,MOTO_IS_CLOSE_Pin)==GPIO_PIN_RESET)
    {
        Switch_GPIO_DeInit();    
        return TRUE;

    }
    Switch_GPIO_DeInit();    
    return FALSE;
}

void RentMgrHandle(NoBikeTaskData_t *app)
{
    switch(GPSData.LockSeqData)
    {
        case LockSeq_Idle:
            if(app->NoBikeProcess ==returnBike_Process)
            {
                GPSData.LockSeqData=LockSeq_WaitForClose;
                app->Time_Delay.Lock_counts_1ms=0;
            }
        break;

        case LockSeq_Open:
            if(app->NoBikeProcess ==rentBike_Process)
            {
                GPRS_GPIO_Init();            
                GPRS_PWR_ON;
                
                Moto_GPIO_Init();
                Switch_GPIO_Init();            
                MOTO_REVERSE();
                        LOG("MOTO_REVERSE++++++++++++++++++++++++++++++");
                
                app->Time_Delay.Lock_counts_1ms=0;
                GPSData.LockSeqData=LockSeq_Delay;
            }
            else
            {
                GPSData.LockSeqData=LockSeq_Idle;
                LOG("app->NoBikeProcess is returnBike_Process");
                
            }
        break;


        case LockSeq_WaitForClose:
            if(app->Time_Delay.Lock_counts_1ms>delay_400ms)
            {
                Moto_GPIO_Init();
                if(LockIsClose())
                {
                    GPSData.LockSeqData=LockSeq_WaitForCloseConfirm;
                }
                else
                {
                    Moto_GPIO_DeInit();
                }
                app->Time_Delay.Lock_counts_1ms=0;
            }
        break; 

        case LockSeq_WaitForCloseConfirm:
            if(app->Time_Delay.Lock_counts_1ms>delay_400ms)
            {
                Moto_GPIO_Init();
                if(LockIsClose())
                {
                    GPSData.LockSeqData=LockSeq_End;
                    produce_record(&gBikeRecord,return_bike);
//                    addRecord2Flash((uint8_t *)&gBikeRecord);
                    app->NoBikeProcess =rentBike_Process;
                    LOG("app->NoBikeProcess is rentBike_Process");
                    
                }
                else
                {
                    Moto_GPIO_DeInit();
                    GPSData.LockSeqData=LockSeq_WaitForClose;
                }
                app->Time_Delay.Lock_counts_1ms=0;
            }
        break;

        case LockSeq_Delay:
            if(app->Time_Delay.Lock_counts_1ms>delay_100ms)
            {
                MOTO_STANDBY();
                app->Time_Delay.Lock_counts_1ms=0;
                GPSData.LockSeqData=LockSeq_LockIsOpen;
                
            }
            
        break;

        case LockSeq_LockIsOpen:
            if(app->Time_Delay.Lock_counts_1ms>delay_400ms)
            {
                if(LockIsOpen())    
                {
                GPSData.LockSeqData=LockSeq_ProduceRecorder;
                app->Time_Delay.Lock_counts_1ms=0;
                
                }
                else
                {
                    LOG("lock open is failed!!!!!");
                    GPSData.LockSeqData=LockSeq_Stop;
                    
                }

            }
            

        break;

        case LockSeq_ProduceRecorder:
            if(app->Time_Delay.Lock_counts_1ms>delay_2s)
            {
                produce_record(&gBikeRecord,rent_bike);
//                addRecord2Flash((uint8_t *)&gBikeRecord);
                app->NoBikeProcess =returnBike_Process;
                GPSData.LockSeqData=LockSeq_Stop;
            }
        break;
        case LockSeq_Stop:
            MOTO_STANDBY();
            Moto_GPIO_DeInit();
            GPSData.LockSeqData=LockSeq_WaitForClose;
            app->Time_Delay.Lock_counts_1ms=0;
        break;
        case LockSeq_End:
        break;
    }
}

GPS_Data_t *getGPSData(void)
{
    return &GPSData;
}

static void PrintConfigData(void)
{
    pNoBikeTaskData_t app = getApp();
    LOGA("BikeArea is:%08x",app->GPRSConfigData.BikeArea[0]);
    LOGA("BikeArea is:%08x",app->GPRSConfigData.BikeArea[1]);
    LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial[0]);
    LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial[1]);
    LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial[2]);
    LOGA("Mode is:%08x",app->GPRSConfigData.System_mode);
    LOGA("IP is:%c%c%c.%c%c%c.%c%c%c.%c%c%c:%c%c%c%c%c",
    app->GPRSConfigData.IP0[0],app->GPRSConfigData.IP0[1],app->GPRSConfigData.IP0[2],
    app->GPRSConfigData.IP1[0],app->GPRSConfigData.IP1[1],app->GPRSConfigData.IP1[2],
    app->GPRSConfigData.IP2[0],app->GPRSConfigData.IP2[1],app->GPRSConfigData.IP2[2],
    app->GPRSConfigData.IP3[0],app->GPRSConfigData.IP3[1],app->GPRSConfigData.IP3[2],
    app->GPRSConfigData.Port[0],app->GPRSConfigData.Port[1],app->GPRSConfigData.Port[2],app->GPRSConfigData.Port[3],app->GPRSConfigData.Port[4]);
#ifdef Neo_M590E


    memcpy((uint8_t *)GPRS_SendMYNETSRV+21,app->GPRSConfigData.IP0,sizeof(app->GPRSConfigData.IP0));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+25,app->GPRSConfigData.IP1,sizeof(app->GPRSConfigData.IP1));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+29,app->GPRSConfigData.IP2,sizeof(app->GPRSConfigData.IP2));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+33,app->GPRSConfigData.IP3,sizeof(app->GPRSConfigData.IP3));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+37,app->GPRSConfigData.Port,sizeof(app->GPRSConfigData.Port));
#elif defined(LONGSUNG_A8300)
    memcpy((uint8_t *)GPRS_SendMYNETSRV+20,app->GPRSConfigData.IP0,sizeof(app->GPRSConfigData.IP0));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+24,app->GPRSConfigData.IP1,sizeof(app->GPRSConfigData.IP1));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+28,app->GPRSConfigData.IP2,sizeof(app->GPRSConfigData.IP2));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+32,app->GPRSConfigData.IP3,sizeof(app->GPRSConfigData.IP3));
    memcpy((uint8_t *)GPRS_SendMYNETSRV+37,app->GPRSConfigData.Port,sizeof(app->GPRSConfigData.Port));

#endif

    
}
bool SaveConfigData(void)
{
    pNoBikeTaskData_t app = getApp();
    
    GPRSConfig_t GPRSConfigData_temp;
    uint32_t dat;
    uint32_t target_add=EEPROM_BASE;

    uint8_t *p=(uint8_t *)&app->GPRSConfigData;

    if(HAL_FLASHEx_DATAEEPROM_Unlock()==HAL_OK)  
    {
        for(uint8_t i=0;i<sizeof(GPRSConfigData_temp)/4;i++)
        {
            dat=p[3]<<24|p[2]<<16|p[1]<<8|p[0];
            HAL_FLASHEx_DATAEEPROM_Erase(target_add);            
            if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,target_add,dat)!=HAL_OK)
            LOG("HAL_FLASHEx_DATAEEPROM_Program:SaveConfigData fails");
                
                
            target_add +=4;
            p +=4;
        }

        
        HAL_FLASHEx_DATAEEPROM_Lock();    
        memcpy((uint8_t*)&GPRSConfigData_temp,(uint8_t*)EEPROM_BASE,sizeof(GPRSConfigData_temp));
        if(Util_Buffer_cmp((uint8_t*)&GPRSConfigData_temp,(uint8_t*)&app->GPRSConfigData,sizeof(GPRSConfigData_temp)))
        {
            LOG("SaveConfigData ok");
//        LOGA("BikeArea is:%08x",app->GPRSConfigData.BikeArea.BITS.dat[0]);
//        LOGA("BikeArea is:%08x",app->GPRSConfigData.BikeArea.BITS.dat[1]);
//        LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial.BITS.dat[0]);
//        LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial.BITS.dat[1]);
//        LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial.BITS.dat[2]);
        PrintConfigData();

        
            
            return TRUE;
        }
        else
        {
            LOG("Util_Buffer_cmp:SaveConfigData fails");
        }
        
    }
    else
    {
        LOG("SaveConfigData fails");
    }
    return FALSE;
}


void GetConfigData(void)
{
    pNoBikeTaskData_t app = getApp();
    uint16_t crc16;


    app->NoBikeProcess =NoBike_Initialising;
     
    
    if(LockIsClose())
    {
        app->NoBikeProcess =rentBike_Process;
        LOG("rentBike_Process");
                                
    }
    else
    {
        app->NoBikeProcess =returnBike_Process;
        LOG("returnBike_Process");
                                
    }
//    GPRSConfig_t GPRSConfigData_temp;
    memcpy((uint8_t*)&app->GPRSConfigData,(uint8_t*)EEPROM_BASE,sizeof(app->GPRSConfigData));
    crc16=get_crc16((uint8_t*)EEPROM_BASE,sizeof(app->GPRSConfigData)-2);
            LOGA("dat_low is:%08x",crc16&0xff);
            LOGA("dat_high is:%08x",crc16>>8);
//        LOGA("BikeArea is:%08x",app->GPRSConfigData.BikeArea.BITS.dat[0]);
//        LOGA("BikeArea is:%08x",app->GPRSConfigData.BikeArea.BITS.dat[1]);
//        LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial.BITS.dat[0]);
//        LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial.BITS.dat[1]);
//        LOGA("BikeSerial is:%08x",app->GPRSConfigData.BikeSerial.BITS.dat[2]);
//    
    
    if(!isEqualcrc16(crc16,app->GPRSConfigData.Crc16_low,app->GPRSConfigData.Crc16_high)
    ||app->GPRSConfigData.Map[0]!=0xff)
    {
        app->GPRSConfigData.BikeArea[0]=0x05;
        app->GPRSConfigData.BikeArea[1]=0x12;
        
        app->GPRSConfigData.BikeSerial[0]=0x01;
        app->GPRSConfigData.BikeSerial[1]=0;
        app->GPRSConfigData.BikeSerial[2]=0;


        app->GPRSConfigData.System_mode=NoBikeMode_Testing;

        app->GPRSConfigData.IP0[0]='1';
        app->GPRSConfigData.IP0[1]='1';
        app->GPRSConfigData.IP0[2]='4';

        app->GPRSConfigData.IP1[0]='2';
        app->GPRSConfigData.IP1[1]='1';
        app->GPRSConfigData.IP1[2]='9';
        

        app->GPRSConfigData.IP2[0]='0';
        app->GPRSConfigData.IP2[1]='4';
        app->GPRSConfigData.IP2[2]='8';

        app->GPRSConfigData.IP3[0]='0';
        app->GPRSConfigData.IP3[1]='6';
        app->GPRSConfigData.IP3[2]='2';

        app->GPRSConfigData.Port[0]='4';
        app->GPRSConfigData.Port[1]='5';
        app->GPRSConfigData.Port[2]='0';
        app->GPRSConfigData.Port[3]='0';
        app->GPRSConfigData.Port[4]='1';

        memset(app->GPRSConfigData.Map,0xff,sizeof(app->GPRSConfigData.Map));

        

        crc16=get_crc16((uint8_t*)&app->GPRSConfigData,sizeof(app->GPRSConfigData)-2);
        app->GPRSConfigData.Crc16_low=crc16&0xff;
        app->GPRSConfigData.Crc16_high=crc16>>8;
        SaveConfigData();

    }
    else
    {

        LOG("GetConfigData is ok");
        PrintConfigData();
        

    }

}

 #define RX_BUF_SIZE (256)

char* Parser_GetString(char** pMsg, uint8_t * error)
{
    char* retval;
    uint8_t myError = ((pMsg == NULL) || (*pMsg == NULL) || (**pMsg == '\0'));

    if (myError)
    {
        retval = NULL;
        if (error != NULL)
        {
            *error = TRUE;
        }
    }
    else
    {
        retval = *pMsg;

        while ((**pMsg != '\0') && (**pMsg != ','))
        {
            (*pMsg)++;
        }

        if (**pMsg != '\0')
        {
            **pMsg = '\0';
        }
        (*pMsg)++;
    }

    return retval;
}

 
 void parse_lock_cmd(uint8_t * pCmd)
 {
     char* pTestCmd = Parser_GetString((char **)&pCmd, NULL);
     if (strcmp(pTestCmd, "u") == 0||strcmp(pTestCmd, "U")==0 )
     {
        OpenUnLockStart();
//                Moto_GPIO_Init();

//                MOTO_REVERSE();

//                Moto_GPIO_DeInit();

        
     }
     else if (strcmp(pTestCmd, "l") == 0||strcmp(pTestCmd, "L")==0 )
     {
//        OpenUnLockStart();

                Moto_GPIO_Init();

MOTO_FORWARD();

                Moto_GPIO_DeInit();

     }
     else if (strcmp(pTestCmd, "s") == 0||strcmp(pTestCmd, "S")==0 )
     {

                Moto_GPIO_Init();

        MOTO_STANDBY();        
     }
 }

 void parse_led_cmd(uint8_t * pCmd)
 {
     char* pTestCmd = Parser_GetString((char **)&pCmd, NULL);
     if (strcmp(pTestCmd, "y") == 0||strcmp(pTestCmd, "Y")==0 )
     {
        GPRS_YELLOW_EN();
        
     }
     else if (strcmp(pTestCmd, "g") == 0||strcmp(pTestCmd, "G")==0 )
     {
        GPRS_GREEN_EN();        
     }
     else if (strcmp(pTestCmd, "r") == 0||strcmp(pTestCmd, "R")==0 )
     {
        GPRS_RED_EN();        
     }
     else if (strcmp(pTestCmd, "o") == 0||strcmp(pTestCmd, "O")==0 )
     {
        GPRS_LED_OFF();        
     }
     
 }
 
 void parse_beep_cmd(uint8_t * pCmd)
 {
     char* pTestCmd = Parser_GetString((char **)&pCmd, NULL);
     if (strcmp(pTestCmd, "on") == 0||strcmp(pTestCmd, "ON")==0 )
     {
        GPRS_BEEP_ON();
        
     }
     else if (strcmp(pTestCmd, "off") == 0||strcmp(pTestCmd, "OFF")==0 )
     {
        GPRS_BEEP_OFF();        
     }

     
 }
 static void IPAddressCopy(uint8_t* src,uint8_t* dst,uint8_t len)
 {
    switch(len)
    {
        case 1:
            *dst++='0';
            *dst++='0';
            *dst=*src;
        break;
        case 2:
            *dst++='0';
            *dst++=src[0];
            *dst=src[1];

        break;    

        case 3:
            *dst++=src[0];
            *dst++=src[1];
            *dst=src[2];
        break;    
    }
 }

 
 void parse_cmd(void)
 {
    pNoBikeTaskData_t app = getApp();
    uint32_t serial;
    uint8_t mode;
     
     static uint8_t  USART_Rx_Buffer[RX_BUF_SIZE];
     static __IO uint32_t USART_Rx_ptr_in;
     uint8_t *dat_h,*dat_t;
     uint8_t len;
     uint16_t crc16;

     
     uint8_t rcv_char = 0;
     rcv_char = (uint8_t)hlpuart1.Instance->RDR; 
             /*received an Enter key*/
     if (rcv_char == '\r') 
    {
        USART_Rx_Buffer[USART_Rx_ptr_in] = '\0';
        LOGA("UART RX: %s",USART_Rx_Buffer);
        USART_Rx_ptr_in = 0;

        // parse the input cmd here
        uint8_t * temp_char = USART_Rx_Buffer;
        char* cmd = Parser_GetString((char**)&temp_char, NULL);

        if(strcmp(cmd,"lock") == 0
            ||strcmp(cmd,"Lock") == 0
            ||strcmp(cmd,"LOCK") == 0)
        {
            parse_lock_cmd(temp_char);
        }
        else if(strcmp(cmd,"led") == 0
            ||strcmp(cmd,"Led") == 0
            ||strcmp(cmd,"LED") == 0)
        {
            parse_led_cmd(temp_char);
        }
        else if(strcmp(cmd,"beep") == 0
            ||strcmp(cmd,"Beep") == 0
            ||strcmp(cmd,"BEEP") == 0)
        {
            parse_beep_cmd(temp_char);
        }

        else if(strcmp(cmd,"erase") == 0
            ||strcmp(cmd,"Erase") == 0
            ||strcmp(cmd,"ERASE") == 0)
        {
                Flash_Bike_Recoder_Init();
                
        }
        
        
        else if(strcmp(cmd,"config") == 0
            ||strcmp(cmd,"Config") == 0
            ||strcmp(cmd,"CONFIG") == 0)
        {

            char delims[] = ",";
            char point[] = ".";
            char colon[] = ":";
            char *result = NULL;
            
            dat_h=temp_char;
            Parser_GetString((char **)&temp_char, NULL);           
            dat_t=temp_char-1;
     
            len=dat_t-dat_h;
            if(len !=GPS_AREA_LEN)
            goto error_handel;    

            memset((uint8_t *)&app->GPRSConfigData,0,sizeof(app->GPRSConfigData));
            app->GPRSConfigData.BikeArea[0]=(*dat_h++-0x30)<<4;
            app->GPRSConfigData.BikeArea[0]|=*dat_h++-0x30;
            app->GPRSConfigData.BikeArea[1]=(*dat_h++-0x30)<<4;
            app->GPRSConfigData.BikeArea[1]|=*dat_h++-0x30;

            dat_h=temp_char;
            Parser_GetString((char **)&temp_char, NULL);         
            dat_t=temp_char-1;
            

            len=dat_t-dat_h;
            if(len < GPS_SERIAL_MIN_LEN)
            goto error_handel;   

            serial=0;
            for(uint8_t i=0;i<len;i++)
            {
                serial += (dat_h[i]-0x30)*pow(10,len-i-1);
            }
            
            LOGA("serial is:%08d",serial);

            app->GPRSConfigData.BikeSerial[0]=serial&0xFF;
            app->GPRSConfigData.BikeSerial[1]=serial>>8&0xFF;
            app->GPRSConfigData.BikeSerial[2]=serial>>16&0xFF;



            dat_h=temp_char;
            Parser_GetString((char **)&temp_char, NULL);         
            dat_t=temp_char-1;

            len=dat_t-dat_h;

            if(len < GPS_MODE_MIN_LEN)
            goto error_handel;   

            mode=0;
            for(uint8_t i=0;i<len;i++)
            {
                mode += (dat_h[i]-0x30)*pow(10,len-i-1);
            }

            app->GPRSConfigData.System_mode=mode;



            result = strtok(temp_char,point);
            if(result==NULL||strlen(result)==0||strlen(result)>3)
            goto error_handel;
            IPAddressCopy(result,app->GPRSConfigData.IP0,strlen(result));

            result = strtok(NULL,point);
            if(result==NULL||strlen(result)==0||strlen(result)>3)
            goto error_handel;
            IPAddressCopy(result,app->GPRSConfigData.IP1,strlen(result));


            result = strtok(NULL,point);
            if(result==NULL||strlen(result)==0||strlen(result)>3)
            goto error_handel;
            IPAddressCopy(result,app->GPRSConfigData.IP2,strlen(result));


            result = strtok(NULL,colon);
            if(result==NULL||strlen(result)==0||strlen(result)>3)
            goto error_handel;
            IPAddressCopy(result,app->GPRSConfigData.IP3,strlen(result));



            result = strtok(NULL,delims);
            if(result==NULL||strlen(result)!=sizeof(app->GPRSConfigData.Port))
            goto error_handel;

            memcpy(app->GPRSConfigData.Port,result,sizeof(app->GPRSConfigData.Port));

            memset(app->GPRSConfigData.Map,0xff,sizeof(app->GPRSConfigData.Map));

            crc16=get_crc16((uint8_t*)&app->GPRSConfigData,sizeof(app->GPRSConfigData)-2);
            
            
            
            app->GPRSConfigData.Crc16_low=crc16&0xff;
            app->GPRSConfigData.Crc16_high=crc16>>8;
            

       //     LOGA("dat_low_config is:%08x",app->GPRSConfigData.CRC16.BITS.dat_low);
        //    LOGA("dat_high_config is:%08x",app->GPRSConfigData.CRC16.BITS.dat_high);

            
            SaveConfigData();
            

            
        }
        
         else
         {
            if(dat_h==NULL)
            goto error_handel;    
         }
    }
         /*reveived a Backspace key */
     else if (rcv_char == '\b')
     {
         if (USART_Rx_ptr_in > 0)
         {
           USART_Rx_ptr_in--;
         }
     }
     else
     {
         USART_Rx_Buffer[USART_Rx_ptr_in++] = rcv_char;
     }

     if (USART_Rx_ptr_in > (RX_BUF_SIZE))
     {
       /**/
         USART_Rx_Buffer[USART_Rx_ptr_in] = '\0';
         USART_Rx_ptr_in = 0;
     }
 return;    
 error_handel:
    USART_Rx_ptr_in = 0;
    memset(USART_Rx_Buffer,0,256);
    printf("Invalid Commands\r\n");
 }



void sendRecord2CenterFromFlashHandle(NoBikeTaskData_t *app)
{
    static uint16_t len;
    
    uint16_t crc16;
    uint32_t target_addr;

    pBike_Record_t Bike_Record_target_addr;    

      switch(GPSData.SendRecorder2CenterSeqData)
    {
        case SendRecorder2CenterSeq_Idle:
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_InitFlash;      
        break;
        case SendRecorder2CenterSeq_InitFlash:
                app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms=0;
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_InitFlashDelay;  
            
        break;


        case SendRecorder2CenterSeq_InitFlashDelay:
                if(app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms>delay_300ms)
                {
                    FlashInit();
                    GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_Start;  
                }
        break;


        case SendRecorder2CenterSeq_Start:
//                    produce_record(&gBikeRecord,return_bike);
//                    addRecord2Flash((uint8_t *)&gBikeRecord);
            
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_ReadHeaderTailer;      
                if(GPSData.FlashWriteErrTimes>FlashWriteErrTimes_MAX)
                {
                    Flash_Bike_Recoder_Init();
                    GPSData.FlashWriteErrTimes=0;
                }
        break;


        case SendRecorder2CenterSeq_ReadHeaderTailer:
            if(app->Flash_Bike_Recorder_Tailer.tailer!=app->Flash_Bike_Recorder_Header.header)  /* need to transmit bike recorder to center*/
            {
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_ReadBikeRecorder;
                LOGA("app->Flash_Bike_Recorder_Header.header:0x%08x",app->Flash_Bike_Recorder_Header.header);
                LOGA("app->Flash_Bike_Recorder_Tailer.tailer:0x%08x",app->Flash_Bike_Recorder_Tailer.tailer);
                
            }
            else
            {
                app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms=0;
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_Interval;
//                HAL_SPI_DeInit(&hspi1);
                if(app->HeartIsStop)
                {
                    app->HeartIsStop=FALSE;
                    GPRSHeartStart();
                }
            }
        break;
        case SendRecorder2CenterSeq_ReadBikeRecorder:
            {
                target_addr=FLASH_Bike_Recorder_Data_START + app->Flash_Bike_Recorder_Header.header* sizeof(Bike_Record_t);
                
                flash_spi_bufferRead((uint8_t*)&gBikeRecord, target_addr, sizeof(Bike_Record_t));

                crc16=get_crc16((uint8_t *)&gBikeRecord,sizeof(Bike_Record_t)-sizeof(gBikeRecord.map)-sizeof(gBikeRecord.response)-2); 
                
                if(isEqualcrc16(crc16,gBikeRecord.crc16_low,gBikeRecord.crc16_high))
                {
                    GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_SendMsg2Center;
                }
                else/*invalid bike recorder*/
                {
                    if(GPSData.crciswongfromFlash_errtimes++>5)/*ignore the current recorder, jump to the next one*/
                    {
                        app->Flash_Bike_Recorder_Header.header=(app->Flash_Bike_Recorder_Header.header+1)%Bike_Recorder_Toltal_Number;
                        Flash_Bike_Recorder_HeaderTailer_Write(app->Flash_Bike_Recorder_Header.header,header);
                        LOG("get_crc16 is wrong!");
                    }
                    GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_Start;
                }
            }
        break;

        case SendRecorder2CenterSeq_SendMsg2Center: 
            if(GPRSOutgoingIsEnd()&&GPRSIncomingIsEnd()&&GPRSEstablishedIsEnd()&&GPRSHeartIsEnd()&&app->GprsIsConnected)
            {
                GPRSLoadDataTransmitStart(GPRSCommType_Record);
                app->RecorderIsSending=TRUE;
                
                app->query_bike_record_rsp_flag=FALSE;
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_WaitForRsp;
                app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms=0;
            }
        break;


        case SendRecorder2CenterSeq_WaitForRsp:
            if(app->query_bike_record_rsp_flag)
            {
                LOG("SendRecorder2CenterSeq_WaitForRsp ok!");
                
                Bike_Record_target_addr=(pBike_Record_t)(FLASH_Bike_Recorder_Data_START + app->Flash_Bike_Recorder_Header.header* sizeof(Bike_Record_t));
                Flash_Bike_Recorder_Response_Verification_Write(recorder_responsed,(uint32_t)&Bike_Record_target_addr->response);                    
                app->Flash_Bike_Recorder_Header.header=(app->Flash_Bike_Recorder_Header.header+1)%Bike_Recorder_Toltal_Number;
                if(!Flash_Bike_Recorder_HeaderTailer_Write(app->Flash_Bike_Recorder_Header.header,header))
                    LOG("Flash_Bike_Recorder_HeaderTailer_Write is  failed!+++++++++");
                    

                app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms=0;
                GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_Interval;
//                LOG("SendRecorder2CenterSeq_WaitForRsp ok!");
            }
            else
            {
                if(app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms>delay_20s)
                {
                    app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms=0;
                    GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_Interval_SendMsg2Center;
                    LOG("SendRecorder2CenterSeq_WaitForRsp failed!");
                    app->RecorderIsSending=FALSE;

                }
            }
            break;

            case SendRecorder2CenterSeq_Interval:
                if(app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms>delay_3s)
                {
                    GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_Start;
                    app->RecorderIsSending=FALSE;
                    
                }
            break;

            case SendRecorder2CenterSeq_Interval_SendMsg2Center:
                if(app->Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms>delay_200ms)
                {
                    GPSData.SendRecorder2CenterSeqData=SendRecorder2CenterSeq_SendMsg2Center;
                }

            break;
            case SendRecorder2CenterSeq_End:
            break;
            default:
            break;
        }
}



