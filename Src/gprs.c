/* Includes ------------------------------------------------------------------*/
#define COMPID "gprs"


#include "main.h"
#include "gprs.h"
#include "gps.h"

#include "util.h"
#include "bsp_printf.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc;


extern Bike_Record_t gBikeRecord;

#ifdef Neo_M590E
static const char *GPRS_SendCmd[]=
{
    "AT\r\n",
    "ATI\r\n",
        
    "ATE0\r\n",
    "AT+CCID\r\n",
    "AT+CPIN?\r\n",
    "AT+CSQ\r\n",
    "AT+CREG?\r\n",

    "AT+CGATT?\r\n",
    "AT$MYNETURC=1\r\n",
    "AT$MYNETACT=0,1\r\n",


    

//    "AT$MYNETSRV=0,0,0,0,\"221.178.130.170:20008\"\r\n",
//    "AT$MYNETSRV=0,0,0,0,\"221.178.130.170:20001\"\r\n",
//    "AT$MYNETSRV=0,0,0,0,\"114.219.048.062:45001\"\r\n",

//    "AT$MYNETOPEN=0\r\n",
//    "AT$MYNETREAD=0,2048\r\n",
};
char  GPRS_SendMYNETSRV[]= "AT$MYNETSRV=0,0,0,0,\"114.219.048.062:45001\"\r\n";
const static char *GPRS_SendMYNETOPEN="AT$MYNETOPEN=0\r\n";

static  char *GPRS_SendMyNetWrite="AT$MYNETWRITE=0,2048\r\n";
static  char *GPRS_SendMyNetRead="AT$MYNETREAD=0,2048\r\n";
static  char *GPRS_SendMyNetACK="AT$MYNETACK=0\r\n";
static  char *GPRS_SendEnPWRSave="AT+ENPWRSAVE=1\r\n";

#elif defined(LONGSUNG_A8300)

static const char *GPRS_SendCmd[]=
{
    "ATI\r\n",
    "ATE0\r\n",
    "AT+CREG?\r\n",
    "AT+CSQ\r\n",
    "AT+MIPMODE=0,0,0\r\n",    
    "AT+SLPTM=25000\r\n",
//    "AT+MIPOPEN=1,\"TCP\",\"221.178.130.170\",20001,7000\r\n",
//    "AT+MIPOPEN=1,\"TCP\",\"101.231.214.90\",60099,7000\r\n",

//    "AT+MIPOPEN?\r\n",
//    "AT+MIPMODE=0,0,0\r\n",
};

char  GPRS_SendMYNETSRV[]= "AT+MIPOPEN=1,\"TCP\",\"221.178.130.170\",20001,7000\r\n";
const static char *GPRS_SendMIPCALL="AT+MIPCALL=1,\"CMNET\"\r\n";
const static char *GPRS_SendMYNETOPEN="AT+MIPOPEN?\r\n";

static  char *GPRS_SendMyNetWrite="AT+MIPSEND=1,2048\r\n";
static  char *GPRS_SendMyNetRead="AT$MYNETREAD=0,2048\r\n";
static  char *GPRS_SendMyNetACK="AT+MIPSTATE?\r\n";
static  char *GPRS_SendEnPWRSave="AT+ENPWRSAVE=1\r\n";

#endif
#if 0
static  const char *GPRS_SendCmd[]=
{
    "AT\r",
    "AT+CCID\r",
    "AT+CPIN?\r",
    "AT+CSQ\r",
    "AT+CREG?\r",

    "AT+CGATT?\r",
    "AT$MYNETURC=1\r",

    "AT$MYNETACT=0,1\r",



    "AT$MYNETSRV=0,0,0,0,\"221.178.130.170:20001\"\r",
    "AT$MYNETOPEN=0\r",
    "AT$MYNETWRITE=0,5\r",
    "12345"
    
};
#endif
//static  char *GPRS_EnterData="\r\n";
//static  char *GPRS_SendAT="AT\r\n";


static GPRS_Data_t GPRSData;



uint8_t receivedata[100];
uint8_t senddata[100];

static void LoadLenForSendMyNetWrite(NoBikeTaskData_t *app)
{
    uint8_t *s;
    memcpy(app->GPRS_SendMyNetWriteData,GPRS_SendMyNetWrite,sizeof(app->GPRS_SendMyNetWriteData));
    
    s=(uint8_t *)strstr((const char *)app->GPRS_SendMyNetWriteData,",")+1;
    *s++=app->comm_len/1000%10+0x30;
    *s++=app->comm_len/100%10+0x30;
    *s++=app->comm_len/10%10+0x30;
    *s++=app->comm_len%10+0x30;
//    *s++=1+0x30;

}
static void GPRSTransmitLoopStart(void)	
{

    pNoBikeTaskData_t app = getApp();
    
    
    GPRSData.GPRSSeqData=GPRSSeq_SendCmdLoop;	
    GPRSData.counts=0;    
    app->Time_Delay.GPRS_counts_1ms=0; 
//    LOG("GPRSTransmitCmdStart"); 
    
}


void GPRSEstabilishStart(void)
{
    pNoBikeTaskData_t app = getApp();

    GPRSData.GPRSSeqData=GPRSSeq_Init;
    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_End;
    GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_End;
    GPRSData.Restart_Times++;
//    app->GprsReConnectTims++;
}


void GPRSReset(void)
{
    pNoBikeTaskData_t app = getApp();
    app->GprsIsConnected=FALSE;
    if (HAL_UART_DeInit(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);

    GPRS_GPIO_Init();
    GPRS_PWR_OFF;
    GPRS_DTR_ON;
}

void GPRSTransmitDataStart(void)
{
    pNoBikeTaskData_t app = getApp();

    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendWriteCmd;
    GPRS_DTR_OFF;
    app->Time_Delay.GPRS_Outgoing_counts_1ms=0;
}
#if 0
void GPRSReceiveDataStart(void)
{
    pNoBikeTaskData_t app;          
    app = getApp();
    GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_SendReadCmd;
    GPRS_DTR_ON;
    app->Time_Delay.GPRS_Incoming_counts_1ms=0;
    LOG("GPRSReceiveDataStart");
    
}
#endif
void GPRSReceiveDataStart(void)
{
    pNoBikeTaskData_t app = getApp();

    GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_WaitEnd;
    app->Time_Delay.GPRS_Incoming_counts_1ms=0;
//    LOG("GPRSReceiveDataStart");
}


static bool GPRSSendCmdDatUart(const char *string)
{
    GPRSData.GPRSFeedBackIsOK=FALSE;
    GPRSData.GPRSRealDataIsArrived=FALSE;
    GPRSData.GPRSRemoteDataReadyForReceive=FALSE;
    GPRSData.GPRSMIPCallIsAck=FALSE;
    GPRSData.GPRSEstabilisedAckIsConnected=FALSE;
    GPRSData.GPRSFeedBackReceiveErr=FALSE;
    if(HAL_UART_Transmit(&huart2,(uint8_t *)string, strlen(string),UART_TIMEOUT) != HAL_OK)             
    {
        LOG("HAL_UART_Transmit is failed 00");
//            MX_USART1_UART_Init();	
        return FALSE;

    }    
    else
    {
        PrintRecieveData((uint8_t *)string,strlen(string)); 
        return TRUE;
        
    }
}

static bool GPRSSendDummyDatUart(void)
{
    uint8_t dummy=0;
    if(HAL_UART_Transmit(&huart2,(uint8_t *)dummy, 1,UART_TIMEOUT) != HAL_OK)             
    {
        LOG("HAL_UART_Transmit is failed 00");
//            MX_USART1_UART_Init();	
        return FALSE;

    }    
    return TRUE;
}


static bool GPRSSendRealData(uint8_t *dat,uint8_t len)
{
//    len=1;
    GPRSData.GPRSFeedBackIsOK=FALSE;
//    for(uint8_t i=0;i<len;i++)
//    dat[i]='1';    
    
    if(HAL_UART_Transmit(&huart2,dat,len,UART_TIMEOUT) != HAL_OK)             
    {
        LOG("HAL_UART_Transmit is failed 01");
        return FALSE;
    }    
    else
    {
        for(uint16_t i=0;i<len;i++)
        LOGA("GPRSSendRealData:%08x",dat[i]);
        return TRUE;
    }
}





void GPRSRecieveMsgParse(NoBikeTaskData_t *app)
{
    static uint16_t pointer_start,pointer_end;
  //  static uint16_t tailer_temp,header_temp;
    uint16_t len,p_len;
 //   bool ParseIsOK=FALSE;
    uint8_t *dat_h,*dat_t;
    static uint16_t dat_len;
    uint16_t header_pre;
    

//    goto test;
    switch(GPRSData.GPRSReceiveMsgParseSeqData)
    {

        case GPRSReceiveMsgParseSeq_Idle:
            GPRSData.GPRSReceiveMsgParseSeqData=GPRSReceiveMsgParseSeq_ParseInit;       
        break;


        case GPRSReceiveMsgParseSeq_ParseInit:
            pointer_start=GPRSData.GprsReceiveData.header;
            GPRSData.GPRSReceiveMsgParseSeqData=GPRSReceiveMsgParseSeq_ParseLoop;
//            header_temp=GPRSData.GprsReceiveData.header;

        break;

        case GPRSReceiveMsgParseSeq_ParseLoop:
            if(GPRSData.GprsReceiveData.tailer!=GPRSData.GprsReceiveData.header)
            {

                if(GPRSData.GprsReceiveData.header==0)
                header_pre=Gprs_Number_of_Receive-1;
                else
                header_pre=GPRSData.GprsReceiveData.header-1;
                    
                
                if((GPRSData.GprsReceiveData.dat[GPRSData.GprsReceiveData.header]=='\n')&&(GPRSData.GprsReceiveData.dat[header_pre]=='\r'))
                {
                       // ParseIsOK=FALSE;
                        pointer_end = (GPRSData.GprsReceiveData.header+1)%Gprs_Number_of_Receive;
                        if(pointer_end>=pointer_start)
                        len=pointer_end-pointer_start;
                        else
                        len=Gprs_Number_of_Receive- pointer_start + pointer_end;

                      
                        for(uint16_t i=0;i<len;i++)
                        {
                            GPRSData.MsgParts[i]=GPRSData.GprsReceiveData.dat[pointer_start];  
                            pointer_start=(pointer_start+1)%Gprs_Number_of_Receive;
                        }
                         
                        if(strstr((const char *)GPRSData.MsgParts,"OK")!=NULL)
                        {
                           // ParseIsOK=TRUE;
                            GPRSData.GPRSFeedBackIsOK=TRUE;
                            pointer_start = pointer_end;
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"ERROR")!=NULL)
                        {
//                            GPRSEstabilishStart();                            
                            pointer_start = pointer_end;
//                            LOG("ERROR: 911");
//                            GPRSReset();
                            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_End;
                            GPRSData.GPRSFeedBackReceiveErr=TRUE;
                            
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"$MYNETWRITE")!=NULL)
                        {
                            GPRSData.GPRSFeedBackIsOK=TRUE;
                            pointer_start = pointer_end;
                            
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"+MIPSTATE: 1")!=NULL)
                        {
                            
                            dat_h=(uint8_t *)strstr((const char *)GPRSData.MsgParts,",");
                            if(dat_h[1]==dat_h[3])
                            GPRSData.GPRSRealDataIsArrived=TRUE;
                            pointer_start = pointer_end;
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"+MIPCALL: 1")!=NULL)
                        {
                            GPRSData.GPRSMIPCallIsAck=TRUE;
                            pointer_start = pointer_end;
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"$MYURCREAD")!=NULL)
                        {
                            GPRSReceiveDataStart();                            
                            pointer_start = pointer_end;

                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"+MIPOPEN: 1,1")!=NULL)
                        {
                            pointer_start = pointer_end;
                            GPRSData.GPRSEstabilisedAckIsConnected=TRUE;
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"$MYURCCLOSE")!=NULL)
                        {
//                            GPRSEstabilishStart();                            
                            pointer_start = pointer_end;
                            GPRSReset();
                            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_End;
                            
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"+MIPDATA")!=NULL)
                        {
                            dat_len=0;
                            dat_h=(uint8_t *)strstr((const char *)GPRSData.MsgParts,",")+1;
                            dat_t=(uint8_t *)strstr((const char *)dat_h,",");
                            p_len=dat_t-dat_h;
                            for(uint16_t i=0;i<p_len;i++)
                            dat_len +=((dat_h[i]-0x30)*(uint16_t)pow(10,p_len-i-1));  
                            LOGA("receive len is:%08x",dat_len);
                            LOGA("dat_h is:%08x",dat_h);
                            LOGA("dat_t is:%08x",dat_t);
                            
                            GPRSData.GPRSFeedBackIsOK=TRUE;                          
                            pointer_start = pointer_end;
                            GPRSData.GPRSRemoteDataReadyForReceive=TRUE;
                            if(GPRSData.GPRSRemoteDataReadyForReceive)
                            {
//                                PrintRecieveData(GPRSData.MsgParts,len);   
                                memset(app->Receive_data,0,sizeof(app->Receive_data));

                                memcpy(app->Receive_data,dat_t+1,dat_len);
                                for(uint16_t i=0;i<dat_len;i++)
                                LOGA("receive:%08x",app->Receive_data[i]);
                                GPRSData.GPRSRemoteDataReadyForReceive=FALSE;
                                GPRSData.GPRSRemoteDataIsReceived=TRUE;

            LOGA("GPRSData.GPRSRemoteDataIsReceived is:%08x",GPRSData.GPRSRemoteDataIsReceived);
                                


                                

                                LOG("GPRSRemoteDataIsReceived is ok!!!!!");
                                
//                                OpenUnLockStart();   
#if 0
test:
                                app->Receive_data[0]=0xaa;
                                app->Receive_data[1]=0x11;
                                app->Receive_data[2]=0x05;
                                app->Receive_data[3]=0x12;
                                app->Receive_data[4]=0x02;
                                app->Receive_data[5]=0;
                                app->Receive_data[6]=0;
                                app->Receive_data[7]=0x03;
                                app->Receive_data[8]=0;
                                app->Receive_data[9]=0x02;
                                app->Receive_data[10]=0;


                                app->Receive_data[11]=0x00;
                                app->Receive_data[12]=0;
                                app->Receive_data[13]=0x00;
                                app->Receive_data[14]=0x05;
                                app->Receive_data[15]=0xa3;
                                app->Receive_data[16]=0xc3;
                                app->Receive_data[17]=0;
                                app->Receive_data[18]=0;

                                dat_len=sizeof(GPRSCommHead_t)+sizeof(GPRSRecorderRsp_t);
                                #endif

                                if(dat_len>=sizeof(GPRSCommHead_t))
                                {
                                    uint16_t type;
                                    uint32_t crc16;
                                    uint8_t crc16_High,crc16_Low;
                                    memcpy((uint8_t *)&app->GPRSCommHeadData,app->Receive_data,sizeof(GPRSCommHead_t));
                                    type=app->GPRSCommHeadData.type[0]|app->GPRSCommHeadData.type[1]<<8;
                                    crc16_Low=app->GPRSCommHeadData.crc[0];
                                    crc16_High=app->GPRSCommHeadData.crc[1];

                                    app->GPRSCommHeadData.crc[0]=0;
                                    app->GPRSCommHeadData.crc[1]=0;

                                    memcpy(app->comm_data,(uint8_t *)&app->GPRSCommHeadData,sizeof(GPRSCommHead_t));
                                    
                                    
                                    switch(type)
                                    {
                                        case GPRSCommType_RemoteRent:
                                            if(dat_len>=sizeof(GPRSCommHead_t)+sizeof(GPRSRemoteRent_t))
                                            {
                                                app->comm_len=sizeof(app->GPRSCommHeadData)+sizeof(app->GPRSRemoteRentData);
                                                memcpy((uint8_t *)&app->comm_data+sizeof(app->GPRSCommHeadData),app->Receive_data+sizeof(app->GPRSCommHeadData),sizeof(app->GPRSRemoteRentData));
                                                crc16=get_crc16(app->comm_data,app->comm_len);
                                                if(isEqualcrc16(crc16,crc16_Low,crc16_High))
                                                {
                                                    memcpy((uint8_t *)&app->GPRSRemoteRentData,app->comm_data+sizeof(app->GPRSCommHeadData),sizeof(app->GPRSRemoteRentData));
                                                    
                                                    OpenUnLockStart();
                                                    LOG("OpenUnLockStart");
                                                    
                                                }
                                            }

                                        break;
                                        case GPRSCommType_Record:
                                            if(dat_len>=sizeof(GPRSCommHead_t)+sizeof(GPRSRecorderRsp_t))
                                            {
                                                app->comm_len=sizeof(app->GPRSCommHeadData)+sizeof(GPRSRecorderRsp_t);
                                                memcpy((uint8_t *)&app->comm_data+sizeof(app->GPRSCommHeadData),app->Receive_data+sizeof(app->GPRSCommHeadData),sizeof(app->GPRSRecorderRspData));
                                                crc16=get_crc16(app->comm_data,app->comm_len);

                                                
                                                if(isEqualcrc16(crc16,crc16_Low,crc16_High))
                                                {
                                                    memcpy((uint8_t *)&app->GPRSRecorderRspData,app->comm_data+sizeof(app->GPRSCommHeadData),sizeof(app->GPRSRecorderRspData));
                                                    app->BikeRecorderIsEnd=TRUE;
                                                    app->query_bike_record_rsp_flag=TRUE;
                                                    LOG("GPRSCommType_Record crc is ok");
                                                    
                                                }
                                                else
                                                {
                                                    LOG("GPRSCommType_Record crc is falied");
//                        PrintRecieveData(GPRSData.MsgParts,len);   

                                for(uint16_t i=0;i<Gprs_Number_of_Receive;i++)
                                LOGA("GPRSData.GprsReceiveData.dat[%04d]:%08x",i,GPRSData.GprsReceiveData.dat[i]);

                        
                                                    LOG("GPRSCommType_Record crc is falied");
//                                                    
//                        PrintRecieveData(GPRSData.GprsReceiveData.dat,Gprs_Number_of_Receive);   
//                                                    


                                                }
                                            }
                                        break;
                                    }
                                }
                            }
                            
                        }
                        else if(strstr((const char *)GPRSData.MsgParts,"AT$MYNETACK=0")!=NULL)
                        {
                            LOG("liu is OK!!!!!!");
                            pointer_start = pointer_end;
                        }
                        else
                        {
//                            if(GPRSData.GPRSRemoteDataReadyForReceive)
                        }
                        PrintRecieveData(GPRSData.MsgParts,len);   
                        memset(GPRSData.MsgParts,0,sizeof(GPRSData.MsgParts));
                }
                GPRSData.GprsReceiveData.header = (GPRSData.GprsReceiveData.header+1)%Gprs_Number_of_Receive;
            }
        break;

        case GPRSReceiveMsgParseSeq_End:

        break;
    }
}
   
bool GPRSEstablishedIsEnd(void)
{
    if(GPRSData.GPRSSeqData==GPRSSeq_End)
    return TRUE;
    return FALSE;
}

bool GPRSOutgoingIsEnd(void)
{
    if(GPRSData.GPRSOutGoingMsgSeqData==GPRSOutGoingMsgSeq_End)
    return TRUE;
    return FALSE;
}
bool GPRSIncomingIsEnd(void)
{
    if(GPRSData.GPRSIncomingMsgSeqData==GPRSIncomingMsgSeq_End)
    return TRUE;
    return FALSE;
}


void GPRSEstablishedHandle(NoBikeTaskData_t *app)
{
    uint8_t* result;
    switch(GPRSData.GPRSSeqData)
    {

        case GPRSSeq_Idle:
            GPRSData.GPRSSeqData=GPRSSeq_End;
//            GPRSEstabilishStart();
//            app->Time_Delay.GPRS_counts_1ms=0;
//            GPRSData.Restart_Times=0;
        break;
        case GPRSSeq_Init:
//            if(GPRSData.Restart_Times>Restart_MaxTimes)
//            GPRSData.Restart_Delay=delay_3min;
//            else
            GPRSData.Restart_Delay=delay_100ms;
            GPRSData.GPRSSeqData=GPRSSeq_Init_Interval;
            app->Time_Delay.GPRS_counts_1ms=0;
        break;


        case GPRSSeq_Init_Interval:
            if(app->Time_Delay.GPRS_counts_1ms>GPRSData.Restart_Delay)
            {
                GPRSReset();
                GPRSData.GPRSSeqData=GPRSSeq_PowerOn;
                app->Time_Delay.GPRS_counts_1ms=0;
            }
            break;
            
            case GPRSSeq_PowerOn:
            if(app->Time_Delay.GPRS_counts_1ms>delay_1s)
            {
                app->Time_Delay.GPRS_counts_1ms=0;
                GPRS_GPIO_Init();            
                GPRS_PWR_ON;
    //            GPRS_PWR_GPIO_DeInit();
                GPRSData.GPRSSeqData=GPRSSeq_PowerOnDelayForUartDTR;
            }
        break;

        case GPRSSeq_PowerOnDelayForUartDTR:
        if(app->Time_Delay.GPRS_counts_1ms>delay_1s)
        {
            app->Time_Delay.GPRS_counts_1ms=0;
            GPRSData.GPRSSeqData=GPRSSeq_WaitForReady;
             GPRS_DTR_ON;
             MX_USART2_UART_Init();
             GPRSData.GPRSRemoteDataIsreached=FALSE;
        }
        break;
        case GPRSSeq_WaitForReady:
        if(app->Time_Delay.GPRS_counts_1ms>delay_15s)
        {
            GPRS_DTR_OFF;
            GPRSSendDummyDatUart(); 
            GPRSTransmitLoopStart();  
            delay_ms(10);
//            GPRS_DTR_ON;
            
        }
        break;     

        case GPRSSeq_SendCmdLoop:
        if(GPRSData.counts<sizeof(GPRS_SendCmd)/sizeof(char *))
        {
            GPRSSendCmdDatUart(GPRS_SendCmd[GPRSData.counts]);
            GPRSData.counts++;
//        LOGA("GPRSData.counts:%08x",GPRSData.counts);
            GPRSData.GprsSendCmdWaitTimes=0;		
            GPRSData.GPRSSeqData=GPRSSeq_SendCmdLoopAckStrCompare;	
            app->Time_Delay.GPRS_counts_1ms=0;
        }
        else
        {

            GPRSData.GPRSSeqData=GPRSSeq_SendCmdMIPCall;	
            GPRSData.GprsSendCmdWaitTimes=0;  
            app->Time_Delay.GPRS_counts_1ms=0;
        }
        break;

        case GPRSSeq_SendCmdLoopAckStrCompare:
        if(app->Time_Delay.GPRS_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
                GPRSData.GPRSSeqData=GPRSSeq_SendCmdLoopAckParse;
                app->Time_Delay.GPRS_counts_1ms=0;

            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSSeqData=GPRSSeq_End;		
                    GPRSReset();
                }
            }
        }
        break;	


        case GPRSSeq_SendCmdLoopAckParse:
            if(app->Time_Delay.GPRS_counts_1ms>delay_100ms)
            {
                GPRSData.GPRSSeqData=GPRSSeq_SendCmdLoop;		
            }
        break;


        case GPRSSeq_SendCmdMIPCall:
//        if(app->Time_Delay.GPRS_counts_1ms>delay_2s)
        {
            
            GPRSSendCmdDatUart(GPRS_SendMIPCALL);
            GPRSData.GPRSSeqData=GPRSSeq_SendCmdMIPCallAckStrCompare;	
            app->Time_Delay.GPRS_counts_1ms=0;
            GPRSData.GprsSendCmdWaitTimes=0;   
        }
        break;

        case GPRSSeq_SendCmdMIPCallAckStrCompare:
        if(app->Time_Delay.GPRS_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK&&GPRSData.GPRSMIPCallIsAck)
            {
                GPRSData.GPRSSeqData=GPRSSeq_SendCmdMYNetSRV;	
                app->Time_Delay.GPRS_counts_1ms=0;
                
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSSeqData=GPRSSeq_End;	
                    GPRSReset();

                }
            }
        }
        break;

        


        case GPRSSeq_SendCmdMYNetSRV:
//        if(app->Time_Delay.GPRS_counts_1ms>delay_2s)
        {
            
            GPRSSendCmdDatUart(GPRS_SendMYNETSRV);
            GPRSData.GPRSSeqData=GPRSSeq_SendCmdMYNetSRVAckStrCompare;	
            app->Time_Delay.GPRS_counts_1ms=0;
            GPRSData.GprsSendCmdWaitTimes=0;   
        }
        break;

        case GPRSSeq_SendCmdMYNetSRVAckStrCompare:
        if(app->Time_Delay.GPRS_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK&&GPRSData.GPRSEstabilisedAckIsConnected)
            {
                GPRSData.GPRSSeqData=GPRSSeq_SendEnPWRSave;	
                app->Time_Delay.GPRS_counts_1ms=0;
                
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSSeqData=GPRSSeq_End;	
                    GPRSReset();

                }
            }
        }
        break;



        case GPRSSeq_SendCmdMYNetOpen:
            GPRSSendCmdDatUart(GPRS_SendMYNETOPEN);
            GPRSData.GPRSSeqData=GPRSSeq_SendCmdMYNetOpenAckStrCompare;	
            app->Time_Delay.GPRS_counts_1ms=0;
            GPRSData.GprsSendCmdWaitTimes=0;            
        break;

        case GPRSSeq_SendCmdMYNetOpenAckStrCompare:
        if(app->Time_Delay.GPRS_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
                GPRSData.GPRSSeqData=GPRSSeq_SendEnPWRSave;	
                GPRSData.GprsSendCmdWaitTimes=0;  
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSSeqData=GPRSSeq_End;	
                    GPRSReset();

                }
            }
        }
        break;
        
        case GPRSSeq_SendEnPWRSave:
//        if(app->Time_Delay.GPRS_counts_1ms>delay_2s)
        {
//            GPRSSendCmdDatUart(GPRS_SendMyNetWrite);
            GPRSData.GPRSSeqData=GPRSSeq_DTROff;	
//            while(1);
        }
//            app->Time_Delay.GPRS_counts_1ms=0;
//            GPRSData.GprsSendCmdWaitTimes=0;
        break;

        case GPRSSeq_SendEnPWRSaveStrCompare:
                GPRSData.GPRSSeqData=GPRSSeq_DTROff;	
#if 0           
        if(app->Time_Delay.GPRS_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
                GPRSData.GPRSSeqData=GPRSSeq_DTROff;	
                app->Time_Delay.GPRS_counts_1ms=0;
                
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSSeqData=GPRSSeq_End;	
                    GPRSReset();

                }
            }
        }
#endif
        break;

        case GPRSSeq_DTROff:
        if(app->Time_Delay.GPRS_counts_1ms>delay_10ms)
        {
            GPRS_DTR_ON;
            app->GprsIsConnected=TRUE;
            GPRSData.GPRSSeqData=GPRSSeq_End;	
//            GPRS_PWR_OFF;


//            while(1);

        }
        break;
        case GPRSSeq_End:     
            
        break;
    }

}

void GPRSOutgoingMsgHandle(NoBikeTaskData_t *app)
{

    switch(GPRSData.GPRSOutGoingMsgSeqData)
    {

        case GPRSOutGoingMsgSeq_Idle:
            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_End;
//            GPRSTransmitDataStart();
        break;


        case GPRSOutGoingMsgSeq_SendWriteCmd:
        if((app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_50ms)&&GPRSIncomingIsEnd()&&GPRSEstablishedIsEnd())
        {
            LoadLenForSendMyNetWrite(app);            
            GPRSSendCmdDatUart((const char*)app->GPRS_SendMyNetWriteData);
            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendWriteCmdAckStrCompare;	
        }
        break;

        case GPRSOutGoingMsgSeq_SendWriteCmdAckStrCompare:
        if(app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_1s)
        {
//            if(GPRSData.GPRSFeedBackIsOK)
            if(GPRSData.GPRSFeedBackReceiveErr)
            {
                    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_OverTime;	
            }
            else
            {


                GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendRealData;
                app->Time_Delay.GPRS_Outgoing_counts_1ms=0;    
                GPRSData.GprsSendCmdWaitTimes=0;       
                
            }
        }
        break;	

        case GPRSOutGoingMsgSeq_SendRealData:
            if(GPRSSendRealData(app->comm_data,app->comm_len))
            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendRealDataAckStrCompare;	
            else
            {
                MX_USART2_UART_Init();
            }
                
            app->Time_Delay.GPRS_Outgoing_counts_1ms=0;	

        break;

        case GPRSOutGoingMsgSeq_SendRealDataAckStrCompare:
        if(app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_500ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
                GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendAck;	
                GPRSData.GprsSendAckConfirmTimes=0;
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                    app->Time_Delay.GPRS_Outgoing_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_OverTime;	
                LOG("GPRSOutGoingMsgSeq_SendRealDataOverTime!!!!!!");
                    
                }
            }
        }
        break;	


        case GPRSOutGoingMsgSeq_SendAck:
            GPRSSendCmdDatUart(GPRS_SendMyNetACK);
                LOG("GPRS_SendMyNetACK!!!!!!");
            
            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendAckStrCompare;	
            app->Time_Delay.GPRS_Outgoing_counts_1ms=0;	
        break;

        case GPRSOutGoingMsgSeq_SendAckStrCompare:
        if(app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_1s)
        {
            if(GPRSData.GPRSRealDataIsArrived)
            {
                GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendEnPWRSave;
                GPRSData.GprsSendCmdWaitTimes=0;         
//            GPRSSendCmdDatUart(GPRS_SendMyNetRead);
            app->Time_Delay.GPRS_Outgoing_counts_1ms=0;	
            
                
            }
            else
            {
                GPRSData.GprsSendAckConfirmTimes++;
                if(GPRSData.GprsSendAckConfirmTimes<GprsSendAckWaitMaxTimes)
                {
                    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendAck;
                }
                else
                {
                    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_OverTime;	
                LOG("GPRSOutGoingMsgSeq_SendAckOverTime!!!!!!");
                    
                }
            }
        }
        break;	




        case GPRSOutGoingMsgSeq_SendEnPWRSave:
//            if(app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_300ms)
            {
//            GPRSSendCmdDatUart(GPRS_SendEnPWRSave);
//            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_SendEnPWRSaveStrCompare;	
                GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_DTROff;	
                app->Time_Delay.GPRS_Outgoing_counts_1ms=0;
            }
        break;

        case GPRSOutGoingMsgSeq_SendEnPWRSaveStrCompare:
        if(app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
                GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_DTROff;	
                app->Time_Delay.GPRS_Outgoing_counts_1ms=0;
                
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_Outgoing_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_OverTime;	
                LOG("GPRSOutGoingMsgSeq_SendEnPWRSaveOverTime!!!!!!");
                    
                }
            }
        }
        break;

        case GPRSOutGoingMsgSeq_DTROff:
        if(app->Time_Delay.GPRS_Outgoing_counts_1ms>delay_10ms)
        {
            GPRS_DTR_ON;
            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_End;		

        }
            

        break;

        case GPRSOutGoingMsgSeq_OverTime:
            GPRSData.GPRSOutGoingMsgSeqData=GPRSOutGoingMsgSeq_End;
            GPRSReset();
//                GPRSEstabilishStart();
        break;

        case GPRSOutGoingMsgSeq_End:

        break;
        
    }

}


void GPRSIncomingMsgHandle(NoBikeTaskData_t *app)
{

    switch(GPRSData.GPRSIncomingMsgSeqData)
    {

        case GPRSIncomingMsgSeq_Idle:
            GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_End;
        break;

        case GPRSIncomingMsgSeq_WaitEnd:
            if(GPRSOutgoingIsEnd()&&GPRSEstablishedIsEnd())
            {
                app->Time_Delay.GPRS_Incoming_counts_1ms=0;
                GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_DTROn;
            }
        break;

        case GPRSIncomingMsgSeq_DTROn:
         if(app->Time_Delay.GPRS_Incoming_counts_1ms>delay_3s)/*need to delay more than 3 senconds*/
         {
            GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_SendReadCmd;
            GPRS_DTR_ON;
            app->Time_Delay.GPRS_Incoming_counts_1ms=0;
            LOG("GPRSReceiveDataStart");
         }
        break;
        case GPRSIncomingMsgSeq_SendReadCmd:
        if(app->Time_Delay.GPRS_Incoming_counts_1ms>delay_50ms)
        {
            app->Time_Delay.GPRS_Incoming_counts_1ms=0;	
            GPRSSendCmdDatUart(GPRS_SendMyNetRead);
//            GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_SendReadCmdAckStrCompare;	
            GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_ReceiveDataAckCompare;	
            app->Time_Delay.GPRS_Incoming_counts_1ms=0;    
            GPRSData.GprsSendCmdWaitTimes=0;
            GPRSData.GPRSRemoteDataIsReceived=FALSE;

        }
        break;

        case GPRSIncomingMsgSeq_SendReadCmdAckStrCompare:
        if(app->Time_Delay.GPRS_Incoming_counts_1ms>delay_50ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
//                GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_ReceiveDataAckCompare;
                GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_ReceiveDataAckCompare;
                app->Time_Delay.GPRS_Incoming_counts_1ms=0;    
                GPRSData.GprsSendCmdWaitTimes=0;
                GPRSData.GPRSRemoteDataIsReceived=FALSE;
            LOG("GPRSFeedBackIsOK is received");
                
                
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_Incoming_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_OverTime;	
                    LOG("GPRSIncomingMsgSeq_SendReadCmdOverTime!!!!!!");
                }
            }
        }
        break;	



        case GPRSIncomingMsgSeq_ReceiveDataAckCompare:
        if(app->Time_Delay.GPRS_Incoming_counts_1ms>delay_100ms)
        {
            if(GPRSData.GPRSRemoteDataIsReceived)
            {
                GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_SendEnPWRSave;	
                GPRSData.GprsSendAckConfirmTimes=0;
                GPRSData.GPRSRemoteDataIsReceived=FALSE;
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                    app->Time_Delay.GPRS_Incoming_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_OverTime;	
                    LOG("GPRSIncomingMsgSeq_ReceiveDataAckCompareOverTime!!!!!!");
                }
            }
        }
        break;	

        case GPRSIncomingMsgSeq_SendEnPWRSave:
            GPRSSendCmdDatUart(GPRS_SendEnPWRSave);
            GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_SendEnPWRSaveStrCompare;	
            app->Time_Delay.GPRS_Incoming_counts_1ms=0;	
        break;

        case GPRSIncomingMsgSeq_SendEnPWRSaveStrCompare:
        if(app->Time_Delay.GPRS_Incoming_counts_1ms>delay_10ms)
        {
            if(GPRSData.GPRSFeedBackIsOK)
            {
                GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_DTROff;		
                app->Time_Delay.GPRS_Incoming_counts_1ms=0;	
            }
            else
            {
                GPRSData.GprsSendCmdWaitTimes++;
                if(GPRSData.GprsSendCmdWaitTimes<GprsSendCmdWaitMaxTimes)
                {
                     app->Time_Delay.GPRS_Incoming_counts_1ms=0;
                }
                else
                {
                    GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_OverTime;	
                    LOG("GPRSIncomingMsgSeq_SendEnPWRSaveOverTime!!!!!!");
                }
            }
        }
        break;

        case GPRSIncomingMsgSeq_DTROff:
        if(app->Time_Delay.GPRS_Incoming_counts_1ms>delay_10ms)
        {
                GPRS_DTR_OFF;
                GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_End;
        }
        break;

        case GPRSIncomingMsgSeq_OverTime:
            GPRSData.GPRSIncomingMsgSeqData=GPRSIncomingMsgSeq_End;
            GPRSReset();
            
//                GPRSEstabilishStart();
        break;
        case GPRSIncomingMsgSeq_End:
        break;
    }
}

void GPRSHeartStart(void)
{
    pNoBikeTaskData_t app = getApp();

    GPRSData.GPRSHeartMsgSeqData=GPRSTransmitMsgSeq_HeartStart;
}


bool GPRSHeartIsEnd(void)
{
    if(GPRSData.GPRSHeartMsgSeqData==GPRSTransmitMsgSeq_End)
    return TRUE;
    return FALSE;
    
}


void GPRSLoadDataTransmitStart(GPRSCommType_t type)
{
    pNoBikeTaskData_t app = getApp();


    uint32_t utctime_int;

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    UTCTime_t UTCTime;
    
    
    uint16_t crc16,len;

    if(!app->GpsIsConnected)
    {
        app->GPS_Lng=0;
        app->GPS_Lat=0;
    }

    memset(app->comm_data,0,sizeof(app->comm_data));

    app->GPRSCommHeadData.header[0]=0xAA;
    app->GPRSCommHeadData.header[1]=0x11;
    
    app->GPRSCommHeadData.bike_area_code[0]=app->GPRSConfigData.BikeArea[0];
    app->GPRSCommHeadData.bike_area_code[1]=app->GPRSConfigData.BikeArea[1];
            app->GPRSCommHeadData.bike_serial[0]=2;
//    app->GPRSCommHeadData.bike_serial[0]=app->GPRSConfigData.BikeSerial[0];
    app->GPRSCommHeadData.bike_serial[1]=app->GPRSConfigData.BikeSerial[1];
    app->GPRSCommHeadData.bike_serial[2]=app->GPRSConfigData.BikeSerial[2];
    app->GPRSCommHeadData.type[0]=type&0xFF;
    app->GPRSCommHeadData.type[1]=type>>8;
    app->GPRSCommHeadData.seq[0]=app->GPRSMsg_Seq&0xff;
    app->GPRSCommHeadData.seq[1]=app->GPRSMsg_Seq>>8;
    app->GPRSCommHeadData.crc[0]=0;
    app->GPRSCommHeadData.crc[1]=0;

    switch(type)
    {
        case GPRSCommType_Heart:
            len=sizeof(app->GPRSHeartData);

            app->GPRSHeartData.BatteryLevel=app->bat_voltage/1000 ;
            app->GPRSHeartData.GPS_Lng[0]=app->GPS_Lng&0xff;
            app->GPRSHeartData.GPS_Lng[1]=app->GPS_Lng>>8&0xff;
            app->GPRSHeartData.GPS_Lng[2]=app->GPS_Lng>>16&0xff;
            app->GPRSHeartData.GPS_Lng[3]=app->GPS_Lng>>24&0xff;
            app->GPRSHeartData.GPS_Lat[0]=app->GPS_Lat&0xff;
            app->GPRSHeartData.GPS_Lat[1]=app->GPS_Lat>>8&0xff;
            app->GPRSHeartData.GPS_Lat[2]=app->GPS_Lat>>16&0xff;
            app->GPRSHeartData.GPS_Lat[3]=app->GPS_Lat>>24&0xff;
            app->GPRSHeartData.Bike_Status=IsUnLock;

            
            app->comm_len=sizeof(app->GPRSCommHeadData)+sizeof(app->GPRSHeartData);
            memcpy(app->comm_data+sizeof(app->GPRSCommHeadData),(uint8_t *)&app->GPRSHeartData,sizeof(app->GPRSHeartData));
        break;
        case GPRSCommType_Record:
            len=sizeof(app->GPRSRecordData);


            app->GPRSCommHeadData.bike_area_code[0]=gBikeRecord.bike_area_code[0];
            app->GPRSCommHeadData.bike_area_code[1]=gBikeRecord.bike_area_code[1];
//            app->GPRSCommHeadData.bike_serial[0]=gBikeRecord.bike_serial[0];
            app->GPRSCommHeadData.bike_serial[0]=2;
            app->GPRSCommHeadData.bike_serial[1]=gBikeRecord.bike_serial[1];
            app->GPRSCommHeadData.bike_serial[2]=gBikeRecord.bike_serial[2];
            

            memcpy((uint8_t *)&app->GPRSRecordData,(uint8_t *)&gBikeRecord,sizeof(GPRSRecord_t));

            app->comm_len=sizeof(app->GPRSCommHeadData)+sizeof(app->GPRSRecordData);
            memcpy(app->comm_data+sizeof(app->GPRSCommHeadData),(uint8_t *)&app->GPRSRecordData,sizeof(app->GPRSRecordData));
            
        break;
    }
    app->GPRSCommHeadData.len[0]=len&0xff;
    app->GPRSCommHeadData.len[1]=len>>8;
    memcpy(app->comm_data,(uint8_t *)&app->GPRSCommHeadData,sizeof(app->GPRSCommHeadData));
    
    crc16=get_crc16(app->comm_data,app->comm_len);
    app->GPRSCommHeadData.crc[0]=crc16&0xff;
    app->GPRSCommHeadData.crc[1]=crc16>>8;
    memcpy(app->comm_data,(uint8_t *)&app->GPRSCommHeadData,sizeof(app->GPRSCommHeadData));
    app->GPRSMsg_Seq++;
    GPRSTransmitDataStart();
    
}

void GPRSTransimtMsgHandle(NoBikeTaskData_t *app)
{
    uint8_t test=0;

    switch(GPRSData.GPRSHeartMsgSeqData)
    {

        case GPRSTransmitMsgSeq_Idle:
            GPRSData.GPRSHeartMsgSeqData=GPRSTransmitMsgSeq_End;
        break;

        case GPRSTransmitMsgSeq_HeartStart:
                if(app->GprsIsConnected)
                {
                    GPRSData.GPRSHeartMsgSeqData=GPRSTransmitMsgSeq_ADConvert;


                    test=app->GpsStartTimes%GpsStartTimes_Interval;
                    if(app->GpsStartTimes%GpsStartTimes_Interval==0)
                    {
                        GPSStart();
                        app->GpsStartTimes=1;
                    }
                    else
                    {

                        app->GpsStartTimes++;
                    }
                }
        break;        

        case GPRSTransmitMsgSeq_ADConvert:
                app->Time_Delay.GPRS_Heart_counts_1ms=0;
                GPRSData.GPRSHeartMsgSeqData=GPRSTransmitMsgSeq_HeartSendData;
                ADConverStart();
        break;


        case GPRSTransmitMsgSeq_HeartSendData:
            if(GPRSOutgoingIsEnd()&&GPRSIncomingIsEnd()&&GPRSEstablishedIsEnd()&&ADConverIsEnd()&&GpsIsEnd())
            {
                GPRSLoadDataTransmitStart(GPRSCommType_Heart);
                app->Time_Delay.GPRS_Heart_counts_1ms=0;
                GPRSData.GPRSHeartMsgSeqData=GPRSTransmitMsgSeq_End;
            }
        break;


        case GPRSTransmitMsgSeq_End:
        break;
    }

}




GPRS_Data_t *getGPRSData(void)
{
    return &GPRSData;
}


