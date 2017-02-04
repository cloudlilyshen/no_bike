#ifndef _GPRS_H_
#define _GPRS_H_

#include "stm32l0xx_hal.h"

#define GprsSendCmdWaitMaxTimes 800
#define GprsSendAckWaitMaxTimes 10

#define Gprs_Check_Interval delay_500ms


#define Gprs_Number_of_Receive 256
#define UART_TIMEOUT 1000
#define Restart_MaxTimes 5
typedef struct CSQReceive_s
{ 
    uint8_t header[6];
	uint8_t inter00[2];
    uint8_t csq_value00;
    uint8_t csq_value01;
	uint8_t inter02[2];
    uint8_t dat1[2];
    uint8_t tailer[8];    
}CSQReceive_t,*pCSQReceive_t;


typedef struct GprsReceive_s
{ 
    uint8_t dat[Gprs_Number_of_Receive];
    uint16_t header;
    uint16_t tailer;    
}GprsReceive_t,*pGprsReceive_t;



typedef struct ZipSend_s
{ 
    uint8_t cmd[13];
	uint8_t len[4];
    uint8_t inter;  
}ZipSend_t,*pZipSend_t;


typedef struct ZipSendAck_s
{ 
    uint8_t cmd[13];
	uint8_t len[4];
    uint8_t inter;
    uint8_t *dat;   
}ZipSendAck_t,*pZipSendAck_t;



typedef enum GPRSOutGoingMsgSeq_e
{
    GPRSOutGoingMsgSeq_Idle,
        
    GPRSOutGoingMsgSeq_SendWriteCmd,
    GPRSOutGoingMsgSeq_SendWriteCmdAckStrCompare,
    GPRSOutGoingMsgSeq_SendDatInterval,


    GPRSOutGoingMsgSeq_SendRealData,
    GPRSOutGoingMsgSeq_SendRealDataAckStrCompare,
    


    GPRSOutGoingMsgSeq_SendAck,
    GPRSOutGoingMsgSeq_SendAckStrCompare,
    

    GPRSOutGoingMsgSeq_SendEnPWRSave,
    GPRSOutGoingMsgSeq_SendEnPWRSaveStrCompare,
    
    GPRSOutGoingMsgSeq_DTROff,

    GPRSOutGoingMsgSeq_OverTime,
    
    
    
    GPRSOutGoingMsgSeq_End
}GPRSOutGoingMsgSeq_t;



typedef enum GPRSIncomingMsgSeq_e
{
    GPRSIncomingMsgSeq_Idle,
    GPRSIncomingMsgSeq_WaitEnd,
        
    GPRSIncomingMsgSeq_DTROn,
        
    GPRSIncomingMsgSeq_SendReadCmd,
    GPRSIncomingMsgSeq_SendReadCmdAckStrCompare,
    GPRSIncomingMsgSeq_ReceiveDataAckCompare,
    GPRSIncomingMsgSeq_SendEnPWRSave,
    GPRSIncomingMsgSeq_SendEnPWRSaveStrCompare,

    GPRSIncomingMsgSeq_DTROff,

    GPRSIncomingMsgSeq_OverTime,
    

    
    
    
    GPRSIncomingMsgSeq_End
}GPRSIncomingMsgSeq_t;

typedef enum GPRSTransimtMsgSeq_e
{
    GPRSTransmitMsgSeq_Idle,
    GPRSTransmitMsgSeq_HeartStart,
    GPRSTransmitMsgSeq_ADConvert,
    GPRSTransmitMsgSeq_GpsStart,
    
    GPRSTransmitMsgSeq_HeartSendData,
        
    GPRSTransmitMsgSeq_HeartStop,

    GPRSTransmitMsgSeq_End
}GPRSTransmitMsgSeq_t;



typedef enum GPRSSeq_e
{
    GPRSSeq_Idle,
    GPRSSeq_Init,
    GPRSSeq_Init_Interval,
    GPRSSeq_PowerOn,
    GPRSSeq_PowerOnDelayForUartDTR,
    
    GPRSSeq_WaitForReady,


    GPRSSeq_SendCmdLoop,
    GPRSSeq_SendCmdLoopAckStrCompare,
    GPRSSeq_SendCmdLoopAckParse,

    GPRSSeq_SendCmdMIPCall,
    GPRSSeq_SendCmdMIPCallAckStrCompare,
    


    GPRSSeq_SendCmdMYNetSRV,
    GPRSSeq_SendCmdMYNetSRVAckStrCompare,


    GPRSSeq_SendCmdMYNetOpen,
    GPRSSeq_SendCmdMYNetOpenAckStrCompare,

    

    GPRSSeq_SendEnPWRSave,
    
    GPRSSeq_SendEnPWRSaveStrCompare,
    GPRSSeq_DTROff,
    GPRSSeq_End
}GPRSSeq_t;



typedef enum GPRSReceiveMsgParseSeq_e
{
    GPRSReceiveMsgParseSeq_Idle,
    GPRSReceiveMsgParseSeq_ParseInit,
    GPRSReceiveMsgParseSeq_ParseLoop,

    
    
    GPRSReceiveMsgParseSeq_End
}GPRSReceiveMsgParseSeq_t;


typedef struct GPRS_Data_s
{ 
    GPRSSeq_t GPRSSeqData:8;
    GPRSIncomingMsgSeq_t GPRSIncomingMsgSeqData:8;
    GPRSReceiveMsgParseSeq_t GPRSReceiveMsgParseSeqData:8;
    GPRSOutGoingMsgSeq_t GPRSOutGoingMsgSeqData:8;
    GPRSTransmitMsgSeq_t GPRSHeartMsgSeqData:8;
    
    bool isReceiveDat;
    char *GPRSSendCmdStrOK;
    char *GPRSSendCmdStrError;
    char *GPRSSendCmdStrMyNetWrite;
    char *GPRSSendCmdStrMyNetAck;
    uint8_t MsgParts[Gprs_Number_of_Receive];
    bool GPRSFeedBackIsOK;
    bool GPRSRealDataIsArrived;
    bool GPRSRemoteDataIsreached;
    bool GPRSRemoteDataReadyForReceive;
    bool GPRSMIPCallIsAck;
    
    bool GPRSRemoteDataIsReceived;
    bool GPRSEstabilisedAckIsConnected;   
    bool GPRSFeedBackReceiveErr;
    

    

    
//    char *GPRSSendCmdStrpPBREADY;
    
	
    GprsReceive_t GprsReceiveData;
    uint16_t GprsSendCmdWaitTimes;
    uint16_t GprsSendAckConfirmTimes;
    
    uint8_t CSQ_Value;
    uint16_t counts;
    uint16_t   Restart_Times;
    uint32_t   Restart_Delay;

}GPRS_Data_t,*pGPRS_Data_t;



void GPRSEstablishedHandle(NoBikeTaskData_t *app);
GPRS_Data_t *getGPRSData(void);
void GPRSRecieveMsgParse(NoBikeTaskData_t *app);
void GPRSOutgoingMsgHandle(NoBikeTaskData_t *app);
void GPRSIncomingMsgHandle(NoBikeTaskData_t *app);
bool GPRSHeartIsEnd(void);
bool GPRSEstablishedIsEnd(void);
bool GPRSOutgoingIsEnd(void);
bool GPRSIncomingIsEnd(void);
void GPRSLoadDataTransmitStart(GPRSCommType_t type);
void GPRSReset(void);












#endif


