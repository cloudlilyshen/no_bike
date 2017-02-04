#ifndef _MAIN_H_
#define _MAIN_H_
#include <string.h>
#include <stdio.h> 
#include <math.h> 

#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"
#include "stm32l0xx_hal_adc.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_hal_spi.h"
#include "stm32l0xx_hal_tim.h"

//#define Neo_M590E
#define LONGSUNG_A8300




#define PRINTF_DEBUG

#ifdef PRINTF_DEBUG

//#define LOGA(info,...)
//#define LOG(info)

#define LOGA(info,...)  do{\
                            printf("[");\
                            printf(COMPID##",");\
                            printf(info,##__VA_ARGS__);\
                            printf("]\r\n");\
                          }while(0);  

#define LOG(info)       do{\
                            printf("[");\
                            printf(COMPID##",");\
                            printf(info);\
                            printf("]\r\n");\
                          }while(0); 
#else
#define LOGA(info,...)
#define LOG(info)
#endif

#ifndef __cplusplus
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif


#define GpsStartTimes_Interval 3
#define MyNetWriteDataLen (22u)
#define  EEPROM_SIZE  (0x800)
#define  EEPROM_BASE (0x08080000)



typedef enum NoBikeProcess_e
{
    NoBike_PoweringOff,   
    NoBike_Initialising,       
    returnBike_Process,    
    rentBike_Process,    
    NoBike_Error,       
    NoBikeStateEnd    
} NoBikeProcess_t;


typedef enum NoBikeMode_e
{
    NoBikeMode_Testing,   
    NoBikeMode_Standby,       
    NoBikeMode_Normal,       
    NoBikeMode_End       
} NoBikeMode_t;



// To be used with 
typedef struct
{
  uint8_t seconds;  // 0-59
  uint8_t minutes;  // 0-59
  uint8_t hours;     // 0-23
  uint8_t date;      // 0-30
  uint8_t month;    // 0-11
  uint8_t reserved;
  uint16_t year;    // 2000+
} UTCTime_t;



//little-endian,4bytes

typedef struct 
{
    
    uint8_t  BikeArea[2];
    uint8_t  BikeSerial[3];
    uint8_t  System_mode;//6


    uint8_t  IP0[3];
    uint8_t  IP1[3];
    uint8_t  IP2[3];
    uint8_t  IP3[3];

    uint8_t Port[5];

    uint8_t Map[3];
    
    
    uint8_t  Crc16_low;
    uint8_t  Crc16_high;
}GPRSConfig_t;


typedef enum GPRSCommType_e
{
    GPRSCommType_Heart=0x0001,
    GPRSCommType_Alarm,
    GPRSCommType_Record,


    GPRSCommType_RemoteRent=0x0006,
    
    GPRSCommType_End
}GPRSCommType_t;



enum 
{
    IsLock,
    IsUnLock
    
};


typedef struct 
{
    union
    {
        struct
        {
            uint8_t dat_low;
            uint8_t dat_high;
            uint8_t crc_low;
            uint8_t crc_high;
        }BITS;
        uint32_t WORD;
    }Id; 
}NoBikeConfig_t;




typedef struct 
{
    uint32_t id0;
    uint32_t id1;
    uint32_t id2;
    
 
}UniqeID_t;
//15
typedef struct 
{
    uint8_t header[2];
    uint8_t bike_area_code[2];
    uint8_t bike_serial[3];
    uint8_t type[2];
    uint8_t seq[2];
    uint8_t len[2];
    uint8_t crc[2];
}GPRSCommHead_t;

//14
typedef struct 
{
    uint8_t BatteryLevel;
    uint8_t GPS_Lng[4];
    uint8_t GPS_Lat[4];
    uint8_t Bike_Status;
}GPRSHeart_t;


typedef struct 
{
    uint8_t Card_ID[4];
    uint8_t Time[4];
    uint8_t GPS_Lng[4];
    uint8_t GPS_Lat[4];
    uint8_t RentType;
}GPRSRecord_t;



typedef enum
{
    no_action,
    rent_bike=0x32,
    return_bike,

    Bike_Action_End
}Bike_Action_t;



/*aligned to 32 */
typedef struct
{
    uint8_t Card_ID[4];
    uint8_t Time[4];
    uint8_t GPS_Lng[4];
    uint8_t GPS_Lat[4];
    uint8_t RentType;
    uint8_t bike_area_code[2];
    uint8_t bike_serial[3];
    uint8_t seq_no[2];        
    uint8_t crc16_low;    
    uint8_t crc16_high;    
    uint16_t response;
    uint8_t map[4];    
}Bike_Record_t,*pBike_Record_t;



typedef struct
{
    uint16_t     header;    
    uint16_t     crc16;    
}Bike_Record_header_t;


typedef struct
{
    uint16_t     tailer;    
    uint16_t     crc16;    
}Bike_Record_tailer_t;




typedef struct 
{
    uint8_t Card[4];
}GPRSRemoteRent_t;

typedef struct 
{
    uint8_t Seq_no[2];
}GPRSRecorderRsp_t;



typedef struct
{
    bool Time_1ms_Mark :1; 
    bool Time_10ms_Mark :1; 
    bool Time_100ms_Mark :1;
    bool Time_1s_Mark :1; 
    bool Time_10s_Mark :1; 
    bool Time_60s_Mark :1; 
} Time_intMark_t;


#define no_delay 0
#define delay_1ms 1
#define delay_2ms 2
#define delay_3ms 3
#define delay_4ms 4
#define delay_5ms 5
#define delay_10ms 10
#define delay_15ms 15
#define delay_20ms 20
#define delay_30ms 30

#define delay_40ms 40
#define delay_50ms 50
#define delay_60ms 50
#define delay_70ms 50

#define delay_80ms 80
#define delay_90ms 90

#define delay_100ms 100
#define delay_120ms 120
#define delay_150ms 150
#define delay_200ms 200
#define delay_250ms 250
#define delay_300ms 300

#define delay_350ms 350

#define delay_400ms 400
#define delay_500ms 500
#define delay_600ms 600

#define delay_700ms 700

#define delay_800ms 800

#define delay_1s 1000
#define delay_2s 2000
#define delay_3s 3000
#define delay_4s 4000
#define delay_5s 5000
#define delay_7s 7000
#define delay_8s 8000

#define delay_10s 10000
#define delay_15s 15000

#define delay_20s 20000
#define delay_30s 30000
#define delay_40s 40000

#define delay_1min (60000*1)
#define delay_1_5min (60000*1.5)
#define delay_2min (60000*2)
#define delay_3min (60000*3)
#define delay_4min (60000*4)
#define delay_5min (60000*5)
#define delay_10min (60000*10)
#define delay_30min (60000*30)

#define SPI_TIMEOUT (1000u)


typedef struct
{
    uint32_t           GPRS_counts_1ms;
    uint32_t           GPS_counts_1ms;
    uint16_t           AD_counts_1ms;
    uint16_t           Speed_counts_1ms;
    uint16_t           GPRS_Outgoing_counts_1ms;
    uint32_t           GPRS_Heart_counts_1ms;
    uint16_t           GPRS_Incoming_counts_1ms;
    uint16_t           Lock_counts_1ms;
    uint16_t           SendRecorder2Center_TIMEOUT_counts_1ms;
    uint32_t           GPRS_Nobike_counts_1ms;
    uint16_t           FM17550_TIMEOUT_counts_1ms;
}Timer_1ms_Counts_t;




typedef enum GPRSNoBikeMsgSeq_e
{
    GPRSNoBikeMsgSeq_Idle,
    GPRSNoBikeMsgSeq_ModeSelect,

    GPRSNoBikeMsgSeq_GPRSConncetStart,
    GPRSNoBikeMsgSeq_GPRSConncetStartDelay,

    GPRSNoBikeMsgSeq_GPRSConncetConfirm,
    GPRSNoBikeMsgSeq_GPSConnectedStart,
    
    GPRSNoBikeMsgSeq_GPRSHeart,
    GPRSNoBikeMsgSeq_GPRSHeartWaitEnd,
    GPRSNoBikeMsgSeq_GPRSHeartInterval,




    GPRSNoBikeMsgSeq_End
}GPRSNoBikeMsgSeq_t;


typedef struct GpsTaskData_s
{
    uint16_t test;

    uint16_t 			     g_intTACount_1ms;
    Time_intMark_t          Time_intMark;
    Timer_1ms_Counts_t Time_Delay;    
    uint8_t liu;
    uint8_t liuyun;
    uint32_t uwADCxConvertedValue[3];
    NoBikeConfig_t NoBikeConfig;
    UniqeID_t uniqe_id;
    uint32_t crc;
    
    uint32_t VrefInt_VAL;
    bool ADIsFinished;
    bool GprsIsConnected;
    uint32_t bat_voltage;

    uint8_t comm_data[128];
    uint16_t comm_len;
    uint8_t Receive_data[20];
    GPRSCommHead_t GPRSCommHeadData;
    GPRSHeart_t GPRSHeartData;
    GPRSRecord_t GPRSRecordData;

    uint8_t GPRS_SendMyNetWriteData[MyNetWriteDataLen];

    GPRSConfig_t GPRSConfigData;
    uint16_t GPRSMsg_Seq;


    uint32_t GPS_Lng;
    uint32_t GPS_Lat;


    GPRSCommType_t GPRSCommTypeData;

    GPRSRemoteRent_t GPRSRemoteRentData;

    GPRSRecorderRsp_t GPRSRecorderRspData;

    Bike_Record_header_t Flash_Bike_Recorder_Header;
    Bike_Record_tailer_t  Flash_Bike_Recorder_Tailer;



    bool query_bike_record_rsp_flag;
    bool BikeRecorderIsEnd;

    bool HeartIsStop;
    NoBikeProcess_t NoBikeProcess;


    GPRSNoBikeMsgSeq_t GPRSNoBikeMsgSeqData;
    NoBikeMode_t NoBikeMode;


    uint8_t GprsReConnectTimes;

    bool RecorderIsSending;
//    uint16_t GpsFailedTimes;
    bool GpsIsConnected;    
    uint8_t GpsStartTimes;

} NoBikeTaskData_t, *pNoBikeTaskData_t;

NoBikeTaskData_t *getApp(void);
void Error_Handler(void);
void MX_ADC_Init(void);
void MX_ADC_DeInit(void);
void MX_DMA_Init(void); 
void ADConverStart(void);
void MX_USART2_UART_Init(void);
void Moto_GPIO_Init(void);
void Moto_GPIO_DeInit(void);
void MX_SPI1_Init(void);
void MX_SPI1_DeInit(void);
void GPRS_GPIO_Init(void);
void GPRS_GPIO_DeInit(void);
void GPS_GPIO_Init(void);
void GPS_GPIO_DeInit(void);
void MX_USART1_UART_Init(void);
void Switch_GPIO_Init(void);
void Switch_GPIO_DeInit(void);
void BEEP_GPIO_Init(void);
void BEEP_GPIO_DeInit(void);
void FM17550_GPIO_Init(void);

#endif




