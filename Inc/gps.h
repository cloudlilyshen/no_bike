#ifndef _GPS_H_
#define _GPS_H_

#include "stm32l0xx_hal.h"

#define Gps_Number_of_Receive 128
#define MMA8652_TIME_OUT 0X02
#define I2C_ADDRESS        (0x1D<<1)
#define I2C_TIME_OUT 0x50

#define VREFINT (12u)
#define VREF_COMP (0500u)

#define flash_spi_bufferWrite_try_maxtimes 1

#define GPS_AREA_LEN (4)
#define GPS_SERIAL_MIN_LEN (1)
#define GPS_MODE_MIN_LEN (1)


/* MMA8652 Register Map Offset Address */
#define CTRL_REG1			0x2A
#define CTRL_REG2			0x2B
#define CTRL_REG3			0x2C
#define CTRL_REG4			0x2D
#define CTRL_REG5			0x2E

#define TRANSIENT_CFG			0x1D
#define TRANSIENT_SRC			0x1E
#define TRANSIENT_THS			0x1F

#define TRANSIENT_COUNT			0x20

#define INT_SOURCE 0x0C
#define FlashWriteErrTimes_MAX 20

#define recorder_responsed 0x1234




enum
{
    tailer,
    header
};




typedef struct GpsReceive_s
{ 
    uint8_t dat[Gps_Number_of_Receive];
    uint8_t header;
    uint8_t tailer;    
}GpsReceive_t,*pGpsReceive_s;

typedef enum GPSSeq_e
{
    GPSSeq_Idle,
    GPSSeq_DatParseInit,
    GPSSeq_DatParse,
    GPSSeq_Standby,



    
    GPSSeq_End
}GPSSeq_t;

typedef enum ADSeq_e
{
    ADSeq_Idle,
    ADSeq_Init,
    ADSeq_ConStart,
    ADSeq_CalValue,
    ADSeq_Deinit,
    ADSeq_End
}ADSeq_t;

typedef enum SpeedSeq_e
{
    SpeedSeq_Idle,


    
    SpeedSeq_End
}SpeedSeq_t;



typedef enum LockSeq_e
{
    LockSeq_Idle,
    LockSeq_Open,
    LockSeq_WaitForClose,
    LockSeq_WaitForCloseConfirm,

    LockSeq_Delay,
    LockSeq_LockIsOpen,
    LockSeq_Stop,

    LockSeq_ProduceRecorder,
    
    LockSeq_End
}LockSeq_t;






typedef enum SendRecorder2CenterSeq_e
{
    SendRecorder2CenterSeq_Idle,

    SendRecorder2CenterSeq_InitFlash,


    SendRecorder2CenterSeq_InitFlashDelay,

    SendRecorder2CenterSeq_Start,
    
    
    SendRecorder2CenterSeq_ReadRAMHeaderTailer,
    SendRecorder2CenterSeq_ReadRAMBikeRecorder,

    SendRecorder2CenterSeq_ReadRAMCpuCardHeaderTailer,
    SendRecorder2CenterSeq_ReadRAMCpuCardRecorder,
    


    SendRecorder2CenterSeq_ReadInterFlashHeaderTailer,
    SendRecorder2CenterSeq_ReadInterFlashBikeRecorder,
    

    SendRecorder2CenterSeq_ReadInterFlashCpuCardTailer,
    SendRecorder2CenterSeq_ReadInterFlashCpuCardRecorder,

    SendRecorder2CenterSeq_ReadHeaderCpuCardTailer,
    SendRecorder2CenterSeq_ReadBikeCpuCardRecorder,    
    
    
        
    SendRecorder2CenterSeq_ReadHeaderTailer,
    SendRecorder2CenterSeq_ReadBikeRecorder,
    SendRecorder2CenterSeq_SendMsg2Center,
    SendRecorder2CenterSeq_SendMsg2Center_CpuCard,
    SendRecorder2CenterSeq_WaitForRsp,
    SendRecorder2CenterSeq_WaitForRsp_CpuCard,
    
    SendRecorder2CenterSeq_Interval,
    SendRecorder2CenterSeq_Interval_SendMsg2Center,

    
    SendRecorder2CenterSeq_End
}SendRecorder2CenterSeq_t;


/*$GPRMC,055733.000,A,3119.2966,N,12042.3364,E,0.78,73.56,280916,,,A*53\r\n*/

#if 0
typedef struct 
{
    uint8_t head;
    uint8_t type[4];
    uint8_t tpye_comma;
    
    uint8_t hour[2];
    uint8_t minute[2];
    uint8_t second[2];
    uint8_t utc_point;
    uint8_t second_decimal[3];
    uint8_t utc_comma;
    
    uint8_t status;
    uint8_t status_comma;
    
    uint8_t lat[4];
    uint8_t lat_point;
    uint8_t lat_decimal[4];
    uint8_t lat_comma;
    
    uint8_t lat_NS;
    uint8_t lat_NS_comma;
    
    uint8_t lon[5];
    uint8_t lon_point;
    uint8_t lon_decimal[4];
    uint8_t lon_comma;
    
    uint8_t lon_EW;
    uint8_t lon_EW_comma;
    
    uint8_t speed[];
    uint8_t lon_EW_comma;
    
    
    
    uint8_t bike_serial[3];
    uint8_t type[2];
    uint8_t seq[2];
    uint8_t len[2];
    uint8_t crc[2];
}GPSGPRMC_t;
#endif


typedef struct 
{
    uint8_t hour[2];
    uint8_t minute[2];
    uint8_t second[2];
    uint8_t utc_point;
    uint8_t second_decimal[3];
}GPSGPRMC_UTC_t;


typedef struct 
{
    uint8_t lat[4];
    uint8_t lat_point;
    uint8_t lat_decimal[4];
}GPSGPRMC_LAT_t;


typedef struct 
{
    uint8_t lon[5];
    uint8_t lon_point;
    uint8_t lon_decimal[4];
}GPSGPRMC_LON_t;

typedef struct 
{
    uint8_t date[2];
    uint8_t month[2];
    uint8_t year[2];
}GPSGPRMC_DATE_t;


typedef struct GPS_Data_s
{ 
    GPSSeq_t GPSSeqData:8;
    GpsReceive_t GpsReceiveData;
    char *GPS_Str_GPGSA;
    char *GPS_Str_GPRMC;
    char *GPS_Str_GPGLL;
    char *GPS_Str_GPVTG;
    char *GPS_Str_GPGSV;


    ADSeq_t ADSeqData:8;
    SpeedSeq_t SpeedSeqData:8;
    LockSeq_t LockSeqData:8;


    uint8_t InterFlashWriteErrTimes;
    uint8_t crciswongfromFlash_errtimes;

    SendRecorder2CenterSeq_t SendRecorder2CenterSeqData:8;

    uint8_t FlashWriteErrTimes;
    uint8_t MsgParts[Gps_Number_of_Receive];
    

	
}GPS_Data_t,*pGPS_Data_t;



void GPSMgrHandle(NoBikeTaskData_t *app);
GPS_Data_t *getGPSData(void);
void ADMgrHandle(NoBikeTaskData_t *app);
void SpeedMgrHandle(NoBikeTaskData_t *app);
 void parse_cmd(void);
void OpenLockStart(void);
void RentMgrHandle(NoBikeTaskData_t *app);
void OpenLockStart(void);
void OpenUnLockStart(void);
void OpenStopStart(void);
bool MMA8652_ReadReg(uint8_t add,uint8_t* dat);
void GetConfigData(void);
void Moto_GPIO_DeInit(void);
bool ADConverIsEnd(void);
bool FlashInit(void);
bool Flash_Bike_Recorder_HeaderTailer_Write(uint16_t val,uint8_t HeaderOrTailer);
void GPSStart(void);
bool GpsIsEnd(void);



#endif


