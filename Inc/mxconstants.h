/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/


#define MOTO_DRV0_Pin GPIO_PIN_15
#define MOTO_DRV0_GPIO_Port GPIOB
#define MOTO_DRV1_Pin GPIO_PIN_8
#define MOTO_DRV1_GPIO_Port GPIOA
//#define MOTO_nSLEEP_Pin GPIO_PIN_14
//#define MOTO_nSLEEP_GPIO_Port GPIOB



  #define MOTO_STANDBY() HAL_GPIO_WritePin(MOTO_DRV0_GPIO_Port, MOTO_DRV0_Pin, GPIO_PIN_RESET);\
                                        HAL_GPIO_WritePin(MOTO_DRV1_GPIO_Port, MOTO_DRV1_Pin, GPIO_PIN_RESET)
#if 1

  #define MOTO_REVERSE() HAL_GPIO_WritePin(MOTO_DRV0_GPIO_Port, MOTO_DRV0_Pin, GPIO_PIN_RESET);\
                                 HAL_GPIO_WritePin(MOTO_DRV1_GPIO_Port, MOTO_DRV1_Pin, GPIO_PIN_SET)

  #define MOTO_FORWARD() HAL_GPIO_WritePin(MOTO_DRV0_GPIO_Port, MOTO_DRV0_Pin, GPIO_PIN_SET);\
                                 HAL_GPIO_WritePin(MOTO_DRV1_GPIO_Port, MOTO_DRV1_Pin, GPIO_PIN_RESET)


#endif

#if 0
  #define MOTO_FORWARD() HAL_GPIO_WritePin(MOTO_DRV0_GPIO_Port, MOTO_DRV0_Pin, GPIO_PIN_RESET);\
                                 HAL_GPIO_WritePin(MOTO_DRV1_GPIO_Port, MOTO_DRV1_Pin, GPIO_PIN_SET)

  #define  MOTO_REVERSE() HAL_GPIO_WritePin(MOTO_DRV0_GPIO_Port, MOTO_DRV0_Pin, GPIO_PIN_SET);\
                                 HAL_GPIO_WritePin(MOTO_DRV1_GPIO_Port, MOTO_DRV1_Pin, GPIO_PIN_RESET)
#endif

#define GPRS_DTR_Pin GPIO_PIN_9
#define GPRS_DTR_GPIO_Port GPIOB

 #define GPRS_DTR_ON HAL_GPIO_WritePin(GPRS_DTR_GPIO_Port, GPRS_DTR_Pin, GPIO_PIN_SET)
 #define GPRS_DTR_OFF HAL_GPIO_WritePin(GPRS_DTR_GPIO_Port, GPRS_DTR_Pin, GPIO_PIN_RESET)


#define GPRS_PWR_EN_Pin GPIO_PIN_15
#define GPRS_PWR_EN_GPIO_Port GPIOA

 #define GPRS_PWR_ON HAL_GPIO_WritePin(GPRS_PWR_EN_GPIO_Port, GPRS_PWR_EN_Pin, GPIO_PIN_SET)
 #define GPRS_PWR_OFF HAL_GPIO_WritePin(GPRS_PWR_EN_GPIO_Port, GPRS_PWR_EN_Pin, GPIO_PIN_RESET)
// #define GPRS_PWR_OFF HAL_GPIO_WritePin(GPRS_PWR_EN_GPIO_Port, GPRS_PWR_EN_Pin, GPIO_PIN_SET)

 

#define MMT_INT_Pin GPIO_PIN_5
#define MMT_INT_GPIO_Port GPIOB



#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOH

#define SPI_FLASH_CS_LOW()       HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin,GPIO_PIN_RESET)
#define SPI_FLASH_CS_HIGH()      HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin,GPIO_PIN_SET)

#define FM17550_CS_Pin GPIO_PIN_4
#define FM17550_CS_GPIO_Port GPIOA


#define SPI_FM17550_CS_LOW()       HAL_GPIO_WritePin(FM17550_CS_GPIO_Port, FM17550_CS_Pin,GPIO_PIN_RESET)
#define SPI_FM17550_CS_HIGH()      HAL_GPIO_WritePin(FM17550_CS_GPIO_Port, FM17550_CS_Pin,GPIO_PIN_SET)



#define FM17550_RST_Pin GPIO_PIN_0
#define FM17550_RST_GPIO_Port GPIOB


#define SPI_FM17550_RST_LOW()       HAL_GPIO_WritePin(FM17550_RST_GPIO_Port, FM17550_RST_Pin,GPIO_PIN_RESET)
#define SPI_FM17550_RST_HIGH()      HAL_GPIO_WritePin(FM17550_RST_GPIO_Port, FM17550_RST_Pin,GPIO_PIN_SET)


//#define FLASH_WP_Pin GPIO_PIN_1
//#define FLASH_WP_GPIO_Port GPIOH
//#define SPI_FLASH_WP_LOW()       HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin,GPIO_PIN_RESET)
//#define SPI_FLASH_WP_HIGH()      HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin,GPIO_PIN_SET)


#define LED0_Pin GPIO_PIN_3
#define LED0_GPIO_Port GPIOB

#define GPRS_LED0_LOW       HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,GPIO_PIN_RESET)
#define GPRS_LED0_HIGH      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,GPIO_PIN_SET)



#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB


#define GPRS_LED1_LOW       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET)
#define GPRS_LED1_HIGH      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET)



#define GPRS_RED_EN() GPRS_LED1_HIGH;\
                                            GPRS_LED0_LOW
                                            
#define GPRS_GREEN_EN() GPRS_LED1_LOW;\
                                            GPRS_LED0_HIGH

#define GPRS_YELLOW_EN() GPRS_LED1_HIGH;\
                                            GPRS_LED0_HIGH


#define GPRS_LED_OFF() GPRS_LED1_LOW;\
                                            GPRS_LED0_LOW

#define BEEP_Pin GPIO_PIN_12
#define BEEP_GPIO_Port GPIOA
#define GPRS_BEEP_ON()      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin,GPIO_PIN_SET)
#define GPRS_BEEP_OFF()      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin,GPIO_PIN_RESET)


#define GPS_PWR_EN_Pin GPIO_PIN_11
#define GPS_PWR_EN_GPIO_Port GPIOA
#define GPS_PWR_ON()      HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin,GPIO_PIN_SET)
#define GPS_PWR_OFF()      HAL_GPIO_WritePin(GPS_PWR_EN_GPIO_Port, GPS_PWR_EN_Pin,GPIO_PIN_RESET)


#define MOTO_IS_CLOSE_Pin GPIO_PIN_13
#define MOTO_IS_CLOSE_GPIO_Port GPIOB
#define MOTO_IS_OPEN_Pin GPIO_PIN_12
#define MOTO_IS_OPEN_GPIO_Port GPIOB






#define FM17550_INT_Pin GPIO_PIN_1
#define FM17550_INT_GPIO_Port GPIOB









#define MS5611_CS_Pin GPIO_PIN_13
#define MS5611_CS_GPIO_Port GPIOC
#define PWR_AD_Pin GPIO_PIN_0
#define PWR_AD_GPIO_Port GPIOA
#define SOLAR_SENSOR_Pin GPIO_PIN_1
#define SOLAR_SENSOR_GPIO_Port GPIOA
#define GPRS_RXD_Pin GPIO_PIN_2
#define GPRS_RXD_GPIO_Port GPIOA
#define GPRS_TXD_Pin GPIO_PIN_3
#define GPRS_TXD_GPIO_Port GPIOA
#define FM1735_PWR_EN_Pin GPIO_PIN_2
#define FM1735_PWR_EN_GPIO_Port GPIOB

#define TPS63020_PG_Pin GPIO_PIN_8
#define TPS63020_PG_GPIO_Port GPIOA
#define GPS_RXD_Pin GPIO_PIN_9
#define GPS_RXD_GPIO_Port GPIOA
#define GPS_TXD_Pin GPIO_PIN_10
#define GPS_TXD_GPIO_Port GPIOA
//#define GPS_RESET_Pin GPIO_PIN_11
//#define GPS_RESET_GPIO_Port GPIOA
#define SHAKE_SENSOR_Pin GPIO_PIN_4
#define SHAKE_SENSOR_GPIO_Port GPIOB
#define GPRS_RST_Pin GPIO_PIN_8
#define GPRS_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
