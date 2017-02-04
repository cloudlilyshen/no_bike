/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "main.h"
#include "gprs.h"
#include "gps.h"



/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;

CRC_HandleTypeDef hcrc;
I2C_HandleTypeDef hi2c1;


IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* Timer handler declaration */
TIM_HandleTypeDef htim21;
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


WWDG_HandleTypeDef hwwdg;
static void MX_I2C1_Init(void);


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define COMPID "main"
static NoBikeTaskData_t app;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemPower_Config(void);

void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_WWDG_Init(void);

static void NoBikeMgrHandle(void);


/* USER CODE BEGIN PFP */
static void MX_PVD_Init(void);
static void MX_TIM21_Init(void);

/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void TimerCountsHandler(void)
{

    if(app.Time_intMark.Time_1ms_Mark)
    {
        app.Time_intMark.Time_1ms_Mark=FALSE;
        app.Time_Delay.GPRS_counts_1ms++;
        app.Time_Delay.GPS_counts_1ms++;
        app.Time_Delay.AD_counts_1ms++;
        app.Time_Delay.Speed_counts_1ms++;
        app.Time_Delay.GPRS_Outgoing_counts_1ms++;
        app.Time_Delay.GPRS_Heart_counts_1ms++;
        app.Time_Delay.GPRS_Incoming_counts_1ms++;
        app.Time_Delay.Lock_counts_1ms++;
        app.Time_Delay.SendRecorder2Center_TIMEOUT_counts_1ms++;
        app.Time_Delay.GPRS_Nobike_counts_1ms++;
        app.Time_Delay.FM17550_TIMEOUT_counts_1ms++;

        
        
//        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);     

    }
    

}


static void Para_Init(void)
{
    app.GpsStartTimes=GpsStartTimes_Interval;
    GetConfigData();
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  SystemPower_Config();
  

  /* Initialize all configured peripherals */

  MX_GPIO_Init();
//  MX_DMA_Init();

//  MX_ADC_Init();

//  MX_COMP1_Init();
//  MX_COMP2_Init();
//  MX_CRC_Init();
//  MX_IWDG_Init();
  MX_LPUART1_UART_Init();

//  MX_RNG_Init();
  MX_RTC_Init();
//  MX_WWDG_Init();
//  MX_PVD_Init();
  MX_I2C1_Init();

  Para_Init();
  //BEEP_GPIO_Init();


  Moto_GPIO_Init();

  MX_TIM21_Init();


  



  

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    TimerCountsHandler();
      #if 1
    RentMgrHandle(&app);
  
    GPRSEstablishedHandle(&app);

    GPRSRecieveMsgParse(&app);
    
    GPRSOutgoingMsgHandle(&app);    
    GPRSTransimtMsgHandle(&app);
    GPRSIncomingMsgHandle(&app);    
//  LOG("MX_LPUART1_UART_Init");
    ADMgrHandle(&app);
    SpeedMgrHandle(&app); 
//    sendRecord2CenterFromFlashHandle(&app);
    #endif
    GPSMgrHandle(&app); 
    NoBikeMgrHandle();
//    FM17550MgrHandle(&app);
  }
  /* USER CODE END 3 */
}

void NoBikeMgrHandle(void)
{
    switch(app.GPRSNoBikeMsgSeqData)
    {
        case GPRSNoBikeMsgSeq_Idle:
            app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_ModeSelect;


        break;

        case GPRSNoBikeMsgSeq_ModeSelect:
                app.NoBikeMode=NoBikeMode_Normal;
                switch(app.NoBikeMode)
                {
                    case NoBikeMode_Testing:


                    break;

                    case NoBikeMode_Standby:


                    break;
                    case NoBikeMode_Normal:
                        app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetStart;
                        app.GprsReConnectTimes=0;
                        
                    break;
                    default:
                    break;
                }
        break;

        case GPRSNoBikeMsgSeq_GPRSConncetStart:
            if(GPRSEstablishedIsEnd())
            {
                GPRSEstabilishStart();
                app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetConfirm;
//                    app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_End;

                
            }
        break;

        case GPRSNoBikeMsgSeq_GPRSConncetConfirm:
            if(GPRSEstablishedIsEnd())
            {
                if(app.GprsIsConnected)
                {
                    app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSHeart;
//                    app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_End;
                    app.GprsReConnectTimes=0;
                }
                else
                {
                    if(app.GprsReConnectTimes++<3)
                    app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetStart;
                    else
                    {
                        app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetStartDelay;     
                        app.Time_Delay.GPRS_Nobike_counts_1ms=0;
                    }
                }
            }
        break;


        case GPRSNoBikeMsgSeq_GPRSConncetStartDelay:
            if(app.Time_Delay.GPRS_Nobike_counts_1ms>delay_10min)
            app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetStart;
        break;

        case GPRSNoBikeMsgSeq_GPRSHeart:
            if(GPRSHeartIsEnd()&&!app.RecorderIsSending)
            {
                if(app.GprsIsConnected)
                {
                    GPRSHeartStart();
                    app.Time_Delay.GPRS_Nobike_counts_1ms=0;
                    app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSHeartWaitEnd;
                }
                else
                {
                        app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetStart;
                        app.GprsReConnectTimes=0;
                }
            }
        break;

        case GPRSNoBikeMsgSeq_GPRSHeartWaitEnd:
            if(GPRSHeartIsEnd())
            {
                if(app.GprsIsConnected)
                {
                
                app.Time_Delay.GPRS_Nobike_counts_1ms=0;
                app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSHeartInterval;
                }
                else
                {
                        app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSConncetStart;
                        app.GprsReConnectTimes=0;
                }
                
            }
        break;
        case GPRSNoBikeMsgSeq_GPRSHeartInterval:
            if(app.Time_Delay.GPRS_Nobike_counts_1ms>delay_3min)
            {
                app.GPRSNoBikeMsgSeqData=GPRSNoBikeMsgSeq_GPRSHeart;
            }
        break;

        case GPRSNoBikeMsgSeq_End:


        break;

        default:


        break;



    }





}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
//                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
//  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
//                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
//                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;

  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  #if 1
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #endif

}

void MX_ADC_DeInit(void)
{
  HAL_ADC_DeInit(&hadc);
  HAL_ADC_MspDeInit(&hadc);  
}


/* COMP1 init function */
static void MX_COMP1_Init(void)
{

  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_VREFINT;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.LPTIMConnection = COMP_LPTIMCONNECTION_DISABLED;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Mode = COMP_POWERMODE_ULTRALOWPOWER;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_COMPEx_EnableVREFINT();

}

/* COMP2 init function */
static void MX_COMP2_Init(void)
{

  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_VREFINT;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.LPTIMConnection = COMP_LPTIMCONNECTION_DISABLED;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_COMPEx_EnableVREFINT();

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  
  }

}


/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* LPUART1 init function */
static void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
   __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);
  LOG("MX_LPUART1_UART_Init");

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
   __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  #ifdef Neo_M590E
  huart2.Init.BaudRate = 9600;
  #elif defined(LONGSUNG_A8300)
  huart2.Init.BaudRate = 115200;
  #endif
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
   __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

}

/* RNG init function */
static void MX_RNG_Init(void)
{

  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }

}


/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 5;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 160;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_RemapConfig(&htim21, TIM21_ETR_COMP2_OUT) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim21);

  if(HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  

}


/* RTC init function */
static void MX_RTC_Init(void)
{


  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

#if 0
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm B 
    */
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_B;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the WakeUp 
    */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

static void Flash_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
    
  /*Configure GPIO pins : FLASH_CS_Pin FLASH_WP_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);
}




static void Flash_GPIO_DeInit(void)
{
  HAL_GPIO_DeInit(FLASH_CS_GPIO_Port,FLASH_CS_Pin);
}


/* SPI1 init function */
void MX_SPI1_Init(void)
{
//  Flash_GPIO_Init();
//  FM17550_GPIO_Init();
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* SPI1 init function */
void MX_SPI1_DeInit(void)
{
  Flash_GPIO_DeInit();
  HAL_SPI_DeInit(&hspi1);
}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}


void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */

    /* Peripheral DMA init*/
  
    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Request = DMA_REQUEST_0;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode = DMA_NORMAL;
    hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc);
   

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);


}
/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow : 
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Wakeup using EXTI Line (Key Button PC.13)
  * @param  None
  * @retval None
  */
static void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Ultra low power mode */
//  HAL_PWREx_EnableUltraLowPower();
  #if 0
  /* Enable the fast wake up from Ultra low power mode */
//  HAL_PWREx_EnableFastWakeUp();

  /* Select HSI as system clock source after Wake Up from Stop mode */
//  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_StopWakeUpClock_HSI);
  
  /* Enable GPIOs clock */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();

  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* Disable GPIOs clock */
  __GPIOA_CLK_DISABLE();
  __GPIOB_CLK_DISABLE();
  __GPIOC_CLK_DISABLE();
  __GPIOD_CLK_DISABLE();
  __GPIOH_CLK_DISABLE();
  #endif

}




void Moto_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
    
  GPIO_InitStruct.Pin = MOTO_DRV0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTO_DRV0_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTO_DRV1_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTO_DRV1_GPIO_Port, &GPIO_InitStruct);
}


void Moto_GPIO_DeInit(void)
{
//    HAL_GPIO_DeInit(MOTO_DRV0_GPIO_Port,MOTO_DRV0_Pin);
//    HAL_GPIO_DeInit(MOTO_DRV1_GPIO_Port,MOTO_DRV1_Pin);
}


void GPRS_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPRS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPRS_PWR_EN_GPIO_Port, &GPIO_InitStruct);
//  GPRS_PWR_OFF;

  GPIO_InitStruct.Pin = GPRS_DTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPRS_DTR_GPIO_Port, &GPIO_InitStruct);
  
}

void GPRS_GPIO_DeInit(void)
{
    HAL_GPIO_DeInit(GPRS_PWR_EN_GPIO_Port,GPRS_PWR_EN_Pin);
    HAL_GPIO_DeInit(GPRS_DTR_GPIO_Port,GPRS_DTR_Pin);
}



void GPS_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_PWR_EN_GPIO_Port, &GPIO_InitStruct);
}

void GPS_GPIO_DeInit(void)
{
    HAL_GPIO_DeInit(GPS_PWR_EN_GPIO_Port,GPS_PWR_EN_Pin);
}

void BEEP_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);
  GPRS_BEEP_ON();
}

void BEEP_GPIO_DeInit(void)
{
    HAL_GPIO_DeInit(BEEP_GPIO_Port,BEEP_Pin);
}


void Switch_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pins : TPS63020_PG_Pin MOTO_IS_CLOSE_Pin */
  GPIO_InitStruct.Pin = MOTO_IS_OPEN_Pin|MOTO_IS_CLOSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTO_IS_OPEN_GPIO_Port, &GPIO_InitStruct);
}

void Switch_GPIO_DeInit(void)
{
    HAL_GPIO_DeInit(MOTO_IS_OPEN_GPIO_Port,MOTO_IS_OPEN_Pin);
    HAL_GPIO_DeInit(MOTO_IS_CLOSE_GPIO_Port,MOTO_IS_CLOSE_Pin);
}

void FM17550_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : FM17550_INT_Pin */
  GPIO_InitStruct.Pin = FM17550_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FM17550_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FM17550_RST_Pin */
  GPIO_InitStruct.Pin = FM17550_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FM17550_RST_GPIO_Port, &GPIO_InitStruct);



  /*Configure GPIO pin : FM17550_CS_Pin */
  GPIO_InitStruct.Pin = FM17550_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FM17550_CS_GPIO_Port, &GPIO_InitStruct);
  
}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  

#if 0  
  /*Configure GPIO pin : MMT_INT_Pin */
  GPIO_InitStruct.Pin = MMT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MMT_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);


  /*Configure GPIO pins : FLASH_CS_Pin FLASH_WP_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|FLASH_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  
  
  /*Configure GPIO pins : FM1735_PWR_EN_Pin LED0_Pin LED1_Pin BEEP_Pin 
                           GPRS_PWR_EN_Pin MOTO_DRV0_Pin MOTO_DRV1_Pin MOTO_nSLEEP_Pin 
                           GPRS_DTR_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif


#if 0
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, FLASH_CS_Pin|FLASH_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FM17550_CS_Pin|GPS_RESET_Pin|GPS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FM17550_RST_Pin|FM1735_PWR_EN_Pin|LED0_Pin|LED1_Pin 
                          |BEEP_Pin|GPRS_PWR_EN_Pin|MOTO_DRV0_Pin|MOTO_DRV1_Pin 
                          |MOTO_nSLEEP_Pin|GPRS_RST_Pin|GPRS_DTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MS5611_CS_Pin */
  GPIO_InitStruct.Pin = MS5611_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MS5611_CS_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pins : FM17550_CS_Pin GPS_RESET_Pin GPS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = FM17550_CS_Pin|GPS_RESET_Pin|GPS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*Configure GPIO pins : FM1735_PWR_EN_Pin LED0_Pin LED1_Pin BEEP_Pin 
                           GPRS_PWR_EN_Pin MOTO_DRV0_Pin MOTO_DRV1_Pin MOTO_nSLEEP_Pin 
                           GPRS_DTR_Pin */
  GPIO_InitStruct.Pin = FM1735_PWR_EN_Pin|LED0_Pin|LED1_Pin|BEEP_Pin 
                          |GPRS_PWR_EN_Pin|MOTO_DRV0_Pin|MOTO_DRV1_Pin|MOTO_nSLEEP_Pin 
                          |GPRS_DTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TPS63020_PG_Pin MOTO_IS_CLOSE_Pin */
  GPIO_InitStruct.Pin = TPS63020_PG_Pin|MOTO_IS_CLOSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTO_IS_OPEN_Pin */
  GPIO_InitStruct.Pin = MOTO_IS_OPEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTO_IS_OPEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPRS_RST_Pin */
  GPIO_InitStruct.Pin = GPRS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPRS_RST_GPIO_Port, &GPIO_InitStruct);
  #endif

}

/* USER CODE BEGIN 4 */


/* PVD init function */
static void MX_PVD_Init(void)
{
    PWR_PVDTypeDef PWR_PVDStruct;
    HAL_PWR_DeInit();   
    PWR_PVDStruct.PVDLevel=PWR_PVDLEVEL_0;
    PWR_PVDStruct.Mode=PWR_PVD_MODE_NORMAL;
    HAL_PWR_ConfigPVD(&PWR_PVDStruct);
    HAL_PWR_EnablePVD();
}




NoBikeTaskData_t *getApp(void)
{
    return &app;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
     LOG("Error_Handler");
    
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

