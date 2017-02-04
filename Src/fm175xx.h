#ifndef FM175X_H
#define FM175X_H

#define Anticollision   0x02

#define CommandReg    0x01
#define ComIEnReg    0x02
#define DivIEnReg    0x03
#define ComIrqReg    0x04
#define DivIrqReg    0x05
#define ErrorReg    0x06
#define Status1Reg    0x07
#define Status2Reg    0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg    0x0C
#define BitFramingReg 0x0D
#define CollReg    0x0E
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxAutoReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define MfTxReg 0x1C
#define MfRxReg 0x1D
#define SerialSpeedReg 0x1F
#define CRCMSBReg 0x21
#define CRCLSBReg 0x22
#define ModWidthReg 0x24
#define GsNOffReg 0x23
#define TxBitPhaseReg 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsPReg 0x28
#define ModGsPReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadMSBReg 0x2C
#define TReloadLSBReg 0x2D
#define TCounterValMSBReg 0x2E
#define TCounterValLSBReg 0x2F
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B

#define Idle    0x00
#define Mem        0x01
#define RandomID    0x02
#define CalcCRC    0x03
#define Transmit    0x04
#define NoCmdChange    0x07
#define Receive    0x08
#define Transceive    0x0C
#define MFAuthent    0x0E
#define SoftReset    0x0F

#define  COMMIEN_DEF          0x80
#define  DIVIEN_DEF           0x80
#define IF_ERR_THEN_RETURN if (ret == FALSE) return FALSE
#define ADC_REFERNCE_MIN 0x0                                            /* ʵ�ʲ�������ֵ��С           */
#define ADC_REFERNCE_MAX 0x7F                                           /* ʵ�ʵ���ֵ���               */


//============================================================================
#define        JREG_EXT_REG_ENTRANCE        0x0F    //ext register entrance
//============================================================================
#define        JBIT_EXT_REG_WR_ADDR        0X40    //wrire address cycle
#define        JBIT_EXT_REG_RD_ADDR        0X80    //read address cycle
#define        JBIT_EXT_REG_WR_DATA        0XC0    //write data cycle
#define        JBIT_EXT_REG_RD_DATA        0X00    //read data cycle


//============================================================================
#define        JREG_LVD_CTRL                0x1D    //Low Votage Detect register
//============================================================================

//============================================================================
#define        JREG_LPCD_CTRL1                0x01    //Lpcd Ctrl register1
//============================================================================
#define        JBIT_LPCD_EN                0x01    //enble LPCD
#define        JBIT_LPCD_RSTN                0X02    //lpcd reset
#define        JBIT_LPCD_CALIBRA_EN        0x04    //into lpcd calibra mode
#define        JBIT_LPCD_CMP_1                0x08    //Compare times 1 or 3
#define        JBIT_LPCD_CMP_3                0x13    //Compare times 1 or 3
#define        JBIT_LPCD_IE                0x10    //Enable LPCD IE
#define        JBIT_BIT_CTRL_SET            0x20    //Lpcd register Bit ctrl set bit
#define        JBIT_BIT_CTRL_CLR            0x00    //Lpcd register Bit ctrl clear bit
//============================================================================

//============================================================================
#define        JREG_LPCD_CTRL2                0x02    //Lpcd Ctrl register2
//============================================================================

//============================================================================
#define        JREG_LPCD_CTRL3                0x03    //Lpcd Ctrl register3
//============================================================================

//============================================================================

//============================================================================
#define        JREG_LPCD_CTRL4                0x04    //Lpcd Ctrl register4
//============================================================================

//============================================================================

//============================================================================
#define        JREG_LPCD_BIAS_CURRENT        0x05    //Lpcd bias current register
//============================================================================

//============================================================================
#define        JREG_LPCD_ADC_REFERECE        0x06    //Lpcd adc reference register 
//============================================================================

//============================================================================
#define        JREG_LPCD_T1CFG                0x07    //T1Cfg[3:0] register 
//============================================================================

//============================================================================
#define        JREG_LPCD_T2CFG                0x08    //T2Cfg[4:0] register 
//============================================================================

//============================================================================
#define        JREG_LPCD_T3CFG                0x09    //T2Cfg[4:0] register 
//============================================================================

//============================================================================
#define        JREG_LPCD_VMIDBD_CFG        0x0A    //VmidBdCfg[4:0] register 
//============================================================================

//============================================================================
#define        JREG_LPCD_AUTO_WUP_CFG            0x0B    //Auto_Wup_Cfg register 
//============================================================================

//============================================================================
#define        JREG_LPCD_ADC_RESULT_L            0x0C    //ADCResult[5:0] Register 
//============================================================================

//============================================================================
#define        JREG_LPCD_ADC_RESULT_H            0x0D    //ADCResult[7:6] Register 
//============================================================================

//============================================================================
#define        JREG_LPCD_THRESHOLD_MIN_L        0x0E    //LpcdThreshold_L[5:0] Register 
//============================================================================

//============================================================================
#define        JREG_LPCD_THRESHOLD_MIN_H        0x0F    //LpcdThreshold_L[7:6] Register 
//============================================================================

//============================================================================
#define        JREG_LPCD_THRESHOLD_MAX_L        0x10    //LpcdThreshold_H[5:0] Register 
//============================================================================

//============================================================================
#define        JREG_LPCD_THRESHOLD_MAX_H        0x11    //LpcdThreshold_H[7:6] Register 
//============================================================================

//============================================================================
#define        JREG_LPCD_IRQ                0x12    //LpcdStatus Register 
//============================================================================
#define        JBIT_CARD_IN_IRQ            0x01    //irq of card in
#define        JBIT_LPCD23_IRQ                0x02    //irq of LPCD 23 end
#define        JBIT_CALIB_IRQ                0x04    //irq of calib end
#define        JBIT_LP10K_TESTOK_IRQ        0x08    //irq of lp osc 10K ok
#define        JBIT_AUTO_WUP_IRQ            0x10    //irq of Auto wake up 
//============================================================================

//============================================================================
#define        JREG_LPCD_RFT1                0x13    //Aux1 select Register 
//============================================================================
//#define        BFL_JBIT_AUX1_CLOSE                0x00    //close aux1 out
//#define        BFL_JBIT_AUX1_VDEM_LPCD            0x01    //vdem of lpcd
//#define        BFL_JBIT_AUX1_VP_LPCD            0x02    //voltage of charecap
//============================================================================

//============================================================================
#define        JREG_LPCD_RFT2                0x14    //Aux2 select Register 
//============================================================================
//#define        BFL_JBIT_AUX2_CLOSE                0x00    //close aux1 out
//#define        BFL_JBIT_AUX2_VDEM_LPCD            0x01    //vdem of lpcd
//#define        BFL_JBIT_AUX2_VP_LPCD            0x02    //voltage of charecap
//============================================================================

//============================================================================
#define        JREG_LPCD_RFT3                0x15    //LPCD test1 Register 
//============================================================================
//#define        BFL_JBIT_LP_OSC10K_EN            0x01    //enable lp osc10k
//#define        BFL_JBIT_LP_OSC_CALIBRA_EN        0x02    //enable lp osc10k calibra mode
//#define        BFL_JBIT_LP_CURR_TEST            0x04    //enable lp t1 current test
//#define        BFL_JBIT_LPCD_TEST2_LPCD_OUT    0x08    //lpcd_test2[3]:LPCD_OUT 
//============================================================================

//============================================================================
#define        JREG_LPCD_RFT4                0x16    //LPCD test2 Register 
//============================================================================
//#define        BFL_JBIT_T1_OUT_EN                0x01    //D5:T1_OUT
//#define        BFL_JBIT_OSCCLK_OUT_EN            0x02    //D4:OSC_CLK_OUT
//#define        BFL_JBIT_OSCEN_OUT_EN            0x04    //D3:OSC_EN
//#define        BFL_JBIT_LP_CLK_LPCD_OUT_EN        0x08    //D2:LP_CLK or LPCD_OUT
//#define        BFL_JBIT_T3_OUT_EN                0x10    //D1:T3_OUT
//============================================================================        

//============================================================================
//#define        BFL_JREG_LP_CLK_CNT1            0x17    //lp_clk_cnt[5:0] Register 
//============================================================================

//============================================================================
//#define        BFL_JREG_LP_CLK_CNT2            0x18    //lp_clk_cnt[7:6] Register 
//============================================================================

//============================================================================
//#define        BFL_JREG_VERSIONREG2            0x19    //VersionReg2[1:0] Register 
//============================================================================

//============================================================================
//#define        BFL_JREG_IRQ_BAK                0x1A    //Irq bak Register 
//============================================================================
//#define        BFL_JBIT_IRQ_INV_BAK            0x01    //Irq Inv backup
//#define        BFL_JBIT_IRQ_PUSHPULL_BAK        0x02    //Irq pushpull backup
//============================================================================


//============================================================================
#define        JREG_LPCD_RFT5                0x1B    //LPCD TEST3 Register 
//============================================================================
//#define        BFL_JBIT_LPCD_TESTEN            0x01    //lPCD test mode
//#define        BFL_JBIT_AWUP_TSEL                0x02    //Auto wakeup test mode
//#define        BFL_JBIT_RNG_MODE_SEL            0x04    //RNG  mode sel
//#define        BFL_JBIT_USE_RET                0x08    //use retention mode
//============================================================================

//============================================================================
#define        JREG_LPCD_MISC                      0x1C    //LPCD Misc Register 
//============================================================================
#define        BFL_JBIT_CALIB_VMID_EN            0x01    //lPCD test mode
#define        BFL_JBIT_AMP_EN_SEL                  0x04    //LPCD amp en select

//============================================================================



/*********************************************************************
*                                                                    *
*   Copyright (c) 2010 Shanghai FuDan MicroElectronic Inc, Ltd.      *
*   All rights reserved. Licensed Software Material.                 *
*                                                                    *
*   Unauthorized use, duplication, or distribution is strictly       *
*   prohibited by law.                                               *
*                                                                    *
*********************************************************************/

//��������,��ʱ�������û��޸�
//<<< Use Configuration Wizard   in Context Menu >>>
// <h> LPCD����
//     <e0>    �Ƿ���AUXͨ��
//       </e>    

//     <o1.0..2> ����Դ������ѡ����ݿ����ֲᣬ�����޸ģ� 
//                                    <0=>50nA(Default) 
//                                    <1=>100nA
//                                    <2=>150nA
//                                     <3=>200nA
//                                     <4=>250nA
//                                     <5=>300nA
//                                     <6=>350nA
//                                     <7=>400nA

//     <o2.0..4> LPCD���׼���׶�ʱ������ �����ݿ����ֲᣬ�����޸ģ�        
//                                    <0=>0ms
//                                    <1=>0ms
//                                    <2=>0.4ms
//                                    <3=>0.5ms
//                                    <4=>0.6ms 
//                                    <5=>0.7ms
//                                    <6=>0.8ms
//                                    <7=>0.9ms
//                                    <8=>1.0ms
//                                    <9=>1.1ms
//                                    <10=>1.2ms
//                                       <11=>1.3ms
//                                    <12=>1.4ms
//                                    <13=>1.5ms(Default)
//                                    <14=>1.6ms
//                                    <15=>1.7ms
//                                    <16=>1.8ms
//                                    <17=>1.9ms
//                                    <18=>2.0ms
//                                    <19=>2.1ms
//                                    <20=>2.2ms
//                                    <21=>2.3ms
//                                    <22=>2.4ms
//                                    <23=>2.5ms
//                                    <24=>2.6ms
//                                    <25=>2.7ms
//                                    <26=>2.8ms
//                                    <27=>2.9ms
//                                    <28=>3.0ms
//                                    <29=>3.1ms
//                                    <30=>3.2ms
//                                    <31=>3.3ms

//     <o3.0..3> �жϵȴ���ʱ���ã����ݿ����ֲᣬ�����޸ģ� 
//                                    <0=>1ms
//                                    <1=>2ms
//                                     <2=>3ms
//                                     <3=>4ms
//                                     <4=>5ms
//                                     <5=>6ms
//                                     <6=>7ms
//                                     <7=>8ms
//                                     <8=>9ms
//                                     <9=>10ms(Default) 
//                                     <10=>11ms
//                                     <11=>12ms
//                                     <12=>13ms
//                                     <13=>14ms
//                                     <14=>15ms
//                                     <15=>16ms

// </h>

// <h> �����������

//     <o4.0..4> ���������ֵ  �����ݿ����ֲᣬ�����޸ģ�
//                                    <10=>5.9us
//                                    <11=>6.5us 
//                                    <12=>7.1us
//                                    <13=>7.6us
//                                    <14=>8.2us
//                                    <15=>8.8us
//                                    <16=>9.4us
//                                    <17=>10us(Default) 
//                                    <18=>10.6us
//                                     <19=>11.2us
//                                     <20=>11.7us
//                                     <21=>12.3us
// </h>

// <h> LPCD��������
//     <o5.0..3> LPCD��ʼ���������ĵ�΢����Χ �����ڶ�ʱ������΢����        
//                                    <1=>1
//                                    <2=>2
//                                    <3=>3
//                                    <4=>4
//                                    <5=>5(Default)
//                                    <6=>6
//                                    <7=>7
//                                    <8=>8
//                                    <9=>9
//                                    <10=>10
// </h>


// <h> LPCD����������

//     <o6.0..1> LPCD����Ƶ���趨 �����ڶ�ʱ������΢����        
//                                    <0=>4��Ƶ
//                                    <1=>8��Ƶ(Default)    
//                                    <2=>16��Ƶ 



//     <o7.0..1> LPCDģʽѡ��
//                                    <0x0=>ģʽ0(Default)
//                                    <0x1=>ģʽ1
//                                    <0x2=>ģʽ2

//     <o8.0..1> LPCDģʽ���룬�������ģʽѡ��
//                                    <0=>�����ģʽ(Default)
//                                    <1=>С����ģʽ

//     <o9.0..1> LPCDģʽ�˳����������ģʽѡ��
//                                    <0=>�����ģʽ(Default)
//                                    <1=>С����ģʽ

//     <o10.0..4> VMID׼���׶�ʱ�� �����ݿ����ֲᣬ�����޸ģ�        
//                                    <0=>0ms
//                                    <1=>0ms
//                                    <2=>0.4ms
//                                    <3=>0.5ms
//                                    <4=>0.6ms 
//                                    <5=>0.7ms
//                                    <6=>0.8ms
//                                    <7=>0.9ms
//                                    <8=>1.0ms(Default)
//                                    <9=>1.1ms
//                                    <10=>1.2ms
//                                       <11=>1.3ms
//                                    <12=>1.4ms
//                                    <13=>1.5ms
//                                    <14=>1.6ms
//                                    <15=>1.7ms
//                                    <16=>1.8ms
//                                    <17=>1.9ms
//                                    <18=>2.0ms
//                                    <19=>2.1ms
//                                    <20=>2.2ms
//                                    <21=>2.3ms
//                                    <22=>2.4ms
//                                    <23=>2.5ms
//                                    <24=>2.6ms
//                                    <25=>2.7ms
//                                    <26=>2.8ms
//                                    <27=>2.9ms
//                                    <28=>3.0ms
//                                    <29=>3.1ms
//                                    <30=>3.2ms
//                                    <31=>3.3ms


//     <o11.0..3> LPCD�󴥷���������Ư�жϷ�Χ
//                                    <1=>1
//                                    <2=>2
//                                    <3=>3
//                                    <4=>4
//                                    <5=>5(Default)
//                                    <6=>6
//                                    <7=>7
//                                    <8=>8
//                                    <9=>9
//                                    <10=>10
// </h>

//1 3 8 9 10 11 19 20

#define      LPCD_AUX_EN                    0           //0
#define      LPCD_BIAS_CURRENT           0x00           //1
#define      TIMER2_CFG                    13            //2
#define         IRQ_TIMEOUT                    9            //3
#define         LP10K_CALIBRA_THRESHOLD     17            //4
#define        ADC_CENTER_RANGE            5            //5
#define         LPCD_SAMPLE_CLK              1            //6
#define        LPCD_MODE                    0x0            //7
#define        LP_OSC_MODE                    0            //8
#define        WUP_OSC_MODE                0            //9
#define        VMID_BG_CFG                    8            //10
#define        ERR_TRIG_JUDGE_RANGE        5            //11
//<<< end of configuration section >>>


#define        LP10K_CNT_DEFAULT            84    //��1.69Mhz��10K�ĸߵ�ƽ���в�����0x54��ȱʡֵ



/*********************************************************************
*                                                                    *
*   Copyright (c) 2010 Shanghai FuDan MicroElectronic Inc, Ltd.      *
*   All rights reserved. Licensed Software Material.                 *
*                                                                    *
*   Unauthorized use, duplication, or distribution is strictly       *
*   prohibited by law.                                               *
*                                                                    *
*********************************************************************/
//<<< Use Configuration Wizard   in Context Menu >>>
#define      TIMER1_CFG                    1        //0
// <h> LPCDʱ������
//     <o0.0..3> LPCD������߽׶�ʱ�����ã��û����ݵ͹���Ҫ�󣬿��޸ģ�         
//                                    <0=>0s
//                                    <1=>300ms    
//                                    <2=>400ms 
//                                    <3=>500ms(Default)
//                                    <4=>600ms
//                                    <5=>700ms 
//                                    <6=>800ms
//                                    <7=>900ms
//                                    <8=>1.0s
//                                    <9=>1.1s
//                                    <10=>1.2s
//                                    <11=>1.3s
//                                       <12=>1.4s
//                                    <13=>1.5s
//                                    <14=>1.6s
//                                    <15=>1.7s


#define      TIMER3_CFG                    12        
//     <o1.0..4> LPCD���׶�ʱ������ ���û����ݵ͹���Ҫ�󣬿��޸ģ�
//                                    <0=>0us
//                                    <1=>0us
//                                    <2=>4.7us                        
//                                    <3=>9.4us    
//                                    <4=>14.1us 
//                                    <5=>18.8us
//                                    <6=>23.5us
//                                    <7=>28.2us 
//                                    <8=>32.9us
//                                    <9=>37.6us
//                                    <10=>42.3us
//                                    <11=>47us
//                                    <12=>51.7us(Default)
//                                    <13=>56.4us
//                                       <14=>61.1us
//                                    <15=>65.8us
//                                    <16=>70.5us
//                                    <17=>75.2us
//                                    <18=>79.9us
//                                    <19=>84.6us
//                                    <20=>89.3us
//                                    <21=>94us
//                                    <22=>98.7us
//                                    <23=>103.4us
//                                    <24=>108.1us
//                                    <25=>112.8us
//                                    <26=>117.5us
//                                    <27=>122.2us
//                                    <28=>126.9us
//                                    <29=>131.6us
//                                    <30=>136.3us
//                                    <31=>141us

// </h>



#define      LPCD_TX2RFEN                1            //2

// <h> LPCD������������
//     <e2>    LPCD�Ƿ�ʹ��TX2����(Defaultʹ��)
//       </e>     
#define        LPCD_CWN                    0x1            //3 0x01��0x00������
//     <o3.0..1> LPCD����NMOS�������ã��û����޸ģ�
//                                    <0=>46ŷķ
//                                    <1=>23ŷķ(Default)

#define        LPCD_CWP                    0x4       //4 ֵԽ��LPCD����Խ����10uA��20��uA
//     <o4.0..2> LPCD����PMOS�������ã��û����޸ģ�
//                                    <0x0=>180ŷķ
//                                    <0x1=>90ŷķ
//                                    <0x2=>46ŷķ
//                                    <0x3=>23ŷķ(Default)
//                                    <0x4=>12ŷķ 
//                                    <0x5=>6ŷķ
//                                    <0x6=>3ŷķ
//                                    <0x7=>1.5ŷķ
// </h>

#define        LPCD_THRESHOLD_RATIO        3//5  �������û��

// <h> LPCD����������
//     <o5.0..5> LPCD������������������         
//                                    <2=>25%     
//                                    <3=>12.5%    
//                                    <4=>6.25%(Default)
//                                    <5=>3.125% 
#define   LPCD_DetectSensitive     0.07 //%5--------������Խ���ܹ��������ľ���ԽС��������ΧӦ��4%~15%֮�䣩

#define        LPCD_AUTO_DETECT_TIMES        1            //6
//     <o6.0> LPCD�Զ���������������ж� 
//                                    <0=>����
//                                    <1=>һ��(Default)
#define      LPCD_IE                     0x01        //7

//     <e7>    LPCD�Ƿ�����ж�(Defaultʹ��)
//       </e>
#define   LPCD_DS           0x00

#define        AUTO_WUP_EN                    1            
//     <e8> LPCDʹ���Զ�����ģʽ ���û����޸ģ�
//       </e>

#define        AUTO_WUP_CFG                2
//     <o9.0..2> LPCD�����Զ�����ʱ�� ���û����޸ģ�
//                                    <0=>6��
//                                    <1=>12��
//                                    <2=>15����(Default)
//                                    <3=>30����
//                                    <4=>1Сʱ
//                                    <5=>1.8Сʱ
//                                    <6=>3.6Сʱ
//                                    <7=>7.2Сʱ
// </h>










//<<< end of configuration section >>>


typedef enum FM17550Seq_e
{
    FM17550Seq_Idle,



    FM17550Seq_RstStart,
    FM17550Seq_RstLow,
    FM17550Seq_RstFinish,

    FM17550Seq_VerifyStart,

    FM17550Seq_Config,
    FM17550Seq_WaitData,


    FM17550Seq_OpenLock,


    FM17550Seq_OpenLock_Delay,


    FM17550Seq_Test,

    
    FM17550Seq_End
}FM17550Seq_t;

typedef struct FM17550_Data_s
{ 
    FM17550Seq_t FM17550SeqData:8;
}FM17550_Data_t,*pFM17550_Data_t;
void FM17550MgrHandle(NoBikeTaskData_t *app);



#endif
