/* 
 * File:   readerDefines.h
 * Author: Dan
 *
 * Created on 01 mai 2015, 08:04
 */

#ifndef EXTRALDEFINES_H
#define	EXTRALDEFINES_H

#ifdef	__cplusplus
extern "C" { 
#endif
   // #define newcs PORTFbits.RF4 
   // #define newdc PORTFbits.RF5 
   #define uart1rx         LATDbits.LATD1      // 
   #define uart1rx_trs     TRISDbits.TRISD5   // 
    
   #define OLED_CS         LATBbits.LATB15      // Chip select
   #define OLED_CS_TRS     TRISBbits.TRISB15   // Chip select TRIS

   #define OLED_RST        LATBbits.LATB6      // Reset
   #define OLED_RST_TRS    TRISBbits.TRISB6    // Reset TRIS

   #define OLED_DC         LATBbits.LATB14     // Data/command
   #define OLED_DC_TRS     TRISBbits.TRISB14    // Data/command TRIS


   #define DHT_TRS    TRISFbits.TRISF3   // Chip select TRIS
   #define DHT        PORTFbits.RF3      // Chip select              

   #define I2C_SCL_TRS    TRISGbits.TRISG2   // CLK TRIS
   #define I2C_SCL       LATGbits.LATG2      // CLK pin    
   #define I2C_SCL_L    I2C_SCL=1
   #define I2C_SCL_HF   I2C_SCL=0
   #define I2C_SCL_State I2C_SCL 

   #define I2C_SDA_TRS    TRISGbits.TRISG3   // CLK TRIS
   #define I2C_SDA       LATGbits.LATG3      // CLK pin    
   #define I2C_SDA_L    I2C_SDA=1
   #define I2C_SDA_HF   I2C_SDA=0
   #define I2C_SDA_State I2C_SDA 

    
 // #define EEPROM_CS_TRS   TRISDbits.TRISD0
//   #define EEPROM_CS       LATDbits.LATD0   
 //  #define EEPROM_CS_L     EEPROM_CS=0
 //  #define EEPROM_CS_H     EEPROM_CS=1
    
    
    
/******************************************************************************/
// includes
/******************************************************************************/
#include <xc.h>
    

/******************************************************************************/
// typedefs
/******************************************************************************/
//typedef char                    int8_t;
typedef int                     int16_t;
typedef long                    int32_t;
typedef long long               int64_t;

typedef unsigned char           uint8_t;
typedef unsigned int            uint16_t;
typedef unsigned long           uint32_t;
typedef unsigned long long      uint64_t;

#define true                    1
#define false                   0

#define DATE_STR                "Date:"
#define TIME_STR                "Time:"
//#define DATE_STR                "Date:"
//#define DATE_STR                "Date:"
//#define DATE_STR                "Date:"


typedef union
{
    uint16_t w;
    uint8_t  b[2];
}uint16_v;

typedef union
{
    uint32_t dw;
    uint16_t w[2];
    uint8_t  b[4];
} uint32_v;

typedef union
{
    uint64_t qw;
    uint32_v dw[2];
    uint16_v w[4];
    uint8_t  b[8];
} uint64_v;
/********************************************/
/*            Time Structures               */
/********************************************/
typedef struct
{
    uint16_v year;
    uint16_v monthDay;
    uint16_v wdayHour;
    uint16_v minSec;
} chipTime_t;

typedef struct
{
    uint16_t sec;
    uint16_t min;
    uint16_t hour;
    uint16_t day;
    uint16_t month;
    uint16_v year;
} localTime_t;

/********************************************/
/*         Detection parameters             */
/********************************************/
typedef struct
{
    volatile uint8_t currValue;
    volatile uint8_t prevValue;

    volatile uint16_t locked;
    uint16_t highThreshold;
    uint16_t lowThreshold;

    uint16_t autoAdjust;
    uint16_t autoAdjustCounter;
    uint16_t autoAdjustLowThreshold;

    uint16_t minMaxDiff;
    uint16_t adjMinSeen;
    uint16_t adjMaxSeen;
    uint16_t count;
    uint16_t pulseWidthIncreased;
    uint16_t pulseWidthAdjCounter;
    uint16_t pulseWidthAdjPeriod;
    uint16_t adjCounter;
    uint16_t secondAdjCounter;

    uint16_t currentPulseWidth;
    uint16_t estimatedPulseWidth;

    uint16_t runningAvg;
    uint16_t pulseSkipPeriod;
    uint16_t pulseWidth;

    uint16_t lowRunningAvg;
    uint16_t lowestSeen;
    uint16_t lowestSeenCounter;
}sensor_t;

/********************************************/
/*      Kamstrup Sensor Head Structure      */
/********************************************/
typedef struct
{
    uint16_t meterAddress;
    uint16_v meterType;
    uint16_v swRev;
    uint32_v serialNumber;
    uint32_t register1;
    uint32_t register2;
    uint32_t register3;
    uint32_t register4;
    uint32_t register5;
    uint32_t register6;
    uint32_t register7;
    uint32_t register8;
} kamstrupMeter_t;

/********************************************/
/*         P1 Slave Meter Structure         */
/********************************************/
typedef struct
{
    uint16_t deviceType;
//    uint8_t equipmentId[96];
//    uint8_t timestamp[13];
    uint32_t value;
//    uint8_t valvePos;
}p1MeterSlaveDevice_t;

/********************************************/
/*          P1 Meter Structure              */
/********************************************/
typedef struct
{
//    uint8_t headerInfo[24];
    uint16_t version;
//    uint8_t timestamp[13];
//    uint8_t equimentId[96];
    uint32_t meterReadingElectricityDeliveredToClientT1;
    uint32_t meterReadingElectricityDeliveredToClientT2;
    uint32_t meterReadingElectricityDeliveredByClientT1;
    uint32_t meterReadingElectricityDeliveredByClientT2;
//    uint16_t tariff;

    uint32_t actualElectricityDelivered;
    uint32_t actualElectricityReceived;
//    uint16_t actualThresholdElectricity;

//    uint8_t switchPosition;

//    uint32_t powerFailures;
//    uint32_t longPowerFailures;
//    uint8_t powerFailureEventLog[23];

//    uint32_t voltageSagsL1;
//    uint32_t voltageSagsL2;
//    uint32_t voltageSagsL3;
//    uint32_t voltageSwellsL1;
//    uint32_t voltageSwellsL2;
//    uint32_t voltageSwellsL3;
    
//    uint8_t textMessageCodes[16];
//    uint8_t textMessage[2048];

//    uint16_t instCurrentL1;
//    uint16_t instCurrentL2;
//    uint16_t instCurrentL3;
//    uint32_t instActivePowerL1Pos;
//    uint32_t instActivePowerL2Pos;
//    uint32_t instActivePowerL3Pos;
//    uint32_t instActivePowerL1Neg;
//    uint32_t instActivePowerL2Neg;
//    uint32_t instActivePowerL3Neg;

    p1MeterSlaveDevice_t slaveMeters[4];
}p1Meter_t;

/********************************************/
/*              M-Bus meter                 */
/********************************************/
typedef struct
{
    uint16_t address;
    uint32_t register1;
    uint32_t register2;
    uint32_t register3;
    uint32_t register4;
    uint32_t register5;
    uint32_t register6;
    uint32_t register7;
    uint32_t register8;
} mBusMeter_t;

/********************************************/
/*              Combined meters             */
/********************************************/
typedef union
{
    kamstrupMeter_t kamstrupMeter;
    p1Meter_t p1Meter;
    mBusMeter_t mBusMeter;
}meter_t;

/********************************************/
/*     Sensor configuration parameters      */
/********************************************/
typedef struct
{   
    uint32_v configPresent;
    // Registers exposed to the user
    uint32_v sensorType;
    uint32_v meterType;
    uint32_v scale;
    uint32_v dualTariff;
    uint32_v tariffStart;
    uint32_v tariffEnd;    
    uint32_v pulseFactor;
    uint32_v calibration;
    uint32_v meterValueConsT1;
    uint32_v meterValueConsT2;
    uint32_v meterValueProdT1;
    uint32_v meterValueProdT2;
    uint32_v keepAwakeInterval;
    uint32_v realtimeInterval;
    uint32_v dataStoreInterval;
    uint32_v meterReportInterval;
    uint32_v realtimeMode;
    uint32_v alarmThreshold;
    uint32_v alarm;
    uint32_v applicationVersion;
    // Registers for internal use
    uint32_v wakeUpInterval;
    uint32_v startAddr;
    uint32_v currAddr;
    uint32_v startTime;
    uint32_v currTime;   
    uint32_v myNodeId;
    uint32_v controllerId;
    uint32_v lifelineNodeId;
    uint32_v assocNodeId;
    uint32_v customerIdLength;
} _params_t;

typedef union
{
    _params_t param;
    uint32_v dw[32];
    uint16_v w[64];
    uint8_t  b[128];
} _cfgParamsP_t;

typedef struct
{
    uint16_t zWaveMeterType;
    uint16_t rate;
    uint16_t activeTariff;
    uint16_t intervalFactor;
    uint16_t alarm;
    uint16_t internalMeterReportInterval;
    uint32_t internalWakeUpInterval;
    uint32_t internalDataStoreInterval;
} _cfgParamsR_t;

typedef struct
{
    uint32_v address;
    uint32_v data;
    uint32_t time;
    localTime_t aecTime;
} aecRecord_t;
/********************************************/
/*           zWave network IDs              */
/********************************************/
typedef struct 
{
    volatile uint16_t timeout;
   
    uint16_t testNodeId;
    uint16_t powerLevel;
    uint16_t status;
    uint16_v ackCount;
    uint16_v testFrameCount;
    uint32_v homeId;
}zWaveConfig_t;
/******************************************************************************/
// peripheral defines
/******************************************************************************/
#define SYSCLK_32MHz                                    0x00
#define SYSCLK_16MHz                                    0x01

#define SET_CLOCK_32MHZ                                 CLKDIVbits.DOZE = 0
#define SET_CLOCK_1MHZ                                  CLKDIVbits.DOZE = 5
#define SET_CLOCK_500KHZ                                CLKDIVbits.DOZE = 6

#define SYSTEM_CLOCK                                    SYSCLK_32MHz

#if SYSTEM_CLOCK == SYSCLK_32MHz
    #define UART_BAUD_115200                            0x0021      // 33
    #define UART_BAUD_9600                              0x019F      // 415
    #define UART_BAUD_2400                              1665
#elif SYSTEM_CLOCK == SYSCLK_16MHz
    #define UART_BAUD_115200                            0x0021
#else
    #define UART_BAUD_115200                            0x0021
#endif

//#define DEBUG_MODE

#define INTERRUPT_PRIORITY_MAX                          7
#define INTERRUPT_PRIORITY_EXT                          6
#define INTERRUPT_PRIORITY_RTC                          5
#define INTERRUPT_PRIORITY_TIMER                        4
#define INTERRUPT_PRIORITY_UART                         3
#define INTERRUPT_PRIORITY_ADC                          2
#define INTERRUPT_PRIORITY_DEFAULT                      1
#define INTERRUPT_PRIORITY_DISABLED                     0

/**************/
/* Trap flags */
/**************/
#define TRAP_MATHERR_CLR                                INTCON1bits.MATHERR = 0
#define TRAP_ADDRERR_CLR                                INTCON1bits.ADDRERR = 0
#define TRAP_STKERR_CLR                                 INTCON1bits.STKERR = 0
#define TRAP_OSCFAIL_CLR                                INTCON1bits.OSCFAIL = 0

/*****************************/
/* Real Time Clock and alarm */
/*****************************/
#define RTC_ENABLE                                      RCFGCALbits.RTCEN = 1
#define RTC_DISABLE                                     RCFGCALbits.RTCEN = 0
#define RTC_WR_ENABLE                                   RCFGCALbits.RTCWREN = 1
#define RTC_WR_DISABLE                                  RCFGCALbits.RTCWREN = 0
#define RTC_PTR                                         RCFGCALbits.RTCPTR
#define RTC_INTERRUPT_ENABLE                            IEC3bits.RTCIE = 1
#define RTC_INTERRUPT_DISABLE                           IEC3bits.RTCIE = 0
#define RTC_INTERRUPT_PRIORITY                          IPC15bits.RTCIP
#define RTC_INTERRUPT_CLEAR                             IFS3bits.RTCIF = 0
#define RTC_INTERRUPT_FLAG                              IFS3bits.RTCIF

#define ALARM_ENABLE                                    ALCFGRPTbits.ALRMEN = 1
#define ALARM_DISABLE                                   ALCFGRPTbits.ALRMEN = 0
#define ALARM_REPEAT_FOREVER                            ALCFGRPTbits.CHIME
#define ALARM_INTERVAL                                  ALCFGRPTbits.AMASK
#define ALARM_PTR                                       ALCFGRPTbits.ARPT
#define ALARM_1S                                        0x01
#define ALARM_10S                                       0x02
#define ALARM_1M                                        0x03
#define ALARM_10M                                       0x04
#define ALARM_1H                                        0x05
#define ALARM_1D                                        0x06
/****************/
/* INT0 defines */
/****************/
#define INT0_ENABLE                                     IEC0bits.INT0IE = 1
#define INT0_DISABLE                                    IEC0bits.INT0IE = 0
#define INT0_CLEAR                                      IFS0bits.INT0IF = 0
#define INT0_FLAG                                       IFS0bits.INT0IF
#define INT0_PRIORITY                                   IPC0bits.INT0IP

/****************/
/* INT1 defines */
/****************/
#define INT1_ENABLE                                     IEC1bits.INT1IE = 1
#define INT1_DISABLE                                    IEC1bits.INT1IE = 0
#define INT1_CLEAR                                      IFS1bits.INT1IF = 0
#define INT1_FLAG                                       IFS1bits.INT1IF
#define INT1_PRIORITY                                   IPC5bits.INT1IP

/*******************/
/* Timer 1 defines */
/*******************/
#define TIMER1_ENABLE                                   T1CONbits.TON = 1
#define TIMER1_DISABLE                                  T1CONbits.TON = 0
#define TIMER1_INTERRUPT_ENABLE                         IEC0bits.T1IE = 1
#define TIMER1_INTERRUPT_DISABLE                        IEC0bits.T1IE = 0
#define TIMER1_INTERRUPT_CLEAR                          IFS0bits.T1IF = 0
#define TIMER1_INTERRUPT_FLAG                           IFS0bits.T1IF
#define TIMER1_INTERRUPT_PRIORITY                       IPC0bits.T1IP

/*******************/
/* Timer 2 defines */
/*******************/
#define TIMER2_ENABLE                                   T2CONbits.TON = 1
#define TIMER2_DISABLE                                  T2CONbits.TON = 0
#define TIMER2_INTERRUPT_ENABLE                         IEC0bits.T2IE = 1
#define TIMER2_INTERRUPT_DISABLE                        IEC0bits.T2IE = 0
#define TIMER2_INTERRUPT_CLEAR                          IFS0bits.T2IF = 0
#define TIMER2_INTERRUPT_FLAG                           IFS0bits.T2IF
#define TIMER2_INTERRUPT_PRIORITY                       IPC1bits.T2IP
#define TIMER2_ENABLED                                  (T2CON & 0x8000)

/*******************/
/* Timer 3 defines */
/*******************/
#define TIMER3_ENABLE                                   T3CONbits.TON = 1
#define TIMER3_DISABLE                                  T3CONbits.TON = 0
#define TIMER3_INTERRUPT_ENABLE                         IEC0bits.T3IE = 1
#define TIMER3_INTERRUPT_DISABLE                        IEC0bits.T3IE = 0
#define TIMER3_INTERRUPT_CLEAR                          IFS0bits.T3IF = 0
#define TIMER3_INTERRUPT_FLAG                           IFS0bits.T3IF
#define TIMER3_INTERRUPT_PRIORITY                       IPC2bits.T3IP

/*******************/
/* Timer 4 defines */
/*******************/
#define TIMER4_ENABLE                                   T4CONbits.TON = 1
#define TIMER4_DISABLE                                  T4CONbits.TON = 0
#define TIMER4_INTERRUPT_ENABLE                         IEC1bits.T4IE = 1
#define TIMER4_INTERRUPT_DISABLE                        IEC1bits.T4IE = 0
#define TIMER4_INTERRUPT_CLEAR                          IFS1bits.T4IF = 0
#define TIMER4_INTERRUPT_FLAG                           IFS1bits.T4IF
#define TIMER4_INTERRUPT_PRIORITY                       IPC6bits.T4IP

/****************/
/* SPI1 defines */
/****************/
#define SPI1_ENABLE                                     SPI1STATbits.SPIEN = 1
#define SPI1_DISABLE                                    SPI1STATbits.SPIEN = 0
#define SPI1_OVERFLOW_FLAG                              SPI1STATbits.SPIROV
#define SPI1_OVERFLOW_CLEAR                             SPI1STATbits.SPIROV = 0
#define SPI1_RX_BUFFER_FULL                             SPI1STATbits.SPIRBF
#define SPI1_TX_BUFFER_FULL                             SPI1STATbits.SPITBF
#define SPI1_INTERRUPT_FLAG                             IFS0bits.SPI1IF
#define SPI1_INTERRUPT_CLEAR                            IFS0bits.SPI1IF = 0
#define SPI1_FAULT_FLAG                                 IFS0bits.SPF1IF
#define SPI1_FAULT_CLEAR                                IFS0bits.SPF1IF = 0
#define SPI1_INTERRUPT_PRIORITY                         IPC2bits.SPI1IP
#define SPI1_ENABLED                                    (SPI1STAT & 0x8000)

/****************/
/* SPI2 defines will use for eeprom */ 
/****************/
#define SPI2_ENABLE                                     SPI2STATbits.SPIEN = 1
#define SPI2_DISABLE                                    SPI2STATbits.SPIEN = 0
#define SPI2_OVERFLOW_FLAG                              SPI2STATbits.SPIROV
#define SPI2_OVERFLOW_CLEAR                             SPI2STATbits.SPIROV = 0
#define SPI2_RX_BUFFER_FULL                             SPI2STATbits.SPIRBF
#define SPI2_TX_BUFFER_FULL                             SPI2STATbits.SPITBF
#define SPI2_INTERRUPT_FLAG                             IFS2bits.SPI2IF
#define SPI2_INTERRUPT_CLEAR                            IFS2bits.SPI2IF = 0
#define SPI2_FAULT_FLAG                                 IFS2bits.SPF2IF
#define SPI2_FAULT_CLEAR                                IFS2bits.SPF2IF = 0
#define SPI2_INTERRUPT_PRIORITY                         IPC8bits.SPI2IP

/*****************/
/* Uart1 defines */
/*****************/
#define UART1_ENABLE                                    U1MODEbits.UARTEN = 1
#define UART1_DISABLE                                   U1MODEbits.UARTEN = 0
#define UART1_TX_ENABLE                                 U1STAbits.UTXEN = 1
#define UART1_TX_DISABLE                                U1STAbits.UTXEN = 0
#define UART1_TX_BUFFER_EMPTY                           U1STAbits.TRMT
#define UART1_RX_DATA_AVAILABLE                         U1STAbits.URXDA
#define UART1_TX_BUFFER_FULL                            U2STAbits.UTXBF
#define UART1_OERR_FLAG                                 U1STAbits.OERR
#define UART1_OERR_CLEAR                                U1STAbits.OERR = 0
#define UART1_WAKE_UP_ENABLE                            U1MODEbits.WAKE = 1

#define UART1_TX_INTERRUPT_FLAG                         IFS0bits.U1TXIF
#define UART1_TX_INTERRUPT_CLEAR                        IFS0bits.U1TXIF = 0
#define UART1_TX_INTERRUPT_ENABLE                       IEC0bits.U1TXIE = 1
#define UART1_TX_INTERRUPT_DISABLE                      IEC0bits.U1TXIE = 0

#define UART1_RX_INTERRUPT_FLAG                         IFS0bits.U1RXIF
#define UART1_RX_INTERRUPT_CLEAR                        IFS0bits.U1RXIF = 0
#define UART1_RX_INTERRUPT_ENABLE                       IEC0bits.U1RXIE = 1
#define UART1_RX_INTERRUPT_DISABLE                      IEC0bits.U1RXIE = 0
#define UART1_RX_INTERRUPT_PRIORITY                     IPC2bits.U1RXIP

/*****************/
/* Uart2 defines */
/*****************/
#define UART2_ENABLE                                    U2MODEbits.UARTEN = 1
#define UART2_DISABLE                                   U2MODEbits.UARTEN = 0
#define UART2_TX_ENABLE                                 U2STAbits.UTXEN = 1
#define UART2_TX_DISABLE                                U2STAbits.UTXEN = 0
#define UART2_TX_BUFFER_EMPTY                           U2STAbits.TRMT
#define UART2_RX_DATA_AVAILABLE                         U2STAbits.URXDA
#define UART2_TX_BUFFER_FULL                            U2STAbits.UTXBF
#define UART2_OERR_FLAG                                 U2STAbits.OERR
#define UART2_OERR_CLEAR                                U2STAbits.OERR = 0
#define UART2_WAKE_UP_ENABLE                            U2MODEbits.WAKE = 1
#define UART2_ENABLED                                   (U2MODE & 0x8000)

#define UART2_TX_INTERRUPT_FLAG                         IFS1bits.U2TXIF
#define UART2_TX_INTERRUPT_CLEAR                        IFS1bits.U2TXIF = 0
#define UART2_TX_INTERRUPT_ENABLE                       IEC1bits.U2TXIE = 1
#define UART2_TX_INTERRUPT_DISABLE                      IEC1bits.U2TXIE = 0

#define UART2_RX_INTERRUPT_FLAG                         IFS1bits.U2RXIF
#define UART2_RX_INTERRUPT_CLEAR                        IFS1bits.U2RXIF = 0
#define UART2_RX_INTERRUPT_ENABLE                       IEC1bits.U2RXIE = 1
#define UART2_RX_INTERRUPT_DISABLE                      IEC1bits.U2RXIE = 0
#define UART2_RX_INTERRUPT_PRIORITY                     IPC7bits.U2RXIP

/*****************/
/* Uart3 defines */
/*****************/
#define UART3_ENABLE                                    U3MODEbits.UARTEN = 1
#define UART3_DISABLE                                   U3MODEbits.UARTEN = 0
#define UART3_TX_ENABLE                                 U3STAbits.UTXEN = 1
#define UART3_TX_DISABLE                                U3STAbits.UTXEN = 0
#define UART3_TX_BUFFER_EMPTY                           U3STAbits.TRMT
#define UART3_RX_DATA_AVAILABLE                         U3STAbits.URXDA
#define UART3_TX_BUFFER_FULL                            U3STAbits.UTXBF
#define UART3_OERR_FLAG                                 U3STAbits.OERR
#define UART3_OERR_CLEAR                                U3STAbits.OERR = 0
#define UART3_WAKE_UP_ENABLE                            U3MODEbits.WAKE = 1

#define UART3_TX_INTERRUPT_FLAG                         IFS5bits.U3TXIF
#define UART3_TX_INTERRUPT_CLEAR                        IFS5bits.U3TXIF = 0
#define UART3_TX_INTERRUPT_ENABLE                       IEC5bits.U3TXIE = 1
#define UART3_TX_INTERRUPT_DISABLE                      IEC5bits.U3TXIE = 0

#define UART3_RX_INTERRUPT_FLAG                         IFS5bits.U3RXIF
#define UART3_RX_INTERRUPT_CLEAR                        IFS5bits.U3RXIF = 0
#define UART3_RX_INTERRUPT_ENABLE                       IEC5bits.U3RXIE = 1
#define UART3_RX_INTERRUPT_DISABLE                      IEC5bits.U3RXIE = 0
#define UART3_RX_INTERRUPT_PRIORITY                     IPC20bits.U3RXIP

/***************/
/* CRC Module  */
/***************/
#define CRC_ENABLE                                      CRCCON1bits.CRCEN = 1
#define CRC_DISABLE                                     CRCCON1bits.CRCEN = 0
#define CRC_START                                       CRCCON1bits.CRCGO = 1
#define CRC_STOP                                        CRCCON1bits.CRCGO = 0
#define CRC_INTERRUPT_FLAG                              IFS4bits.CRCIF

/***********************/
/* Change notification */
/***********************/
#define CN_INTERRUPT_FLAG                               IFS1bits.CNIF
#define CN_INTERRUPT_CLEAR                              IFS1bits.CNIF = 0
#define CN_INTERRUPT_ENABLE                             IEC1bits.CNIE = 1
#define CN_INTERRUPT_DISABLE                            IEC1bits.CNIE = 0
#define CN_INTERRUPT_PRIORITY                           IPC4bits.CNIP

/***************/
/* ADC defines */
/***************/
#define ADC_ENABLE                                      AD1CON1bits.ADON = 1
#define ADC_DISABLE                                     AD1CON1bits.ADON = 0
#define ADC_AUTO_SAMPLE_START                           AD1CON1bits.ASAM = 1
#define ADC_AUTO_SAMPLE_STOP                            AD1CON1bits.ASAM = 0
#define ADC_DONE                                        AD1CON1bits.DONE
#define ADC_CHANNEL                                     AD1CHSbits.CH0SA
#define ADC_SAMPLING_TIME                               AD1CON3bits.SAMC
#define ADC_INTERRUPT_ENABLE                            IEC0bits.AD1IE = 1
#define ADC_INTERRUPT_DISABLE                           IEC0bits.AD1IE = 0
#define ADC_INTERRUPT_FLAG                              IFS0bits.AD1IF
#define ADC_INTERRUPT_CLEAR                             IFS0bits.AD1IF = 0
#define ADC_INTERRUPT_PRIORITY                          IPC3bits.AD1IP

/******************************************************************************/
/*                              IO defines                                    */
/******************************************************************************/
/*********/
// Button
/*********/
#define BUTTON_PIN                                      PORTFbits.RF6
#define BUTTON_DIR                                      TRISFbits.TRISF6

/********/
/* LEDs */
/********/
#define LED_RED_PIN                                     PORTDbits.RD8
#define LED_GRN_PIN                                     PORTDbits.RD10
#define LED_BLU_PIN                                     PORTDbits.RD9

#define LED_RED_ON                                      LED_RED_PIN = 1
#define LED_RED_OFF                                     LED_RED_PIN = 0
#define LED_GREEN_ON                                    LED_GRN_PIN = 1
#define LED_GREEN_OFF                                   LED_GRN_PIN = 0
#define LED_BLUE_ON                                     LED_BLU_PIN = 1
#define LED_BLUE_OFF                                    LED_BLU_PIN = 0

#define JACK_VDD_PIN                                    PORTBbits.RB2
#define JACK_POWER_ON                                   PORTBbits.RB1 = 1
#define JACK_POWER_OFF                                  PORTBbits.RB1 = 0
/*******************/
// UART pins:
// UART1 -> Z-Wave
// UART2 -> Jack
// UART3 -> Debug
/*******************/
#define UART1_TX_PIN                                    PORTDbits.RD5
#define UART1_TX_DIR                                    TRISDbits.TRISD5
#define UART1_RX_PIN                                    PORTDbits.RD4
#define UART1_RX_DIR                                    TRISDbits.TRISD4

#define UART2_TX_PIN                                    PORTBbits.RB6
#define UART2_TX_DIR                                    TRISBbits.TRISB6
#define UART2_RX_PIN                                    PORTBbits.RB7
#define UART2_RX_DIR                                    TRISBbits.TRISB7

#define UART3_TX_PIN                                    PORTGbits.RG6
#define UART3_TX_DIR                                    TRISGbits.TRISG6
#define UART3_RX_PIN                                    PORTGbits.RG7
#define UART3_RX_DIR                                    TRISGbits.TRISG7

/*************/
// Z-Wave Pins
/*************/
#define ZWAVE_RESET_PIN                                 PORTDbits.RD6
#define ZWAVE_RESET_DIR                                 TRISDbits.TRISD6
#define ZWAVE_MISO_PIN                                  PORTDbits.RD1
#define ZWAVE_MOSI_PIN                                  PORTDbits.RD2
#define ZWAVE_CLK_PIN                                   PORTDbits.RD3
#define ZWAVE_INT1_PIN                                  PORTDbits.RD7

#define ZWAVE_MISO_DIR                                  TRISBbits.TRISB1
#define ZWAVE_MOSI_DIR                                  TRISBbits.TRISB2
#define ZWAVE_CLK_DIR                                   TRISBbits.TRISB3

/************/
/* SPI Pins */
/************/
#define SPI_MISO_PIN                                    PORTDbits.RD9
#define SPI_MOSI_PIN                                    PORTDbits.RD8
#define SPI_CLK_PIN                                     PORTDbits.RD10
#define SPI_CS_EEPROM_PIN                               PORTDbits.RD0
#define EEPROM_CS_HIGH                                  SPI_CS_EEPROM_PIN = 1
#define EEPROM_CS_LOW                                   SPI_CS_EEPROM_PIN = 0

/****************/
/* IR Tx and Rx */
/****************/
#define IR_TX_PIN                                       PORTBbits.RB6
#define IR_TX_DIR                                       TRISBbits.TRISB6
#define IR_TX_ON                                        IR_TX_PIN = 1
#define IR_TX_OFF                                       IR_TX_PIN = 0

#define IR_RX_PIN                                       PORTBbits.RB7
#define IR_RX_DIR                                       TRISBbits.TRISB7
#define IR_RX_ANS                                       ANSBbits.ANSB7

/**************/
// ADC Channels
/**************/
#define ADC_CHANNEL_IR_RX                               0x07
#define ADC_CHANNEL_VBAT                                0x1F

/******************************************************************************/
//
//                          application defines
//
/******************************************************************************/
/***********************/
// Device runtime modes
/***********************/
#define RUNTIME_AWAKE                                   0
#define RUNTIME_POWER_DOWN_ZWAVE_CHIP                   1
#define RUNTIME_POWER_DOWN_WAIT                         2
#define RUNTIME_POWER_DOWN_PIC_CHIP                     3
#define RUNTIME_SLEEP                                   4

/************************************/
/* Uart receive and transmit states */
/************************************/
#define UART_RX_IDLE                                    0
#define UART_RX_IN_PROGRESS                             1
#define UART_RX_SUCCESS                                 2 
#define UART_RX_FAILED                                  3

#define UART_TX_IDLE                                    0
#define UART_TX_IN_PROGRESS                             1
#define UART_TX_FRAME_SUCCESS                           2
#define UART_TX_FRAME_FAILED                            3
#define UART_TX_ZWAVE_FINISHED                          4
//#define UART_TX_TIMEOUT                                 3
//#define UART_TX_ZWAVE_FAILED                            5 
//#define UART_TX_ZWAVE_SUCCESS                           6

/***********************/
// Firmware Update steps
/***********************/
#define FIRMWARE_UPDATE_IDLE                            0
#define FIRMWARE_UPDATE_DOWNLOAD_IN_PROGRESS            1
#define FIRMWARE_UPDATE_DOWNLOAND_WAIT                  2
#define FIRMWARE_UPDATE_DOWNLOAD_COMPLETE               3
#define FIRMWARE_UPDATE_SUCCESS                         4
#define FIRMWARE_UPDATE_TIMEOUT                         5
#define FIRMWARE_UPDATE_FAILED                          6
#define FIRMWARE_UPDATE_CLEAR                           0xFF

/******************/
/* Button actions */
/******************/
#define ACTION_REALTME_ENTER                            1
#define ACTION_REATIME_EXIT                             2
#define ACTION_PAIR                                     3
#define ACTION_TEST                                     4
#define ACTION_CALIBRATE                                5
#define ACTION_LOAD_DEFAULTS                            7
#define ACTION_REBOOT                                   10
/*************************/
// Timeouts and time bases
/*************************/
#define ADC_SAMPLING_TIME_BATTERY                       0x1F
#define ADC_SAMPLING_TIME_DEFAULT                       0x03

#define TIMER1_PERIOD_EA                                546     // 16  ms
#define TIMER1_PERIOD_GA                                1638    // 50  ms
#define TIMER1_PERIOD_LED                               80      // 2.5 ms
#define TIMER2_PERIOD                                   625     // 10  ms
#define TIMER3_PERIOD                                   15625   // 250 ms
#define TIMER4_PERIOD                                   3200    // 5   KHz

#define LED_FADE_PERIOD                                 55
#define LED_FADE_DUTY_MIN                               3
#define LED_FADE_DUTY_MAX                               54

#define KEEP_AWAKE_3S                                   300     // 3 seconds
#define KEEP_AWAKE_5S                                   500     // 5 seconds
#define KEEP_AWAKE_10S                                  1000    // 10 seconds

#define ADC_SAMPLE_COUNT                                8
#define ADC_SAMPLE_DIVIDER                              5       // 3 from 8 samples + 2 from 10bit to 8 bit

#define BUTTON_PRESSED                                  0
#define BUTTON_RELEASED                                 1
#define BUTTON_VALID_PRESS                              5
#define BUTTON_RELEASED_TIMEOUT                         150     // 1.5 seconds

#define OUTPUT_FLASH_DURATION                           5       // 50ms
#define OUTPUT_FLASH_PERIOD                             35      // 350ms

#define LED_BLINK_PERIOD                                35      // 350 ms
#define LED_BLINK_DURATION                              30      // 5 ms = 35 - 30

// The following timeouts are based on Timer3 @ 250ms
#define ZWAVE_TX_TIMEOUT                                8       // 2 seconds
#define ZWAVE_RX_TIMEOUT                                6       // 1.5 seconds
#define ZWAVE_LEARN_TIMEOUT                             20      // 5 seconds
#define FIRMWARE_UPDATE_RECEIVE_TIMEOUT                 40      // 10 seconds

#define UART2_RX_TIMEOUT                                20      // 5 seconds
#define METER_DATA_ACQ_INTERVAL                         120     // 30 seconds
/******************************************************************************/
// zWave Learn Mode Steps
/******************************************************************************/
#define ZWAVE_LEARN_IDLE                                0
#define ZWAVE_LEARN_SEND_NIF                            1
#define ZWAVE_LEARN_IN_PROGRESS                         2
#define ZWAVE_LEARN_SUCCESS                             3
#define ZWAVE_LEARN_FAILED                              4

/******************************************************************************/
// Sensor and meter types
/******************************************************************************/
#define SENSOR_TYPE_NONE                                0
#define SENSOR_TYPE_ELECTRIC                            1
#define SENSOR_TYPE_GAS                                 2
#define SENSOR_TYPE_WATER                               3
#define SENSOR_TYPE_KAMSTRUP                            4
#define SENSOR_TYPE_P1                                  5
#define SENSOR_TYPE_MBUS                                6
#define SENSOR_TYPE_NORTHQ                              0xFF

#define METER_TYPE_NONE                                 0
#define METER_TYPE_PULSE                                1
#define METER_TYPE_ANALOG                               2
#define METER_TYPE_LED                                  3

#define METER_TYPE_P1_V1                                1
#define METER_TYPE_P1_V2                                2

/******************************************************************************/
// Supported scales, rates, types according to Z-Wave documentation
/******************************************************************************/
#define ZWAVE_METER_TYPE_NONE                           0x00
#define ZWAVE_METER_TYPE_ELECTRIC                       0x01
#define ZWAVE_METER_TYPE_GAS                            0x02
#define ZWAVE_METER_TYPE_WATER                          0x03
#define ZWAVE_METER_TYPE_HEATING                        0x08
#define ZWAVE_METER_TYPE_COOLING                        0x09
#define ZWAVE_METER_TYPE_HEATING_COOLING                0x0A

#define RATE_TYPE_NONE                                  0x00
#define RATE_TYPE_IMPORT                                0x01
#define RATE_TYPE_EXPORT                                0x02
#define RATE_TYPE_BOTH                                  0x03
// Units for Electricity meters
#define SCALE_EL_KWH                                    0x00
// Units for Gas and Water meters
#define SCALE_G_W_CUBIC_METER                           0x00    
#define SCALE_G_W_CUBIC_FEET                            0x01
#define SCALE_G_W_US_GALLON                             0x02
#define SCALE_G_W_IMP_GALLON                            0x04
#define SCALE_G_W_LITER                                 0x05
// Units for Heating and Cooling meters
#define SCALE_H_C_CUBIC_METER                           0x00
#define SCALE_H_C_METRIC_TON                            0x01
#define SCALE_H_C_CUBIC_METER_PER_HOUR                  0x02
#define SCALE_H_C_LITER_PER_HOUR                        0x03
#define SCALE_H_C_KW                                    0x04
#define SCALE_H_C_MW                                    0x05
#define SCALE_H_C_KWH                                   0x06
#define SCALE_H_C_MWH                                   0x07
#define SCALE_H_C_GJ                                    0x08
#define SCALE_H_C_GCAL                                  0x09
#define SCALE_H_C_CELSIUS                               0x0A
/******************************************************************************/
// Configuration registers
/******************************************************************************/
#define CONFIG_ID_PARAM_COUNT                           21
#define CONFIG_ID_TOTAL_PARAM_COUNT                     31

#define CONFID_ID_PRESENT_MARK                          0
#define CONFIG_ID_SENSOR_TYPE                           1
#define CONFIG_ID_METER_TYPE                            2
#define CONFIG_ID_SCALE                                 3
#define CONFIG_ID_TARIFF                                4
#define CONFIG_ID_TARIFF_START                          5
#define CONFIG_ID_TARIFF_END                            6
#define CONFIG_ID_PULSE_FACTOR                          7
#define CONFIG_ID_CALIBRATION                           8
#define CONFIG_ID_METER_VALUE_CONS_T1                   9
#define CONFIG_ID_METER_VALUE_CONS_T2                   10
#define CONFIG_ID_METER_VALUE_PROD_T1                   11
#define CONFIG_ID_METER_VALUE_PROD_T2                   12
#define CONFIG_ID_INTERVAL_KEEP_AWAKE                   13
#define CONFIG_ID_INTERVAL_REALTIME                     14
#define CONFIG_ID_INTERVAL_DATA_STORE                   15
#define CONFIG_ID_INTERVAL_METER_REPORT                 16
#define CONFIG_ID_REALTIME_MODE                         17
#define CONFIG_ID_THRESHOLD_ALARM_VALUE                 18
#define CONFIG_ID_THRESHOLD_ALARM                       19
#define CONFIG_ID_APPLICATION_VERSION                   20 

#define CONFIG_ID_INTERVAL_WAKEUP                       21
#define CONFIG_ID_DATA_START_IDX                        22
#define CONFIG_ID_DATA_CURR_IDX                         23
#define CONFIG_ID_DATA_START_TIME                       24
#define CONFIG_ID_DATA_CURR_TIME                        25
#define CONFIG_ID_NODE_ID                               26
#define CONFIG_ID_CONTROLLER_ID                         27
#define CONFIG_ID_LIFELINE_ID                           28
#define CONFIG_ID_ASSOC_ID                              29
#define CONFIG_ID_CUSTOMER_ID_LENGTH                    30

#define REG_SIZE_CONFIG_MARK                            2
#define REG_SIZE_CONFIG_RESERVED                        2
#define REG_SIZE_SENSOR_TYPE                            2
#define REG_SIZE_METER_TYPE                             2
#define REG_SIZE_SCALE                                  2
#define REG_SIZE_TARIFF                                 2
#define REG_SIZE_INTERVAL_TARIFF                        4
#define REG_SIZE_PULSE_FACTOR                           4
#define REG_SIZE_METER_VALUE                            4
#define REG_SIZE_INTERVAL_KEEP_AWAKE                    2
#define REG_SIZE_INTERVAL_REALTIME                      2
#define REG_SIZE_INTERVAL_WAKEUP                        4
#define REG_SIZE_INTERVAL_DATA_STORE                    4
#define REG_SIZE_INTERVAL_METER_REPORT                  2
#define REG_SIZE_DATA_START_IDX                         4
#define REG_SIZE_DATA_CURR_IDX                          4
#define REG_SIZE_DATA_START_TIME                        4
#define REG_SIZE_DATA_CURR_TIME                         4
#define REG_SIZE_NODE_ID                                2
#define REG_SIZE_CONTROLLER_ID                          2
#define REG_SIZE_LIFELINE_ID                            2
#define REG_SIZE_ASSOC_ID                               2
#define REG_SIZE_CUSTOMER_ID_LENGTH                     2
#define REG_SIZE_ALARM_THRESHOLD_VALUE                  4
/******************************************************************************/
/*                         EEPROM CONFIGURATION MAP                           */
/******************************************************************************/
#define EEPROM_CONFIG_START                             0x00000000
#define EEPROM_CONFIG_END                               0x000003FF
#define EEPROM_CONFIG_SIZE                              0x00000400  // 1024 Bytes
#define EEPROM_DATA_START                               0x00000400
#define EEPROM_DATA_END                                 0x0003FFFF
#define EEPROM_DATA_SIZE                                0x0003FC00  // 261120 Bytes
#define EEPROM_FIRMWARE_START                           0x00010000
#define EEPROM_FIRMWARE_MARK                            0x000003F8
#define EEPROM_SOFT_RESET_MARK                          0x000003F0

#define EEPROM_CONFIG_PRESENT                           0xA5A5

#define EEADDR_CONFIG                                   0x00000000          // 2B + 2 reserved
#define EEADDR_SENSOR_TYPE                              EEADDR_CONFIG + 4
#define EEADDR_METER_TYPE                               EEADDR_SENSOR_TYPE + 4
#define EEADDR_SCALE                                    EEADDR_METER_TYPE + 4
#define EEADDR_TARIFF                                   EEADDR_SCALE + 4
#define EEADDR_TARIFF_START                             EEADDR_TARIFF + 4
#define EEADDR_TARIFF_END                               EEADDR_TARIFF_START + 4
#define EEADDR_PULSE_FACTOR                             EEADDR_TARIFF_END + 4
#define EEADDR_CALIBRATION                              EEADDR_PULSE_FACTOR + 4
#define EEADDR_METER_VALUE_CONS_T1                      EEADDR_CALIBRATION + 4
#define EEADDR_METER_VALUE_CONS_T2                      EEADDR_METER_VALUE_CONS_T1 + 4
#define EEADDR_METER_VALUE_PROD_T1                      EEADDR_METER_VALUE_CONS_T2 + 4
#define EEADDR_METER_VALUE_PROD_T2                      EEADDR_METER_VALUE_PROD_T1 + 4
#define EEADDR_INTERVAL_KEEPAWAKE                       EEADDR_METER_VALUE_PROD_T2 + 4
#define EEADDR_INTERVAL_REALIME                         EEADDR_INTERVAL_KEEPAWAKE + 4
#define EEADDR_INTERVAL_DATA_STORE                      EEADDR_INTERVAL_REALIME + 4
#define EEADDR_INTERVAL_METER_REPORT                    EEADDR_INTERVAL_DATA_STORE + 4
#define EEADDR_REALTIME_MODE                            EEADDR_INTERVAL_METER_REPORT + 4
#define EEADDR_ALARM_THRESHOLD_VALUE                    EEADDR_REALTIME_MODE + 4
#define EEADDR_ALARM                                    EEADDR_ALARM_THRESHOLD_VALUE + 4
#define EEADDR_APPLICATION_VERSION                      EEADDR_ALARM + 4
#define EEADDR_INTERVAL_WAKEUP                          EEADDR_APPLICATION_VERSION + 4
#define EEADDR_DATA_START_IDX                           EEADDR_INTERVAL_WAKEUP + 4
#define EEADDR_DATA_CURR_IDX                            EEADDR_DATA_START_IDX + 4                        
#define EEADDR_DATA_START_TIME                          EEADDR_DATA_CURR_IDX + 4 
#define EEADDR_DATA_CURR_TIME                           EEADDR_DATA_START_TIME + 4
#define EEADDR_MY_NODE_ID                               EEADDR_DATA_CURR_TIME + 4
#define EEADDR_CONTROLLER_ID                            EEADDR_MY_NODE_ID + 4
#define EEADDR_LIFELINE_ID                              EEADDR_CONTROLLER_ID + 4
#define EEADDR_ASSOC_ID                                 EEADDR_LIFELINE_ID + 4
#define EEADDR_CUSTOMER_ID_LENGH                        EEADDR_ASSOC_ID + 4

#define EEADDR_CONFIG_END                               EEADDR_CUSTOMER_ID_LENGH + 4

#define EEPROM_CONFIG_PARAM_SIZE                        EEADDR_CONFIG_END - REG_SIZE_CONFIG_MARK - REG_SIZE_CONFIG_RESERVED

#define EEADDR_CUSTOMER_ID                              0x0300
/******************************************************************************/
// Application tasks
/******************************************************************************/
// tasks belonging to Communication and hw
#define TASK1_UART1_DATA_AVAILABLE_SET                  0x0001
#define TASK1_UART1_DATA_AVAILABLE_CLR                  0xFFFE

#define TASK1_UART2_DATA_AVAILABLE_SET                  0x0002
#define TASK1_UART2_DATA_AVAILABLE_CLR                  0xFFFD

#define TASK1_UART3_DATA_AVAILABLE_SET                  0x0004
#define TASK1_UART3_DATA_AVAILABLE_CLR                  0xFFFB

#define TASK1_UART1_DATA_SEND_SET                       0x0008
#define TASK1_UART1_DATA_SEND_CLR                       0xFFF7

#define TASK1_UART2_DATA_SEND_SET                       0x0010
#define TASK1_UART2_DATA_SEND_CLR                       0xFFEF

#define TASK1_UART3_DATA_SEND_SET                       0x0020
#define TASK1_UART3_DATA_SEND_CLR                       0xFFDF

#define TASK1_ADC_ACTIVE_SET                            0x0040
#define TASK1_ADC_ACTIVE_CLR                            0xFFBF

#define TASK1_BATTERY_LEVEL_SET                         0x0080
#define TASK1_BATTERY_LEVEL_CLR                         0xFF7F

#define TASK1_METER_DATA_REQUEST_SET                    0x0100
#define TASK1_METER_DATA_REQUEST_CLR                    0xFEFF

#define TASK1_WAKEUP_FROM_SLEEP_SET                     0x0200
#define TASK1_WAKEUP_FROM_SLEEP_CLR                     0xFDFF

#define TASK1_SEND_DEBUG_DATA_SET                       0x0400
#define TASK1_SEND_DEBUG_DATA_CLR                       0xFBFF

#define TASK1_HW_TEST_SET                               0x0800
#define TASK1_HW_TEST_CLR                               0xF7FF

#define TASK1_REBOOT_SET                                0x1000
#define TASK1_REBOOT_CLR                                0xEFFF

#define TASK1_WIRELESS_UPDATE_SET                       0x2000
#define TASK1_WIRELESS_UPDATE_CLR                       0xDFFF

#define TASK1_ZWAVE_PROGRAM_SET                         0x4000
#define TASK1_ZWAVE_PROGRAM_CLR                         0xBFFF

#define TASK1_DEBUG_SET                                 0x8000
#define TASK1_DEBUG_CLR                                 0x7FFF

// Config and control tasks
#define TASK2_LOAD_DEFAULTS_SET                         0x0001
#define TASK2_LOAD_DEFAULTS_CLR                         0xFFFE

#define TASK2_CHANGE_CONFIG_SET                         0x0002
#define TASK2_CHANGE_CONFIG_CLR                         0xFFFD

#define TASK2_DATA_STORE_SET                            0x0004
#define TASK2_DATA_STORE_CLR                            0xFFFB

#define TASK2_CLEAR_HISTORICAL_DATA_SET                 0x0008
#define TASK2_CLEAR_HISTORICAL_DATA_CLR                 0xFFF7

#define TASK2_DETECTION_SET                             0x0010
#define TASK2_DETECTION_CLR                             0xFFEF

#define TASK2_CALIBRATION_SET                           0x0020
#define TASK2_CALIBRATION_CLR                           0xFFDF

#define TASK2_PERIODIC_WAKEUP_SET                       0x0040
#define TASK2_PERIODIC_WAKEUP_CLR                       0xFFBF

#define TASK2_REALTIME_SET                              0x0080
#define TASK2_REALTIME_CLR                              0xFF7F

#define TASK2_ALWAYS_ON_SET                             0x0100
#define TASK2_ALWAYS_ON_CLR                             0xFEFF

#define TASK2_KEEP_AWAKE_SET                            0x0200
#define TASK2_KEEP_AWAKE_CLR                            0xFDFF

// Tasks belonging to ZWave API
#define TASK3_ZWAVE_INIT_SET                            0x0001
#define TASK3_ZWAVE_INIT_CLR                            0xFFFE

#define TASK3_LEARN_MODE_SET                            0x0002
#define TASK3_LEARN_MODE_CLR                            0xFFFD

#define TASK3_SEND_WAKEUP_NOTIFICATION_SET              0x0004
#define TASK3_SEND_WAKEUP_NOTIFICATION_CLR              0xFFFB

#define TASK3_SEND_REALTIME_REPORT_SET                  0x0008
#define TASK3_SEND_REALTIME_REPORT_CLR                  0xFFF7

#define TASK3_SEND_METER_REPORT_SET                     0x0010
#define TASK3_SEND_METER_REPORT_CLR                     0xFFEF

#define TASK3_SEND_HISTORICAL_REPORT_SET                0x0020
#define TASK3_SEND_HISTORICAL_REPORT_CLR                0xFFDF

#define TASK3_HANDLE_MULTI_CMD_SET                      0x0040
#define TASK3_HANDLE_MULTI_CMD_CLR                      0xFFBF

#define TASK3_POWER_LEVEL_CHANGE_SET                    0x0080
#define TASK3_POWER_LEVEL_CHANGE_CLR                    0xFF7F

#define TASK3_POWER_LEVEL_TEST_SET                      0x0100
#define TASK3_POWER_LEVEL_TEST_CLR                      0xFEFF

#define TASK3_SEND_CONFIG_BULK_REPORT_SET               0x0200
#define TASK3_SEND_CONFIG_BULK_REPORT_CLR               0xFDFF

#define TASK3_ZWAVE_CHIP_RESET_SET                      0x8000
#define TASK3_ZWAVE_CHIP_RESET_CLR                      0x7FFF
/******************************************************************************/
// constants
/******************************************************************************/
#define EEPROM_BUFFER_SIZE                              256

#define ZWAVE_TX_BUFFER_SIZE                            256
#define ZWAVE_RX_BUFFER_SIZE                            256

#define UART1_TX_BUFFER_SIZE                            256
#define UART1_RX_BUFFER_SIZE                            256
#define UART2_TX_BUFFER_SIZE                            256
#define UART2_RX_BUFFER_SIZE                            1024
#define UART3_TX_BUFFER_SIZE                            256
#define UART3_RX_BUFFER_SIZE                            256
/******************************************************************************/
// variables
/******************************************************************************/
extern uint8_t  zwaveRxBuffer[ZWAVE_RX_BUFFER_SIZE];
extern uint8_t  zwaveTxBuffer[ZWAVE_TX_BUFFER_SIZE];
extern uint16_t zwaveRxBufferCount;
extern uint16_t zwaveTxBufferCount;

extern uint8_t  uart2RxBuffer[UART2_RX_BUFFER_SIZE];
extern uint8_t  uart2TxBuffer[UART2_TX_BUFFER_SIZE];
extern uint16_t uart2RxBufferCount;
extern uint16_t uart2TxBufferCount;

extern uint8_t  uart3RxBuffer[UART3_RX_BUFFER_SIZE];
extern uint8_t  uart3TxBuffer[UART3_TX_BUFFER_SIZE];
extern uint16_t uart3RxBufferCount;
extern uint16_t uart3TxBufferCount;

extern uint8_t  eepromReadBuffer[EEPROM_BUFFER_SIZE];
extern uint8_t  eepromWriteBuffer[EEPROM_BUFFER_SIZE];
extern uint16_t eepromReadBufferCount;
extern uint16_t eepromWriteBufferCount;

extern _cfgParamsP_t cfgParamsP;
extern _cfgParamsR_t cfgParamsR;
extern zWaveConfig_t zWaveStatus;
extern meter_t       connectedMeter;
extern sensor_t      detectionSensor;
extern localTime_t   localTime;
extern aecRecord_t   aecRecord;

extern uint32_v applicationVersion;
extern uint32_v serialNumber;

extern uint16_t transmitPowerInterval;

extern volatile uint16_t pendingTasks1;
extern volatile uint16_t pendingTasks2;
extern volatile uint16_t pendingTasks3;
extern volatile uint16_t batteryLevel;
extern volatile uint16_t uartDebug;
extern volatile uint16_t adcInUse;
extern volatile uint16_t resetReason;

extern volatile uint16_t ledRedBlinkTimes;
extern volatile uint16_t ledGrnBlinkTimes;
extern volatile uint16_t ledBluBlinkTimes;

extern volatile uint16_t batteryLevelTimeout;
extern volatile uint16_t batteryLevelCounter;
extern volatile uint32_t dataStoreCounter;
extern volatile uint32_t realtimeCounter;
extern volatile uint32_t keepAwakeCounter;
extern volatile uint32_t wakeUpCounter;

extern volatile uint16_t zWaveLearnModeProgress;
extern volatile uint16_t zWaveLearnModeTimeoutCounter;
extern volatile uint16_t zWaveTxFrameTimeoutCounter;
extern volatile uint16_t zWaveRxFrameTimeoutCounter;
extern volatile uint16_t zWaveTxFrameStatus;
extern volatile uint16_t zWaveTxFrameRetry;
extern volatile uint16_t zWaveTxOnlyAckRequired;
extern volatile uint16_t uart2RxTimeoutCounter;
extern volatile uint16_t uart2RxProgress;

extern volatile uint16_t firmwareUpdateProgress;
extern volatile uint16_t firmwareUpdateTimeout;

extern volatile uint16_t ledFadeDir;
extern volatile uint16_t ledFadeDuty;
extern volatile uint16_t ledFadePeriod;

extern volatile uint32_t pulseCountT1;
extern volatile uint32_t pulseCountT2;

extern uint16_t crc16EncapRequired;
extern uint16_t transmitPower;
extern uint16_t transmitPowerCount;

extern volatile uint16_t deviceSleepStep;

extern volatile uint16_t uartDebugCounter;
extern volatile uint16_t delayCounter;
/******************************************************************************/
// functions
/******************************************************************************/

/******************************************************************************/
#ifdef	__cplusplus
}
#endif

#endif	/* READERDEFINES_H */

