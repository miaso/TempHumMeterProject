/*
 * File:   mainPIC.c
 * Author: Arturas
 *
 * Created on 24. februar 2016, 11:05
 */


#include "xc.h"
#include "oled.h"
#include <stdint.h>
#include <stdio.h>
#include "hwConfig.h"
#include "uart1.h"
#include "extralDefines.h"
#include "rtcc.h"
#include <stdlib.h>
#include "spiDriver.h"
#include "i2c1.h"




#define FCY     16000000ULL //TO DO

#include <libpic30.h>
#include "dht.h"
#include "si7020.h"
#include "timers.h"

//#include "oled_draw.h"
//





/// CONFIGURRATION BITS 
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & LPCFG_ON & ICS_PGx3 & WINDIS_OFF & FWDTEN_WDT_DIS & FWPSA_PR32 & WDTPS_PS1)
_CONFIG2(IESO_OFF & ALTVREF_ALT_AV_ALT_CV & FNOSC_FRCPLL & FCKSM_CSDCMD & OSCIOFCN_ON & IOL1WAY_OFF & POSCMD_NONE)
_CONFIG3(WPEND_WPENDMEM & WPCFG_WPCFGEN & WPDIS_WPEN & BOREN_OFF & WDTWIN_PS50_0 & SOSCSEL_ON & VBTBOR_OFF & WPFP_WPFP82)
_CONFIG4(DSSWEN_OFF & DSWDTEN_OFF & DSBOREN_OFF & DSWDTOSC_SOSC & DSWDTPS_DSWDTPS0)



struct tm currentTime;
volatile uint16_t timer1;

//Repeater main function

void RepeaterProcessEvents() {
    unsigned char data = 0;

    //wait for data to be received
    data = UART1GetChar();

    //send data back on UART TX line
    UART1PutChar(data);
}




//RTCTime time; // declare the type of the time object

int main(void) {
    timer1=0;
    volatile uint16_t lalala;
    
    OSCCON = 0x1102; // clock setup   0001 0001 0000 0010
    CLKDIV = 0x0000;
    OSCTUN = 0x0000;
    RCON |= 0x1100;


    /// Hardware setup
    hwInit();
    RTCC_Initialize();
    I2C1_Initialize();
   TimerInit();
   // INITIALIZE_SI7020();


    //    MICI: MI2C1 - I2C1 Master Events
    //    Priority: 1
   IPC4bits.MI2C1P = 1;
    //    SICI: SI2C1 - I2C1 Slave Events
    //    Priority: 1
   IPC4bits.SI2C1P = 1;

    // Display setup and test
    oled_init();
    oled_clearDisplay();
    oled_prints(1, 6, "Temp:"); // print on center of the screen
    oled_prints(1, 8, "Humidity:");
    oled_render();



    __delay_ms(1000);

    UART1Init(51);

    UART1PutChar('b');

    UART1PutStr("Init complete");
    
    
    SPI2STAT = 0x20AC;
    SPI2CON1 = 0x013D;
    SPI2CON2 = 0x0000;
    SPI2_INTERRUPT_CLEAR;
    SPI2_INTERRUPT_PRIORITY = INTERRUPT_PRIORITY_DISABLED;
    SPI2CON1bits.CKE = 0;
    SPI2CON1bits.CKP = 1;
    SPI2CON1bits.MSTEN = 1;
    SPI2_ENABLE;

    
    //eepromWrite();
    
    uint32_v address;
    
    
    while (1) {
       UART1PutChar('B');
        
        UART1PutChar('A');
        //lalala = 0;
        //lalala = eepromReadStatusRegister();
        
        //testINT=eepromReadStatusRegister();
        
        
      //  uint16_t sddsd=eepromTest();
     // if(sddsd==0){UART1PutChar('N');             
      // }else{UART1PutChar('Y');}
        
        //makeHTmeasurements(); 
        
        
        __delay_ms(100);
    }
    return (0);
}

// 15 NA 14-12 Fast RC Oscillator (FRC) 000 // 11 NA 10-8 000 Fast RC Oscillator (FRC) // bit7 0 CLLOCK // Bit 6 NA // bit 5=0 LOCK //bit 4 NA// bit 3 CLOCK fail // bit2 NA // bit 1 Secondary osc  0 // bit 0 OSWEN 
//Loop forever



