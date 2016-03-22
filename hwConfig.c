#include "extralDefines.h"
#include "hwConfig.h"
#include "spiDriver.h"

/******************************************************************************/
// Hardware Init:
// Init peripherals
// Configure pins
/******************************************************************************/
void hwInit(void)
{
    /***************/
    /* PPS mapping */
    /***************/
    asm volatile("disi	#6");
    asm volatile("MOV   #OSCCON, w1");
    asm volatile("MOV   #0x46,   w2");
    asm volatile("MOV   #0x57,   w3");
    asm volatile("MOV.b w2,    [w1]");
    asm volatile("MOV.b w3,    [w1]");
    asm volatile("BCLR  OSCCON,  #6");

    RPOR12bits.RP24R = 3;       // U1 Tx   
    //me
    RPINR18bits.U1RXR = 20;     // U1 Rx
    
    // RPOR11bits.RP23R = 3;       // U1 Tx    
     //vova
    // RPINR18bits.U1RXR = 24;     // U1 Rx
    RPOR3bits.RP6R = 5;         // U2 Tx
    RPINR19bits.U2RXR = 7;      // U2 Rx
    RPOR10bits.RP21R = 28;      // U3 Tx
    RPINR17bits.U3RXR = 26;     // U3 RxhhLSB
    
    
    RPINR20bits.SDI1R = 17;     // SPI1 MISO    
    RPOR5bits.RP10R =7;        // SPI1 MOSI new
    RPOR8bits.RP17R = 8;       // SPI1 CLK new
   
   //RPOR5bits.RP10R5= ; 
    
    //RPINR0bits.INT1R = 7;       // Ext Int 1
    RPINR22bits.SDI2R = 4;     // SPI2 MISO
    RPOR1bits.RP2R = 10;      // SPI2 MOSI
    RPOR1bits.RP3R = 11;      // SPI2 CLK
    

    

    asm volatile("disi	#6");
    asm volatile("MOV   #OSCCON, w1");
    asm volatile("MOV   #0x46,   w2");
    asm volatile("MOV   #0x57,   w3");
    asm volatile("MOV.b w2,    [w1]");
    asm volatile("MOV.b w3,    [w1]");
    asm volatile("BSET  OSCCON,  #6");

    /******************************/
    /* Disable Unused peripherals */
    /******************************/
//    PMD1 = 0xC080;
//    PMD2 = 0xFFFF;
//    PMD3 = 0xFDF7;
//    PMD4 = 0xFFFD;
//    PMD6 = 0xFFFF;
//    PMD7 = 0xFFFF;
    
    /***********/
    /* IO init */
    /***********/
    TRISB = 0x0000;
    PORTB = 0x0000;
    ANSB  = 0x0000;

    TRISC = 0x2000;
    PORTC = 0x2000;

    TRISD = 0x0012;    
    
   // TRISDBITS.TRISD8 = 0;
   // TRISDBITS.TRISD10 = 0;
    TRISDbits.TRISD9 = 1 ;
    
   //  TRISDBITS.TRISD11 ;
    
    
    PORTD = 0x00B2;
    ANSD  = 0x0000;

    TRISE = 0x0000;
    PORTE = 0x0000;
    ANSE  = 0x0000;

    TRISF = 0x0000;
    PORTF = 0x0000;
   

    
    
    TRISG = 0x0000;
    PORTG = 0x0000;
    ANSG  = 0x0000;

 //   DHT_TRS = 0; //Configure RD0 as output
    //DHT = 1; //RD0 sends 0 to the sensor
}


void enableRTCC(void){
    // unlock RTCC
asm volatile ("disi #5");
asm volatile ("mov #0x55,w7");
asm volatile ("mov w7,_NVMKEY");
asm volatile ("mov #0xAA, w8");
asm volatile ("mov w8, _NVMKEY");
asm volatile ("bset _RCFGCAL,#13");
asm volatile ("nop");
asm volatile ("nop");

//_RTCEN = 0; // disable the module

//_RTCPTR = 3; // start the loading sequence
//RTCVAL = 0x2010; // YEAR
//RTCVAL = 0x1216; // MONTH-1/DAY-1
//RTCVAL = 0x0403; // WEEKDAY/HOURS
//RTCVAL = 0x0130; // MINUTES/SECONDS


}

/******************************************************************************/
//
/******************************************************************************/
void disableHw(void)
{
    INT0_DISABLE;
    INT0_CLEAR;
    INT1_DISABLE;
    INT1_CLEAR;
    TIMER1_DISABLE;
    TIMER1_INTERRUPT_CLEAR;
    TIMER2_DISABLE;
    TIMER2_INTERRUPT_CLEAR;
    ADC_DISABLE;
    ADC_INTERRUPT_CLEAR;
}
