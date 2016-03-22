/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.c

  @Summary:
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB(c) Code Configurator interrupts.
    Generation Information : 
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC24FJ128GA306
        Version           :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.26
        MPLAB             :  MPLAB X 3.20

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


/**
    Section: Includes
*/
#include <xc.h>
#include "pin_manager.h"

/**
    void PIN_MANAGER_Initialize(void)
*/
void PIN_MANAGER_Initialize(void)
{
    /****************************************************************************
     * Setting the GPIO of PORTB
     ***************************************************************************/
    LATB = 0x0;
 
    /****************************************************************************
     * Setting the GPIO of PORTF
     ***************************************************************************/
    LATF = 0x0;
 
    /****************************************************************************
     * Setting the GPIO of PORTE
     ***************************************************************************/
    LATE = 0x0;
 
    /****************************************************************************
     * Setting the GPIO of PORTD
     ***************************************************************************/
    LATD = 0x0;
 
    /****************************************************************************
     * Setting the GPIO of PORTC
     ***************************************************************************/
    LATC = 0x0;
 
    /****************************************************************************
     * Setting the GPIO of PORTG
     ***************************************************************************/
    LATG = 0x0;
 
    /****************************************************************************
     * Setting the GPIO of PORTF
     ***************************************************************************/
    TRISF = 0x7F;
 
    /****************************************************************************
     * Setting the GPIO of PORTG
     ***************************************************************************/
    TRISG = 0x3CC;
 
    /****************************************************************************
     * Setting the GPIO of PORTD
     ***************************************************************************/
    TRISD = 0xEFF;
 
    /****************************************************************************
     * Setting the GPIO of PORTE
     ***************************************************************************/
    TRISE = 0xFF;
 
    /****************************************************************************
     * Setting the GPIO of PORTB
     ***************************************************************************/
    TRISB = 0xFFFF;
 
    /****************************************************************************
     * Setting the GPIO of PORTC
     ***************************************************************************/
    TRISC = 0x9000;
 


    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR
     ***************************************************************************/
    ANSD = 0x0CC0;
    ANSE = 0x00F0;
    ANSB = 0xFFFF;
    ANSG = 0x03C0;

}
