/*
 * File:   timers.c
 * Author: IdeaPad
 *
 * Created on 21. marts 2016, 10:16
 */


#include "xc.h"



    void TimerInit(void) {
        

        
        PR1 = 0x8000;//4000; //set to (2^13), since 32.768kHz / 2^13 = 2 Hz /// FIND a better value
        IPC0bits.T1IP = 5; //set interrupt priority
        T1CON = 0b1000000000000010; //turn on the timer
        IFS0bits.T1IF = 0; //reset interrupt flag
        IEC0bits.T1IE = 1; //turn on the timer1 interrupt
    }

