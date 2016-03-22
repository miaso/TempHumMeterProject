/*
 * File:   uart1.c
 * Author: IdeaPad
 *
 * Created on 1. marts 2016, 13:03
 */


#include "xc.h"

#include "uart1.h"
#include "extralDefines.h"

//Initiation function
//parameter BAUDRATEREG1 determines baud speed
void UART1Init(int BAUDRATEREG1)
{
    uart1rx_trs=1;
   //Set up registers
   U1BRG = BAUDRATEREG1;	//set baud speed
   U1MODE	=	0x8000;	 //turn on module
   U1STA	=	0x8400;	 //set interrupts
   //reset RX interrupt flag
   IFS0bits.U1RXIF = 0;
}

//UART transmit function, parameter Ch is the character to send
void UART1PutChar(char Ch)
{
   //transmit ONLY if TX buffer is empty
   while(U1STAbits.UTXBF == 1);
   U1TXREG = Ch;
}

void UART1PutStr(char* s){
    	char c = 0;
        for (;(( c=*s)!=0);s++){
		UART1PutChar(*s); 
	}
}

//UART receive function, returns the value received.
char UART1GetChar()
{
   char Temp;
   //wait for buffer to fill up, wait for interrupt
   while(IFS0bits.U1RXIF == 0);
   Temp = U1RXREG;
   //reset interrupt
   IFS0bits.U1RXIF = 0;
   //return my received byte
   return Temp;
}