/*
 * File:   dht.c
 * Author: IdeaPad
 *
 * Created on 2. marts 2016, 11:12
 */


#include "xc.h"
#include "extralDefines.h"
#define FCY     16000000ULL //TO DO
#include <libpic30.h>




unsigned char Check, T_byte1, T_byte2,
RH_byte1, RH_byte2, Ch;
unsigned Temp, RH, Sum;

void StartSignal() {

    DHT_TRS = 0; //Configure RD0 as output
    // DHT=1 ;
    //__delay_ms(250);
    DHT = 0; //RD0 sends 0 to the sensor
    __delay_ms(20);
    DHT = 1; //RD0 sends 1 to the sensor

    __delay_us(30);
    DHT_TRS = 1; //Configure RD0 as input

}
//////////////////////////////

void CheckResponse() {
    Check = 0;
    __delay_us(50);

    if (DHT == 0) {
        __delay_us(80);


        if (DHT == 1) {
            Check = 1;
            __delay_us(40);
        }
    }

}
//////////////////////////////

char ReadData() {
    char i, j;
    for (j = 0; j < 8; j++) {
        while (!DHT); //Wait until PORTD.F0 goes HIGH
        __delay_us(30);
        if (DHT != 0)
            i &= ~(1 << (7 - j)); //Clear bit (7-b)
        else {
            i |= (1 << (7 - j)); //Set bit (7-b)
            while (DHT);
        } //Wait until PORTD.F0 goes LOW
    }
    return i;
}

