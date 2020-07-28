/*
 * File:   IncubatorRoutine.c
 * Author: rjalva
 *
 * Created on October 24, 2019, 2:49 PM
 */

// PIC16F1829 Configuration Bit Settings
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>


/*Serial Configuration*/
#define BAUD 9600   //Bits per second transfer rate
#define FOSC 4000000L   //Frequency Oscillator
#define DIVIDER ((int)(FOSC/(16UL * BAUD) -1))  //Should be 25 for 9600/4MhZ
#define NINE_BITS 0
#define SPEED 0x4       //T Speed
#define RX_PIN TRISC5   //Recieve Pin
#define TX_PIN TRISC4   //Transmit Pin
/*Xtal Macro*/
#define _XTAL_FREQ 4000000.0    /*for 4mhz*/
//*Function Prototypes*//
void pinConfig(void);
void setup_comms(void);
void putch(unsigned char);
unsigned char getch(void);
unsigned char getche(void);
//*Global Vars*//
unsigned int k=0, time[5]={0}, Ctemp;

int main(int argc, char** argv) {
 //int a = 35;
 pinConfig();
 setup_comms(); // set up the USART - settings defined in usart.h
 // Get set up for A2D
 ADCON1 = 0xC0; //Right justify and Fosc/4 and Vss and Vdd references
 while (1){
    /*Internal Temperature Read*/
        FVRCON = 0b1111010;
        ADCON0 = 0x75; // set up for the Temperature input channel
        __delay_ms(10); //Allow cap to recharge
        ADGO = 1; // initiate conversion on the selected channel
        while(ADGO)continue;
        Ctemp = ((0.659-((1.65)*(1-((float)((ADRESH<<8)+(ADRESL))/1023))))/0.00132)-40+554.5; //Store 10 bits into temp, 8 + 2
    /*Write to Terminal + Temperature Math*/
        if(time[4]==0)//14 days have not passed
            if(time[1]<1&&time[0]<28)
                RA5=1;//relay to motor is on
            else
                RA5=0;//relay to motor is off
        else
            RA5=0;
        printf("Temp in C is %d  | Time= %d day %d hr %d min %d sec  \n\r", Ctemp, time[3], time[2], time[1], time[0]);
        __delay_ms(1000);
        //if (a==35){ //not enough program space to implement
            if (Ctemp < 37)
                RA2 = 1; //lightbulb turns on when less than 37 C
            else
                RA2 = 0; //lightbulb turns off when greater than 37 C
        /*}
        if (Ctemp < a){
            RC6 = 1; //Backup lightbulb turns on when drops below 35 C  
            a = 37; //Backup lightbulb will now function as primary lightbulb and code for original lightbulb will be ignored
        }
        else
            RC6 = 0;*/
 }
 return (EXIT_SUCCESS);
}
void setup_comms(void){
    RX_PIN = 1;
    TX_PIN = 1;
    SPBRG = DIVIDER;
    RCSTA = (NINE_BITS | 0x90);
    TXSTA = (SPEED | NINE_BITS | 0x20);
    TXEN = 1;
    SYNC = 0;
    SPEN = 1;
    BRGH = 1;
}
void putch(unsigned char byte){
    /* output one byte */
    while(!TXIF) /* set when register is empty */
    continue;
    TXREG = byte;
}
unsigned char getch(){
    /* retrieve one byte */
    while(!RCIF) /* set when register is not empty */
    continue;
    return RCREG;
}
unsigned char getche(void){
    unsigned char c;
    putch(c = getch());
    return c;
}
void pinConfig(void){
 OSCCON = 0x6A; /* b6..4 = 1101 = 4MHz */
 TXCKSEL = 1; // both bits in APFCON0 MUST BE 1 for 1829 0 for 1825
 RXDTSEL = 1; /* makes RC4 & 5 TX & RX for USART (Allows ICSP)*/
 INTCON = 0xA0;
 OPTION_REG = 0x8F;
 TRISA = 0x00;
 ANSELA =0x00;
}

int __interrupt () ISR (void) {
    if(TMR0IF){//Check if TMR0 caused the interrupt 256/1000000
        k++;
        TMR0IF=0;//Clear interrupt flag
        if(k==3906){//256*62*63/1000000 = .999936 seconds
            time[0]++;//increment every second
            k=0;
            if(time[0]==60){
                time[1]++;//increment every minute
                time[0]=0;//seconds reset every minute
                if(time[1]==60){//256*62*63*60*60/1000000 = 3599.7696 s = 59.99616 min = .999936 hr
                    time[2]++;//increment every hour
                    time[1]=0;//minutes reset every hour
                    if(time[2]==24){
                        time[3]++;//increment every day
                        time[2]=0;//hours reset every day
                        if(time[3]==14){//256*62*63*60*60*24*14/1000000 = 1209522.586 s = 20158.70976 min = 335.978496 hr = 13.999104 days
                         time[4]=1;//signals that 14 days have passed
                        }
                    }                        
                }
            }
        }
    return(time[5]);
    }
}