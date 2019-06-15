//****************************************************************************
//Author:		Paul Jerrold Biglete
//Date:  		4-22-2018
//Version:		0.75
//Device:		16f18324
//Description:	Mini Project - Digital Thermometer
//Complier:		XC8
//
//****************************************************************************
//****************************************************************************
// Configuration
// Window -> PIC Memory Views -> Configuration Bits
//****************************************************************************
// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits->Oscillator not enabled
#pragma config RSTOSC = HFINT1    // Power-up default value for COSC bits->HFINTOSC
#pragma config CLKOUTEN = OFF    // Clock Out Enable bit->CLKOUT function is disabled; I/O or oscillator function on OSC2
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config MCLRE = ON    // Master Clear Enable bit->MCLR/VPP pin function is MCLR; Weak pull-up enabled
#pragma config PWRTE = OFF    // Power-up Timer Enable bit->PWRT disabled
#pragma config WDTE = OFF    // Watchdog Timer Enable bits->WDT disabled; SWDTEN is ignored
#pragma config LPBOREN = OFF    // Low-power BOR enable bit->ULPBOR disabled
#pragma config BOREN = ON    // Brown-out Reset Enable bits->Brown-out Reset enabled, SBOREN bit ignored
#pragma config BORV = LOW    // Brown-out Reset Voltage selection bit->Brown-out voltage (Vbor) set to 2.45V
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a Reset
#pragma config DEBUG = OFF    // Debugger enable bit->Background debugger disabled

// CONFIG3
#pragma config WRT = OFF    // User NVM self-write protection bits->Write protection off
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.

// CONFIG4
#pragma config CP = OFF    // User NVM Program Memory Code Protection bit->User NVM code protection disabled
#pragma config CPD = OFF    // Data NVM Memory Code Protection bit->Data NVM code protection disabled

//****************************************************************************
// Includes
//****************************************************************************
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//#include "pic16f18324.h";

//****************************************************************************
// Global Variables
//****************************************************************************
#define LOAD_CS PORTCbits.RC3
#define _XTAL_FREQ 4000000

uint16_t count = 0;
uint8_t ADC_H;
int Hot_Flag = 0;
int Cold_Flag = 0;

void ADC_Init();
void Timer0_Init();
void MAX7219_Init();
void MAX7219_Transfer(uint8_t address, uint8_t value);
uint8_t SPI_SHIFT_8(uint8_t data);
void TestDisplay();
void UpdateDisplay();

//****************************************************************************
// Main
//****************************************************************************
void main(void)
{     
    OSCCON1 = 0x60; // HFINTOSC   
    OSCFRQ = 0x03;  // HFFRQ 4_MHz; 
    
    ADC_Init();
    Timer0_Init();
      
    TRISC = 0b1111111; //disable output driver   
    TRISCbits.TRISC5 = 0; //Set as Output, SDO
    TRISCbits.TRISC4 = 0; //Set as Output, CLK
    TRISCbits.TRISC3 = 0; //Set as Output, Slave Select
    TRISCbits.TRISC2 = 1; //Temperature Sensor Pin to Analog Input
    
    /*
        RC5 -> SDO
        RC4 -> SCK
        RC3 -> CS
    */
    
    //SPI Init  
    SSP1CON1 =  0b00110000; //SSPEN = 1, CKP = 1, SPI Master Mode = FOSC/4
    RC5PPS = 0b11001; //SDO onto DI on RC5
    RC4PPS = 0b11000; //SCK on RC4    
    PPSLOCK = 1;
    
    MAX7219_Init();
    TestDisplay();
    
    //Set Brightness
    MAX7219_Transfer(0x0A, 0x02); // 21%
    
    TMR0IF = 0;
        
    while (1)
    {
       __delay_ms(150); //Update Display every 150 ms
       UpdateDisplay();  
    } 
}

void ADC_Init()
{
    //ADC Conversion    
    TRISCbits.TRISC2 = 1; //Set C2 to Tri-state    
    ANSELCbits.ANSC2 = 1; //Set C2 as Analog Input   
    ADCON0 = 0b01001001; //ACS to ANC2, ADON enable
    ADCON1 = 0b01000011;  //ADC ---> FOSC/4, left justified
    FVRCON = 0b11000010; //Fixed Voltage, ADC FVR Buffer set to 2.048V
}

void MAX7219_Init()
{
    //MAX7219 Init
    LOAD_CS = 0;
    MAX7219_Transfer(0x00, 0x00); //Clear Bits
    MAX7219_Transfer(0x0F, 0x01); //Display Test
    MAX7219_Transfer(0x0F, 0x00); //Turns off Display
    MAX7219_Transfer(0x09, 0x0F); //Code B decode for digits 3-0
    MAX7219_Transfer(0x0A, 0x0F); //Intensity to 31/32
    MAX7219_Transfer(0x0B, 0x01); //Scan for Two Digits Only
    MAX7219_Transfer(0x0C, 0x01); //Enable Normal Operation    
    MAX7219_Transfer(0x00, 0x00); //Clear Bits
}

void Timer0_Init()
{
    T0CON0 = 0b10000000; //enable timer 0
    T0CON1 = 0b01000110; //enable FOSC/4 and set Prescale to 64   
    PIE0 = 0b00100000; //enable TMR0 overflow interrupt enable bit
    INTCONbits.GIE = 1; // enable Interrupt Enable, 
    TMR0Hbits.TMR0H = 157; //10 ms  
}

void MAX7219_Transfer(uint8_t address, uint8_t value)
{
    SPI_SHIFT_8(address);
    SPI_SHIFT_8(value);
    LOAD_CS = 0; //LOW
    __delay_ms(5);
    LOAD_CS = 1; //HIGH
}

void interrupt my_isr(void)
{    
    if(TMR0IF && TMR0IE)
    {                
       while(ADCON0bits.ADGO == 1){} //Conversion Done? if not start again  
       ADC_H = ADRESH;
           
       TMR0IF = 0;
       ADCON0bits.ADGO = 1;      
    }
}

void WarningFlash()
{
    if(Hot_Flag == 1)
    {
        for(int i = 0; i < 3; i++)
        {
            __delay_ms(750);
            MAX7219_Transfer(0x01, 12); // H
            MAX7219_Transfer(0x02, 12); // H
            __delay_ms(500);
            MAX7219_Transfer(0x01, 15); // blank
            MAX7219_Transfer(0x02, 15); // blank      
        }
        Hot_Flag = 0;
    }
    
    if(Cold_Flag == 1)
    {
        for(int i = 0; i < 3; i++)
        {
            __delay_ms(750);
            MAX7219_Transfer(0x01, 13); // L
            MAX7219_Transfer(0x02, 13); // L
            __delay_ms(500);
            MAX7219_Transfer(0x01, 15); // blank
            MAX7219_Transfer(0x02, 15); // blank  
        }
        Cold_Flag = 0;
    }
}

void UpdateDisplay()
{
       if(ADC_H >= 19 && ADC_H <= 28) //0F to 9F
       {
          MAX7219_Transfer(0x01, 0);   //First Digit       
          
          //Second Digit
          if(ADC_H == 20) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 21) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 22) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 23) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 24) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 25) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 26) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 27) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 28) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 29 && ADC_H <= 38) //10F to 19F
       {
          MAX7219_Transfer(0x01, 1);   //First Digit       
          
          //Second Digit
          if(ADC_H == 30) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 31) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 32) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 33) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 34) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 35) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 36) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 37) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 38) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 39 && ADC_H <= 48) //20F to 29F
       {
          MAX7219_Transfer(0x01, 2);   //First Digit       
          
          //Second Digit
          if(ADC_H == 40) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 41) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 42) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 43) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 44) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 45) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 46) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 47) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 48) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 49 && ADC_H <= 58) //30F to 39F
       {
          MAX7219_Transfer(0x01, 3);   //First Digit       
          
          //Second Digit
          if(ADC_H == 50) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 51) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 52) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 53) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 54) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 55) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 56) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 57) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 58) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 59 && ADC_H <= 68) //40F to 49F
       {
          MAX7219_Transfer(0x01, 4);   //First Digit       
          
          //Second Digit
          if(ADC_H == 60) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 61) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 62) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 63) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 64) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 65) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 66) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 67) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 68) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 69 && ADC_H <= 78) //50F to 59F
       {
          MAX7219_Transfer(0x01, 5);   //First Digit       
          
          //Second Digit
          if(ADC_H == 70) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 71) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 72) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 73) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 74) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 75) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 76) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 77) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 78) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 79 && ADC_H <= 88) //60F to 69F
       {
          MAX7219_Transfer(0x01, 6);   //First Digit       
          
          //Second Digit
          if(ADC_H == 80) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 81) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 82) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 83) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 84) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 85) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 86) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 87) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 88) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 89 && ADC_H <= 98) //70F to 79F
       {
          MAX7219_Transfer(0x01, 7);   //First Digit       
          
          //Second Digit
          if(ADC_H == 90) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 91) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 92) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 93) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 94) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 95) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 96) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 97) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 98) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 99 && ADC_H <= 108) //80F to 89F
       {
          MAX7219_Transfer(0x01, 8);   //First Digit       
          
          //Second Digit
          if(ADC_H == 100) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 101) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 102) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 103) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 104) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 105) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 106) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 107) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 108) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 109 && ADC_H <= 118) //80F to 89F
       {
          MAX7219_Transfer(0x01, 8);   //First Digit       
          
          //Second Digit
          if(ADC_H == 110) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 111) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 112) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 113) MAX7219_Transfer(0x02, 4);
          else if (ADC_H == 114) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 115) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 116) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 117) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 118) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       }
       else if(ADC_H >= 119 && ADC_H <= 128) //90F to 99F
       {
          MAX7219_Transfer(0x01, 9);   //First Digit       
          
          //Second Digit
          if(ADC_H == 120) MAX7219_Transfer(0x02, 1);
          else if (ADC_H == 121) MAX7219_Transfer(0x02, 2);
          else if (ADC_H == 122) MAX7219_Transfer(0x02, 3);
          else if (ADC_H == 123) MAX7219_Transfer(0x02, 4); 
          else if (ADC_H == 124) MAX7219_Transfer(0x02, 5);
          else if (ADC_H == 125) MAX7219_Transfer(0x02, 6);
          else if (ADC_H == 126) MAX7219_Transfer(0x02, 7);
          else if (ADC_H == 127) MAX7219_Transfer(0x02, 8);
          else if (ADC_H == 128) MAX7219_Transfer(0x02, 9);
          else MAX7219_Transfer(0x02, 0);              
       } 
       else //if no matching ADC value to temp, output EE on display
       {
          MAX7219_Transfer(0x01, 11); // E
          MAX7219_Transfer(0x02, 11); // E
       }
       
       if(ADC_H >= 123) //if Temp is 95 degrees, flash display
       {         
           Hot_Flag = 1;
           WarningFlash();
       }
       
       if(ADC_H <= 59) // if Temp is below or equal to 40, flash display
       {
           Cold_Flag = 1;
           WarningFlash();
       }
}

void TestDisplay()
{
    int i = 0; 
    int j = 0;
    
    __delay_ms(500);
    MAX7219_Transfer(0x01, 14); // P
    MAX7219_Transfer(0x02, 14); // P
    __delay_ms(500);
    
    /* Loop Through 00 - 99 to test display */
    for(i = 0; i < 10; i++)
    {
        MAX7219_Transfer(0x0A, 0x00); //Adjust Brightness 3%
        switch(i)
        {
            case 0:
                MAX7219_Transfer(0x0A, 0x01); // 9%
                break;
            case 1:
                MAX7219_Transfer(0x0A, 0x03); // 21%
                break;
            case 2:
                MAX7219_Transfer(0x0A, 0x05); // 34%
                break;
            case 3:
                MAX7219_Transfer(0x0A, 0x07); // 46%
                break;
            case 4: 
                MAX7219_Transfer(0x0A, 0x09); // 60%
                break;
            case 5:
                MAX7219_Transfer(0x0A, 0x0B); // 72%
                break;
            case 6:
                MAX7219_Transfer(0x0A, 0x0C); // 78%
                break;
            case 7:
                MAX7219_Transfer(0x0A, 0x0D); // 84%
                break;
            case 8:
                MAX7219_Transfer(0x0A, 0x0E); // 90%
                break;
            case 9:
                MAX7219_Transfer(0x0A, 0x0F); // 97%
                break;
            default:
                MAX7219_Transfer(0x0A, 0x0F); // 97%
                break;               
        }
        
        MAX7219_Transfer(0x01, i);
        for(j = 0; j < 10; j++)
        {
            __delay_ms(25);
            MAX7219_Transfer(0x02, j);
        }          
    }
      
    __delay_ms(500);
    MAX7219_Transfer(0x01, 10); // Dash
    MAX7219_Transfer(0x02, 10); // Dash
    __delay_ms(500);
    
    MAX7219_Transfer(0x00, 0x00); //Clear Bits
}

uint8_t SPI_SHIFT_8(uint8_t data)
{
    SSP1CONbits.WCOL = 0;
    SSP1BUF = data;
    while(SSP1STATbits.BF == 0){}
    return (SSP1BUF);
}
