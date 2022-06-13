#include "lab04.h"
#define _USE_MATH_DEFINES // for C
#include <math.h>
#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "math.h"

#include "types.h"
#include "lcd.h"
#include "led.h"
#include "dac.h"

// signal parameter
#define sin_frequency 10
#define min_volt 1
#define max_volt 3
#define sample_rate  20*sin_frequency
#define offset 2


#define pi 3.14159265358979323846

#define calc_timer_ticks(f) (FCY/(256.0*(20*f)))



/*
 * Timer code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03


int count=1;

void timer_initialize()
{
    // Timer 3 for measuring
    CLEARBIT(T3CONbits.TON); // Disable Timer
    CLEARBIT(T3CONbits.TCS); // Select internal instruction cycle clock
       
    CLEARBIT(T3CONbits.TGATE); // Disable Gated Timer mode
    TMR3 = 0x00; // Clear timer register
    T3CONbits.TCKPS = 0b11; // Select 1:1 Prescaler
    PR3 = calc_timer_ticks(sin_frequency); // Load the period value
    
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    CLEARBIT(IFS0bits.T3IF); // Clear Timer3 Interrupt Flag
    SETBIT(IEC0bits.T3IE); // Enable Timer3 interrupt
    
    SETBIT(T3CONbits.TON); // Start Timer 
    
}

float Voltage_calculated(float t)
{
    // Calculate the output voltage of sin function
    float v_out = ((max_volt-min_volt)*sin(2.0*pi*sin_frequency*t))+offset;
    return v_out;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void) { 
    TOGGLEBIT(LED1_PORT);
    Nop();
    float v_out=Voltage_calculated(1.0/200*count);
    v_out = (v_out/4.096)*4096;//convert the v_out between 0V-4.096V and to 12 bits binary value
    count++;
    uint16_t v = v_out+4096; //choose the mode among 13-16 bits
    dac_convert_milli_volt(v);
    CLEARBIT(IFS0bits.T3IF);
    

}



/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab04: Wave");
    lcd_locate(0, 1);
    lcd_printf("Group: GroupName");
    
    while(TRUE) { }
}
