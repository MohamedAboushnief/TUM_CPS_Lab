#include "lab02.h"

#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

#define FCY_EXT 32768

volatile int seconds = 0;
volatile int milliseconds = 0;
volatile int minutes=0;


void initialize_timer() {
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    // Disable the Timers

    // Set Prescaler

    // Set Clock Source

    // Set Gated Timer Mode -> don't use gating

    // T1: Set External Clock Input Synchronization -> no sync

    // Load Timer Periods

    // Reset Timer Values

    // Set Interrupt Priority

    // Clear Interrupt Flags

    // Enable Interrupts

    // Enable the Timers




    // Timer 1 1Sec for LED2
    CLEARBIT(T1CONbits.TON); //Disable Timer
    SETBIT(T1CONbits.TCS); //Select external clock  32.768 kHz 
    CLEARBIT(T1CONbits.TSYNC); //Disable Synchronization     
    T1CONbits.TCKPS = 0b11; //Select 1:256 Prescaler
    TMR1 = 0x00; //Clear timer register
    PR1 = 128; //Load the period value
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag
    SETBIT(IEC0bits.T1IE); // Enable Timer1 interrupt
    SETBIT(T1CONbits.TON); // Start Timer  

    // Timer 2 5ms for LED1
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    PR2 = 100; // Load the period value
    IPC1bits.T2IP = 0x01; // Set Timer2 Interrupt Priority Level
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 Interrupt Flag
    SETBIT(IEC0bits.T2IE); // Enable Timer2 interrupt
    SETBIT(T2CONbits.TON); // Start Timer 
    
    // Timer 3 for measuring
    CLEARBIT(T3CONbits.TON); // Disable Timer
    CLEARBIT(T3CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T3CONbits.TGATE); // Disable Gated Timer mode
    TMR3 = 0x00; // Clear timer register
    T3CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    PR3 = 65535; // Load the period value
    SETBIT(T3CONbits.TON); // Start Timer 
    


}

void timer_loop() {
    // print assignment information
    lcd_printf("Lab02: Int & Timer");
    lcd_locate(0, 1);
    lcd_printf("Group: Group2");
    
    lcd_locate(0, 5);
    lcd_printf("Time: ");
    
    
    lcd_locate(0, 6);
    lcd_printf("Cycles: ");
    
    lcd_locate(0, 7);
    lcd_printf("d: ");



    
    int count= 0;
    while (TRUE) {
        count++;
        
        if(count==2000)
        {
            TOGGLEBIT(LED3_PORT);    // not sure if it is toggling at the right speed
            Nop();
            count=0;
            
            // display the time
            lcd_locate(6, 5);
            lcd_printf("%d",minutes);
            lcd_printf(":");
            lcd_printf("%d",seconds);
            lcd_printf(":");
            lcd_printf("%d",milliseconds);
            
            
            lcd_locate(8, 6);
            
            //print cycles
            lcd_printf("%u",TMR3);
            
            
            
            //print period
            float period = ((float)((float)(TMR3/1000))/12800);
            period = period * 1000;
            lcd_locate(8, 7);
            lcd_printf("%f", period);
            
            TMR3=0x00;






            
            

        }
        
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void) { // invoked every 1 Sec

    TOGGLEBIT(LED2_PORT);
    Nop();
    CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag
    seconds++;
    
    if(seconds==60)
    {
        seconds=0;
        minutes++;
    }
    
    // we will increment seconds independently of ms while we think it is not correct
    // and we should only use the millisecond for counting 



}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void) { // invoked every 2 ms
    TOGGLEBIT(LED1_PORT);
    Nop();
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 Interrupt Flag
    milliseconds=milliseconds+2;
    
    if(milliseconds>=999)
    {
        milliseconds=0;
    }

}
