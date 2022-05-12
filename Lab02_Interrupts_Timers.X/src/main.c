#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"
#include "lab02.h"

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// OSC2 Pin Function: OSC2 is Clock Output
#pragma config OSCIOFNC = ON
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF


void init_interrupts()
{
    // Enable Interrupts
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    
    IPC1bits.T2IP = 0x01; // Set Timer2 Interrupt Priority Level
    IFS0bits.T2IF = 0; // Clear Timer2 Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Timer2 interrupt
    
//    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
//    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
//    IEC0bits.T3IE = 1; // Enable Timer3 interrupt
    



}

int main(){
    //Init LCD and LEDs
    lcd_initialize();
    led_init();
	
    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);
    
    init_interrupts();
    
    // Initialize Lab 02 Timers
    initialize_timer();
    
    // Start Lab02 Main Program
    timer_loop();
    
    
    
    
    
    // Stop
    while(1)
        ;
}

