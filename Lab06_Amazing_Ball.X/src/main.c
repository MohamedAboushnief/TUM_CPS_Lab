#include <xc.h>
//do not change the order of the following 2 lines
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"
#include "lab06.h"

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

int main(){
    //Init LCD, LED
    lcd_initialize();
    led_initialize();
	
    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);
    
    // Start Lab06 Main Program
    main_loop();
    
    // Stop
    while(1)
        ;
}
