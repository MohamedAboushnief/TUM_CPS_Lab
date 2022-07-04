#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


typedef enum{
    SERVO_1,
    SERVO_2
}SERVO;


/*
 * PWM code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

/*
 * touch screen code
 */
void pwm_init(SERVO servo_x ){
    
    
    if(servo_x==SERVO_1)
    {
            //setup Timer 2
        CLEARBIT(T2CONbits.TON); // Disable Timer
        CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
        CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
        TMR2 = 0x00; // Clear timer register
        T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
        CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
        PR2 = 4000; // Set timer period 20ms: // 4000= 20*10^-3 * 12.8*10^6 * 1/64
        //setup OC8
        CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
        OC8R = 1000; /* Set the initial duty cycle to 5ms*/
        OC8RS = 1000; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */
        SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    }
    else if(servo_x==SERVO_2)
    {
            //setup Timer 2
        CLEARBIT(T2CONbits.TON); // Disable Timer
        CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
        CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
        TMR2 = 0x00; // Clear timer register
        T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
        CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
        CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
        PR2 = 4000; // Set timer period 20ms: // 4000= 20*10^-3 * 12.8*10^6 * 1/64
        //setup OC8
        CLEARBIT(TRISDbits.TRISD6); /* Set OC7 as output */
        OC7R = 1000; /* Set the initial duty cycle to 5ms*/
        OC7RS = 1000; /* Load OCRS: next pwm duty cycle */
        OC7CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */
        SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
    }
    

}


// initializing servo motors
void servo_init(SERVO servo_x)
{
    if(servo_x==SERVO_1)
    {
        
    }
    else if(servo_x==SERVO_2)
    {
        
    }
    
}

/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: GroupName");
    
    // initialize touchscreen
    
    // initialize servos
    
    while(TRUE) {
        
    }
}
