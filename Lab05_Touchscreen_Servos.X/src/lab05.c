
#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


// C function showing how to do time delay
#include <stdio.h>
// To use time library of C
#include <time.h>

typedef enum {
    SERVO_1,
    SERVO_2
} SERVO;

typedef enum {
    x,
    y

} DIMENSION;


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

void adc_init(void) {
    //disable ADC
    CLEARBIT(AD1CON1bits.ADON);
    //disable ADC
    //CLEARBIT(AD2CON1bits.ADON);

    //initialize PINS
    SETBIT(TRISEbits.TRISE8);
    //SETBIT(TRISBbits.TRISB15); //set TRISE RB15 to input
    CLEARBIT(AD1PCFGLbits.PCFG15); //set AD1 AN15 input pin as analog

    //SETBIT(TRISBbits.TRISB9); //set TRISE RB9  to input
    CLEARBIT(AD1PCFGLbits.PCFG9); //set AD2 AN9 input pin as analog

    //Configure AD1CON1
    CLEARBIT(AD1CON1bits.AD12B); //set 10b Operation Mode
    AD1CON1bits.FORM = 0; //set integer output
    AD1CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD1CON2
    AD1CON2 = 0; //not using scanning sampling
    //Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC); //internal clock source
    AD1CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD1CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD1CON4 at its default value
    //enable ADC
    SETBIT(AD1CON1bits.ADON);
    
    //Configure AD2CON1
    /*CLEARBIT(AD2CON1bits.AD12B); //set 10b Operation Mode
    AD2CON1bits.FORM = 0; //set integer output
    AD2CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD2CON2
    AD2CON2 = 0; //not using scanning sampling
    //Configure AD2CON3
    CLEARBIT(AD2CON3bits.ADRC); //internal clock source
    AD2CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD2CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD2CON4 at its default value
    //enable ADC
    SETBIT(AD2CON1bits.ADON);*/
}

/*
 * touch screen code
 */
void touch_screen_init(void) {

    adc_init();
    //set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1); //I/O pin set to output
    Nop();
    Nop();
    CLEARBIT(TRISEbits.TRISE2); //I/O pin set to output
    Nop();
    Nop();
    CLEARBIT(TRISEbits.TRISE3); //I/O pin set to output
    Nop();
    Nop();

    // x,y standby mode

    SETBIT(PORTEbits.RE1);
    Nop();
    Nop();
    SETBIT(PORTEbits.RE2);
    Nop();
    Nop();
    CLEARBIT(PORTEbits.RE3);
    Nop();
    Nop();


}

touch_screen_dimension_set(DIMENSION dim) {
    if (dim == x) {
        // set x coordinate as read
        
        CLEARBIT(PORTEbits.RE1);
        Nop();
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();
        Nop();
        __delay_ms(50);
    } else if (dim == y) {
        // set y coordinate as read
        
        SETBIT(PORTEbits.RE1);
        Nop();
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();
        Nop();
        __delay_ms(50);
    }
}

uint16_t touch_screen_position_read_x(void) {
    
    AD1CHS0bits.CH0SA = 0x0F; //set ADC to Sample AN15 pin
    SETBIT(AD1CON1bits.SAMP); //start to sample
    while (!AD1CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC1BUF0; //return sample

}

uint16_t touch_screen_position_read_y(void) {
    
    AD1CHS0bits.CH0SA = 0x09; //set ADC to Sample AN9 pin
    SETBIT(AD1CON1bits.SAMP); //start to sample
    while (!AD1CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC1BUF0; //return sample

}

void init_timer() {
    //setup Timer 2
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    PR2 = 4000; // Set timer period 20ms: // 4000= 20*10^-3 * 12.8*10^6 * 1/64
    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */
}

void pwm_init(SERVO servo_x) {


    if (servo_x == SERVO_1) {

        //setup OC8
        CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
        OC8R = 3820; /* Set the initial duty cycle to 5ms*/
        OC8RS = 3820; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */

    } else if (servo_x == SERVO_2) {

        //setup OC8
        CLEARBIT(TRISDbits.TRISD6); /* Set OC7 as output */
        OC7R = 3820; /* Set the initial duty cycle to 5ms*/
        OC7RS = 3820; /* Load OCRS: next pwm duty cycle */
        OC7CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */

    }


}


// initializing servo motors

void servo_init(SERVO servo_x) {
    if (servo_x == SERVO_1) {
        pwm_init(SERVO_1);
    } else if (servo_x == SERVO_2) {
        pwm_init(SERVO_2);
    }

}

void servo_duty_cycle_set(SERVO servo_x, float duty_cycle) {
    if (servo_x == SERVO_1) {

        OC8R = (duty_cycle / 20.0)*4000;
        OC8RS = (duty_cycle / 20.0)*4000; /* Load OCRS: next pwm duty cycle */

    } else if (servo_x == SERVO_2) {
        OC7R = (duty_cycle / 20.0)*4000;
        OC7RS = (duty_cycle / 20.0)*4000; /* Load OCRS: next pwm duty cycle */
    }

}

void servo_control(float servo_1, float servo_2) {
    servo_duty_cycle_set(SERVO_1, servo_1);
    Nop();
    Nop();
    servo_duty_cycle_set(SERVO_2, servo_2);
    Nop();
    Nop();
}

/*
 * main loop
 */

void main_loop() {
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: GroupName");

    // initialize timer2
    init_timer();
    // initialize touchscreen

    // initialize servos
    servo_init(SERVO_1);
    Nop();
    Nop();
    servo_init(SERVO_2);
    Nop();
    Nop();

    touch_screen_init();


    

    int count = 0;
    __delay_ms(5000);
    while (TRUE) {


      if (count == 0) {
            servo_control(20 - 0.9, 20 - 0.9);
            
            
        } else if (count == 1) {
            servo_control(20 - 0.9, 20 - 2.1);
            
            
        } else if (count == 2) {
            servo_control(20 - 2.1, 20 - 2.1);
            
        } else if (count == 3) {
            servo_control(20 - 2.1, 20 - 0.9);
            

        }


        count++;

        if (count == 4) {
            count = 0;
        }

        
        



        


        touch_screen_dimension_set(x);
        uint16_t x_pos = touch_screen_position_read_x();
        touch_screen_dimension_set(y);
        uint16_t y_pos = touch_screen_position_read_y();
        lcd_locate(0, 7);
        lcd_printf("                ");
        lcd_locate(0, 7);
        lcd_printf("X/Y: %d/%d", x_pos, y_pos);
        
        
        
        __delay_ms(4000);


        






    }
}