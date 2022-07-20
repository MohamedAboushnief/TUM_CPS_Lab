#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


typedef enum {
    SERVO_1,
    SERVO_2
} SERVO;

typedef enum {
    x,
    y

} DIMENSION;

volatile int counterHundredHz = 0;
volatile int counterFiftyHz = 0;
volatile int counterFiveHz = 0;

volatile bool trigger_read_touchscreen = false;
volatile bool trigger_pd_controller = false;
volatile bool trigger_print_missed_deadlines = false;
/*
 * Parameter
 */
#define pi 3.14159265358979323846
#define center_x 409
#define center_y 371

#define radius 224
#define frequency 1
#define angular_velocity 2*pi*frequency





/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03



/*
 * Global Variables
 */
uint16_t x_pos = 0;
uint16_t y_pos = 0;
bool trigger_exec = false;
int deadlines_missed = 0;

/*
 * Timer Code
 */
void initialize_timer() {
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);


    // Timer 1 10 ms 
    CLEARBIT(T1CONbits.TON); //Disable Timer
    CLEARBIT(T1CONbits.TCS); //Select internal clock
    CLEARBIT(T1CONbits.TGATE); // Disable Gated Timer mode
    CLEARBIT(T1CONbits.TSYNC); //Disable Synchronization     
    T1CONbits.TCKPS = 0b11; //Select 1:256 Prescaler
    TMR1 = 0x00; //Clear timer register
    PR1 = 500; //Load the period value
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag
    SETBIT(IEC0bits.T1IE); // Enable Timer1 interrupt
    SETBIT(T1CONbits.TON); // Start Timer  


    //    //setup Timer 2
    //    CLEARBIT(T2CONbits.TON); // Disable Timer
    //    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    //    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    //    TMR2 = 0x00; // Clear timer register
    //    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    //    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    //    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    //    PR2 = 4000; // Set timer period 20ms: // 4000= 20*10^-3 * 12.8*10^6 * 1/64
    //    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */





}

/*
 * Servo Code
 */

void adc_init(void) {
    //disable ADC
    CLEARBIT(AD1CON1bits.ADON);

    //initialize PINS
    SETBIT(TRISBbits.TRISB15); //set TRISE RB15 to input
    CLEARBIT(AD1PCFGLbits.PCFG15); //set AD1 AN15 input pin as analog

    SETBIT(TRISBbits.TRISB9); //set TRISE RB9  to input
    CLEARBIT(AD1PCFGLbits.PCFG9); //set AD1 AN9 input pin as analog

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
}

void pwm_init(SERVO servo_x) {


    if (servo_x == SERVO_1) {

        //setup OC8
        CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
        OC8R = 3820; /* Set the initial duty cycle to 1,5ms*/
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

    servo_duty_cycle_set(SERVO_2, servo_2);
    Nop();
}

/*
 * Touch screen code
 */
void touch_screen_init(void) {

    adc_init();
    //set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1); //I/O pin set to output
    CLEARBIT(TRISEbits.TRISE2); //I/O pin set to output
    CLEARBIT(TRISEbits.TRISE3); //I/O pin set to output

    // x,y standby mode

    SETBIT(PORTEbits.RE1);
    SETBIT(PORTEbits.RE2);
    CLEARBIT(PORTEbits.RE3);


}

touch_screen_dimension_set(DIMENSION dim) {
    if (dim == x) {
        // set x coordinate as read
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();

    } else if (dim == y) {
        // set y coordinate as read
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();

    }
}

uint16_t touch_screen_position_read_x(void) {
    __delay_ms(10);
    AD1CHS0bits.CH0SA = 0x0F; //set ADC to Sample AN15 pin
    SETBIT(AD1CON1bits.SAMP); //start to sample
    while (!AD1CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC1BUF0; //return sample

}

uint16_t touch_screen_position_read_y(void) {
    __delay_ms(10);
    AD1CHS0bits.CH0SA = 0x09; //set ADC to Sample AN9 pin
    SETBIT(AD1CON1bits.SAMP); //start to sample
    while (!AD1CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return ADC1BUF0; //return sample

}

uint16_t butterworth_filter_x(x) {
    x = filter(x);
    return x;
}

uint16_t butterworth_filter_y(y) {
    y = filter(y);
    return y;
}

void read_touchscreen() {
    
    //Read the touchscreen
    touch_screen_dimension_set(x);
    x_pos = touch_screen_position_read_x();
    touch_screen_dimension_set(y);
    y_pos = touch_screen_position_read_y();


}

/*
 * PD Controller
 */
void pd_controller(x_pos, y_pos) {
    // filter x coordinate
    x_pos = butterworth_filter_x(x_pos);
    y_pos = butterworth_filter_y(y_pos);
    
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */








void print_missed_deadlines() {
    lcd_locate(0, 7);
    lcd_printf("missed deadlines: %u", deadlines_missed);

   
}

/*
 * main loop
 */
void main_loop() {
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: 2");
    lcd_locate(0, 2);

    initialize_timer();

    touch_screen_init();

    while (TRUE) {

        //        servo_control(20 - 0.9, 20 - 2.1);
        //        if (trigger_exec == true) {
        if (trigger_read_touchscreen == true) {
            read_touchscreen();
            trigger_read_touchscreen = false;

        }

        if (trigger_pd_controller == true) {
            pd_controller();
            trigger_pd_controller = false;
        }

        if (trigger_print_missed_deadlines == true) {
            print_missed_deadlines();
            trigger_print_missed_deadlines = false;
        }



        //            trigger_exec = false;
        //        }

        lcd_locate(0, 5);
        lcd_printf("x pos: %u", x_pos);
        lcd_locate(0, 6);
        lcd_printf("y pos: %u", y_pos);


    }
}
// triggered each 10 ms

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void) { // invoked every 10 ms


    CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag

    // 100Hz for Task1  , 10 ms
    counterHundredHz++;
    if (counterHundredHz >= 1) {
        counterHundredHz = 0;
        trigger_read_touchscreen = true;

    }

    // 50Hz for Task2 , 20 ms
    counterFiftyHz++;
    if (counterFiftyHz >= 2) {
        counterFiftyHz = 0;
        trigger_pd_controller = true;
    }

    // 5Hz for Task3 , 200 ms
    counterFiveHz++;
    if (counterFiveHz >= 20) {
        counterFiveHz = 0;
        trigger_print_missed_deadlines = true;
    }

    //    if (trigger_exec == true) {
    //        // deadline missed
    //        deadlines_missed++;
    //    }
    //
    //    trigger_exec = true;






}


// only one timer should be used

// don't forget to add saturation

// deadline 100Hz, display of deadline 5Hz