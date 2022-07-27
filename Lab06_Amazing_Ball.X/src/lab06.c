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


/*
 * Parameter
 */
#define pi 3.14159265358979323846
#define center_x 409
#define center_y 310

#define radius 200
#define frequency 2.5
#define angular_velocity_w 2*pi*frequency


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

volatile int counterHundredHz = 0;
volatile int counterFiftyHz = 0;
volatile int counterFiveHz = 0;

volatile bool trigger_read_touchscreen = false;
volatile bool trigger_pd_controller = false;
volatile bool trigger_print_missed_deadlines = false;
volatile bool trigger_exec = true;

// PID Variables
volatile double kpx = 0.3; // Proportional sControl Constant
volatile int kdx = 80; // Derivative Control Constant
volatile double kpy = 0.2; // Proportional Control Constant
volatile int kdy = 60; // Derivative Control Constant


float t = 0.01;

uint16_t x_current_pos = 0;
uint16_t y_current_pos = 0;
uint16_t x_current_filtered = 0;
uint16_t x_previous_pos = 0;
uint16_t x_previous_filtered = 0;

uint16_t y_current_filtered = 0;
uint16_t y_previous_pos = 0;
uint16_t y_previous_filtered = 0;
uint16_t x_set_pos = 0;
uint16_t y_set_pos = 0;

int error_x = 0;
int derivative_x = 0;
int last_error_x = 0;
int error_y = 0;
int derivative_y = 0;
int last_error_y = 0;
int theta_x = 0;
int theta_y = 0;
double dutycicle_x = .0;
double dutycicle_y = .0;


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

void read_touchscreen() {

    //Read the touchscreen
    touch_screen_dimension_set(x);
    x_current_pos = touch_screen_position_read_x();
    touch_screen_dimension_set(y);
    y_current_pos = touch_screen_position_read_y();


}

void setPointGenerator() {

    x_set_pos = center_x + radius * sin(angular_velocity_w * t);
    y_set_pos = center_y + radius * cos(angular_velocity_w * t);
    t = t + 0.01;
    // returns set point (X,Y) as they are located on circle
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 *
 * Butterworth formula from Matlab
 * [b,a]=butter(N,Wn,ftype)
 * N: Nth-order
 * Wn: Normalized cutoff frequency
 * ftype: high or low
 * Wn = cufOffFrequency/(0.5*samplingRate)      Nyquist Theorem
 * 
 * N=1
 * Wn=3/(0.5*50)=3/25=0.12
 * Using [b,a]=butter(1, 0.12, 'low') we got:
 * b =
 *      0.1602  0.1602
 * a = 
 *      1.0000  -0.6796
 */

uint16_t butterworth_filter_x(x) {
    x_current_filtered = (0.1602 * x) + (0.1602 * x_previous_pos) + (0.6796 * x_previous_filtered);
    x_previous_pos = x;
    x_previous_filtered = x_current_filtered;
    return x_current_filtered;
}

uint16_t butterworth_filter_y(y) {
    y_current_filtered = (0.1602 * y) + (0.1602 * y_previous_pos) + (0.6796 * y_previous_filtered);
    y_previous_pos = y;
    y_previous_filtered = y_current_filtered;
    return y_current_filtered;
}

/*
 * PD Controller
 */
void pd_controller(x_current_pos, y_current_pos) {
    setPointGenerator(); // generate setpoint x and y

    //        lcd_locate(0, 3);
    //        lcd_printf("x set pos: %u", x_set_pos);
    //        lcd_locate(0, 4);
    //        lcd_printf("y set pos: %u", y_set_pos);

    // filter x coordinate and y coordinate
    x_current_pos = butterworth_filter_x(x_current_pos);
    // calculate the error
    error_x = x_set_pos - x_current_pos;
    // calculate the derivative
    derivative_x = error_x - last_error_x;

    // calculate the control variable, period of the loop is 20ms
    theta_x = (kpx * error_x)+(kdx * (derivative_x / 20));

    // Saturate (map) the pwm control variable to protect motor


    last_error_x = error_x;

    y_current_pos = butterworth_filter_y(y_current_pos);
    //    lcd_locate(0, 3);
    //    lcd_printf("x_f_pos: %u", x_current_pos);
    //    lcd_locate(0, 4);
    //    lcd_printf("y_f_pos: %u", y_current_pos);


    // calculate the error
    error_y = y_set_pos - y_current_pos;
    // calculate the derivative
    derivative_y = error_y - last_error_y;
    // calculate the control variable, period of the loop is 20ms
    theta_y = (kpy * error_y)+(kdy * derivative_y / 20);

    // Saturate (map) the pwm control variable to protect motor


    last_error_y = error_y;



    if (theta_x<-300) {
        theta_x = -300;
    } else if (theta_x > 300) {
        theta_x = 300;
    }
    if (theta_y<-300) {
        theta_y = -300;
    } else if (theta_y > 300) {
        theta_y = 300;
    }

    //        lcd_locate(0, 3);
    //        lcd_printf("                         ");
    //        lcd_locate(0, 3);
    //        lcd_printf("thetaX: %d", theta_x);
    //        lcd_locate(0, 4);
    //        lcd_printf("                         ");
    //        lcd_locate(0, 4);
    //        lcd_printf("thetaY: %d", theta_y);

    // transfer the range from -400-400 to 0.9-2.1
    dutycicle_x = (theta_x - (-300)) / 600.0 * (2.1 - 0.9) + 0.9;
    dutycicle_y = (theta_y - (-300)) / 600.0 * (2.1 - 0.9) + 0.9;
    //        lcd_locate(0, 3);
    //        lcd_printf("                         ");
    //        lcd_locate(0, 3);
    //        lcd_printf("thetaX: %f", dutycicle_x);
    //        lcd_locate(0, 4);
    //        lcd_printf("                         ");
    //        lcd_locate(0, 4);
    //        lcd_printf("thetaY: %f", dutycicle_y);
}

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

    // initialize servos
    servo_init(SERVO_1);
    Nop();
    Nop();
    servo_init(SERVO_2);
    Nop();
    Nop();

    while (TRUE) {

        if (trigger_read_touchscreen == true) {
            read_touchscreen();
            trigger_read_touchscreen = false;

        }



        if (trigger_pd_controller == true) {
            pd_controller(x_current_pos, y_current_pos);
            trigger_pd_controller = false;
        }
        servo_control(20 - dutycicle_x, 20 - dutycicle_y);

        if (trigger_print_missed_deadlines == true) {
            print_missed_deadlines();
            trigger_print_missed_deadlines = false;
        }




    }
}
// triggered each 10 ms

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void) { // invoked every 10 ms


    CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag

    // 100Hz for Task1  , 10 ms
    counterHundredHz++;
    if (counterHundredHz >= 1) {
        counterHundredHz = 0;

        if (trigger_read_touchscreen == true) {
            deadlines_missed++;
        } else {
            trigger_read_touchscreen = true;

        }

    }

    // 50Hz for Task2 , 20 ms
    counterFiftyHz++;
    if (counterFiftyHz >= 2) {
        counterFiftyHz = 0;
        if (trigger_pd_controller == true) {
            deadlines_missed++;
        } else {
            trigger_pd_controller = true;

        }
    }

    // 5Hz for Task3 , 200 ms
    counterFiveHz++;
    if (counterFiveHz >= 20) {
        counterFiveHz = 0;
        trigger_print_missed_deadlines = true;
    }


}

