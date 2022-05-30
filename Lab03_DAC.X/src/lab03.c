#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * DAC code
 */

#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13



void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    // set RD8, RB10, RB11, RB13 as output pins
    SETBIT(DAC_SCK_AD1CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_SCK_AD2CFG); // set Pin to Digital
    Nop();
    CLEARBIT(DAC_SCK_TRIS); // set Pin to Output
    Nop();
    
    SETBIT(DAC_SDI_AD1CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_SDI_AD2CFG); // set Pin to Digital
    Nop();
    CLEARBIT(DAC_SDI_TRIS); // set Pin to Output
    Nop();

    SETBIT(DAC_LDAC_AD1CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_LDAC_AD2CFG); // set Pin to Digital
    Nop();
    CLEARBIT(DAC_LDAC_TRIS); // set Pin to Output
    Nop();

    
    //make cs as output
    CLEARBIT(DAC_CS_TRIS);
    Nop();

    
    
    
   
    
    // set default state: CS=??, SCK=??, SDI=??, LDAC=??
    SETBIT(DAC_CS_PORT);
    Nop();
    CLEARBIT(DAC_SCK_PORT);
    Nop();
    CLEARBIT(DAC_SDI_PORT);
    Nop();
    SETBIT(DAC_LDAC_PORT);
    Nop();

    
    
    
    
}

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

void timer_initialize()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    // configure timer
    
    // Timer 2 2ms for LED1
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
    
}

// interrupt service routine?
volatile int milliseconds = 0;
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void) { // invoked every 2 ms
    
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 Interrupt Flag
    
    milliseconds=milliseconds+2;
    switch(milliseconds){
        
    case 2:
        DAC_procedure(14287,1); 
        TOGGLEBIT(LED1_PORT); 
        Nop();
        CLEARBIT(IFS0bits.T2IF);
        break;
    
    case 502:
        DAC_procedure(6595,2);
        TOGGLEBIT(LED1_PORT); 
        Nop();
        CLEARBIT(IFS0bits.T2IF);
        break;
      
    case 2502:
        DAC_procedure(7595,3);
        TOGGLEBIT(LED1_PORT); 
        Nop();
        CLEARBIT(IFS0bits.T2IF);
        break;
        
    case 3500:
        milliseconds = 0;
        break;
            
    }
    
  

}


void DAC_procedure(uint16_t voltage, int row)
{
    // clear CS
    CLEARBIT(DAC_CS_PORT);
    Nop();
    int i=15;
    lcd_locate(0, row);
    while(i>=0)
    {
        uint16_t value = voltage&BV(i);
        value = value >> i;
        
        
        DAC_SDI_PORT= value;
        Nop();
         
//        lcd_printf("%d", value);
        
        
        // toggle clock
        SETBIT(DAC_SCK_PORT);
        Nop();
        CLEARBIT(DAC_SCK_PORT);
        Nop(); 
        
        i--;
    }
    
    //clear CS
    SETBIT(DAC_CS_PORT);
    Nop();
    // clear SDI
    CLEARBIT(DAC_SDI_PORT);
    Nop();
    
    
    //toggle LDAC
    CLEARBIT(DAC_LDAC_PORT);
    Nop(); 
    Nop(); 
    SETBIT  (DAC_LDAC_PORT);
    Nop();


    
    

    
}


/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: Group 2");
    
    uint16_t volt_1= 14287;   // 0011011111001111     1.0 volt
    uint16_t volt_2= 6595;    // 0001100111000011     2.5 volt
    uint16_t volt_3= 7595;    //0001110110101011      3.5 volt
    
    
    
    while(TRUE)
    {
        /*int i=3;
        DAC_procedure(volt_1,i);
        DAC_procedure(volt_2,i+1);
        DAC_procedure(volt_3,i+2);*/
           
    }
}
