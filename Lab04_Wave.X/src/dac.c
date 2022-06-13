#include "dac.h"
#include "types.h"

// tristate register
#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13

// port register
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

// analog to digital converter 1 port configuration register
#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

// analog to digital converter 2 port configuration register
#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    
    // set RD8, RB10, RB11, RB13 as output pins
    
    // set default state: CS=on, SCK=off, SDI=off, LDAC=on
    
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




void dac_convert_milli_volt(uint16_t voltage)
{
    // clear CS
    CLEARBIT(DAC_CS_PORT);
    Nop();
    int i=15;
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
