/* 
 * File:   led.h
 * Author: giovani
 *
 * Created on April 24, 2019, 10:46 AM
 */

#include <p33Fxxxx.h>
#include "types.h"

#ifndef LED_H
#define	LED_H

// Tri-state registers
#define LEDTRIS		TRISA
#define LED1_TRIS	TRISAbits.TRISA4
#define LED2_TRIS	TRISAbits.TRISA5
#define LED3_TRIS	TRISAbits.TRISA9
#define LED4_TRIS	TRISAbits.TRISA10
#define LED5_TRIS	TRISAbits.TRISA0

// Port registers using predefined structs
#define LEDPORT		PORTA
#define LED1_PORT	PORTAbits.RA4
#define LED2_PORT	PORTAbits.RA5
#define LED3_PORT	PORTAbits.RA9
#define LED4_PORT	PORTAbits.RA10
#define LED5_PORT	PORTAbits.RA0

// LEDPORT Bitwise definitions
#define LED1        4
#define LED2        5
#define LED3        9
#define LED4        10
#define LED5        0

// inserting Nop(): one cycle delay required for writes (see datasheet 11.4)
#define SETLED(BIT) do { SETBIT(BIT); Nop(); } while(0)
#define CLEARLED(BIT) do { CLEARBIT(BIT); Nop(); } while(0)
#define TOGGLELED(BIT) do { TOGGLEBIT(BIT); Nop(); } while(0)

void led_init(void);

#endif	/* LED_H */
