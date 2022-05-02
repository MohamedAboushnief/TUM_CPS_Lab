#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"
#include <stdio.h>
#include <time.h>

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF


void ledBinary(int binaryValueArr[]);
void delay(long int count);
void ledOutputInit(void);
void decToBinary(int n);



void ledBinary(int binaryValueArr[]) {
    //do the LED part

    if (binaryValueArr[0] == 0) {
        CLEARLED(LED5_PORT);
        Nop();
    } else {
        SETLED(LED5_PORT);
        Nop();
    }
    if (binaryValueArr[1] == 0) {
        CLEARLED(LED4_PORT);
        Nop();
    } else {
        SETLED(LED4_PORT);
        Nop();
    }
    if (binaryValueArr[2] == 0) {
        CLEARLED(LED3_PORT);
        Nop();
    } else {
        SETLED(LED3_PORT);
        Nop();
    }
    if (binaryValueArr[3] == 0) {
        CLEARLED(LED2_PORT);
        Nop();
    } else {
        SETLED(LED2_PORT);
        Nop();
    }
    if (binaryValueArr[4] == 0) {
        CLEARLED(LED1_PORT);
        Nop();
    } else {
        SETLED(LED1_PORT);
        Nop();
    }


}

void ledOutputInit(void) {
    CLEARLED(LED1_TRIS);
    Nop();
    CLEARLED(LED2_TRIS);
    Nop();
    CLEARLED(LED3_TRIS);
    Nop();
    CLEARLED(LED4_TRIS);
    Nop();
    CLEARLED(LED5_TRIS);
    Nop();
}

void decToBinary(int n) {
    // array to store binary number
    int binaryNum[5] = {0, 0, 0, 0, 0};

    // counter for binary array
    int i = 0;
    while (n > 0) {

        // storing remainder in binary array
        binaryNum[i] = n % 2;
        n = n / 2;
        i++;
    }


    ledBinary(binaryNum);
}

void delay(long int count) {
    long int i = 0;
    while (i < count) {
        i++;
    }
}

int main() {
    //Init LCD and LEDs
    lcd_initialize();
    led_init();

    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);


    lcd_printf("Mohamed Aboushenif");
    lcd_locate(0, 1);
    lcd_printf("Xinlan Zheng\n");
    lcd_locate(0, 2);
    lcd_printf("Qiu Rui");



    ledOutputInit();


    lcd_locate(0, 5);
    lcd_printf("Counter:");
    lcd_locate(10, 5);




    int counter = 0;
    while (1) {



        decToBinary(counter);

        lcd_printf("%d", counter);
        lcd_locate(10, 5);

        // maximum number in 5 binary digits is 31
        if (counter == 32) {
            counter = 0;
            lcd_printf("  ");
            lcd_locate(10, 5);

        } else {
            counter++;
        }


        delay(1000000);




    }

}

