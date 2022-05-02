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


void turnLedOn(int counter);
void delay(long int count);
void clearAllLeds(void);



void turnLedOn(int counter) {
    //do the LED part
    switch (counter) {
        case 1:
            SETLED(LED1_PORT);
            Nop();
            lcd_printf("1");
            lcd_locate(10, 5);
            break;
        case 2:
            SETLED(LED2_PORT);
            Nop();
            lcd_printf("2");
            lcd_locate(10, 5);
            break;
        case 3:
            SETLED(LED3_PORT);
            Nop();
            lcd_printf("3");
            lcd_locate(10, 5);
            break;
        case 4:
            SETLED(LED4_PORT);
            Nop();
            lcd_printf("4");
            lcd_locate(10, 5);
            break;
        case 5:
            SETLED(LED5_PORT);
            Nop();
            lcd_printf("5");
            lcd_locate(10, 5);
            break;
        case 6:
            CLEARLED(LED1_PORT);
            Nop();
            CLEARLED(LED2_PORT);
            Nop();
            CLEARLED(LED3_PORT);
            Nop();
            CLEARLED(LED4_PORT);
            Nop();
            CLEARLED(LED5_PORT);
            break;
    }
}


void clearAllLeds(void){
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
//
//
//void decToBinary(int n)
//{
//    // array to store binary number
//    int binaryNum[32];
// 
//    // counter for binary array
//    int i = 0;
//    while (n > 0) {
// 
//        // storing remainder in binary array
//        binaryNum[i] = n % 2;
//        n = n / 2;
//        i++;
//    }
// 
//    // printing binary array in reverse order
//    for (int j = i - 1; j >= 0; j--){
//        lcd_locate(10, 5);
//        lcd_printf("%c",binaryNum[j]);
////        cout << binaryNum[j];
//    }
//}


void delay(long int count){
    long int i=0;
    while(i<count){
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


        // Print Hello World
        lcd_printf("Mohamed Aboushenif");
        lcd_locate(0, 1);
        lcd_printf("Xinlan Zheng\n");
        lcd_locate(0, 2);
        lcd_printf("Qiu Rui");

        

        clearAllLeds();


        lcd_locate(0, 5);
        lcd_printf("Counter:");
        lcd_locate(10, 5);

        int counter = 1;
        while (1) {

            
            
            turnLedOn(counter);


            if (counter == 6) {
                counter = 1;

            }
            else{
                counter++;
            }

            
            delay(1000000);
            
            
            

        }

    }

