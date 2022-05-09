#include "led.h"

void led_init(void){
	CLEARBIT(LED1_TRIS); // set Pin to Output
	Nop();
	CLEARBIT(LED2_TRIS); // set Pin to Output
	Nop();
	CLEARBIT(LED3_TRIS); // set Pin to Output
	Nop();
	CLEARBIT(LED4_TRIS); // set Pin to Output
	Nop();
	CLEARBIT(LED5_TRIS); // set Pin to Output
	Nop();
}
