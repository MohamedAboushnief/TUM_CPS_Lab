#include <xc.h>

#ifndef DAC_H
#define	DAC_H

// initialize pins for serial communication with MCP4822
void dac_initialize();

// send digital conversion value to the DAC
void dac_convert_milli_volt(uint16_t milliVolt);

#endif	/* DAC_H */
