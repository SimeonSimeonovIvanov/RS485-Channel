#ifndef __DIGITAL_IN_OUT_FUNC_H__
#define __DIGITAL_IN_OUT_FUNC_H__

void initAddressSwitch(void);
uint8_t readAddressSwitch(void);

void initDigitalInput(void);
void readDigitalInput(uint8_t in[25]);

void initDigitalOutput(void);
void writeDigitalOutput(uint8_t out[16]);

void initHmiLed( void );
void writeHmiLed( uint8_t in[40] );

#endif
