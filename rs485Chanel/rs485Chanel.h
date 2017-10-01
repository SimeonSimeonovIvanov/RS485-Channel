#ifndef __RS485_CHANEL_H__
#define __RS485_CHANEL_H__

#include <avr/interrupt.h>
#include "mb.h"

#define MYBRR(BAUD, FOSC) (FOSC/16/BAUD-1)

typedef struct rs485Chanel OBJ_RS485_CHANEL;
typedef struct rs485Chanel *LP_OBJ_RS485_CHANEL;

struct rs485Chanel {
	void *lpData;

	uint16_t msReadTimeOut;

	uint8_t ucEnableRequest, ucFlag, rxErrorCounter;

	void (*rs485SetTimeOutError)( void *lpObject );
	void (*rs485ClrTimeOutError)( void *lpObject );

	void (*rs485SetUartSetings)( void *lpObject );
	void (*rs485RestoreUartSetings)( void *lpObject );

	void (*rs485SendRequestFunc)( void *lpObject );
	uint8_t (*rs485GetResponseFunc)( uint8_t *rxBuffer, uint8_t len, void *lpObject );
};


void rs485TaskEnable( void );
void rs485TaskDisable( void );
uint8_t rs485TaskIsEnable( void );

void rs485Task( void );
void rs485TaskInit( void );
uint8_t rs485AddChanel( LP_OBJ_RS485_CHANEL lpChanel );
void rs485ChanelDefInit( LP_OBJ_RS485_CHANEL lpChanel );

void rs485SetUartSetingsNullFunc( void *lpObject );
void rs485RestoreUartSetingsNullFunc( void *lpObject );

void rs485sendRequestNullFunc( void *lpObject );
uint8_t rs485getResponseNullFunc( uint8_t *rxBuffer, uint8_t len, void *lpObject );

void rs485SetTimeOutErrorNullFunc( void *lpObject );
void rs485ClrTimeOutErrorNullFunc( void *lpObject );

void initRS485( unsigned long ulBaudRate, uint8_t ucDataBits, uint8_t ucStopBits, uint8_t ucParity );
void rs485SendBuffer( uint8_t *buffer, uint8_t len );

#endif
