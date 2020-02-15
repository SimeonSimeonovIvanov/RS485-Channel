#ifndef __RS485_CHANNEL_H__
#define __RS485_CHANNEL_H__

#include <avr/interrupt.h>

#define MYBRR(BAUD, FOSC) (FOSC/16/BAUD-1)

typedef struct rs485Channel
{
	void *lpObject;

	uint16_t msReadTimeOut;

	uint8_t ucEnableRequest, ucFlag, rxErrorCounter;

	void (*rs485SetTimeOutError)( void *lpObject );
	void (*rs485ClrTimeOutError)( void *lpObject );

	void (*rs485SetUartSetings)( void *lpObject );
	void (*rs485RestoreUartSetings)( void *lpObject );

	void (*rs485SendRequestFunc)( void *lpObject );
	uint8_t (*rs485GetResponseFunc)( void *lpObject, uint8_t *rxBuffer, uint8_t len );
} OBJ_RS485_CHANNEL, *LP_OBJ_RS485_CHANNEL;

void rs485TaskEnable( void );
void rs485TaskDisable( void );
uint8_t rs485TaskIsEnable( void );

void rs485Task( void );
void rs485TaskInit( void );
void rs485TimerIsr( void );
uint8_t rs485AddChannel( LP_OBJ_RS485_CHANNEL lpChannel );
void rs485ChannelDefInit( LP_OBJ_RS485_CHANNEL lpChannel );

void rs485SetUartSetingsNullFunc( void *lpObject );
void rs485RestoreUartSetingsNullFunc( void *lpObject );

void rs485sendRequestNullFunc( void *lpObject );
uint8_t rs485getResponseNullFunc( void *lpObject, uint8_t *rxBuffer, uint8_t len );

void rs485SetTimeOutErrorNullFunc( void *lpObject );
void rs485ClrTimeOutErrorNullFunc( void *lpObject );

void initRS485( unsigned long ulBaudRate, uint8_t ucDataBits, uint8_t ucStopBits, uint8_t ucParity );
void rs485SendBuffer( uint8_t *buffer, uint8_t len );

#endif
