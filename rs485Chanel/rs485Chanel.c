#include "..//main.h"
#include "rs485Chanel.h"

volatile static uint8_t __RS485_PAUSE_FOR_NEXT_REQUEST__ = 2;

volatile uint16_t uiSysRS485SendRequestTimer = 5;
volatile uint16_t uiSysRS485ReciverTimer = 0;

volatile static uint8_t ucRS485txrxBuffer[260];

volatile static uint8_t ucRS485txState;
volatile static uint8_t *ucRS485txBuffer;
volatile static uint8_t ucRS485txBufferLenght;

volatile static uint8_t ucRS485rxState;
volatile static uint8_t *ucRS485rxBuffer;

volatile static uint16_t rs485ReciverTimerOld = 0;

volatile static uint8_t ucChanelCount = 0;
volatile static uint8_t ucRS485ChanelIndex = 0;
volatile static uint8_t ucRS485ChanelCounter = 0;
volatile static LP_OBJ_RS485_CHANEL lpArrRS485Chanel[ 50 ];

volatile static uint8_t usRS485PortAsMaster = 0;

void rs485Task( void )
{
	if( !ucChanelCount || !usRS485PortAsMaster ) {
		return;
	}

	if( !uiSysRS485SendRequestTimer ) {

		ucRS485ChanelIndex = ucRS485ChanelCounter;
		if( ++ucRS485ChanelCounter >= ucChanelCount ) {
			ucRS485ChanelCounter = 0;
			PORTG ^= MCU_PG1_bm;
		}

		if( NULL == lpArrRS485Chanel[ ucRS485ChanelIndex ] ) {
			return;
		}

		/*if( !lpArrRS485Chanel[ ucRS485ChanelIndex ]->ucEnableRequest ) {
			return;
		}*/

		/*
			На всеки xx мс. се изпраща запитване към следващото устройство
			свързано към RS485 линията.
		*/
		lpArrRS485Chanel[ ucRS485ChanelIndex ]->ucFlag = 0;
		lpArrRS485Chanel[ ucRS485ChanelIndex ]->rs485SetUartSetings( lpArrRS485Chanel[ ucRS485ChanelIndex ]->lpData );
		lpArrRS485Chanel[ ucRS485ChanelIndex ]->rs485SendRequestFunc( lpArrRS485Chanel[ ucRS485ChanelIndex ]->lpData );

		// Пауза за следващото запитване + Timeout
		uiSysRS485ReciverTimer		= lpArrRS485Chanel[ ucRS485ChanelIndex ]->msReadTimeOut;
		uiSysRS485SendRequestTimer	= lpArrRS485Chanel[ ucRS485ChanelIndex ]->msReadTimeOut + __RS485_PAUSE_FOR_NEXT_REQUEST__;
	}

	if( uiSysRS485ReciverTimer ) {
		if( 1 ) {//|| rs485ReciverTimer != rs485ReciverTimerOld ) {
			rs485ReciverTimerOld = uiSysRS485ReciverTimer;

			if( lpArrRS485Chanel[ ucRS485ChanelIndex ]->rs485GetResponseFunc( (uint8_t*)ucRS485rxBuffer, ucRS485rxState, lpArrRS485Chanel[ ucRS485ChanelIndex ]->lpData ) ) {
				lpArrRS485Chanel[ ucRS485ChanelIndex ]->rs485ClrTimeOutError( lpArrRS485Chanel[ ucRS485ChanelIndex ]->lpData );
				lpArrRS485Chanel[ ucRS485ChanelIndex ]->rxErrorCounter = 0;
				lpArrRS485Chanel[ ucRS485ChanelIndex ]->ucFlag = 1; // OK

				uiSysRS485ReciverTimer = 0;
			}
		}
	} else {
		lpArrRS485Chanel[ ucRS485ChanelIndex ]->rs485RestoreUartSetings( lpArrRS485Chanel[ ucRS485ChanelIndex ]->lpData );

		if( !lpArrRS485Chanel[ ucRS485ChanelIndex ]->ucFlag ) {
			lpArrRS485Chanel[ ucRS485ChanelIndex ]->ucFlag = 2; // Timeout

			// Изчаква N timeout грешки за вдигане на timeout флаг
			if( lpArrRS485Chanel[ ucRS485ChanelIndex ]->rxErrorCounter < 5 ) {
				++lpArrRS485Chanel[ ucRS485ChanelIndex ]->rxErrorCounter;
			} else {
				lpArrRS485Chanel[ ucRS485ChanelIndex ]->rs485SetTimeOutError( lpArrRS485Chanel[ ucRS485ChanelIndex ]->lpData );
			}
		}
	}
}

void rs485TaskEnable( void )
{
	usRS485PortAsMaster = 1;
}

void rs485TaskDisable( void )
{
	usRS485PortAsMaster = 0;
}

uint8_t rs485TaskIsEnable( void )
{
	return usRS485PortAsMaster;
}

void rs485TaskInit( void )
{
	uint8_t i;

	rs485TaskDisable();

	ucRS485txBuffer = ucRS485txrxBuffer;
	ucRS485rxBuffer = ucRS485txrxBuffer;

	ucChanelCount = 0;
	for( i = 0; i < size_of_array( lpArrRS485Chanel ); i++ ) {
		lpArrRS485Chanel[i] = NULL;
	}
}

uint8_t rs485AddChanel( LP_OBJ_RS485_CHANEL lpChanel )
{
	lpArrRS485Chanel[ ucChanelCount ] = lpChanel;

	if( ++ucChanelCount > size_of_array( lpArrRS485Chanel ) ) {
		ucChanelCount = size_of_array( lpArrRS485Chanel );
		return 0;
	}

	return 1;
}

void rs485ChanelDefInit( LP_OBJ_RS485_CHANEL lpChanel )
{
	lpChanel->lpData = NULL;

	lpChanel->msReadTimeOut = 100;
	lpChanel->ucEnableRequest = 0;
	lpChanel->rxErrorCounter = 0;
	lpChanel->ucFlag = 0;

	lpChanel->rs485SetUartSetings = rs485SetUartSetingsNullFunc;
	lpChanel->rs485RestoreUartSetings = rs485RestoreUartSetingsNullFunc;

	lpChanel->rs485SendRequestFunc = rs485sendRequestNullFunc;
	lpChanel->rs485GetResponseFunc = rs485getResponseNullFunc;

	lpChanel->rs485SetTimeOutError = rs485SetTimeOutErrorNullFunc;
	lpChanel->rs485ClrTimeOutError = rs485ClrTimeOutErrorNullFunc;
}

void rs485sendRequestNullFunc( void *lpData )
{
}

uint8_t rs485getResponseNullFunc( uint8_t *rxBuffer, uint8_t len, void *lpData )
{
	return 1;
}

void rs485SetUartSetingsNullFunc( void *lpData )
{
}

void rs485RestoreUartSetingsNullFunc( void *lpData )
{
}

void rs485SetTimeOutErrorNullFunc( void *lpData )
{
}

void rs485ClrTimeOutErrorNullFunc( void *lpData )
{
}

void rs485SendBuffer( uint8_t *buffer, uint8_t len )
{
	memcpy( (char*)ucRS485txBuffer, buffer, len );

	ucRS485txBufferLenght = len;
	ucRS485rxState = ucRS485txState = 0;

	UCSR0B &= ~( _BV( RXEN0 ) |  _BV( RXCIE0 ) );
	enable_rs485_transmit();

	UDR0 = ucRS485txBuffer[ ucRS485txState ];
}

ISR( USART0_TX_vect )
{
	if( !usRS485PortAsMaster ) {
		disable_rs485_transmit();
	} else {
		if( ++ucRS485txState != ucRS485txBufferLenght ) {
			UDR0 = ucRS485txBuffer[ ucRS485txState ];
		} else {
			disable_rs485_transmit();
			UCSR0B |= _BV( RXEN0 ) | _BV( RXCIE0 );

			ucRS485txState = ucRS485txBufferLenght = 0;
		}
	}
}

ISR( USART0_RX_vect )
{
	if( !usRS485PortAsMaster ) {
		pxMBFrameCBByteReceived();
	} else {
		volatile char c = UDR0;

		if( ucRS485rxState < 260 ) {
			ucRS485rxBuffer[ ucRS485rxState ] = c;

			++ucRS485rxState;
		} else {
			ucRS485rxState = 0;
		}
	}
}

ISR( USART0_UDRE_vect )
{
	if( !usRS485PortAsMaster ) {
		pxMBFrameCBTransmitterEmpty();
	}
}

void initRS485( unsigned long ulBaudRate, uint8_t ucDataBits, uint8_t ucStopBits, uint8_t ucParity )
{
	if( usRS485PortAsMaster ) {
		disable_rs485_transmit();
		
		UCSR0A = 0;
		UCSR0B = 0;
		UCSR0C = 0;

		switch( ulBaudRate ) {
		case 2400: ulBaudRate = 416; break;
		case 4800: ulBaudRate = 207; break;
		case 9600: ulBaudRate = 103; break;
		case 14400: ulBaudRate = 68; break;
		case 19200: ulBaudRate = 51; break;
		case 28800: ulBaudRate = 34; break;
		case 38400: ulBaudRate = 25; break;
		case 57600: ulBaudRate = 16; break;
		case 76800: ulBaudRate = 12; break;
		case 115200: ulBaudRate = 8; break;
		//case 115200: UCSR0C |= 1<<UMSEL0; UCSR0A |= 1<<U2X0; ulBaudRate = 16; break;
		case 230400: ulBaudRate = 3; break;
		case 250000: ulBaudRate = 3; break;
		case 500000: ulBaudRate = 1; break;
		case 1000000: ulBaudRate = 0; break;
		//default: ulBaudRate = UART_BAUD_CALC( ulBaudRate, F_CPU );
		}

		UBRR0H = ulBaudRate>>8;
		UBRR0L = ulBaudRate;

		switch ( ucDataBits ) {
		case 8:
			UCSR0C |= _BV( UCSZ01 ) | _BV( UCSZ00 );
		 break;
		case 7:
			UCSR0C |= _BV( UCSZ01 );
		 break;
		}

		switch ( ucStopBits ) {
		case 1: 
		 break;
		case 2:
			UCSR0C |= _BV( USBS0 );
		 break;
		}

		switch ( ucParity ) {
		case 0: // PAR_NONE:
		 break;

		case 1: // PAR_ODD
			UCSR0C |= _BV( UPM01 ) | _BV( UPM00 );
		 break;

		case 2: // PAR_EVEN
			UCSR0C |= _BV( UPM01 );
		 break;
		}

		UCSR0B = _BV( TXEN0 ) | _BV( TXCIE0 );
	}
}
