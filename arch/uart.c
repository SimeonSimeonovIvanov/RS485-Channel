
/*
 * Copyright (c) 2006-2008 by Roland Riegel <feedback@roland-riegel.de>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>

#include "uart.h"
#include "../FreeMODBUS/port/port.h"

/**
 * \addtogroup arch
 *
 * @{
 */
/**
 * \addtogroup arch_uart Hardware UART
 *
 * Functions for sending and receiving data via UART.
 *
 * \note Currently, the UART is set to a fixed speed of 115200 Baud.
 *
 * @{
 */
/**
 * \file
 * UART implementation (license: GPLv2)
 *
 * \author Roland Riegel
 */

#define TERMINAL_UBRR		UBRR0
#define TERMINAL_UBRRH		UBRR0H
#define TERMINAL_UBRRL		UBRR0L
#define TERMINAL_UCSRA		UCSR0A
#define TERMINAL_UCSRB		UCSR0B
#define TERMINAL_UCSRC		UCSR0C
#define TERMINAL_UDR		UDR0

#define TERMINAL_BAUD		115200UL
#define TERMINAL_UBRRVAL	( F_CPU / ( TERMINAL_BAUD * 16 ) - 1 )

static int _uart_putc(char c, FILE* stream);
static int _uart_getc(FILE* stream);

static FILE stdio = FDEV_SETUP_STREAM( _uart_putc, _uart_getc, _FDEV_SETUP_RW );

/**
 * Initializes the UART.
 */
void uart_init()
{
	/* set baud rate */
	//TERMINAL_UBRRH = UBRRVAL >> 8;
	//TERMINAL_UBRRL = UBRRVAL & 0xff;
	TERMINAL_UBRRH = 0;
	TERMINAL_UBRRL = 8;
	/* set frame format: 8 bit, no parity, 1 bit */
	TERMINAL_UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
	/* enable serial receiver and transmitter */
	TERMINAL_UCSRB = (1 << RXEN) | (1 << TXEN);
}

/**
 * Connects the UART with the I/O functions of the C Standard Library.
 */
void uart_connect_stdio()
{
	stdin = stdout = stderr = &stdio;
}

/**
 * Transmits a single character/byte via the UART.
 *
 * \param[in] c The byte to send.
 */
void uart_putc(uint8_t c)
{
	if( '\n' == c ) {
		uart_putc('\r');
	}
	/* wait until transmit buffer is empty */
	loop_until_bit_is_set( TERMINAL_UCSRA, UDRE );
	asm("nop\n");
	/* send next byte */
	TERMINAL_UDR = c;
	loop_until_bit_is_set( TERMINAL_UCSRA, UDRE );
}

/**
 * Adaptor function for transmission of a character with the C Standard Library.
 *
 * \param[in] c The character to write to the \a stream.
 * \param[in] stream The stream to which to write the character (unused).
 * \returns \c 0.
 */
int _uart_putc(char c, FILE* stream)
{
	uart_putc(c);
	return 0;
}

/**
 * Transmits a byte in hexadecimal ASCII representation via the UART.
 *
 * \param[in] b The byte to transmit.
 */
void uart_putc_hex(uint8_t b)
{
	/* upper nibble */
	if( (b >> 4) < 0x0a ) {
		uart_putc((b >> 4) + '0');
	} else {
		uart_putc((b >> 4) - 0x0a + 'a');
	}

	/* lower nibble */
	if( (b & 0x0f) < 0x0a ) {
		uart_putc((b & 0x0f) + '0');
	} else {
		uart_putc((b & 0x0f) - 0x0a + 'a');
	}
}

/**
 * Receives a byte from the UART.
 *
 * \returns The byte received.
 */
uint8_t uart_getc()
{
	uint8_t b;

	/* wait until receive buffer is full */
	loop_until_bit_is_set( TERMINAL_UCSRA, RXC );

	b = UDR;

	if(b == '\r') {
		b = '\n';
	}

	return b;
}

/**
 * Receives a byte from the UART without blocking.
 *
 * \returns The byte received if available, \c 0 otherwise.
 */
uint8_t uart_getc_try()
{
	if( !(TERMINAL_UCSRA & (1 << RXC)) ) {
		return 0x00;
	}

	return uart_getc();
}

/**
 * Adaptor function for reception of a character with the C Standard Library.
 *
 * \param[in] stream The stream from which to read the character (unused).
 * \returns The character received.
 */
int _uart_getc(FILE* stream)
{
	return uart_getc();
}

/**
 * @}
 * @}
 */
