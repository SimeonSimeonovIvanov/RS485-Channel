/*
	diy-VT100 firmware
	==================

	Source code of diy-VT100 firmware.
	Run on STM32F767VxTx.

	URL: https://www.madresistor.com/diy-vt100
	Hardware design: https://gitlab.com/madresistor/diy-vt100-hardware

	If you are looking for the last tested design,
	checkout tag "manuf-27-march-2017" from firmware repo and hardware design repo.

	Dependencies
	============
*/

#ifndef BIT_ARRAY_H
#define BIT_ARRAY_H

#include <stdint.h>
#include <stdbool.h>


#define mask( index )   ( 1<<( 0x7 & index ) )
#define offset( index ) ( index>>3 )

void bitarr_high(uint8_t *data, unsigned index);
void bitarr_low(uint8_t *data, unsigned index);
void bitarr_write(uint8_t *data, unsigned index, bool value);
bool bitarr_read(const uint8_t *data, unsigned index);
void bitarr_flip(uint8_t *data, unsigned index);

#endif
