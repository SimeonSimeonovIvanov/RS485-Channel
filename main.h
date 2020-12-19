#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

//#include "veznaEEP.h"
#include "arch/spi.h"
#include "bit-array/bit-array.h"
#include "digital_in_out_func/digital_in_out_func.h"
#include "modbus/port/port.h"
#include "modbus_master/modbus-rtu.h"
#include "rs485Channel/rs485Channel.h"

// ============================================================================
// PORTA

#define MCU_D7_bp						( PA7 )
#define MCU_D6_bp						( PA6 )
#define MCU_D5_bp						( PA5 )
#define MCU_D4_bp						( PA4 )
#define MCU_D3_bp						( PA3 )
#define MCU_D2_bp						( PA2 )
#define MCU_D1_bp						( PA1 )
#define MCU_D0_bp						( PA0 )

#define MCU_D7_bm						( 1<<MCU_D7_bp )
#define MCU_D6_bm						( 1<<MCU_D6_bp )
#define MCU_D5_bm						( 1<<MCU_D5_bp )
#define MCU_D4_bm						( 1<<MCU_D4_bp )
#define MCU_D3_bm						( 1<<MCU_D3_bp )
#define MCU_D2_bm						( 1<<MCU_D2_bp )
#define MCU_D1_bm						( 1<<MCU_D1_bp )
#define MCU_D0_bm						( 1<<MCU_D0_bp )

// ============================================================================
// PORTB

#define MCU_RUN_STOP_bp					( PB7 )
#define MCU_DO_CS0_bp					( PB6 )
#define MCU_CS2_bp						( PB5 )
#define MCU_CS1_bp						( PB4 )
#define MCU_MISO_bp						( PB3 )
#define MCU_MOSI_bp						( PB2 )
#define MCU_SCK_bp						( PB1 )
#define MCU_CS0_bp						( PB0 )

#define MCU_RUN_STOP_bm					( 1<<MCU_RUN_STOP_bp )
#define MCU_DO_CS0_bm					( 1<<MCU_DO_CS0_bp )
#define MCU_CS2_bm						( 1<<MCU_CS2_bp )
#define MCU_CS1_bm						( 1<<MCU_CS1_bp )
#define MCU_MISO_bm						( 1<<MCU_MISO_bp )
#define MCU_MOSI_bm						( 1<<MCU_MOSI_bp )
#define MCU_SCK_bm						( 1<<MCU_SCK_bp )
#define MCU_CS0_bm						( 1<<MCU_CS0_bp )

// ============================================================================
// PORTC

#define MCU_DI0_bp						( PC7 )
#define MCU_DI1_bp						( PC6 )
#define MCU_DI2_bp						( PC5 )
#define MCU_DI3_bp						( PC4 )
#define MCU_DI4_bp						( PC3 )
#define MCU_DI5_bp						( PC2 )
#define MCU_DI6_bp						( PC1 )
#define MCU_DI7_bp						( PC0 )

#define MCU_DI0_bm						( 1<<MCU_DI0_bp )
#define MCU_DI1_bm						( 1<<MCU_DI1_bp )
#define MCU_DI2_bm						( 1<<MCU_DI2_bp )
#define MCU_DI3_bm						( 1<<MCU_DI3_bp )
#define MCU_DI4_bm						( 1<<MCU_DI4_bp )
#define MCU_DI5_bm						( 1<<MCU_DI5_bp )
#define MCU_DI6_bm						( 1<<MCU_DI6_bp )
#define MCU_DI7_bm						( 1<<MCU_DI7_bp )

// ============================================================================
// PORTD

#define MCU_DI8_bp						( PD7 )
#define MCU_DI9_bp						( PD6 )
#define MCU_DI10_bp						( PD5 )
#define MCU_DI11_bp						( PD4 )
#define MCU_DI12_bp						( PD3 )
#define MCU_DI13_bp						( PD2 )
#define MCU_DI14_bp						( PD1 )
#define MCU_DI15_bp						( PD0 )

#define MCU_DI8_bm						( 1<<MCU_DI8_bp )
#define MCU_DI9_bm						( 1<<MCU_DI9_bp )
#define MCU_DI10_bm						( 1<<MCU_DI10_bp )
#define MCU_DI11_bm						( 1<<MCU_DI11_bp )
#define MCU_DI12_bm						( 1<<MCU_DI12_bp )
#define MCU_DI13_bm						( 1<<MCU_DI13_bp )
#define MCU_DI14_bm						( 1<<MCU_DI14_bp )
#define MCU_DI15_bm						( 1<<MCU_DI15_bp )

// ============================================================================
// PORTE

#define MCU_DO_DIAG0_bp					( PE7 )
#define MCU_DO_WR_bp					( PE6 )
#define MCU_DO_DIS_bp					( PE5 )
#define MCU_DO_CS1_bp					( PE4 )
#define MCU_DO_DIAG1_bp					( PE3 )
#define MCU_RS485_DE_bp					( PE2 )
#define MCU_RS485_TX_bp					( PE1 )
#define MCU_RS485_RX_bp					( PE0 )

#define MCU_DO_DIAG0_bm					( 1<<MCU_DO_DIAG0_bp )
#define MCU_DO_WR_bm					( 1<<MCU_DO_WR_bp )
#define MCU_DO_DIS_bm					( 1<<MCU_DO_DIS_bp )
#define MCU_DO_CS1_bm					( 1<<MCU_DO_CS1_bp )
#define MCU_DO_DIAG1_bm					( 1<<MCU_DO_DIAG1_bp )
#define MCU_RS485_DE_bm					( 1<<MCU_RS485_DE_bp )
#define MCU_RS485_TX_bm					( 1<<MCU_RS485_TX_bp )
#define MCU_RS485_RX_bm					( 1<<MCU_RS485_RX_bp )

// ============================================================================
// PORTF

#define MCU_RS485_AN_bp					( PF3 )
#define MCU_RS485_A2_bp					( PF2 )
#define MCU_RS485_A1_bp					( PF1 )
#define MCU_RS485_A0_bp					( PF0 )

#define MCU_RS485_AN_bm					( 1<<MCU_RS485_AN_bp )
#define MCU_RS485_A2_bm					( 1<<MCU_RS485_A2_bp )
#define MCU_RS485_A1_bm					( 1<<MCU_RS485_A1_bp )
#define MCU_RS485_A0_bm					( 1<<MCU_RS485_A0_bp )

// ============================================================================
// PORTG

#define MCU_VCC_FB0_bp					( PG4 )
#define MCU_VCC_FB1_bp					( PG3 )
#define MCU_IO_RESET_bp					( PG2 )
#define MCU_PG1_bp						( PG1 )
#define MCU_PG0_bp						( PG0 )

#define MCU_VCC_FB0_bm					( 1<<MCU_VCC_FB0_bp )
#define MCU_VCC_FB1_bm					( 1<<MCU_VCC_FB1_bp )
#define MCU_IO_RESET_bm					( 1<<MCU_IO_RESET_bp )
#define MCU_PG1_bm						( 1<<MCU_PG1_bp )
#define MCU_PG0_bm						( 1<<MCU_PG0_bp )

// ============================================================================

#define enable_rs485_transmit()			( PORTE |= MCU_RS485_DE_bm )
#define disable_rs485_transmit()		( PORTE &= ~MCU_RS485_DE_bm )

#define strobe_hmi_led()				( PORTB |= MCU_CS2_bm )
#define unstrobe_hmi_led()				( PORTB &= ~MCU_CS2_bm )

#define len_of_array( arr )				( sizeof( arr ) / sizeof( *arr ) )

void initBoard(void);
void byteArrToBitArr( uint8_t *lpBit, const uint8_t *lpByte, uint16_t bit_count );

void mbMasterClrTimeOutError( void *lpObject );
void veznaEepSetUartSetings( void *lpObject );
void veznaEepRestoreUartSetings( void *lpObject );

#endif
