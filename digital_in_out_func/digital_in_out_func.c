#include "../main.h"
#include "digital_in_out_func.h"

void readDigitalInput( uint8_t in[25] )
{
	uint8_t i, in0, in1, bit_mask = 0x80;

	in0 = PINC;
	in1 = PIND;

	for( i = 0; i < 8; i++ ) {
		( in0 & bit_mask ) ? ( in[ i + 0 ] |= 1 ) : ( in[ i + 0 ] &= ~1 );
		( in1 & bit_mask ) ? ( in[ i + 8 ] |= 1 ) : ( in[ i + 8 ] &= ~1 );
		bit_mask >>= 1;
	}

	( PING & MCU_VCC_FB0_bm ) ? ( in[ 20 ] = 1 ) : ( in[ 20 ] = 0 );
	( PING & MCU_VCC_FB1_bm ) ? ( in[ 21 ] = 1 ) : ( in[ 21 ] = 0 );
	( PINE & MCU_DO_DIAG0_bm ) ? ( in[ 22 ] = 1 ) : ( in[ 22 ] = 0 );
	( PINE & MCU_DO_DIAG1_bm ) ? ( in[ 23 ] = 1 ) : ( in[ 23 ] = 0 );

	( PINB & MCU_RUN_STOP_bm ) ? ( in[ 24 ] = 1 ) : ( in[ 24 ] = 0 );
}

void initDigitalInput( void )
{
	DDRC = (uint8_t)~(
		MCU_DI0_bm | MCU_DI1_bm | MCU_DI2_bm | MCU_DI3_bm |
		MCU_DI4_bm | MCU_DI5_bm | MCU_DI6_bm | MCU_DI7_bm
	);

	DDRD = (uint8_t)~(
		MCU_DI8_bm | MCU_DI9_bm | MCU_DI10_bm | MCU_DI11_bm |
		MCU_DI12_bm | MCU_DI13_bm | MCU_DI14_bm | MCU_DI15_bm
	);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writeDigitalOutput( uint8_t out[16] )
{
	uint8_t i, out0 = 0, out1 = 0, bit_mask = 0x80;

	for( i = 0; i < 8; i++ ) {
		( 1 & out[i + 0] ) ? ( out0 |= bit_mask ) : 0;
		( 1 & out[i + 8] ) ? ( out1 |= bit_mask ) : 0;
		bit_mask >>= 1;
	}

	PORTA = out0;
	PORTB &= ~MCU_DO_CS0_bm;
	asm("nop\n");
	PORTE &= ~MCU_DO_WR_bm;
	asm("nop\n");
	PORTE |= MCU_DO_WR_bm;
	asm("nop\n");
	PORTB |= MCU_DO_CS0_bm;

	PORTA = out1;
	PORTE &= ~MCU_DO_CS1_bm;
	asm("nop\n");
	PORTE &= ~MCU_DO_WR_bm;
	asm("nop\n");
	PORTE |= MCU_DO_WR_bm;
	asm("nop\n");
	PORTE |= MCU_DO_CS1_bm;

	PORTA = 0;
}

void initDigitalOutput(void)
{
	PORTE &= ~MCU_DO_DIS_bm;
	PORTE |= MCU_DO_CS1_bm | MCU_DO_WR_bm;
	DDRE |= MCU_DO_WR_bm | MCU_DO_DIS_bm | MCU_DO_CS1_bm;

	PORTA = 0;
	DDRA =
	(
		MCU_D7_bm | MCU_D6_bm | MCU_D5_bm | MCU_D4_bm |
		MCU_D3_bm | MCU_D2_bm | MCU_D1_bm | MCU_D0_bm
	);

	PORTB |= MCU_DO_CS0_bm;
	DDRB |= MCU_DO_CS0_bm;

	PORTE |= MCU_DO_DIS_bm;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writeHmiLed( uint8_t in[40] )
{
	uint8_t i, j, n, bit_mask0, bit_mask1;
	uint8_t data[5] = { 0 };

	n = 0;
	for( i = 0; i < 20; ) {
		n = i>>2;

		bit_mask0 = 16;
		bit_mask1 = 1;
		
		for( j = 0; j < 4; j++ ) {
			if( 1 & in[ i +  0 ] ) {
				data[n] |= bit_mask0;
			}

			if( 1 & in[ i + 20 ] ) {
				data[n] |= bit_mask1;
			}

			bit_mask0 <<= 1;
			bit_mask1 <<= 1;

			++i;
		}
	}

	spi_send_byte( data[0] );
	spi_send_data( &data[1], 4 );

	strobe_hmi_led();
	unstrobe_hmi_led();
}

void initHmiLed( void )
{
	uint8_t buffer[40] = { 0 };

	DDRB |= MCU_CS2_bm;

	writeHmiLed( buffer );

	strobe_hmi_led();
	asm("nop\n");
	unstrobe_hmi_led();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t readAddressSwitch(void)
{
	uint8_t i, address = 0;

	for( i = 0; i < 8; i++ ) {
		PORTF &= ~( MCU_RS485_A2_bm | MCU_RS485_A1_bm | MCU_RS485_A0_bm );
		PORTF |= i;
		asm("nop\n");asm("nop\n");asm("nop\n");
		address |= ( ( ( PINF & MCU_RS485_AN_bm ) ? 1:0 )<<i );
		asm("nop\n");asm("nop\n");asm("nop\n");
	}

	return address;
}

void initAddressSwitch(void)
{
	DDRF &= ~( MCU_RS485_AN_bm );
	DDRF |= ( MCU_RS485_A2_bm | MCU_RS485_A1_bm | MCU_RS485_A0_bm );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
