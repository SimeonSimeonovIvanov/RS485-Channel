#include "modbus-rtu.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Coil Status (FC=01)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Status (FC=02)

void mbSendRequestReadInputStatus( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;
	uint8_t txBuffer[8] = { 0 };

	lpData->exception_code = 0;

	lpData->rxtx_byte_count = lpData->data_count>>3;
	if( !lpData->rxtx_byte_count )
	{
		lpData->rxtx_byte_count = 1;
	}

	txBuffer[0] = lpData->address;
	txBuffer[1] = 2;

	txBuffer[2] = lpData->data_address>>8;
	txBuffer[3] = lpData->data_address;

	txBuffer[4] = lpData->data_count>>8;
	txBuffer[5] = lpData->data_count;

	mbRtuAddCrcAndSendBuffer( txBuffer, 8 );
}

uint8_t mbReceiveRequestReadInputStatus( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address ) {

		if( 2 == rxBuffer[1] ) {
			if( len == 5 + lpData->rxtx_byte_count &&
				check_crc16( rxBuffer, 3 + lpData->rxtx_byte_count )
			) {
				if( NULL != lpData->lpReadData ) {
					uint8_t i;

					for( i = 0; i < lpData->rxtx_byte_count; i++ )
					{
						( (uint8_t*)lpData->lpReadData )[ i ] = rxBuffer[ 3 + i ];
					}
				}

				lpData->exception_code = 0;
				return 1;
			}

			return 0;
		}

		return mbCheckExceptionForResponse( &lpData->exception_code, 2, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Holding Registers (FC=03)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Registers (FC=04)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Single Coil (FC=05)

void mbSendRequestForceSingleCoil( void *lpObject )
{
	LP_MB_MASTER_WRITE_SINGLE_COIL lpData = (LP_MB_MASTER_WRITE_SINGLE_COIL)lpObject;
	uint8_t txBuffer[8] = { 0 };

	lpData->exception_code = 0;

	txBuffer[0] = lpData->address;
	txBuffer[1] = 5;

	txBuffer[2] = lpData->coil_address>>8;
	txBuffer[3] = lpData->coil_address;

	if( *lpData->lpCoil ) {
		txBuffer[4] = 0xff;
	}

	mbRtuAddCrcAndSendBuffer( txBuffer, 8 );
}

uint8_t mbReceiveRequestForceSingleCoil( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_WRITE_SINGLE_COIL lpData = (LP_MB_MASTER_WRITE_SINGLE_COIL)lpObject;

	if( rxBuffer[0] == lpData->address ) {

		if( 5 == rxBuffer[1] ) {
			if( 8 == len &&
				check_crc16( rxBuffer, 6 )
			) {
				if( rxBuffer[2] == lpData->coil_address>>8	&&
					rxBuffer[3] == lpData->coil_address		&&
					rxBuffer[5] == 0
				) {
					if( NULL != lpData->lpCoilNew ) {
						if( 0xff == rxBuffer[4] ) {
							*lpData->lpCoilNew = 1;
						}

						if( 0x00 == rxBuffer[4] ) {
							*lpData->lpCoilNew = 0;
						}
					}

					lpData->exception_code = 0;

					return 1;
				}
			}

			return 0;
		}
		
		return mbCheckExceptionForResponse( &lpData->exception_code, 5, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Single Register (FC=06)

void mbSendRequestPresetSingleRegister( void *lpObject )
{
	LP_MB_MASTER_PRESET_SINGLE_REGISTER lpData = (LP_MB_MASTER_PRESET_SINGLE_REGISTER)lpObject;
	uint8_t txBuffer[8] = { 0 };

	lpData->exception_code = 0;

	txBuffer[0] = lpData->address;
	txBuffer[1] = 6;

	txBuffer[2] = lpData->register_address>>8;
	txBuffer[3] = lpData->register_address;

	txBuffer[4] = *lpData->lpRegister>>8;
	txBuffer[5] = *lpData->lpRegister;

	mbRtuAddCrcAndSendBuffer( txBuffer, 8 );
}

uint8_t mbReceiveRequestPresetSingleRegister( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_PRESET_SINGLE_REGISTER lpData = (LP_MB_MASTER_PRESET_SINGLE_REGISTER)lpObject;

	if( rxBuffer[0] == lpData->address ) {

		if( 6 == rxBuffer[1] ) {
			if( 8 == len &&
				check_crc16( rxBuffer, 6 )
			) {
				if( rxBuffer[2] == lpData->register_address>>8	&&
					rxBuffer[3] == lpData->register_address
				) {
					if( NULL != lpData->lpRegisterNew ) {
						*lpData->lpRegisterNew = ( rxBuffer[4]<<8 ) | rxBuffer[5];
					}

					lpData->exception_code = 0;

					return 1;
				}
			}

			return 0;
		}
		
		return mbCheckExceptionForResponse( &lpData->exception_code, 6, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Multiple Coils (FC=15)

void mbSendRequestForceMultipleCoils( void *lpObject )
{
	LP_MB_MASTER_RW_COILS lpData = (LP_MB_MASTER_RW_COILS)lpObject;
	uint8_t txBuffer[260] = { 0 };
	uint8_t i, byte_number;

	lpData->exception_code = 0;

	txBuffer[0] = lpData->address;
	txBuffer[1] = 15;

	txBuffer[2] = lpData->coils_address>>8;
	txBuffer[3] = lpData->coils_address;

	txBuffer[4] = lpData->coils_number>>8;
	txBuffer[5] = lpData->coils_number;

	byte_number = lpData->coils_number / 8;
	if( 8 * byte_number < lpData->coils_number ) {
		++byte_number;
	}

	txBuffer[6] = byte_number;
	for( i = 0; i < byte_number; i++ ) {
		txBuffer[ i + 7 ] = lpData->lpCoils[i];
	}

	mbRtuAddCrcAndSendBuffer( txBuffer, 9 + byte_number );
}

uint8_t mbReceiveRequestForceMultipleCoils( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_RW_COILS lpData = (LP_MB_MASTER_RW_COILS)lpObject;

	if( rxBuffer[0] == lpData->address ) {
		
		if( 15 == rxBuffer[1] ) {
			if( 8 == len &&
				check_crc16( rxBuffer, 6 )
			) {
				uint16_t coils_address, coils_number;

				coils_address = rxBuffer[2]<<8 | rxBuffer[3];
				coils_number = rxBuffer[4]<<8 | rxBuffer[5];

				lpData->exception_code = 0;

				return 1;
			}

			return 0;
		}

		return mbCheckExceptionForResponse( &lpData->exception_code, 15, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Multiple Registers (FC=16)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mbRtuAddCrcAndSendBuffer( uint8_t *txBuffer, uint16_t len )
{
	uint16_t check_sum = usMBCRC16( txBuffer, len - 2 );

	txBuffer[ len - 2 ] = check_sum;
	txBuffer[ len - 1 ] = check_sum>>8;

	rs485SendBuffer( txBuffer, len );
}

uint8_t mbCheckExceptionForResponse( uint8_t *lpExceptionCode, uint8_t txFunctionCode, uint8_t *rxBuffer, uint8_t len )
{
	if( ( 0x80 + txFunctionCode ) == rxBuffer[1] ) {
		if( 5 == len ) {
			uint16_t check_sum;

			check_sum = usMBCRC16( rxBuffer, 3 );

			if( ( ( 0xff & check_sum>>0 ) == rxBuffer[3] ) &&
				( ( 0xff & check_sum>>8 ) == rxBuffer[4] )
			) {
				*lpExceptionCode = rxBuffer[2];
				return 1;
			}
		}
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t check_crc16( uint8_t *buffer, uint8_t len )
{
	uint16_t check_sum;

	check_sum = usMBCRC16( buffer, len );

	if( ( ( 0xff & check_sum>>0 ) == buffer[ len + 0 ] ) &&
		( ( 0xff & check_sum>>8 ) == buffer[ len + 1 ] )
	) {
		return 1;
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
