#include "modbus-rtu.h"

void mbMasterDefInit( LP_MB_MASTER_DATA lpMbMaster )
{
	lpMbMaster->address = 0;
	lpMbMaster->exception_code = 0;
	lpMbMaster->data_address = 0;
	lpMbMaster->lpSocket = NULL;
	lpMbMaster->lpReadData = NULL;
	lpMbMaster->lpWriteData = NULL;

	lpMbMaster->tx_byte_count = 0;
	lpMbMaster->tx_address = 0;
	lpMbMaster->tx_count = 0;
	lpMbMaster->rx_byte_count = 0;
	lpMbMaster->rx_address = 0;
	lpMbMaster->rx_count = 0;

	lpMbMaster->lpUserData = NULL;
	lpMbMaster->lpRxCallback = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Coil Status (FC=01) / Read Coils (0x01)

void mbSendRequestReadCoils( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	lpData->exception_code = 0;

	lpData->rx_byte_count = lpData->rx_count>>3;
	if( !lpData->rx_byte_count )
	{
		lpData->rx_byte_count = 1;
	}

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 0x01;

	lpData->txBuffer[2] = lpData->rx_address>>8;
	lpData->txBuffer[3] = lpData->rx_address;

	lpData->txBuffer[4] = lpData->rx_count>>8;
	lpData->txBuffer[5] = lpData->rx_count;

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 8 );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Status (FC=02) / Read Discrete Inputs (0x02)

void mbSendRequestReadDiscreteInputs( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	lpData->exception_code = 0;

	lpData->rx_byte_count = lpData->rx_count>>3;
	if( !lpData->rx_byte_count )
	{
		lpData->rx_byte_count = 1;
	}

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 2;

	lpData->txBuffer[2] = lpData->rx_address>>8;
	lpData->txBuffer[3] = lpData->rx_address;

	lpData->txBuffer[4] = lpData->rx_count>>8;
	lpData->txBuffer[5] = lpData->rx_count;

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 8 );
}

uint8_t mbReceiveRequestReadDiscreteInputs( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address )
	{
		if( 2 == rxBuffer[1] )
		{
			if( len == 5 + lpData->rx_byte_count && check_crc16( rxBuffer, 3 + lpData->rx_byte_count ) )
			{
				lpData->exception_code = 0;

				if( NULL != lpData->lpReadData )
				{
					for( uint8_t i = 0; i < lpData->rx_byte_count; i++ )
					{
						( (uint8_t*)lpData->lpReadData )[ i ] = rxBuffer[ 3 + i ];
					}
				}

				if( NULL != lpData->lpRxCallback )
				{
					lpData->lpRxCallback( lpData , rxBuffer, lpData->rx_byte_count );
				}

				return 1;
			}

			return 0;
		}

		return mbCheckExceptionForResponse( lpData, 2, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Holding Registers (FC=03) / Read Holding Registers (0x03)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Registers (FC=04) / Read Input Register (0x04)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Single Coil (FC=05) / Write Single Coil (0x05)

void mbSendRequestForceSingleCoil( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	lpData->exception_code = 0;

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 5;

	lpData->txBuffer[2] = lpData->data_address>>8;
	lpData->txBuffer[3] = lpData->data_address;

	if( *(uint8_t*)lpData->lpWriteData )
	{
		lpData->txBuffer[4] = 0xff;
	}

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 8 );
}

uint8_t mbReceiveRequestForceSingleCoil( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address )
	{
		if( 5 == rxBuffer[1] )
		{
			if( 8 == len && check_crc16( rxBuffer, 6 ) )
			{
				if( rxBuffer[2] == lpData->data_address>>8 && rxBuffer[3] == lpData->data_address && !rxBuffer[5] )
				{
					lpData->exception_code = 0;

					if( NULL != lpData->lpReadData )
					{
						if( 0xff == rxBuffer[4] )
						{
							*(uint8_t*)lpData->lpReadData = 1;
						}

						if( 0x00 == rxBuffer[4] )
						{
							*(uint8_t*)lpData->lpReadData = 0;
						}
					}

					if( NULL != lpData->lpRxCallback )
					{
						lpData->lpRxCallback( lpData , rxBuffer, lpData->rx_byte_count );
					}

					return 1;
				}
			}

			return 0;
		}
		
		return mbCheckExceptionForResponse( lpData, 5, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Single Register (FC=06) / Write Single Register (0x06)

void mbSendRequestPresetSingleRegister( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	lpData->exception_code = 0;

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 6;

	lpData->txBuffer[2] = lpData->data_address>>8;
	lpData->txBuffer[3] = lpData->data_address;

	lpData->txBuffer[4] = *(uint16_t*)lpData->lpWriteData>>8;
	lpData->txBuffer[5] = *(uint16_t*)lpData->lpWriteData;

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 8 );
}

uint8_t mbReceiveRequestPresetSingleRegister( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address )
	{

		if( 6 == rxBuffer[1] )
		{
			if( 8 == len && check_crc16( rxBuffer, 6 ) )
			{
				if( rxBuffer[2] == lpData->data_address>>8 && rxBuffer[3] == lpData->data_address )
				{
					lpData->exception_code = 0;

					if( NULL != lpData->lpReadData )
					{
						*(uint16_t*)lpData->lpReadData = ( rxBuffer[4]<<8 ) | rxBuffer[5];
					}

					if( NULL != lpData->lpRxCallback )
					{
						lpData->lpRxCallback( lpData , rxBuffer, lpData->rx_byte_count );
					}

					return 1;
				}
			}

			return 0;
		}
		
		return mbCheckExceptionForResponse( lpData, 6, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Multiple Coils (FC=15) / Write Multiple Coils (0x0F)

void mbSendRequestForceMultipleCoils( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	lpData->exception_code = 0;

	lpData->tx_byte_count = lpData->tx_count>>3;
	if( !lpData->tx_byte_count )
	{
		lpData->tx_byte_count = 1;
	}

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 15;

	lpData->txBuffer[2] = lpData->data_address>>8;
	lpData->txBuffer[3] = lpData->data_address;

	lpData->txBuffer[4] = lpData->tx_count>>8;
	lpData->txBuffer[5] = lpData->tx_count;

	lpData->txBuffer[6] = lpData->tx_byte_count;

	for( uint8_t i = 0; i < lpData->tx_byte_count; i++ )
	{
		lpData->txBuffer[ i + 7 ] = ( (uint8_t*)lpData->lpWriteData )[i];
	}

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 9 + lpData->tx_byte_count );
}

uint8_t mbReceiveRequestForceMultipleCoils( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address )
	{
		if( 15 == rxBuffer[1] )
		{
			if( 8 == len && check_crc16( rxBuffer, 6 ) )
			{
				uint16_t coils_address, coils_number;

				coils_address = rxBuffer[2]<<8 | rxBuffer[3];
				coils_number = rxBuffer[4]<<8 | rxBuffer[5];

				coils_address = coils_address;
				coils_number = coils_number;

				lpData->exception_code = 0;

				if( NULL != lpData->lpRxCallback )
				{
					lpData->lpRxCallback( lpData , rxBuffer, lpData->tx_byte_count );
				}

				return 1;
			}

			return 0;
		}

		return mbCheckExceptionForResponse( lpData, 15, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Multiple Registers (FC=16) / Write Multiple Registers (0x10)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Report Slave ID (FC=17) / Report Slave ID (0x11)

void mbSendRequestReportSlaveID( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	lpData->exception_code = 0;
	lpData->rx_byte_count = 0;

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 0x11;

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 4 );
}

uint8_t mbReceiveRequestReportSlaveID( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address )
	{
		if( 0x11 == rxBuffer[1] )
		{
			if( len >= 5 )
			{
				lpData->rx_byte_count = rxBuffer[4] + 4;

				if( len >= lpData->rx_byte_count && check_crc16( rxBuffer, lpData->rx_byte_count ) )
				{
					lpData->exception_code = 0;

					if( NULL != lpData->lpReadData )
					{
						/*
						 * TDOD: V lpReadData se kopira prietiq masiv.
						 * Poneje ne se znae kolko shte e to lpReadData da e ukazatel kam 256 bajta?
						 */
					}

					if( NULL != lpData->lpRxCallback )
					{
						lpData->lpRxCallback( lpData, rxBuffer, lpData->rx_byte_count );
					}

					return 1;
				}
			}

			return 0;
		}

		return mbCheckExceptionForResponse( lpData, 5, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read/Write Multiple Registers (FC=23) / Read/Write Multiple Registers (0x17)

void mbSendRequestReadWriteMultipleRegisters( void *lpObject )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;
	uint8_t i;

	lpData->exception_code = 0;

	lpData->rx_byte_count = lpData->rx_count; // Quantity to Read
	if( !lpData->rx_byte_count )
	{
		lpData->rx_byte_count = 1;
	}
	lpData->rx_byte_count *= 2;

	lpData->tx_byte_count = lpData->tx_count; // Quantity to Write
	if( !lpData->tx_byte_count )
	{
		lpData->tx_byte_count = 1;
	}
	lpData->tx_byte_count *= 2;

	lpData->txBuffer[0] = lpData->address;
	lpData->txBuffer[1] = 23;

	lpData->txBuffer[2] = lpData->rx_address>>8;
	lpData->txBuffer[3] = lpData->rx_address;

	lpData->txBuffer[4] = lpData->rx_count>>8;
	lpData->txBuffer[5] = lpData->rx_count;

	lpData->txBuffer[6] = lpData->tx_address>>8;
	lpData->txBuffer[7] = lpData->tx_address;

	lpData->txBuffer[8] = lpData->tx_count>>8;
	lpData->txBuffer[9] = lpData->tx_count;

	lpData->txBuffer[10] = lpData->tx_byte_count;

	for( i = 0; i < lpData->tx_byte_count; i += 2 )
	{
		lpData->txBuffer[ 11 + i ] = *((uint16_t*)(lpData->lpWriteData) + (i>>1))>>8;
		lpData->txBuffer[ 12 + i ] = *((uint16_t*)(lpData->lpWriteData) + (i>>1));
	}

	mbRtuAddCrcAndSendBuffer( lpData->txBuffer, 11 + lpData->tx_byte_count + 2);
}

uint8_t mbReceiveRequestReadWriteMultipleRegisters( void *lpObject, uint8_t *rxBuffer, uint8_t len )
{
	LP_MB_MASTER_DATA lpData = (LP_MB_MASTER_DATA)lpObject;

	if( rxBuffer[0] == lpData->address )
	{
		if( 23 == rxBuffer[1] )
		{
			if( rxBuffer[2] == lpData->rx_byte_count )
			{
				if( (13 + lpData->rx_byte_count >= len) && check_crc16(rxBuffer, 11 + lpData->rx_byte_count) )
				{
					uint8_t rx_cnt;

					lpData->exception_code = 0;

					rx_cnt = rxBuffer[2];

					if( NULL != lpData->lpReadData )
					{
						for( uint8_t i = 0; i < rx_cnt; i+=2 )
						{
							( (uint16_t*)lpData->lpReadData )[ i ]  = rxBuffer[ 3 + i ]<<8;
							( (uint16_t*)lpData->lpReadData )[ i ] |= rxBuffer[ 4 + i ];
						}
					}

					if( NULL != lpData->lpRxCallback )
					{
						lpData->lpRxCallback( lpData , rxBuffer, lpData->rx_byte_count );
					}

					return 1;
				}
			}

			return 0;
		}

		return mbCheckExceptionForResponse( lpData, 15, rxBuffer, len );
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t mbCheckExceptionForResponse( LP_MB_MASTER_DATA lpData, uint8_t txFunctionCode, uint8_t *rxBuffer, uint8_t len )
{
	if( ( 0x80 + txFunctionCode ) == rxBuffer[1] )
	{
		if( 5 == len && check_crc16( rxBuffer, 3 ) )
		{
			lpData->exception_code = rxBuffer[2];

			if( NULL != lpData->lpRxCallback )
			{
				lpData->lpRxCallback( lpData, &rxBuffer[2], len );
			}

			return 1;
		}
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mbRtuAddCrcAndSendBuffer( uint8_t *txBuffer, uint16_t len )
{
	uint16_t check_sum = usMBCRC16( txBuffer, len - 2 );

	txBuffer[ len - 2 ] = check_sum;
	txBuffer[ len - 1 ] = check_sum>>8;

	rs485SendBuffer( txBuffer, len );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t check_crc16( uint8_t *buffer, uint8_t len )
{
	uint16_t check_sum;

	check_sum = usMBCRC16( buffer, len );

	if( (( 0xff & check_sum>>0 ) == buffer[ len + 0 ]) && (( 0xff & check_sum>>8 ) == buffer[ len + 1 ]) )
	{
		return 1;
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
