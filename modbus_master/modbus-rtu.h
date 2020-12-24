#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <stdio.h>

#include "mb.h"
#include "mbcrc.h"
#include "..//rs485Channel//rs485Channel.h"

typedef struct stMbMaster
{
	uint8_t *txBuffer;

	uint8_t address, exception_code;
	uint16_t data_address;
	void *lpSocket, *lpReadData, *lpWriteData;

	uint8_t rx_byte_count, tx_byte_count;
	uint16_t rx_address, rx_count;
	uint16_t tx_address, tx_count;

	void *lpUserData;
	void (*lpRxCallback)( void *lpObject, uint8_t *rxBuffer, uint8_t len );
} MB_MASTER_DATA, *LP_MB_MASTER_DATA;

void mbMasterDefInit( LP_MB_MASTER_DATA lpMbMaster );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Coil Status (FC=01) / Read Coils (0x01)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Status (FC=02) / Read Discrete Inputs (0x02)

void mbSendRequestReadDiscreteInputs( void *lpObject );
uint8_t mbReceiveRequestReadDiscreteInputs( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Holding Registers (FC=03) / Read Holding Registers (0x03)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Registers (FC=04) / Read Input Register (0x04)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Single Coil (FC=05) / Write Single Coil (0x05)

void mbSendRequestForceSingleCoil( void *lpObject );
uint8_t mbReceiveRequestForceSingleCoil( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Single Register (FC=06) / Write Single Register (0x06)

void mbSendRequestPresetSingleRegister( void *lpObject );
uint8_t mbReceiveRequestPresetSingleRegister( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Multiple Coils (FC=15) / Write Multiple Coils (0x0F)

void mbSendRequestForceMultipleCoils( void *lpObject );
uint8_t mbReceiveRequestForceMultipleCoils( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Multiple Registers (FC=16) / Write Multiple Registers (0x10)



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Report Slave ID (FC=17) / Report Slave ID (0x11)

void mbSendRequestReportSlaveID( void *lpObject );
uint8_t mbReceiveRequestReportSlaveID( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read/Write Multiple Registers (FC=23) / Read/Write Multiple Registers (0x17)

void mbSendRequestReadWriteMultipleRegisters( void *lpObject );
uint8_t mbReceiveRequestReadWriteMultipleRegisters( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mbRtuAddCrcAndSendBuffer( uint8_t *txBuffer, uint16_t len );
uint8_t mbCheckExceptionForResponse( LP_MB_MASTER_DATA lpData, uint8_t txFunctionCode, uint8_t *rxBuffer, uint8_t len );

uint8_t check_crc16( uint8_t *buffer, uint8_t len );

#endif
