#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <stdio.h>

#include "mb.h"
#include "mbcrc.h"
#include "..//rs485Channel//rs485Channel.h"

typedef struct 
{
	uint8_t address, exception_code;

	uint16_t data_address, data_count;
	uint8_t rxtx_byte_count;
	void *lpReadData;
	void *lpWriteData;

	void *lpSocket;

	uint32_t baud_rate;

	//void *lpUserData;
	//void *(lpUserCallback( void *lpObject ));
} MB_MASTER_DATA, *LP_MB_MASTER_DATA;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Coil Status (FC=01)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Status (FC=02)

void mbSendRequestReadInputStatus( void *lpObject );
uint8_t mbReceiveRequestReadInputStatus( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Holding Registers (FC=03)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Input Registers (FC=04)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Single Coil (FC=05)

typedef struct
{
	uint8_t exception_code;
	uint8_t address, *lpCoil, *lpCoilNew, coil_address;
} MB_MASTER_WRITE_SINGLE_COIL, *LP_MB_MASTER_WRITE_SINGLE_COIL;

void mbSendRequestForceSingleCoil( void *lpObject );
uint8_t mbReceiveRequestForceSingleCoil( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Single Register (FC=06)

typedef struct
{
	uint8_t exception_code;
	uint8_t address;
	uint16_t register_address, *lpRegister, *lpRegisterNew;
} MB_MASTER_PRESET_SINGLE_REGISTER, *LP_MB_MASTER_PRESET_SINGLE_REGISTER;

void mbSendRequestPresetSingleRegister( void *lpObject );
uint8_t mbReceiveRequestPresetSingleRegister( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Force Multiple Coils (FC=15)

void mbSendRequestForceMultipleCoils( void *lpObject );
uint8_t mbReceiveRequestForceMultipleCoils( void *lpObject, uint8_t *rxBuffer, uint8_t len );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preset Multiple Registers (FC=16)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mbRtuAddCrcAndSendBuffer( uint8_t *txBuffer, uint16_t len );
uint8_t mbCheckExceptionForResponse( uint8_t *lpExceptionCode, uint8_t txFunctionCode, uint8_t *rxBuffer, uint8_t len );

uint8_t check_crc16( uint8_t *buffer, uint8_t len );

#endif
