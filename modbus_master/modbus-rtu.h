#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include <stdio.h>
#include "..//rs485Chanel/rs485Chanel.h"

typedef struct {
	uint8_t exception_code;
	uint8_t address, *lpInputs, inputs_address;
	uint16_t inputs_number;
	uint8_t byte_number;
} MB_MASTER_READ_INPUT_STATUS, *LP_MB_MASTER_READ_INPUT_STATUS;

typedef struct {
	uint8_t exception_code;
	uint8_t address, *lpCoil, *lpCoilNew, coil_address;
} MB_MASTER_WRITE_SINGLE_COIL, *LP_MB_MASTER_WRITE_SINGLE_COIL;

typedef struct {
	uint8_t exception_code;
	uint8_t address;
	uint16_t register_address, *lpRegister, *lpRegisterNew;
} MB_MASTER_PRESET_SINGLE_REGISTER, *LP_MB_MASTER_PRESET_SINGLE_REGISTER;

typedef struct {
	uint8_t exception_code;
	uint8_t address, *lpCoils, coils_address;
	uint16_t coils_number;
} MB_MASTER_RW_COILS, *LP_MB_MASTER_RW_COILS;

void mbSendRequestReadInputStatus( void *lpObject );
uint8_t mbReceiveRequestReadInputStatus( uint8_t *rxBuffer, uint8_t len, void *lpObject );

void mbSendRequestForceSingleCoil( void *lpObject );
uint8_t mbReceiveRequestForceSingleCoil( uint8_t *rxBuffer, uint8_t len, void *lpObject );

void mbSendRequestForceMultipleCoils( void *lpObject );
uint8_t mbReceiveRequestForceMultipleCoils( uint8_t *rxBuffer, uint8_t len, void *lpObject );

void mbSendRequestPresetSingleRegister( void *lpObject );

void mbRtuAddCrcAndSendBuffer( uint8_t *txBuffer, uint16_t len );
uint8_t mbCheckExceptionForResponse( uint8_t *lpExceptionCode, uint8_t txFunctionCode, uint8_t *rxBuffer, uint8_t len );

#endif
