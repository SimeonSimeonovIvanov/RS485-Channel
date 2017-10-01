#include "main.h"

/* ----------------------- AVR includes -------------------------------------*/
#include "avr/io.h"
#include "avr/interrupt.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/

// MB_FUNC_READ_DISCRETE_INPUTS					( 2 )
#define REG_DISC_START							1
#define REG_DISC_SIZE							100

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MB_FUNC_READ_COILS							( 1 )
// MB_FUNC_WRITE_SINGLE_COIL					( 5 )
// MB_FUNC_WRITE_MULTIPLE_COILS					( 15 )
#define REG_COILS_START							1
#define REG_COILS_SIZE							100

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MB_FUNC_READ_INPUT_REGISTER					( 4 )
#define REG_INPUT_START							1
#define REG_INPUT_NREGS							100

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MB_FUNC_WRITE_REGISTER						( 6 )
// MB_FUNC_READ_HOLDING_REGISTER				( 3 )
// MB_FUNC_WRITE_MULTIPLE_REGISTERS				( 16 )
// MB_FUNC_READWRITE_MULTIPLE_REGISTERS			( 23 )
#define REG_HOLDING_START						1
#define REG_HOLDING_NREGS						100

/* ----------------------- Static variables ---------------------------------*/

/*
	BYTE0:					inPort[0..7]				X0 - X7
	BYTE1:					inPort[8..15]				X8 - X15
	BYTE2:
		BIT[0..3]			inPort[16..19]				I16 - I19 ( Spare )
		BIT4				inPort[20]					VCC_FB0
		BIT5 				inPort[21]					VCC_FB0
		BIT6 				inPort[22]					OUT_DIAG0
		BIT7 				inPort[23]					OUT_DIAG1
	BYTE3:
		BIT[0]				inPort[24]					Run switch
		BIT[1]				inPort[25]					Run status
		BIT[2]				inPort[26]					Trig OUT0 Error
		BIT[3]				inPort[27]					Trig OUT1 Error
		BIT[4]				inPort[28]					Timeout error
*/

// MB_FUNC_READ_DISCRETE_INPUTS					( 2 )
volatile static uint8_t ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0 };

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/*
	BYTE0:					outPort[0..7]				Y0 - Y7
	BYTE1:					outPort[8..15]				Y8 - Y15
	BYTE2:
		BIT[0]				outPort[16]					Remote Run
		BIT[1]				outPort[17]					Remote Reset
		BIT[2]				outPort[18]					Disable reset out in Stop mode ( but port set in 0 )
*/

// MB_FUNC_READ_COILS							( 1 )
// MB_FUNC_WRITE_SINGLE_COIL					( 5 )
// MB_FUNC_WRITE_MULTIPLE_COILS					( 15 )
volatile static uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = { 0 };

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MB_FUNC_READ_INPUT_REGISTER					( 4 )
volatile static uint16_t uiRegInputBuf[REG_INPUT_NREGS] = { 0 };

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MB_FUNC_WRITE_REGISTER						( 6 )
// MB_FUNC_READ_HOLDING_REGISTER				( 3 )
// MB_FUNC_WRITE_MULTIPLE_REGISTERS				( 16 )
// MB_FUNC_READWRITE_MULTIPLE_REGISTERS			( 23 )
volatile static uint16_t uiRegHolding[REG_HOLDING_NREGS] = { 0 };

/* ----------------------- Variables ---------------------------------*/

extern volatile uint16_t uiSysRS485SendRequestTimer;
extern volatile uint16_t uiSysRS485ReciverTimer;

volatile uint8_t isRun = 0;
volatile uint8_t inPort[32] = { 0 };
volatile uint8_t outPort[32] = { 0 };
volatile uint8_t hmiLed[40] = { 0 };
volatile uint8_t uc10msTimerEvent = 0;
volatile uint8_t uc100msTimerEvent = 0;
volatile uint16_t uc1000msTimerEvent = 0;

volatile uint8_t ucFlagTimeOutOutError = 0;
volatile uint16_t uiModbusTimeOutCounter = 0;

OBJ_RS485_CHANEL arrRS485Chanel[10];

volatile MB_MASTER_READ_INPUT_STATUS stSlaveChanelReadInputsA3, stSlaveChanelReadInputsA11;
volatile MB_MASTER_RW_COILS stSlaveChanelWriteCoilsA3, stSlaveChanelWriteCoilsA11;

volatile MB_MASTER_PRESET_SINGLE_REGISTER stSlaveChanelPresetRegisterA10;

volatile uint8_t ucRegDiscBufA3[2], ucRegCoilsBufA3[2];
volatile uint8_t ucRegDiscBufA11[2], ucRegCoilsBufA11[2];

volatile uint8_t ucRegDiscBufA3Temp[2];

/* ----------------------- Start implementation -----------------------------*/

int main(void)
{
	uint8_t fFirstRun = 1, ucRS485_address, ucRS485_address_old = 0;
	UCHAR ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
	eMBErrorCode eStatus;
	uint16_t remote_run = 0;

	volatile uint8_t ucFlagOutError, ucTrigOut0Error = 0, ucTrigOut1Error = 0;
	uint8_t ucAutoStopIfOutError = 1;
	uint8_t i, n;

	initBoard();
	
	rs485TaskInit();

	///////////////////////////////////////////////////////////////////////

	rs485ChanelDefInit( &arrRS485Chanel[1] );

	stSlaveChanelReadInputsA3.address = 3;
	stSlaveChanelReadInputsA3.inputs_address = 0;
	stSlaveChanelReadInputsA3.inputs_number = 10;
	stSlaveChanelReadInputsA3.lpInputs = (uint8_t*)ucRegDiscBufA3;

	arrRS485Chanel[1].lpData = (void*)&stSlaveChanelReadInputsA3;
	arrRS485Chanel[1].ucEnableRequest = 1;
	arrRS485Chanel[1].msReadTimeOut = 5;

	arrRS485Chanel[1].rs485SendRequestFunc = mbSendRequestReadInputStatus;
	arrRS485Chanel[1].rs485GetResponseFunc = mbReceiveRequestReadInputStatus;
	arrRS485Chanel[1].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChanel( &arrRS485Chanel[1] );

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	rs485ChanelDefInit( &arrRS485Chanel[2] );

	stSlaveChanelReadInputsA11.address = 11;
	stSlaveChanelReadInputsA11.inputs_address = 0;
	stSlaveChanelReadInputsA11.inputs_number = 10;
	stSlaveChanelReadInputsA11.lpInputs = (uint8_t*)ucRegDiscBufA11;

	arrRS485Chanel[2].lpData = (void*)&stSlaveChanelReadInputsA11;
	arrRS485Chanel[2].ucEnableRequest = 1;
	arrRS485Chanel[2].msReadTimeOut = 5;

	arrRS485Chanel[2].rs485SendRequestFunc = mbSendRequestReadInputStatus;
	arrRS485Chanel[2].rs485GetResponseFunc = mbReceiveRequestReadInputStatus;
	arrRS485Chanel[2].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChanel( &arrRS485Chanel[2] );

	///////////////////////////////////////////////////////////////////////

	rs485ChanelDefInit( &arrRS485Chanel[3] );

	stSlaveChanelWriteCoilsA3.address = 3;
	stSlaveChanelWriteCoilsA3.coils_address = 0;
	stSlaveChanelWriteCoilsA3.coils_number = 12;
	stSlaveChanelWriteCoilsA3.lpCoils = (uint8_t*)ucRegDiscBufA11;

	arrRS485Chanel[3].lpData = (void*)&stSlaveChanelWriteCoilsA3;
	arrRS485Chanel[3].ucEnableRequest = 1;
	arrRS485Chanel[3].msReadTimeOut = 5;

	arrRS485Chanel[3].rs485SendRequestFunc = mbSendRequestForceMultipleCoils;
	arrRS485Chanel[3].rs485GetResponseFunc = mbReceiveRequestForceMultipleCoils;		
	arrRS485Chanel[3].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChanel( &arrRS485Chanel[3] );
	
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	rs485ChanelDefInit( &arrRS485Chanel[4] );

	stSlaveChanelWriteCoilsA11.address = 11;
	stSlaveChanelWriteCoilsA11.coils_address = 0;
	stSlaveChanelWriteCoilsA11.coils_number = 12;
	stSlaveChanelWriteCoilsA11.lpCoils = (uint8_t*)ucRegDiscBufA3Temp;

	arrRS485Chanel[4].lpData = (void*)&stSlaveChanelWriteCoilsA11;
	arrRS485Chanel[4].ucEnableRequest = 1;
	arrRS485Chanel[4].msReadTimeOut = 5;

	arrRS485Chanel[4].rs485SendRequestFunc = mbSendRequestForceMultipleCoils;
	arrRS485Chanel[4].rs485GetResponseFunc = mbReceiveRequestForceMultipleCoils;		
	arrRS485Chanel[4].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChanel( &arrRS485Chanel[4] );

	///////////////////////////////////////////////////////////////////////

	rs485ChanelDefInit( &arrRS485Chanel[5] );

	stSlaveChanelPresetRegisterA10.address = 11;
	stSlaveChanelPresetRegisterA10.register_address = 15;
	stSlaveChanelPresetRegisterA10.lpRegister = &remote_run;
	stSlaveChanelPresetRegisterA10.lpRegisterNew = NULL;

	arrRS485Chanel[5].lpData = (void*)&stSlaveChanelPresetRegisterA10;
	arrRS485Chanel[5].ucEnableRequest = 1;
	arrRS485Chanel[5].msReadTimeOut = 5;

	arrRS485Chanel[5].rs485SendRequestFunc = mbSendRequestPresetSingleRegister;
	arrRS485Chanel[5].rs485GetResponseFunc = rs485getResponseNullFunc; // ???
	arrRS485Chanel[5].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChanel( &arrRS485Chanel[5] );

	///////////////////////////////////////////////////////////////////////

	sei();
	while( 1 ) {
		readDigitalInput( (uint8_t*)inPort );

		ucTrigOut0Error |= ( !inPort[22] && inPort[20] ); // !MCU_DO_DIAG0_bm && MCU_VCC_FB0_bm
		ucTrigOut1Error |= ( !inPort[23] && inPort[21] ); // !MCU_DO_DIAG1_bm && MCU_VCC_FB1_bm
		ucFlagOutError = ucTrigOut1Error || ucTrigOut0Error;

		inPort[25] = isRun;
		inPort[26] = ucTrigOut0Error;
		inPort[27] = ucTrigOut1Error;
		inPort[28] = ucFlagTimeOutOutError;

		ucRS485_address = readAddressSwitch();

		if( fFirstRun || ( ucRS485_address != ucRS485_address_old ) ) {
			ucRS485_address_old = ucRS485_address;

			if( !ucRS485_address ) {
				rs485TaskEnable();
			} else {
				rs485TaskDisable();
			}

			eStatus = eMBDisable();

			if( !rs485TaskIsEnable() ) {
				eStatus = eMBInit( MB_RTU, ucRS485_address, 0, 115200, MB_PAR_EVEN );
				eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 );
				eStatus = eMBEnable();
			} else {
				initRS485( 115200, 8, 1, 1 );
			}
		}

		byteArrToBitArr( (uint8_t*)ucRegDiscBuf, (uint8_t*)inPort, 42 );

		if( uc10msTimerEvent ) {
			uc10msTimerEvent = 0;
		}

		if( uc100msTimerEvent ) {
			uc100msTimerEvent = 0;
			ucRegDiscBufA3Temp[0] ^= 1;
		}

		if( uc1000msTimerEvent ) {
			uc1000msTimerEvent = 0;
		}

		if( !rs485TaskIsEnable() ) {
			eMBPoll();
		} else {
			ucRegCoilsBuf[2] |= 1;
			remote_run = isRun;

			rs485Task();
		}

		uint8_t temp = ( ~1 & ucRegDiscBufA3[0] );

		temp |= ( 1 & ucRegDiscBufA3Temp[0] );

		ucRegDiscBufA3Temp[0] = temp;
		ucRegDiscBufA3Temp[1] = ucRegDiscBufA3[1];

		if( !inPort[24]									|| // !MCU_RUN_STOP_bm
			ucFlagTimeOutOutError						||
			( ucAutoStopIfOutError && ucFlagOutError )
		) {
			ucRegDiscBuf[2] &= ~1; // Clear run 'switch'
		}

		if( !inPort[24]		||	// !MCU_RUN_STOP_bm
			!outPort[16]		// !RUN_STOP 'switch'
		) {
			if( !outPort[18] || ucFlagOutError || ucFlagTimeOutOutError ) {
				ucRegCoilsBuf[ 0 ] = 0;
				ucRegCoilsBuf[ 1 ] = 0;
			}

			PORTE &= ~MCU_DO_DIS_bm;
			isRun = 0;
		} else {
			PORTE |= MCU_DO_DIS_bm;
			isRun = outPort[16];
		}

		n = 0;
		for( i = 0; i < 19; i++ ) {
			n = i / 8;
			outPort[ i ] = 0;
			if( ucRegCoilsBuf[ n ] & ( 1<<(i - 8 * n) ) ) {
				outPort[i] = 1;
			}
		}

		if(	( !inPort[24] || outPort[17] ) && // Reset errors
			( ( inPort[22] && inPort[20] ) && ( inPort[23] && inPort[21] ) )
		) {
			ucTrigOut0Error = 0;
			ucTrigOut1Error = 0;
			ucFlagTimeOutOutError = 0;
		}

		memcpy( (char*)hmiLed, (char*)inPort, 16 );
		memcpy( (char*)&hmiLed[20], (char*)outPort, 16 );

		( ( PORTE & MCU_RS485_DE_bm ) || ucFlagTimeOutOutError ) ? ( hmiLed[19] = 1 ) : ( hmiLed[19] = 0 );

		hmiLed[16] = isRun;
		hmiLed[36] = inPort[20];		// MCU_VCC_FB0_bm
		hmiLed[37] = inPort[21];		// MCU_VCC_FB1_bm
		hmiLed[38] = ucTrigOut0Error;	// !MCU_DO_DIAG0_bm && MCU_VCC_FB0_bm
		hmiLed[39] = ucTrigOut1Error;	// !MCU_DO_DIAG1_bm && MCU_VCC_FB1_bm

		writeHmiLed( (uint8_t*)hmiLed );

		writeDigitalOutput( (uint8_t*)outPort );

		fFirstRun = 0;
	}

	return 0;
}

void mbMasterClrTimeOutError( void *lpObject )
{
	uiModbusTimeOutCounter = uiRegHolding[18];
}

void veznaEepSetUartSetings( void *lpObject )
{
	initRS485( 9600, 8, 1, 0 );
}

void veznaEepRestoreUartSetings( void *lpObject )
{
	initRS485( 115200, 8, 1, 1 );
}

eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
	short iNDiscrete = ( short )usNDiscrete;
	unsigned short usBitOffset;

	// MB_FUNC_READ_DISCRETE_INPUTS          ( 2 )
	/* Check if we have registers mapped at this block. */
	if( (usAddress >= REG_DISC_START) &&
		(usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE)
	) {
		usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
		while(iNDiscrete > 0) {
			*pucRegBuffer++ =
			xMBUtilGetBits
			(
				ucRegDiscBuf, usBitOffset,
				(unsigned char)(iNDiscrete>8? 8:iNDiscrete)
			);
			iNDiscrete -= 8;
			usBitOffset += 8;
		}

		uiModbusTimeOutCounter = uiRegHolding[18];
		return MB_ENOERR;
	}
	
	return MB_ENOREG;
}

eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress,
							USHORT usNCoils, eMBRegisterMode eMode
						  )
{
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;
	
	/* Check if we have registers mapped at this block. */
	if( (usAddress >= REG_COILS_START) &&
		(usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE)
	) {
		usBitOffset = (unsigned short)(usAddress - REG_COILS_START);
		switch(eMode) {
		/*
			Read current values and pass to protocol stack.
			MB_FUNC_READ_COILS						( 1 )
		*/
		case MB_REG_READ:
			while( iNCoils > 0 ) {
				*pucRegBuffer++ =
				xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
								(unsigned char)((iNCoils > 8) ? 8 : iNCoils)
				);
				usBitOffset += 8;
				iNCoils -= 8;
			}

			uiModbusTimeOutCounter = uiRegHolding[18];
		 return MB_ENOERR;
		 
		 /*
		 	Update current register values.
		 	MB_FUNC_WRITE_SINGLE_COIL				( 5 )
			MB_FUNC_WRITE_MULTIPLE_COILS			( 15 )
		 */
		 case MB_REG_WRITE:
			uiModbusTimeOutCounter = uiRegHolding[18];

		 	while( iNCoils > 0 ) {
				xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
								(unsigned char)((iNCoils > 8) ? 8 : iNCoils),
								*pucRegBuffer++
				);
				usBitOffset += 8;
				iNCoils -= 8;
			}

			uiModbusTimeOutCounter = uiRegHolding[18];
		 return MB_ENOERR;
		}
	}
	
	return MB_ENOREG;
}

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	unsigned int iRegIndex;
 
	// MB_FUNC_READ_INPUT_REGISTER           (  4 )
	if( (usAddress >= REG_INPUT_START) &&
		(usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS)
	) {
		iRegIndex = (int)(usAddress - REG_INPUT_START);
		while( usNRegs > 0 ) {
			*pucRegBuffer++ = (unsigned char)(uiRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (unsigned char)(uiRegInputBuf[iRegIndex] & 0xFF);
			++iRegIndex;
			--usNRegs;
		}

		uiModbusTimeOutCounter = uiRegHolding[18];
		return MB_ENOERR;
	}
	return MB_ENOREG;
}

eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress,
							  USHORT usNRegs, eMBRegisterMode eMode
)
{
	unsigned int iRegIndex;
	
	/* Check if we have registers mapped at this block. */
	if( (usAddress >= REG_HOLDING_START) &&
		(usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)
	) {
		switch(eMode) {
		case MB_REG_READ:
			iRegIndex = (int)(usAddress - 1);
			while( usNRegs > 0 ) {
				*pucRegBuffer++ = uiRegHolding[iRegIndex]>>8;
				*pucRegBuffer++ = uiRegHolding[iRegIndex];
				++iRegIndex;
				--usNRegs;
			}

			uiModbusTimeOutCounter = uiRegHolding[18];
		 return MB_ENOERR;

		/*
		 	Update current register values.
		 	MB_FUNC_WRITE_MULTIPLE_REGISTERS             (16)
		*/
		case MB_REG_WRITE: {
			iRegIndex = (int)(usAddress - 1);
			while( usNRegs > 0 ) {
				uiRegHolding[iRegIndex]  = (*pucRegBuffer++)<<8;
				uiRegHolding[iRegIndex] |= *pucRegBuffer++;
				++iRegIndex;
				--usNRegs;
			}

			uiModbusTimeOutCounter = uiRegHolding[18];
		}
		 return MB_ENOERR;
		}
	}

	return MB_ENOREG;
}

void byteArrToBitArr( uint8_t *lpBit, const uint8_t *lpByte, uint16_t bit_count )
{
	uint16_t i, n;

	for( i = 0; i < bit_count; i++ ) {
		n = i / 8;
		if( lpByte[i] ) {
			lpBit[ n ] |=  ( 1<<(i - 8 * n) );
		} else {
			lpBit[ n ] &= ~( 1<<(i - 8 * n) );
		}
	}
}

ISR( TIMER3_COMPA_vect )
{
	volatile static uint16_t n0 = 0, n1 = 0, n2 = 0;

	PORTG ^= MCU_PG0_bm;

	if( 10 == ++n0 ) {
		uc10msTimerEvent = 1;
		n0 = 0;
	}

	if( 100 == ++n1 ) {
		uc100msTimerEvent = 1;
		n1 = 0;
	}

	if( 1000 == ++n2 ) {
		uc1000msTimerEvent = 1;
		n2 = 0;
	}

	rs485TimerIsr();

	if( uiRegHolding[18] ) {
		if( uiModbusTimeOutCounter ) {
			--uiModbusTimeOutCounter;
		} else {
			ucFlagTimeOutOutError = 1;
		}
	} else {
		ucFlagTimeOutOutError = 0;
	}
}

void initBoard(void)
{
	PORTG &= ~MCU_IO_RESET_bm;
	DDRG |= MCU_IO_RESET_bm | MCU_PG1_bm | MCU_PG0_bm;

	initDigitalOutput();

	PORTB |= MCU_CS1_bm | MCU_CS0_bm;
	DDRB |= ( MCU_CS1_bm | MCU_CS0_bm );

	DDRE |= MCU_RS485_DE_bm;
	disable_rs485_transmit();

	spi_init();
	initHmiLed();
	initAddressSwitch();
	initDigitalInput();

	PORTG |= MCU_IO_RESET_bm;

	OCR3A = 249;
	TCCR3A = 0;
	TCCR3B = (1<<WGM32) | (1<<CS31) | (1<<CS30);
	ETIMSK |= (1<<OCIE3A);
}
