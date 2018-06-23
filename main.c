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
		BIT[4]				inPort[28]					Trig Timeout Error
		BIT[5]				inPort[29]					Trig Out Error
		BIT[5]				inPort[30]					Timeout ERROR is Enable
*/

// MB_FUNC_READ_DISCRETE_INPUTS					( 2 )
volatile static uint8_t ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0 };

#define MCU_VCC_FB0			inPort[20]
#define MCU_VCC_FB1			inPort[21]
#define MCU_DO_DIAG0		inPort[22]
#define MCU_DO_DIAG1		inPort[23]
#define MCU_RUN_SWITCH		inPort[24]

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

#define QOIL_REMOTE_RUN							outPort[16]
#define QOIL_REMOTE_RESET						outPort[17]
#define QOIL_DISABLE_OUTPUT_RESET_IN_RUN		outPort[18]

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

volatile uint8_t isRun = 0;
volatile uint8_t inPort[32] = { 0 };
volatile uint8_t outPort[32] = { 0 };
volatile uint8_t hmiLed[40] = { 0 };
volatile uint8_t uc10msTimerEvent = 0;
volatile uint8_t uc100msTimerEvent = 0;
volatile uint16_t uc500msTimerEvent = 0;
volatile uint16_t uc1000msTimerEvent = 0;

volatile uint8_t ucFlagTimeOutOutError = 0;
volatile uint16_t uiModbusTimeOutCounter = 0;

OBJ_RS485_CHANNEL arrRS485Channel[10];

volatile MB_MASTER_READ_INPUT_STATUS stSlaveChannelReadInputsA3, stSlaveChannelReadInputsA11;
volatile MB_MASTER_RW_COILS stSlaveChannelWriteCoilsA3, stSlaveChannelWriteCoilsA11;

volatile MB_MASTER_PRESET_SINGLE_REGISTER stSlaveChannelPresetRegisterA10;

//volatile VEZNA_ELICOM_EEP veznaEEP;

volatile uint8_t ucRegDiscBufA3[2], ucRegCoilsBufA3[2];
volatile uint8_t ucRegDiscBufA11[2], ucRegCoilsBufA11[2];

volatile uint8_t ucRegDiscBufA3Temp[2];

/* ----------------------- Start implementation -----------------------------*/

int main(void)
{
	uint8_t fFirstRun = 1, ucRS485_address, ucRS485_address_old = 0;
	UCHAR ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
	eMBErrorCode eStatus;
	uint16_t remote_run = 0, flashTimer = 0;

	volatile uint8_t ucTrigOutError = 0, ucTrigOut0Error = 0, ucTrigOut1Error = 0;
	volatile uint8_t ucTrigTimeOutError = 0;

	uint8_t ucAutoStopIfOutError = 1;
	uint8_t i, n;

	initBoard();
	
	rs485TaskInit();

	///////////////////////////////////////////////////////////////////////////

	rs485ChannelDefInit( &arrRS485Channel[0] );

	stSlaveChannelWriteCoilsA11.address = 11;
	stSlaveChannelWriteCoilsA11.coils_address = 11;
	stSlaveChannelWriteCoilsA11.lpCoils = (uint8_t*)&inPort[15];

	arrRS485Channel[0].lpData = (void*)&stSlaveChannelWriteCoilsA11;
	arrRS485Channel[0].ucEnableRequest = 1;
	arrRS485Channel[0].msReadTimeOut = 8;

	arrRS485Channel[0].rs485SendRequestFunc = mbSendRequestForceSingleCoil;
	arrRS485Channel[0].rs485GetResponseFunc = mbReceiveRequestForceSingleCoil;		
	arrRS485Channel[0].rs485SetTimeOutError = rs485SetTimeOutErrorNullFunc;
	arrRS485Channel[0].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	//rs485AddChannel( &arrRS485Channel[0] );

	///////////////////////////////////////////////////////////////////////////

	rs485ChannelDefInit( &arrRS485Channel[1] );

	stSlaveChannelReadInputsA3.address = 3;
	stSlaveChannelReadInputsA3.inputs_address = 0;
	stSlaveChannelReadInputsA3.inputs_number = 10;
	stSlaveChannelReadInputsA3.lpInputs = (uint8_t*)ucRegDiscBufA3;

	arrRS485Channel[1].lpData = (void*)&stSlaveChannelReadInputsA3;
	arrRS485Channel[1].ucEnableRequest = 1;
	arrRS485Channel[1].msReadTimeOut = 8;

	arrRS485Channel[1].rs485SendRequestFunc = mbSendRequestReadInputStatus;
	arrRS485Channel[1].rs485GetResponseFunc = mbReceiveRequestReadInputStatus;
	arrRS485Channel[1].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[1] );

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	rs485ChannelDefInit( &arrRS485Channel[2] );

	stSlaveChannelReadInputsA11.address = 11;
	stSlaveChannelReadInputsA11.inputs_address = 0;
	stSlaveChannelReadInputsA11.inputs_number = 10;
	stSlaveChannelReadInputsA11.lpInputs = (uint8_t*)ucRegDiscBufA11;

	arrRS485Channel[2].lpData = (void*)&stSlaveChannelReadInputsA11;
	arrRS485Channel[2].ucEnableRequest = 1;
	arrRS485Channel[2].msReadTimeOut = 10;

	arrRS485Channel[2].rs485SendRequestFunc = mbSendRequestReadInputStatus;
	arrRS485Channel[2].rs485GetResponseFunc = mbReceiveRequestReadInputStatus;
	arrRS485Channel[2].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[2] );

	///////////////////////////////////////////////////////////////////////////

	rs485ChannelDefInit( &arrRS485Channel[3] );

	stSlaveChannelWriteCoilsA3.address = 3;
	stSlaveChannelWriteCoilsA3.coils_address = 0;
	stSlaveChannelWriteCoilsA3.coils_number = 12;
	stSlaveChannelWriteCoilsA3.lpCoils = (uint8_t*)ucRegDiscBufA11;

	arrRS485Channel[3].lpData = (void*)&stSlaveChannelWriteCoilsA3;
	arrRS485Channel[3].ucEnableRequest = 1;
	arrRS485Channel[3].msReadTimeOut = 8;

	arrRS485Channel[3].rs485SendRequestFunc = mbSendRequestForceMultipleCoils;
	arrRS485Channel[3].rs485GetResponseFunc = mbReceiveRequestForceMultipleCoils;		
	arrRS485Channel[3].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[3] );
	
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	rs485ChannelDefInit( &arrRS485Channel[4] );

	stSlaveChannelWriteCoilsA11.address = 11;
	stSlaveChannelWriteCoilsA11.coils_address = 0;
	stSlaveChannelWriteCoilsA11.coils_number = 12;
	stSlaveChannelWriteCoilsA11.lpCoils = (uint8_t*)ucRegDiscBufA3Temp;

	arrRS485Channel[4].lpData = (void*)&stSlaveChannelWriteCoilsA11;
	arrRS485Channel[4].ucEnableRequest = 1;
	arrRS485Channel[4].msReadTimeOut = 10;

	arrRS485Channel[4].rs485SendRequestFunc = mbSendRequestForceMultipleCoils;
	arrRS485Channel[4].rs485GetResponseFunc = mbReceiveRequestForceMultipleCoils;		
	arrRS485Channel[4].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[4] );

	///////////////////////////////////////////////////////////////////////////

	rs485ChannelDefInit( &arrRS485Channel[5] );

	stSlaveChannelPresetRegisterA10.address = 11;
	stSlaveChannelPresetRegisterA10.register_address = 15;
	stSlaveChannelPresetRegisterA10.lpRegister = &remote_run;
	stSlaveChannelPresetRegisterA10.lpRegisterNew = NULL;

	arrRS485Channel[5].lpData = (void*)&stSlaveChannelPresetRegisterA10;
	arrRS485Channel[5].ucEnableRequest = 1;
	arrRS485Channel[5].msReadTimeOut = 8;

	arrRS485Channel[5].rs485SendRequestFunc = mbSendRequestPresetSingleRegister;
	arrRS485Channel[5].rs485GetResponseFunc = mbReceiveRequestPresetSingleRegister;
	arrRS485Channel[5].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[5] );

	///////////////////////////////////////////////////////////////////////////

	/*rs485ChannelDefInit( &arrRS485Channel[6] );

	int16_t temp;

	veznaEEP.address = 15;
	veznaEEP.lpFlag = &temp;
	veznaEEP.lpKg = &temp;
	veznaEEP.lpGr = &temp;
	veznaEEP.lpOldKg = &temp;
	veznaEEP.lpOldGr = &temp;

	arrRS485Channel[6].lpData = (void*)&veznaEEP;
	arrRS485Channel[6].ucEnableRequest = 1;
	arrRS485Channel[6].msReadTimeOut = 100;

	arrRS485Channel[6].rs485SetUartSetings = veznaEepSetUartSetings;
	arrRS485Channel[6].rs485RestoreUartSetings = veznaEepRestoreUartSetings;

	arrRS485Channel[6].rs485SendRequestFunc = sendRequestVeznaEEP_P04;
	arrRS485Channel[6].rs485GetResponseFunc = getResponseVeznaEEP_P04;
	arrRS485Channel[6].rs485ClrTimeOutError = mbMasterClrTimeOutError;*/

	//rs485AddChannel( &arrRS485Channel[6] );*/

	///////////////////////////////////////////////////////////////////////////

	sei();
	while( 1 ) {
		readDigitalInput( (uint8_t*)inPort );

		ucTrigOut0Error |= ( !MCU_DO_DIAG0 && MCU_VCC_FB0 ); // !MCU_DO_DIAG0_bm && MCU_VCC_FB0_bm
		ucTrigOut1Error |= ( !MCU_DO_DIAG1 && MCU_VCC_FB1 ); // !MCU_DO_DIAG1_bm && MCU_VCC_FB1_bm
		ucTrigOutError	|= ( ucTrigOut1Error || ucTrigOut0Error );

		ucTrigTimeOutError |= ucFlagTimeOutOutError;

		inPort[25] = isRun;
		inPort[26] = ucTrigOut0Error;
		inPort[27] = ucTrigOut1Error;
		inPort[28] = ucTrigTimeOutError;
		inPort[29] = ucTrigOutError;
		inPort[30] = uiRegHolding[18] ? 1 : 0;

		//=====================================================================

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
				eStatus = eMBInit( MB_RTU, ucRS485_address, 0, 115200/2, MB_PAR_EVEN );
				eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 );
				eStatus = eMBEnable();
				ucRegCoilsBuf[2] &= ~1;
			} else {
				initRS485( 115200, 8, 1, 1 );
				ucRegCoilsBuf[2] |= 1;
			}
		}

		//=====================================================================

		if( uc10msTimerEvent ) {
			static uint8_t n = 0;

			uc10msTimerEvent = 0;

			if( ++n == 50 ) {
				ucRegDiscBufA3Temp[0] ^= 1;
				n = 0;
			}
		}

		if( uc100msTimerEvent ) {
			uc100msTimerEvent = 0;
		}

		if( uc500msTimerEvent ) {
			uc500msTimerEvent = 0;

			flashTimer = !flashTimer;
		}

		if( uc1000msTimerEvent ) {
			uc1000msTimerEvent = 0;
		}

		//=====================================================================

		byteArrToBitArr( (uint8_t*)ucRegDiscBuf, (uint8_t*)inPort, 42 );

		//=====================================================================

		if( !rs485TaskIsEnable() ) {
			eMBPoll();
		} else {
			uint8_t temp;

			ucRegCoilsBuf[2] |= 1;
			remote_run = isRun;

			rs485Task();

			temp  = ( ~1 & ucRegDiscBufA3[0] );
			temp |= ( 1 & ucRegDiscBufA3Temp[0] );

			ucRegDiscBufA3Temp[0] = temp;
			ucRegDiscBufA3Temp[1] = ucRegDiscBufA3[1];
		}

		//=====================================================================

		if( !MCU_RUN_SWITCH || ucTrigTimeOutError || ( ucAutoStopIfOutError && ucTrigOutError ) ) {
			ucRegCoilsBuf[2] &= ~1; // Clear QOIL_REMOTE_RUN
		}

		if( !MCU_RUN_SWITCH || !QOIL_REMOTE_RUN ) {
			if( !QOIL_DISABLE_OUTPUT_RESET_IN_RUN || ucTrigOutError || ucTrigTimeOutError ) {
				ucRegCoilsBuf[ 0 ] = 0;
				ucRegCoilsBuf[ 1 ] = 0;
			}

			PORTE &= ~MCU_DO_DIS_bm;
		} else {
			PORTE |= MCU_DO_DIS_bm;
		}

		//=====================================================================

		n = 0;
		for( i = 0; i < 19; i++ ) {
			n = i / 8;
			outPort[ i ] = 0;
			if( ucRegCoilsBuf[ n ] & ( 1<<(i - 8 * n) ) ) {
				outPort[i] = 1;
			}
		}

		writeDigitalOutput( (uint8_t*)outPort );

		//=====================================================================

		isRun = QOIL_REMOTE_RUN;

		if(	( !MCU_RUN_SWITCH || QOIL_REMOTE_RESET )// &&
			//( ( MCU_VCC_FB1 && MCU_DO_DIAG1 ) && ( MCU_VCC_FB0 && MCU_DO_DIAG0 ) )
		) {
			ucTrigTimeOutError = 0;
			ucTrigOut0Error = 0;
			ucTrigOut1Error = 0;

			ucFlagTimeOutOutError = 0;
			ucTrigOutError = 0;
		}

		//=====================================================================
		// HMI:
		memcpy( (char*)hmiLed, (char*)inPort, 16 );
		memcpy( (char*)&hmiLed[20], (char*)outPort, 16 );

		hmiLed[16] = !isRun || ( isRun && flashTimer );
		hmiLed[18] = ucTrigTimeOutError;
		hmiLed[19] = ( PORTE & MCU_RS485_DE_bm ) ? 1 : 0;
		hmiLed[36] = MCU_VCC_FB0;
		hmiLed[37] = MCU_VCC_FB1;
		hmiLed[38] = ucTrigOut0Error;
		hmiLed[39] = ucTrigOut1Error;

		writeHmiLed( (uint8_t*)hmiLed );
		//=====================================================================

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
	volatile static uint16_t n0 = 0, n1 = 0, n2 = 0, n3 = 0;

	PORTG |= MCU_PG0_bm;

	if( 10 == ++n0 ) {
		uc10msTimerEvent = 1;
		n0 = 0;
	}

	if( 100 == ++n1 ) {
		uc100msTimerEvent = 1;
		n1 = 0;
	}

	if( 500 == ++n2 ) {
		uc500msTimerEvent = 1;
		n2 = 0;
	}

	if( 1000 == ++n3 ) {
		uc1000msTimerEvent = 1;
		n3 = 0;
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

	PORTG &= ~MCU_PG0_bm;
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
