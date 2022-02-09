#include "main.h"

/* ----------------------- AVR includes -------------------------------------*/
#include "avr/io.h"
#include "avr/interrupt.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
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
		BIT0				inPort[16]					I16			/ New: Run status /
		BIT1				inPort[17]					I17			/ New: Trig Timeout Error /
		BIT2				inPort[18]					I18			/ New  Trig Out Error /
		BIT3				inPort[19]					I17			/ New: Run switch /
		BIT4				inPort[20]					VCC_FB0
		BIT5 				inPort[21]					VCC_FB1
		BIT6 				inPort[22]					OUT_DIAG0
		BIT7 				inPort[23]					OUT_DIAG1

		BIT0				inPort[16]					VCC_FB0
		BIT1 				inPort[17]					VCC_FB1
		BIT2 				inPort[18]					OUT_DIAG0
		BIT3 				inPort[19]					OUT_DIAG1
		BIT4 				inPort[20]					Run switch
		BIT5 				inPort[21]					Run status
		BIT6 				inPort[22]					Error/Fault
		BIT7 				inPort[23]					Power Up


	BYTE3:
		BIT[0]				inPort[24]					Trig VCC_FB0
		BIT[1]				inPort[25]					Trig VCC_FB1
		BIT[2]				inPort[26]					Trig OUT0 Error
		BIT[3]				inPort[27]					Trig OUT1 Error
		BIT[4]				inPort[28]					Trig Out Error *
		BIT[5]				inPort[29]					Trig Timeout Error *
		BIT[6]				inPort[30]					Timeout ERROR is Enable
		BIT[7]				inPort[31]					Spare
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

#define SET_QOIL_REMOTE_RUN						( ucRegCoilsBuf[2] |=  1 )
#define CLR_QOIL_REMOTE_RUN						( ucRegCoilsBuf[2] &= ~1 )
#define SET_QOIL_REMOTE_RESET					( ucRegCoilsBuf[2] |=  2 )
#define CLR_QOIL_REMOTE_RESET					( ucRegCoilsBuf[2] &= ~2 )

#define OUT_REMOTE_RUN							( outPort[16] ) // ( ( ucRegCoilsBuf[2] & 1 ) ? 1 : 0 )
#define OUT_REMOTE_RESET						( outPort[17] ) // ( ( ucRegCoilsBuf[2] & 2 ) ? 1 : 0 )
#define OUT_DISABLE_OUTPUT_RESET_IN_RUN			( outPort[18] ) // ( ( ucRegCoilsBuf[2] & 4 ) ? 1 : 0 )

#define SET_REMOTE_RUN							\
{												\
	SET_QOIL_REMOTE_RUN;						\
}

#define CLR_REMOTE_RUN							\
{												\
	CLR_QOIL_REMOTE_RUN;						\
}

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

UCHAR ucSlaveID[7] =
{
	0x00,		// Len
	0x01, 0x00, // HW
	0x01, 0x00, // FW
	0x00, 0x00  // CRC16
};
uint16_t check_sum;
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

//volatile VEZNA_ELICOM_EEP veznaEEP;

volatile uint8_t ucRegDiscBufA3[2], ucRegCoilsBufA3[2];
volatile uint8_t ucRegDiscBufA10[5];
volatile uint8_t ucRegDiscBufA11[2], ucRegCoilsBufA11[2];

volatile uint8_t ucRegDiscBufA3Temp[2];

volatile MB_MASTER_DATA stSlaveChannelWriteCoilsA3, stSlaveChannelWriteCoilsA11;
volatile MB_MASTER_DATA stSlaveChannelPresetRegisterA10, stSlaveChannelReadInputsA3, stSlaveChannelReadInputsA10, stSlaveChannelReadInputsA11;

void mbMasterSetBaudRate( void *lpObject );
void mbMasterClrTimeOutError1( void *lpObject );
void mbMasterSetTimeOutError1( void *lpObject );

void mbMasterSetTimeOutError( void *lpObject );

/* ----------------------- Start implementation -----------------------------*/

volatile char lpBit[4];

int main(void)
{
	uint8_t fFirstRun = 1, ucRS485_address, ucRS485_address_old = 0;
	eMBErrorCode eStatus;
	uint16_t remote_run = 0, flashTimer = 0;

	volatile uint8_t ucTrigOutError = 0, ucTrigOut0Error = 0, ucTrigOut1Error = 0;
	volatile uint8_t ucTrigTimeOutError = 0;

	uint8_t ucAutoStopIfOutError = 1;
	uint8_t i;

	while( 0 )
	{
		byteArrToBitArr( (uint8_t*)lpBit, (uint8_t*)inPort, 32 );
	}

	ucSlaveID[ 0 ] = len_of_array( ucSlaveID );
	check_sum = usMBCRC16( ucSlaveID, len_of_array( ucSlaveID ) - 2 );
	ucSlaveID[ len_of_array( ucSlaveID ) - 2 ] = check_sum;
	ucSlaveID[ len_of_array( ucSlaveID ) - 1 ] = check_sum>>8;

	initBoard();
	rs485TaskInit();

	///////////////////////////////////////////////////////////////////////////

	rs485ChannelDefInit( &arrRS485Channel[0] );

	stSlaveChannelReadInputsA10.address = 10;
	stSlaveChannelReadInputsA10.data_address = 1;
	stSlaveChannelReadInputsA10.rx_count = 40;
	stSlaveChannelReadInputsA10.lpReadData = (uint8_t*)ucRegDiscBufA10;
	//stSlaveChannelReadInputsA10.baud_rate = 57600;
	stSlaveChannelReadInputsA10.lpSocket = &arrRS485Channel[0];

	arrRS485Channel[0].lpObject = (void*)&stSlaveChannelReadInputsA10;
	arrRS485Channel[0].ucEnableRequest = 1;
	arrRS485Channel[0].msReadTimeOut = 18;

	arrRS485Channel[0].rs485SetUartSetings = mbMasterSetBaudRate;
	arrRS485Channel[0].rs485SendRequestFunc = mbSendRequestReadDiscreteInputs;
	arrRS485Channel[0].rs485GetResponseFunc = mbReceiveRequestReadDiscreteInputs;
	arrRS485Channel[0].rs485SetTimeOutError = mbMasterSetTimeOutError1;
	arrRS485Channel[0].rs485ClrTimeOutError = mbMasterClrTimeOutError1;

	rs485AddChannel( &arrRS485Channel[0] );


	uint16_t baud = 192;
	rs485ChannelDefInit( &arrRS485Channel[1] );

	stSlaveChannelPresetRegisterA10.address = 10;
	stSlaveChannelPresetRegisterA10.data_address = 19;
	stSlaveChannelPresetRegisterA10.lpWriteData = &baud;
	stSlaveChannelPresetRegisterA10.lpReadData = NULL;
	//stSlaveChannelPresetRegisterA10.baud_rate = 9600;
	stSlaveChannelPresetRegisterA10.lpSocket = &arrRS485Channel[1];

	arrRS485Channel[1].lpObject = (void*)&stSlaveChannelPresetRegisterA10;
	arrRS485Channel[1].ucEnableRequest = 1;
	arrRS485Channel[1].msReadTimeOut = 40;

	arrRS485Channel[1].rs485SetUartSetings = mbMasterSetBaudRate;
	arrRS485Channel[1].rs485SendRequestFunc = mbSendRequestPresetSingleRegister;
	arrRS485Channel[1].rs485GetResponseFunc = mbReceiveRequestPresetSingleRegister;
	//arrRS485Channel[1].rs485ClrTimeOutError = mbMasterClrTimeOutError1;

	rs485AddChannel( &arrRS485Channel[1] );

	///////////////////////////////////////////////////////////////////////////

	/*rs485ChannelDefInit( &arrRS485Channel[0] );

	stSlaveChannelWriteCoilsA11.address = 11;
	stSlaveChannelWriteCoilsA11.coils_address = 11;
	stSlaveChannelWriteCoilsA11.lpCoils = (uint8_t*)&inPort[15];

	arrRS485Channel[0].lpObject = (void*)&stSlaveChannelWriteCoilsA11;
	arrRS485Channel[0].ucEnableRequest = 1;
	arrRS485Channel[0].msReadTimeOut = 8;

	arrRS485Channel[0].rs485SendRequestFunc = mbSendRequestForceSingleCoil;
	arrRS485Channel[0].rs485GetResponseFunc = mbReceiveRequestForceSingleCoil;		
	arrRS485Channel[0].rs485SetTimeOutError = rs485SetTimeOutErrorNullFunc;
	arrRS485Channel[0].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[0] );*/

	///////////////////////////////////////////////////////////////////////////

	/*rs485ChannelDefInit( &arrRS485Channel[1] );

	stSlaveChannelReadInputsA3.address = 3;
	stSlaveChannelReadInputsA3.data_address = 0;
	stSlaveChannelReadInputsA3.data_count = 10;
	stSlaveChannelReadInputsA3.lpReadData = (uint8_t*)ucRegDiscBufA3;

	arrRS485Channel[1].lpObject = (void*)&stSlaveChannelReadInputsA3;
	arrRS485Channel[1].ucEnableRequest = 1;
	arrRS485Channel[1].msReadTimeOut = 8;

	arrRS485Channel[1].rs485SendRequestFunc = mbSendRequestReadInputStatus;
	arrRS485Channel[1].rs485GetResponseFunc = mbReceiveRequestReadInputStatus;
	arrRS485Channel[1].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	rs485AddChannel( &arrRS485Channel[1] );*/

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	//rs485ChannelDefInit( &arrRS485Channel[2] );

	stSlaveChannelReadInputsA11.address = 11;
	stSlaveChannelReadInputsA11.data_address = 0;
	stSlaveChannelReadInputsA11.rx_count = 10;
	stSlaveChannelReadInputsA11.lpReadData = (uint8_t*)ucRegDiscBufA11;

	arrRS485Channel[2].lpObject = (void*)&stSlaveChannelReadInputsA11;
	arrRS485Channel[2].ucEnableRequest = 1;
	arrRS485Channel[2].msReadTimeOut = 10;

	arrRS485Channel[2].rs485SendRequestFunc = mbSendRequestReadDiscreteInputs;
	arrRS485Channel[2].rs485GetResponseFunc = mbReceiveRequestReadDiscreteInputs;
	arrRS485Channel[2].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	//rs485AddChannel( &arrRS485Channel[2] );

	///////////////////////////////////////////////////////////////////////////

	rs485ChannelDefInit( &arrRS485Channel[3] );

	stSlaveChannelWriteCoilsA3.address = 3;
	stSlaveChannelWriteCoilsA3.data_address = 0;
	stSlaveChannelWriteCoilsA3.tx_count = 12;
	stSlaveChannelWriteCoilsA3.lpWriteData = (uint8_t*)ucRegDiscBufA11;

	arrRS485Channel[3].lpObject = (void*)&stSlaveChannelWriteCoilsA3;
	arrRS485Channel[3].ucEnableRequest = 1;
	arrRS485Channel[3].msReadTimeOut = 8;

	arrRS485Channel[3].rs485SendRequestFunc = mbSendRequestForceMultipleCoils;
	arrRS485Channel[3].rs485GetResponseFunc = mbReceiveRequestForceMultipleCoils;		
	arrRS485Channel[3].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	//rs485AddChannel( &arrRS485Channel[3] );
	
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	
	//rs485ChannelDefInit( &arrRS485Channel[4] );

	stSlaveChannelWriteCoilsA11.address = 11;
	stSlaveChannelWriteCoilsA11.data_address = 0;
	stSlaveChannelWriteCoilsA11.tx_count = 12;
	stSlaveChannelWriteCoilsA11.lpWriteData = (uint8_t*)ucRegDiscBufA3Temp;

	arrRS485Channel[4].lpObject = (void*)&stSlaveChannelWriteCoilsA11;
	arrRS485Channel[4].ucEnableRequest = 1;
	arrRS485Channel[4].msReadTimeOut = 10;

	arrRS485Channel[4].rs485SendRequestFunc = mbSendRequestForceMultipleCoils;
	arrRS485Channel[4].rs485GetResponseFunc = mbReceiveRequestForceMultipleCoils;		
	arrRS485Channel[4].rs485ClrTimeOutError = mbMasterClrTimeOutError;

	//rs485AddChannel( &arrRS485Channel[4] );

	///////////////////////////////////////////////////////////////////////////

	//rs485ChannelDefInit( &arrRS485Channel[5] );

	/*stSlaveChannelPresetRegisterA10.address = 11;
	stSlaveChannelPresetRegisterA10.register_address = 15;
	stSlaveChannelPresetRegisterA10.lpRegister = &remote_run;
	stSlaveChannelPresetRegisterA10.lpRegisterNew = NULL;

	arrRS485Channel[5].lpObject = (void*)&stSlaveChannelPresetRegisterA10;
	arrRS485Channel[5].ucEnableRequest = 1;
	arrRS485Channel[5].msReadTimeOut = 8;

	arrRS485Channel[5].rs485SendRequestFunc = mbSendRequestPresetSingleRegister;
	arrRS485Channel[5].rs485GetResponseFunc = mbReceiveRequestPresetSingleRegister;
	arrRS485Channel[5].rs485ClrTimeOutError = mbMasterClrTimeOutError;*/

	//rs485AddChannel( &arrRS485Channel[5] );

	///////////////////////////////////////////////////////////////////////////

	//rs485ChannelDefInit( &arrRS485Channel[6] );

	/*int16_t temp;

	veznaEEP.address = 15;
	veznaEEP.lpFlag = (uint16_t*)&temp;
	veznaEEP.lpKg = &temp;
	veznaEEP.lpGr = &temp;
	veznaEEP.lpOldKg = &temp;
	veznaEEP.lpOldGr = &temp;

	arrRS485Channel[6].lpObject = (void*)&veznaEEP;
	arrRS485Channel[6].ucEnableRequest = 1;
	arrRS485Channel[6].msReadTimeOut = 100;

	arrRS485Channel[6].rs485SetUartSetings = veznaEepSetUartSetings;
	arrRS485Channel[6].rs485RestoreUartSetings = veznaEepRestoreUartSetings;

	arrRS485Channel[6].rs485SendRequestFunc = sendRequestVeznaEEP_P04;
	arrRS485Channel[6].rs485GetResponseFunc = getResponseVeznaEEP_P04;
	arrRS485Channel[6].rs485ClrTimeOutError = mbMasterClrTimeOutError;*/

	//rs485AddChannel( &arrRS485Channel[6] );

	///////////////////////////////////////////////////////////////////////////
	uiModbusTimeOutCounter = uiRegHolding[18] = 1000;
	
	sei();
	while( 1 ) {
		readDigitalInput( (uint8_t*)inPort );

		ucTrigOut0Error |= ( !MCU_DO_DIAG0 && MCU_VCC_FB0 ); // !MCU_DO_DIAG0_bm && MCU_VCC_FB0_bm
		ucTrigOut1Error |= ( !MCU_DO_DIAG1 && MCU_VCC_FB1 ); // !MCU_DO_DIAG1_bm && MCU_VCC_FB1_bm
		ucTrigOutError	|= ( ucTrigOut1Error || ucTrigOut0Error );

		ucTrigTimeOutError |= ucFlagTimeOutOutError;

		inPort[16] = isRun;
		inPort[17] = ucTrigTimeOutError;
		inPort[18] = ucTrigOutError;
		inPort[19] = MCU_RUN_SWITCH;
		/*inPort[20] = MCU_VCC_FB0;
		inPort[21] = MCU_VCC_FB1;
		inPort[22] = MCU_DO_DIAG0;
		inPort[23] = MCU_DO_DIAG1;*/

		//inPort[24] <> MCU_RUN_SWITCH
		inPort[25] = isRun;
		inPort[26] = ucTrigOut0Error;
		inPort[27] = ucTrigOut1Error;
		inPort[28] = ucTrigTimeOutError;
		inPort[29] = ucTrigOutError;
		inPort[30] = uiRegHolding[18] ? 1 : 0;
		inPort[31] = 0;

		//=====================================================================
		if( uc10msTimerEvent )
		{
			uc10msTimerEvent = 0;
		}

		if( uc100msTimerEvent )
		{
			uc100msTimerEvent = 0;
		}

		if( uc500msTimerEvent )
		{
			uc500msTimerEvent = 0;

			flashTimer = !flashTimer;
			ucRegDiscBufA3Temp[0] ^= 1;
		}

		if( uc1000msTimerEvent )
		{
			uc1000msTimerEvent = 0;
		}
		//=====================================================================
		ucRS485_address = readAddressSwitch();

		if( fFirstRun || ( ucRS485_address != ucRS485_address_old ) )
		{
			ucRS485_address_old = ucRS485_address;

			if( !ucRS485_address ) {
				rs485TaskEnable();
			} else {
				rs485TaskDisable();
			}

			eStatus = eMBDisable();

			if( !rs485TaskIsEnable() )
			{
				eStatus = eMBInit( MB_RTU, ucRS485_address, 0, /*115200*/2*250000, MB_PAR_NONE );
				eStatus = eMBEnable();

				CLR_QOIL_REMOTE_RUN;
			} else {
				initRS485( 9600, 8, 1, 1 );

				SET_QOIL_REMOTE_RUN;
			}
		}
		eMBSetSlaveID( ucRS485_address, isRun, ucSlaveID, len_of_array( ucSlaveID ) );
		//=====================================================================
		byteArrToBitArr( (uint8_t*)ucRegDiscBuf, (uint8_t*)inPort, 42 );
		//=====================================================================

		if( !rs485TaskIsEnable() )
		{
			uiRegHolding[45]  = ucRegDiscBuf[0];
			uiRegHolding[45] |= ucRegDiscBuf[1]<<8;
			uiRegHolding[46]  = ucRegDiscBuf[2];
			uiRegHolding[46] |= ucRegDiscBuf[3]<<8;

			eMBPoll();
		} else {
			uint8_t temp;

			SET_QOIL_REMOTE_RUN;
			remote_run = isRun;

			rs485Task();

			temp  = ( ~1 & ucRegDiscBufA3[0] );
			temp |= ( 1 & ucRegDiscBufA3Temp[0] );

			ucRegDiscBufA3Temp[0] = temp;
			ucRegDiscBufA3Temp[1] = ucRegDiscBufA3[1];


			ucRegCoilsBuf[0] = ucRegDiscBufA10[0];
			ucRegCoilsBuf[1] = ucRegDiscBufA10[1];
		}

		//=====================================================================
		for( i = 0; i < 19; i++ )
		{
			outPort[i] = bitarr_read((uint8_t*)&ucRegCoilsBuf, i);
		}

		isRun = OUT_REMOTE_RUN;
		//=====================================================================
		if( !MCU_RUN_SWITCH || ucTrigTimeOutError || ( ucAutoStopIfOutError && ucTrigOutError ) )
		{
			CLR_QOIL_REMOTE_RUN;
		}

		if( !MCU_RUN_SWITCH || !OUT_REMOTE_RUN )
		{
			if( !OUT_DISABLE_OUTPUT_RESET_IN_RUN || ucTrigOutError || ucTrigTimeOutError )
			{
				ucRegCoilsBuf[ 0 ] = 0;
				ucRegCoilsBuf[ 1 ] = 0;
			}

			PORTE &= ~MCU_DO_DIS_bm;
		} else {
			PORTE |= MCU_DO_DIS_bm;
		}
		//=====================================================================
		writeDigitalOutput( (uint8_t*)outPort );
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

		if(	( !MCU_RUN_SWITCH || OUT_REMOTE_RESET ) //&&
			//( ( MCU_VCC_FB1 && MCU_DO_DIAG1 ) && ( MCU_VCC_FB0 && MCU_DO_DIAG0 ) )
		)
		{
			ucTrigTimeOutError = 0;
			ucTrigOut0Error = 0;
			ucTrigOut1Error = 0;

			ucFlagTimeOutOutError = 0;
			ucTrigOutError = 0;

			CLR_QOIL_REMOTE_RESET;
		}

		//=====================================================================

		fFirstRun = 0;
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
void mbMasterSetBaudRate( void *lpObject )
{
	//LP_MB_MASTER_DATA lpData= (LP_MB_MASTER_DATA)lpObject;
	//initRS485( lpData->baud_rate, 8, 1, 1 );
}

void mbMasterSetTimeOutError1( void *lpObject )
{
	arrRS485Channel[1].ucEnableRequest = 1;
}

void mbMasterClrTimeOutError1( void *lpObject )
{
	arrRS485Channel[1].ucEnableRequest = 0;
}
///////////////////////////////////////////////////////////////////////////////

void mbMasterClrTimeOutError( void *lpObject )
{
	uiModbusTimeOutCounter = uiRegHolding[18];
}

void veznaEepSetUartSetings( void *lpObject )
{
	initRS485( 9600, 8, 1, 1 );
}

void veznaEepRestoreUartSetings( void *lpObject )
{
	initRS485( 115200, 8, 1, 1 );
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
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
				(uint8_t*)ucRegDiscBuf, usBitOffset,
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

eMBErrorCode eMBRegCoilsCB( UCHAR *pucRegBuffer, USHORT usAddress,
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
				xMBUtilGetBits( (uint8_t*)ucRegCoilsBuf, usBitOffset,
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
				xMBUtilSetBits( (uint8_t*)ucRegCoilsBuf, usBitOffset,
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

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
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

eMBErrorCode eMBRegHoldingCB( UCHAR *pucRegBuffer, USHORT usAddress,
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

				if( 50 == iRegIndex )
				{
					ucRegCoilsBuf[0] = uiRegHolding[iRegIndex];
					ucRegCoilsBuf[1] = uiRegHolding[iRegIndex]>>8;
				}
				if( 51 == iRegIndex )
				{
					ucRegCoilsBuf[2] = uiRegHolding[iRegIndex];
					ucRegCoilsBuf[3] = uiRegHolding[iRegIndex]>>8;
				}
				if( 52 == iRegIndex )
				{
					switch( 0xff & (uiRegHolding[iRegIndex]>>8) )
					{
					case 1:
						uiRegHolding[18] = (0xff & uiRegHolding[iRegIndex])<<2;
					break;
					}
				}

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
	uint16_t i ;

	for( i = 0; i < bit_count; i++ ) {
		bitarr_write(lpBit, i, (1 & lpByte[i] ) ? 1:0 );
	}

	/*for( uint16_t i = 0; i < bit_count; i++ ) {
		uint16_t n;

		n = i / 8;
		if( lpByte[i] ) {
			lpBit[ n ] |=  ( 1<<(i - 8 * n) );
		} else {
			lpBit[ n ] &= ~( 1<<(i - 8 * n) );
		}
	}*/
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
