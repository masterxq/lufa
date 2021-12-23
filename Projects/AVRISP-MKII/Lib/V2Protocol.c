/*
             LUFA Library
     Copyright (C) Dean Camera, 2021.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2021  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  V2Protocol handler, to process V2 Protocol commands used in Atmel programmer devices.
 */

#define  INCLUDE_FROM_V2PROTOCOL_C
#include "V2Protocol.h"

/** Current memory address for FLASH/EEPROM memory read/write commands */
uint32_t CurrentAddress;

/** Flag to indicate that the next read/write operation must update the device's current extended FLASH address */
bool MustLoadExtendedAddress;

#if ARCH == ARCH_AVR8
/** ISR to manage timeouts whilst processing a V2Protocol command */
ISR(TIMER0_COMPA_vect, ISR_NOBLOCK)
{
	if (TimeoutTicksRemaining)
	  TimeoutTicksRemaining--;
	else
	  TCCR0B = 0;
}
#elif ARCH == ARCH_XMEGA
ISR(DELAY_TIMER_OVF_vect, ISR_NOBLOCK)
{
	if (TimeoutTicksRemaining)
	  TimeoutTicksRemaining--;
	else
		DELAY_TIMER.CTRLA = TC_CLKSEL_OFF_gc; 
}
#endif

/** Initializes the hardware and software associated with the V2 protocol command handling. */
void V2Protocol_Init(void)
{
	#if defined(ADC) && !defined(NO_VTARGET_DETECT)
	/* Initialize the ADC converter for VTARGET level detection on supported AVR models */
	ADC_Init(ADC_FREE_RUNNING | ADC_PRESCALE_128);
	ADC_SetupChannel(VTARGET_ADC_CHANNEL);
	ADC_StartReading(VTARGET_REF_MASK | ADC_RIGHT_ADJUSTED | VTARGET_ADC_CHANNEL_MASK);
	#elif !defined(NO_VTARGET_DETECT) && (ARCH == ARCH_XMEGA)
	//TODO: Write LUFA XMega ADC Driver
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t cal = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	ADCA.CALL = cal;

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	cal = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	ADCA.CALH = cal;
	ADCA.CTRLB = ADC_FREERUN_bm | ADC_IMPMODE_bm;
	PORTA.DIRCLR = PIN0_bm;
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;
	ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc;
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;
	ADCA.CTRLA = ADC_ENABLE_bm;
	#endif

	/* Timeout timer initialization (~10ms period) */
#if ARCH == ARCH_AVR8
	OCR0A  = (((F_CPU / 1024) / 100) - 1);
	TCCR0A = (1 << WGM01);
	TIMSK0 = (1 << OCIE0A);
#elif ARCH == ARCH_XMEGA
	DELAY_TIMER.PER = (((F_CPU / 1024) / 100) - 1);
	DELAY_TIMER.CTRLB = TC_WGMODE_NORMAL_gc;
	DELAY_TIMER.INTCTRLA = TC_OVFINTLVL_MED_gc;
#endif

	V2Params_LoadNonVolatileParamValues();

	#if defined(ENABLE_ISP_PROTOCOL)
	ISPTarget_ConfigureRescueClock();
	#endif
}

/** Master V2 Protocol packet handler, for received V2 Protocol packets from a connected host.
 *  This routine decodes the issued command and passes off the handling of the command to the
 *  appropriate function.
 */
void V2Protocol_ProcessCommand(void)
{
	uint8_t V2Command = Endpoint_Read_8();

	/* Reset timeout counter duration and start the timer */
	TimeoutTicksRemaining = COMMAND_TIMEOUT_TICKS;
#if ARCH == ARCH_AVR8
	TCCR0B = ((1 << CS02) | (1 << CS00));
#elif ARCH == ARCH_XMEGA
	DELAY_TIMER.CTRLA = TC_CLKSEL_DIV1024_gc; 
#endif
	switch (V2Command)
	{
		case CMD_SIGN_ON:
			V2Protocol_SignOn();
			break;
		case CMD_SET_PARAMETER:
		case CMD_GET_PARAMETER:
			V2Protocol_GetSetParam(V2Command);
			break;
		case CMD_LOAD_ADDRESS:
			V2Protocol_LoadAddress();
			break;
		case CMD_RESET_PROTECTION:
			V2Protocol_ResetProtection();
			break;
#if defined(ENABLE_ISP_PROTOCOL)
		case CMD_ENTER_PROGMODE_ISP:
			ISPProtocol_EnterISPMode();
			break;
		case CMD_LEAVE_PROGMODE_ISP:
			ISPProtocol_LeaveISPMode();
			break;
		case CMD_PROGRAM_FLASH_ISP:
		case CMD_PROGRAM_EEPROM_ISP:
			ISPProtocol_ProgramMemory(V2Command);
			break;
		case CMD_READ_FLASH_ISP:
		case CMD_READ_EEPROM_ISP:
			ISPProtocol_ReadMemory(V2Command);
			break;
		case CMD_CHIP_ERASE_ISP:
			ISPProtocol_ChipErase();
			break;
		case CMD_OSCCAL:
			ISPProtocol_Calibrate();
			break;
		case CMD_READ_FUSE_ISP:
		case CMD_READ_LOCK_ISP:
		case CMD_READ_SIGNATURE_ISP:
		case CMD_READ_OSCCAL_ISP:
			ISPProtocol_ReadFuseLockSigOSCCAL(V2Command);
			break;
		case CMD_PROGRAM_FUSE_ISP:
		case CMD_PROGRAM_LOCK_ISP:
			ISPProtocol_WriteFuseLock(V2Command);
			break;
		case CMD_SPI_MULTI:
			ISPProtocol_SPIMulti();
			break;
#endif
#if defined(ENABLE_XPROG_PROTOCOL)
		case CMD_XPROG_SETMODE:
			XPROGProtocol_SetMode();
			break;
		case CMD_XPROG:
			XPROGProtocol_Command();
			break;
#endif
		default:
			V2Protocol_UnknownCommand(V2Command);
			break;
	}

	/* Disable the timeout management timer */
#if ARCH == ARCH_AVR8
	TCCR0B = 0;
#elif ARCH == ARCH_XMEGA
	DELAY_TIMER.CTRLA = TC_CLKSEL_OFF_gc; 
#endif
	
	Endpoint_WaitUntilReady();
	Endpoint_SelectEndpoint(AVRISP_DATA_OUT_EPADDR);
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_OUT);
}

/** Handler for unknown V2 protocol commands. This discards all sent data and returns a
 *  STATUS_CMD_UNKNOWN status back to the host.
 *
 *  \param[in] V2Command  Issued V2 Protocol command byte from the host
 */
static void V2Protocol_UnknownCommand(const uint8_t V2Command)
{
	/* Discard all incoming data */
	while (Endpoint_BytesInEndpoint() == AVRISP_DATA_EPSIZE)
	{
		Endpoint_ClearOUT();
		Endpoint_WaitUntilReady();
	}

	Endpoint_ClearOUT();
	Endpoint_SelectEndpoint(AVRISP_DATA_IN_EPADDR);
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);

	Endpoint_Write_8(V2Command);
	Endpoint_Write_8(STATUS_CMD_UNKNOWN);
	Endpoint_ClearIN();
}

/** Handler for the CMD_SIGN_ON command, returning the programmer ID string to the host. */
static void V2Protocol_SignOn(void)
{
	Endpoint_ClearOUT();
	Endpoint_SelectEndpoint(AVRISP_DATA_IN_EPADDR);
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);

	Endpoint_Write_8(CMD_SIGN_ON);
	Endpoint_Write_8(STATUS_CMD_OK);
	Endpoint_Write_8(sizeof(PROGRAMMER_ID) - 1);
	Endpoint_Write_Stream_LE(PROGRAMMER_ID, (sizeof(PROGRAMMER_ID) - 1), NULL);
	Endpoint_ClearIN();
}

/** Handler for the CMD_RESET_PROTECTION command, implemented as a dummy ACK function as
 *  no target short-circuit protection is currently implemented.
 */
static void V2Protocol_ResetProtection(void)
{
	Endpoint_ClearOUT();
	Endpoint_SelectEndpoint(AVRISP_DATA_IN_EPADDR);
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);

	Endpoint_Write_8(CMD_RESET_PROTECTION);
	Endpoint_Write_8(STATUS_CMD_OK);
	Endpoint_ClearIN();
}


/** Handler for the CMD_SET_PARAMETER and CMD_GET_PARAMETER commands from the host, setting or
 *  getting a device parameter's value from the parameter table.
 *
 *  \param[in] V2Command  Issued V2 Protocol command byte from the host
 */
static void V2Protocol_GetSetParam(const uint8_t V2Command)
{
	uint8_t ParamID = Endpoint_Read_8();
	uint8_t ParamValue;

	if (V2Command == CMD_SET_PARAMETER)
	  ParamValue = Endpoint_Read_8();

	Endpoint_ClearOUT();
	Endpoint_SelectEndpoint(AVRISP_DATA_IN_EPADDR);
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);

	Endpoint_Write_8(V2Command);

	uint8_t ParamPrivs = V2Params_GetParameterPrivileges(ParamID);

	if ((V2Command == CMD_SET_PARAMETER) && (ParamPrivs & PARAM_PRIV_WRITE))
	{
		Endpoint_Write_8(STATUS_CMD_OK);
		V2Params_SetParameterValue(ParamID, ParamValue);
	}
	else if ((V2Command == CMD_GET_PARAMETER) && (ParamPrivs & PARAM_PRIV_READ))
	{
		Endpoint_Write_8(STATUS_CMD_OK);
		Endpoint_Write_8(V2Params_GetParameterValue(ParamID));
	}
	else
	{
		Endpoint_Write_8(STATUS_CMD_FAILED);
	}

	Endpoint_ClearIN();
}

/** Handler for the CMD_LOAD_ADDRESS command, loading the given device address into a
 *  global storage variable for later use, and issuing LOAD EXTENDED ADDRESS commands
 *  to the attached device as required.
 */
static void V2Protocol_LoadAddress(void)
{
	Endpoint_Read_Stream_BE(&CurrentAddress, sizeof(CurrentAddress), NULL);

	Endpoint_ClearOUT();
	Endpoint_SelectEndpoint(AVRISP_DATA_IN_EPADDR);
	Endpoint_SetEndpointDirection(ENDPOINT_DIR_IN);

	if (CurrentAddress & (1UL << 31))
	  MustLoadExtendedAddress = true;

	Endpoint_Write_8(CMD_LOAD_ADDRESS);
	Endpoint_Write_8(STATUS_CMD_OK);
	Endpoint_ClearIN();
}

