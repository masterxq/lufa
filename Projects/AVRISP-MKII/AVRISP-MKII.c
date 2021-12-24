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
 *  Main source file for the AVRISP project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "AVRISP-MKII.h"

#if (BOARD != BOARD_NONE)
  /* Some board hardware definitions (e.g. the Arduino Micro) have their LEDs defined on the same pins
     as the ISP, PDI or TPI interfaces (see the accompanying project documentation). If a board other
     than NONE is selected (to enable the LED driver with the programmer) you should double-check that
     no conflicts will occur. If there is a conflict, turn off the LEDs (set BOARD to NONE in the makefile)
     or define a custom board driver (see the LUFA manual) with alternative LED mappings.
  */
  #warning Board specific drivers have been selected; make sure the board LED driver does not conflict with the programmer ISP/PDI/TPI interfaces.
#endif


/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another. This is for the first CDC interface,
 *  which sends strings to the host for each joystick movement.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial1_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC1_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC1_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC1_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC1_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another. This is for the second CDC interface,
 *  which echos back all received data from the host.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial2_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC2_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC2_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC2_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC2_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},

			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	V2Protocol_Init();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		#if (BOARD == BOARD_USBTINYMKII)
		/* On the USBTINY-MKII target, there is a secondary LED which indicates the current selected power
		   mode - either VBUS, or sourced from the VTARGET pin of the programming connectors */
		LEDs_ChangeLEDs(LEDMASK_VBUSPOWER, (PIND & (1 << 0)) ? 0 : LEDMASK_VBUSPOWER);
		#endif
		
		int16_t ReceivedByte2 = CDC_Device_ReceiveByte(&VirtualSerial1_CDC_Interface);

		/* Echo all received data on the second CDC interface */
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial2_CDC_Interface);
		if (!(ReceivedByte < 0))
		  CDC_Device_SendByte(&VirtualSerial1_CDC_Interface, (uint8_t)ReceivedByte);

		if(!(ReceivedByte2 < 0))
			CDC_Device_SendByte(&VirtualSerial2_CDC_Interface, (uint8_t)ReceivedByte2);

		AVRISP_Task();
		CDC_Device_USBTask(&VirtualSerial1_CDC_Interface);
		CDC_Device_USBTask(&VirtualSerial2_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	// Start the 32MHz internal RC oscillator and start the DFLL to increase it to F_USB using the USB SOF as a reference
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();
#endif

	/* Hardware Initialization */
	LEDs_Init();
	#if defined(RESET_TOGGLES_LIBUSB_COMPAT)
	UpdateCurrentCompatibilityMode();
	#endif
	
	/* USB Stack Initialization */
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup AVRISP Data OUT endpoint */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(AVRISP_DATA_OUT_EPADDR, EP_TYPE_BULK, AVRISP_DATA_EPSIZE, 1);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial1_CDC_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial2_CDC_Interface);

	/* Setup AVRISP Data IN endpoint if it is using a physically different endpoint */
#if (ARCH != ARCH_XMEGA)
	if ((AVRISP_DATA_IN_EPADDR & ENDPOINT_EPNUM_MASK) != (AVRISP_DATA_OUT_EPADDR & ENDPOINT_EPNUM_MASK))
#endif
	  ConfigSuccess &= Endpoint_ConfigureEndpoint(AVRISP_DATA_IN_EPADDR, EP_TYPE_BULK, AVRISP_DATA_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial1_CDC_Interface);
	CDC_Device_ProcessControlRequest(&VirtualSerial2_CDC_Interface);
// 	CDC_Device_SendString(&VirtualSerial1_CDC_Interface, "Control");
}

/** Processes incoming V2 Protocol commands from the host, returning a response when required. */
void AVRISP_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	V2Params_UpdateParamValues();

	Endpoint_SelectEndpoint(AVRISP_DATA_OUT_EPADDR);

	/* Check to see if a V2 Protocol command has been received */
	if (Endpoint_IsOUTReceived())
	{
		LEDs_SetAllLEDs(LEDMASK_BUSY);

		/* Pass off processing of the V2 Protocol command to the V2 Protocol handler */
		V2Protocol_ProcessCommand();

		LEDs_SetAllLEDs(LEDMASK_USB_READY);
	}
}


/** CDC class driver callback function the processing of changes to the virtual
 *  control lines sent from the host..
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	/* You can get changes to the virtual CDC lines in this callback; a common
	   use-case is to use the Data Terminal Ready (DTR) flag to enable and
	   disable CDC communications in your application when set to avoid the
	   application blocking while waiting for a host to become ready and read
	   in the pending data from the USB endpoints.
	*/
	bool HostReady = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;
	uint32_t bps = CDCInterfaceInfo->State.LineEncoding.BaudRateBPS;
	(void)HostReady;

// 	char buffer[128];
// 	sprintf(buffer, "BPS: %lu\r\n", bps);
	if (CDCInterfaceInfo == &VirtualSerial1_CDC_Interface)
	{
// 		CDC_Device_SendString(&VirtualSerial2_CDC_Interface, buffer);
		// CDC interface 1's host is ready to send/receive data if HostReady is true
	}
	else
	{
// 		CDC_Device_SendString(&VirtualSerial1_CDC_Interface, buffer);
		// CDC interface 2's host is ready to send/receive data if HostReady is true
	}
}


void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool HostReady = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;
// 	uint32_t bps = CDCInterfaceInfo->State.LineEncoding.BaudRateBPS;
	(void)HostReady;

// 	char buffer[128];
	if (CDCInterfaceInfo == &VirtualSerial1_CDC_Interface)
	{
// 		CDC_Device_SendString(&VirtualSerial2_CDC_Interface, buffer);
		// CDC interface 1's host is ready to send/receive data if HostReady is true
	}
	else
	{
// 		CDC_Device_SendString(&VirtualSerial1_CDC_Interface, buffer);
		// CDC interface 2's host is ready to send/receive data if HostReady is true
	}
}



/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 *
 *  \param[in]  wValue                 Descriptor type and index to retrieve
 *  \param[in]  wIndex                 Sub-index to retrieve (such as a localized string language)
 *  \param[out] DescriptorAddress      Address of the retrieved descriptor
 *
 *  \return Length of the retrieved descriptor in bytes, or NO_DESCRIPTOR if the descriptor was not found
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
	return AVRISP_GetDescriptor(wValue, wIndex, DescriptorAddress);
}

