/*
             LUFA Library
     Copyright (C) Dean Camera, 2019.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2019  Dean Camera (dean [at] fourwalledcubicle [dot] com)

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
 *  Main source file for the GenericHID demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "GenericHID.h"

// #define UARTDEBUG
// #define UARTDEBUG_RUMBLE

typedef enum
{
	NothingToDo,
	FirstMagicPacket,
	SecondMagicPacket,
	RequestMacAddress,
	Handshake,
	EnableUSBHIDJoystickReport,
	UART_BluetoothManualPairing,
	UART_RequestDeviceInfo,
	UART_Others,
	UART_TriggerButtonsElapsedTime,
	UART_SetNFCIRMCUConfiguration,
	SPI_ReadData,
	PadData,
	RumbleOnly
} CommandDescription_t;

CommandDescription_t ProControllerCommand;
uint8_t ResponseUART;
uint8_t ResponseSPI[5];
bool JoystickReport = false;
// uint8_t MACAddr[] = {0x00, 0x00, 0x5e, 0x00, 0x53, 0x5e};
const uint8_t InitialInput[] = {0x81, 0x00, 0x80, 0x00, 0x00, 0x08, 0x80, 0x00, 0x08, 0x80, 0x00};
#define BUFFERSIZE GENERIC_REPORT_SIZE * 2
char buffer[BUFFERSIZE + 1]; // \0

PROGMEM const uint8_t SPIROM0x6000[] = 
{
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x03, 0xA0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF,
	0x37, 0xFF, 0x42, 0xFF, 0x69, 0xFF, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x2E, 0x00, 0xC1, 0xFF,
	0xDF, 0xFF, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x07, 0x7F,
	0x00, 0x08, 0x80, 0x00, 0x08, 0x80, 0xF0, 0x07, 0x7F, 0x00, 0x08, 0x80, 0x00, 0x08, 0x80, 0xFF,
	0x42, 0x87, 0xF5, 0x2B, 0x2B, 0x2B, 0xF3, 0xC2, 0x02, 0x32, 0x6D, 0xB3, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x5E, 0x01, 0x00, 0x00, 0xF1, 0x0F, 0x19, 0x00, 0x49, 0xBC, 0x40, 0xE1, 0xDA, 0xA2, 0x2D, 0xDA,
	0xA2, 0x2D, 0xA6, 0x6A, 0xAA, 0x90, 0x04, 0x49, 0x19, 0x00, 0x49, 0xBC, 0x40, 0xE1, 0xDA, 0xA2,
	0x2D, 0xDA, 0xA2, 0x2D, 0xA6, 0x6A, 0xAA, 0x90, 0x04, 0x49, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

PROGMEM const uint8_t SPIROM0x8000[] = 
{
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xB2, 0xA1, 0xCF, 0x00, 0xC8, 0xFF, 0x69, 0xFF, 0x00, 0x40,
	0x00, 0x40, 0x00, 0x40, 0x3D, 0x00, 0xBD, 0xFF, 0xDA, 0xFF, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34
};

void Response(uint8_t* DataArray, uint8_t Code, uint8_t Cmd, const uint8_t* Data, int SizeofData)
{
	DataArray[0] = Code;
	DataArray[1] = Cmd;
	for(int i = 0; i < SizeofData; i++) {
		if(i + 2 >= GENERIC_REPORT_SIZE) {
			break;
		}
		DataArray[i + 2] = Data[i];
	}
}

void UART_Response(uint8_t* DataArray, uint8_t Code, uint8_t Subcmd, const uint8_t* Data, int SizeofData)
{
	// uint8_t Buff[SizeofData + sizeof(InitialInput) + 1] = {};
	uint8_t Buff[GENERIC_REPORT_SIZE] = {};
	for(int i = 0; i < sizeof(InitialInput); i++) {
		Buff[i] = InitialInput[i];
	}
	Buff[sizeof(InitialInput)] = Code;
	Buff[sizeof(InitialInput) + 1] = Subcmd;
	for(int i = 0; i < SizeofData; i++) {
		if(i + sizeof(InitialInput) + 2 >= GENERIC_REPORT_SIZE) {
			break;
		}
		Buff[i + sizeof(InitialInput) + 2] = Data[i];
	}
	Response(DataArray, 0x21, GetCounter(), Buff, sizeof(Buff));
}

void SPI_Response(uint8_t* DataArray, uint8_t* Addr)
{
	uint8_t Buff[GENERIC_REPORT_SIZE] = {};
	const uint8_t* ROMAddr;
	memcpy(Buff, Addr, sizeof(ResponseSPI));
	if(Addr[1] == 0x60) {
		ROMAddr = SPIROM0x6000;
	} else/* if(Addr[1] == 0x80)*/ {
		ROMAddr = SPIROM0x8000;
	}
	memcpy_P(&Buff[sizeof(ResponseSPI)], &ROMAddr[Addr[0]], Addr[4]);
	UART_Response(DataArray, 0x90, 0x10, Buff, 2 + sizeof(ResponseSPI) + Addr[4]);
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	#if defined UARTDEBUG || defined UARTDEBUG_RUMBLE
		transmitUartStringCRLF("Init");
	#endif

	for (;;)
	{
		HID_Task();
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
	#if defined UARTDEBUG || defined UARTDEBUG_RUMBLE
		initUart();
	#endif
	InitHardware();
	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	USB_Init();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management task.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the generic HID device endpoints.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(GENERIC_IN_EPADDR, EP_TYPE_INTERRUPT, GENERIC_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(GENERIC_OUT_EPADDR, EP_TYPE_INTERRUPT, GENERIC_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				uint8_t GenericData[GENERIC_REPORT_SIZE] = {};
				CreateGenericHIDReport(GenericData);

				Endpoint_ClearSETUP();

				/* Write the report data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&GenericData, sizeof(GenericData));
				Endpoint_ClearOUT();
			}

			break;
		case HID_REQ_SetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				uint8_t GenericData[GENERIC_REPORT_SIZE] = {};

				Endpoint_ClearSETUP();

				/* Read the report data from the control endpoint */
				Endpoint_Read_Control_Stream_LE(&GenericData, sizeof(GenericData));
				Endpoint_ClearIN();

				ProcessGenericHIDReport(GenericData);
			}

			break;
	}
}

/** Function to process the last received report from the host.
 *
 *  \param[in] DataArray  Pointer to a buffer where the last received report has been stored
 */
void ProcessGenericHIDReport(uint8_t* DataArray)
{
	/*
		This is where you need to process reports sent from the host to the device. This
		function is called each time the host has sent a new report. DataArray is an array
		holding the report sent from the host.
	*/
	ProControllerCommand = NothingToDo;
	if(DataArray[0] == 0x80) {
		if(DataArray[1] == 0x01) {
			ProControllerCommand = RequestMacAddress;
		} else if(DataArray[1] == 0x02) {
			ProControllerCommand = Handshake;
		} else if(DataArray[1] == 0x04) {
			ProControllerCommand = EnableUSBHIDJoystickReport;
			JoystickReport = true;
		}
	} else if(DataArray[0] == 0x01) {
		ResponseUART = DataArray[10];
		if(DataArray[10] == 0x00) {
			ProControllerCommand = PadData;
			NormalizeRumble(DataArray + 2);
			#if defined UARTDEBUG_RUMBLE
				PrintRumble();
			#endif
		} else if(DataArray[10] == 0x01) {
			ProControllerCommand = UART_BluetoothManualPairing;
		} else if(DataArray[10] == 0x02) {
			ProControllerCommand = UART_RequestDeviceInfo;
		} else if(DataArray[10] == 0x03 || DataArray[10] == 0x08 || DataArray[10] == 0x30 || \
		          DataArray[10] == 0x38 || DataArray[10] == 0x40 || DataArray[10] == 0x48) {
			ProControllerCommand = UART_Others;
		} else if(DataArray[10] == 0x04) {
			ProControllerCommand = UART_TriggerButtonsElapsedTime;
		} else if(DataArray[10] == 0x21) {
			ProControllerCommand = UART_SetNFCIRMCUConfiguration;
		} else if(DataArray[10] == 0x10) {
			memcpy(ResponseSPI, DataArray + 11, sizeof(ResponseSPI));
			ProControllerCommand = SPI_ReadData;
		}
	} else if(DataArray[0] == 0x10) {
		ProControllerCommand = RumbleOnly;
		NormalizeRumble(DataArray + 2);
		#if defined UARTDEBUG_RUMBLE
			PrintRumble();
		#endif
	}
	#ifdef UARTDEBUG
		if(ProControllerCommand == NothingToDo) {
			transmitUartString("!!> ");
		} else {
			transmitUartString(">>> ");
		}
			if(DataArray[0] != 0x00) {
				for(int i = 0; i < GENERIC_REPORT_SIZE; i++) {
					ByteHexDump(buffer + i * 2, DataArray[i]);
				}
				transmitUartString(buffer);
			}
			transmitUartStringCRLF("");
	#endif
}

/** Function to create the next report to send back to the host at the next reporting interval.
 *
 *  \param[out] DataArray  Pointer to a buffer where the next report data should be stored
 */
void CreateGenericHIDReport(uint8_t* DataArray)
{
	/*
		This is where you need to create reports to be sent to the host from the device. This
		function is called each time the host is ready to accept a new report. DataArray is
		an array to hold the report to the host.
	*/
	const uint8_t sm[] = {0x00, 0x03};
	const uint8_t mc[] = {0x00, 0x03, 0x1f, 0x86, 0x1d, 0xd6, 0x03, 0x04}; // '0003' + mac_addr
	const uint8_t bt[] = {0x03};
	const uint8_t di[] = {0x03, 0x48, 0x03, 0x02, 0x04, 0x03, 0xd6, 0x1d, 0x86, 0x1f, 0x03, 0x01}; // '03480302' + mac_addr[::-1] + '0301'
	const uint8_t ni[] = {0x01, 0x00, 0xff, 0x00, 0x03, 0x00, 0x05, 0x01};
	uint8_t Temp[GENERIC_REPORT_SIZE] = {};

	static uint8_t FirstTime = 0;
	if(FirstTime == 0){
		FirstTime = 1;
		ProControllerCommand = FirstMagicPacket;
	} else if(FirstTime == 1){
		FirstTime = 2;
		ProControllerCommand = SecondMagicPacket;
	}

	switch(ProControllerCommand)
	{
	case FirstMagicPacket:
		Response(DataArray, 0x81, 0x03, Temp, 0);
		break;
	case SecondMagicPacket:
		Response(DataArray, 0x81, 0x01, sm, sizeof(sm));
		break;
	case RequestMacAddress:
		// memcpy(DataArray, mac, sizeof(mac));
		Response(DataArray, 0x81, 0x01, mc, sizeof(mc));
		break;
	case Handshake:
		// memcpy(DataArray, hs, sizeof(hs));
		Response(DataArray, 0x81, 0x02, Temp, 0);
		break;
	// case EnableUSBHIDJoystickReport:
	// 	JoystickReport = true;
	// 	break;
	case UART_BluetoothManualPairing:
		UART_Response(DataArray, 0x81, ResponseUART, bt, sizeof(bt));
		break;
	case UART_RequestDeviceInfo:
		UART_Response(DataArray, 0x82, ResponseUART, di, sizeof(di));
		break;
	case UART_Others:
		UART_Response(DataArray, 0x80, ResponseUART, Temp, 0);
		break;
	case UART_TriggerButtonsElapsedTime:
		UART_Response(DataArray, 0x83, ResponseUART, Temp, 0);
		break;
	case UART_SetNFCIRMCUConfiguration:
		UART_Response(DataArray, 0xA0, ResponseUART, ni, sizeof(ni));
		break;
	case SPI_ReadData:
		SPI_Response(DataArray, ResponseSPI);
		break;
	case PadData:
	// default:
		if(JoystickReport) {
			memcpy(Temp, InitialInput, sizeof(InitialInput));
			GetControllerInputData(Temp);
			Response(DataArray, 0x30, GetCounter(), Temp, sizeof(InitialInput));
		}
		break;
	case RumbleOnly:
		break;
	default:
		break;
	}
	#ifdef UARTDEBUG
		transmitUartString("<<< ");
		if(DataArray[0] != 0x00) {
			for(int i = 0; i < GENERIC_REPORT_SIZE; i++) {
				ByteHexDump(buffer + i * 2, DataArray[i]);
			}
			transmitUartString(buffer);
		}
		transmitUartStringCRLF("");
	#endif
}

void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	ProControllerCommand = PadData;

	Endpoint_SelectEndpoint(GENERIC_OUT_EPADDR);
	/* Check to see if a packet has been sent from the host */
	if (Endpoint_IsOUTReceived())
	{
		// transmitUartStringCRLF("O");
		/* Check to see if the packet contains data */
		if (Endpoint_IsReadWriteAllowed())
		{
			/* Create a temporary buffer to hold the read in report from the host */
			uint8_t GenericData[GENERIC_REPORT_SIZE] = {};
			/* Read Generic Report Data */
			#ifdef UARTDEBUG
				ByteHexDump(buffer, 
			#endif
				Endpoint_Read_Stream_LE(&GenericData, sizeof(GenericData), NULL)
			#ifdef UARTDEBUG
				)
			#endif
				;
			#ifdef UARTDEBUG
				buffer[2] = '\0';
				transmitUartStringCRLF(buffer);
			#endif
			/* Process Generic Report Data */
			ProcessGenericHIDReport(GenericData);
		}

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearOUT();
	}

	Endpoint_SelectEndpoint(GENERIC_IN_EPADDR);

	/* Check to see if the host is ready to accept another packet */
	if (Endpoint_IsINReady())
	{
		// transmitUartStringCRLF("I");
		/* Create a temporary buffer to hold the report to send to the host */
		uint8_t GenericData[GENERIC_REPORT_SIZE] = {};

		/* Create Generic Report Data */
		CreateGenericHIDReport(GenericData);

		/* Write Generic Report Data */
		Endpoint_Write_Stream_LE(&GenericData, sizeof(GenericData), NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();
	}
}

