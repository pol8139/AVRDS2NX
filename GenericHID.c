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

// #include <stdio.h>
#include "GenericHID.h"
#include "mega32u4_dualshock2/mega32u4_uart.h"

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
	SPI_SerialNumber,
	SPI_BodyRGBColor,
	SPI_6AxisHorizontalOffsets,
	SPI_FactoryStickDeviceParameters2,
	SPI_FactoryConfigurationCalibration2,
	SPI_UserAnalogSticksCalibration,
	SPI_User6AxisMotionSensorCalibration,
	PadData
} CommandDescription_t;

CommandDescription_t ProControllerCommand;
uint8_t ResponseUART;
uint8_t ResponseSPI[2];
bool JoystickReport = false;
// uint8_t MACAddr[] = {0x00, 0x00, 0x5e, 0x00, 0x53, 0x5e};
uint8_t InitialInput[] = {0x81, 0x00, 0x80, 0x00, 0xf8, 0xd7, 0x7a, 0x22, 0xc8, 0x7b, 0x00};
uint16_t GlobalCounter = 0;
#define BUFFERSIZE GENERIC_REPORT_SIZE * 2
char buffer[BUFFERSIZE + 1]; // \0

void Response(uint8_t* DataArray, uint8_t Code, uint8_t Cmd, uint8_t* Data, int SizeofData)
{
	DataArray[0] = Code;
	DataArray[1] = Cmd;
	for(int i = 0; i < SizeofData; i++) {
		if(i + 2 >= GENERIC_REPORT_SIZE) {
			break;
		}
		DataArray[i + 2] = Data[i];
	}
	// for(int i = 0; i < GENERIC_REPORT_SIZE - 2 - SizeofData; i++) {
	// 	DataArray[i + 2 + SizeofData] = 0;
	// }
}

void UART_Response(uint8_t* DataArray, uint8_t Code, uint8_t Subcmd, uint8_t* Data, int SizeofData)
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
	Response(DataArray, 0x21, GlobalCounter, Buff, sizeof(Buff));
}

void SPI_Response(uint8_t* DataArray, uint8_t* Addr, int SizeofAddr, uint8_t* Data, int SizeofData)
{
	uint8_t Buff[GENERIC_REPORT_SIZE] = {};
	for(int i = 0; i < SizeofAddr; i++) {
		Buff[i] = Addr[i];
	}
	Buff[SizeofAddr] = 0x00;
	Buff[SizeofAddr + 1] = 0x00;
	// Buff[sizeof(Addr) + 2] = SizeofData;
	Buff[SizeofAddr + 2] = SizeofData & 0xFF;
	for(int i = 0; i < SizeofData; i++) {
		Buff[i + SizeofAddr + 3] = Data[i];
	}
	UART_Response(DataArray, 0x90, 0x10, Buff, sizeof(Buff));
}

char SingleHex2Char(uint8_t in)
{
	if(in < 10) {
		return in + 48;
	} else {
		return in + 55;
	}
}

void ByteHexDump(char* out, uint8_t in)
{
	out[0] = SingleHex2Char(in >> 4);
	out[1] = SingleHex2Char(in & 0x0F);
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	transmitUartStringCRLF("Init");

	for (;;)
	{
		HID_Task();
		USB_USBTask();
		// GlobalCounter += 8;
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	initUart();
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
	transmitUartString("EVENT_USB_Device_ConfigurationChanged");
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(GENERIC_IN_EPADDR, EP_TYPE_INTERRUPT, GENERIC_EPSIZE, 1);
	if(ConfigSuccess) {
		transmitUartString(" Success");
	} else {
		transmitUartString(" Failure");
	}
	ConfigSuccess &= Endpoint_ConfigureEndpoint(GENERIC_OUT_EPADDR, EP_TYPE_INTERRUPT, GENERIC_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
	if(ConfigSuccess) {
		transmitUartStringCRLF(" Success");
	} else {
		transmitUartStringCRLF(" Failure");
	}
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	transmitUartString("EVENT_USB_Device_ControlRequest ");
	ByteHexDump(buffer, USB_ControlRequest.bRequest);
	ByteHexDump(buffer + 2, USB_ControlRequest.bmRequestType);
	buffer[4] = '\0';
	transmitUartStringCRLF(buffer);
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
			ProControllerCommand = NothingToDo;
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
			ResponseSPI[0] = DataArray[11];
			ResponseSPI[1] = DataArray[12];
			if(DataArray[11] == 0x00 && DataArray[12] == 0x60) {
				ProControllerCommand = SPI_SerialNumber;
			} else if(DataArray[11] == 0x50 && DataArray[12] == 0x60) {
				ProControllerCommand = SPI_BodyRGBColor;
			} else if(DataArray[11] == 0x80 && DataArray[12] == 0x60) {
				ProControllerCommand = SPI_6AxisHorizontalOffsets;
			} else if(DataArray[11] == 0x98 && DataArray[12] == 0x60) {
				ProControllerCommand = SPI_FactoryStickDeviceParameters2;
			} else if(DataArray[11] == 0x3d && DataArray[12] == 0x60) {
				ProControllerCommand = SPI_FactoryConfigurationCalibration2;
			} else if(DataArray[11] == 0x10 && DataArray[12] == 0x80) {
				ProControllerCommand = SPI_UserAnalogSticksCalibration;
			} else if(DataArray[11] == 0x28 && DataArray[12] == 0x80) {
				ProControllerCommand = SPI_User6AxisMotionSensorCalibration;
			}
		}
	}

	if(DataArray[0] != 0x00) {
		transmitUartString(">>> ");
		for(int i = 0; i < GENERIC_REPORT_SIZE; i++) {
			// sprintf(buffer + i * 2, "%02x", DataArray[i]);
			ByteHexDump(buffer + i * 2, DataArray[i]);
		}
		transmitUartStringCRLF(buffer);
	}
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
	uint8_t sm[] = {0x00, 0x03};
	uint8_t mc[] = {0x00, 0x03, 0x1f, 0x86, 0x1d, 0xd6, 0x03, 0x04}; // '0003' + mac_addr
	// uint8_t hs[] = {0x81, 0x82};
	uint8_t bt[] = {0x03};
	uint8_t di[] = {0x03, 0x48, 0x03, 0x02, 0x04, 0x03, 0xd6, 0x1d, 0x86, 0x1f, 0x03, 0x01}; // '03480302' + mac_addr[::-1] + '0301'
	uint8_t ni[] = {0x01, 0x00, 0xff, 0x00, 0x03, 0x00, 0x05, 0x01};
	uint8_t sn[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	uint8_t bc[] = {0xbc, 0x11, 0x42, 0x75, 0xa9, 0x28, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	uint8_t ho[] = {0x50, 0xfd, 0x00, 0x00, 0xc6, 0x0f, 0x0f, 0x30, 0x61, 0x96, 0x30, 0xf3, 0xd4, 0x14, 0x54, 0x41, 0x15, 0x54, 0xc7, 0x79, 0x9c, 0x33, 0x36, 0x63};
	uint8_t d2[] = {0x0f, 0x30, 0x61, 0x96, 0x30, 0xf3, 0xd4, 0x14, 0x54, 0x41, 0x15, 0x54, 0xc7, 0x79, 0x9c, 0x33, 0x36, 0x63};
	uint8_t c2[] = {0xba, 0x15, 0x62, 0x11, 0xb8, 0x7f, 0x29, 0x06, 0x5b, 0xff, 0xe7, 0x7e, 0x0e, 0x36, 0x56, 0x9e, 0x85, 0x60, 0xff, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff};
	uint8_t ac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb2, 0xa1};
	uint8_t ms[] = {0xbe, 0xff, 0x3e, 0x00, 0xf0, 0x01, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0xfe, 0xff, 0xfe, 0xff, 0x08, 0x00, 0xe7, 0x3b, 0xe7, 0x3b, 0xe7, 0x3b};
	uint8_t Temp[GENERIC_REPORT_SIZE] = {};

	static uint8_t FirstTime = 0;
	if(FirstTime == 0){
		FirstTime = 1;
		ProControllerCommand = FirstMagicPacket;
	} else if(FirstTime == 1){
		FirstTime = 2;
		ProControllerCommand = SecondMagicPacket;
	}

	GlobalCounter += 8;

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
		UART_Response(DataArray, 0xa0, ResponseUART, ni, sizeof(ni));
		break;
	case SPI_SerialNumber:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), sn, sizeof(sn));
		break;
	case SPI_BodyRGBColor:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), bc, sizeof(bc));
		break;
	case SPI_6AxisHorizontalOffsets:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), ho, sizeof(ho));
		break;
	case SPI_FactoryStickDeviceParameters2:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), d2, sizeof(d2));
		break;
	case SPI_FactoryConfigurationCalibration2:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), c2, sizeof(c2));
		break;
	case SPI_UserAnalogSticksCalibration:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), ac, sizeof(ac));
		break;
	case SPI_User6AxisMotionSensorCalibration:
		SPI_Response(DataArray, ResponseSPI, sizeof(ResponseSPI), ms, sizeof(ms));
		break;
	case PadData:
	// default:
		if(JoystickReport) {
			memcpy(Temp, InitialInput, sizeof(InitialInput));
			if(GlobalCounter >> 7) {
				Temp[1] = (1 << 3);
			}
			Response(DataArray, 0x30, GlobalCounter, Temp, sizeof(InitialInput));
		}
	break;
	default:
		break;
	}
	// DataArray[GENERIC_REPORT_SIZE - 1] = 0x34;
	// DataArray[GENERIC_REPORT_SIZE - 2] = 0x12;

	transmitUartString("<<< ");
	if(DataArray[0] != 0x00) {
		for(int i = 0; i < GENERIC_REPORT_SIZE; i++) {
			// sprintf(buffer + i * 2, "%02x", DataArray[i]);
			ByteHexDump(buffer + i * 2, DataArray[i]);
		}
		transmitUartString(buffer);
	}
	transmitUartStringCRLF("");
}

void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	ProControllerCommand = PadData;

	Endpoint_SelectEndpoint(GENERIC_OUT_EPADDR);
	// if(GlobalCounter == 0) transmitUart('.');
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
			Endpoint_Read_Stream_LE(&GenericData, sizeof(GenericData), NULL);

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

		// if(ProControllerCommand != NothingToDo) {
			/* Write Generic Report Data */
			Endpoint_Write_Stream_LE(&GenericData, sizeof(GenericData), NULL);

			/* Finalize the stream transfer to send the last packet */
			Endpoint_ClearIN();
		// }
	}
}

