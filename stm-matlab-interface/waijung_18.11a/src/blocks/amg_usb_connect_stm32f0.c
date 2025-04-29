#include "mex.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Usage:
 * 1. Check connection:
 *    => [sta, msg] = amg_usb_connect_stm32f0('connect')
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 2. Write program:
 *    => [sta, msg] = amg_usb_connect_stm32f0('writeflash', <membase>, <memsize>, <filename>);
 *    =>Example, [sta, msg] = amg_usb_connect_stm32f0('writeflash', hex2dec('8000000'), hex2dec('20000'), 'digital_io_demo.bin');
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 3. Full Erase:
 *    => [sta, msg] = amg_usb_connect_stm32f0('fullerase');
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 4. Run:
 *    => [sta, msg] = amg_usb_connect_stm32f0('run');
 *    Where, sta = 1 Success, 0 - Error.
 */

#include "..\..\utils\devices\aMG_USBConnect\ftd2xx.h"
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\i386\\ftd2xx.lib")
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\amd64\\ftd2xx.lib")


extern DWORD WINAPI GetTickCount(void);

// ========================================================================
// COM interface
// ========================================================================
typedef struct {
	HANDLE hHandle;
	HANDLE hEvtOverlapped;
	LONG lLastError;
	DWORD dwEventMask;
	//
	DWORD dwInQueue;
	DWORD dwOutQueue;
} SERIAL_PORT_STRUCT;

// Prototype
BOOL Serial_Purge(SERIAL_PORT_STRUCT *serial);

BOOL Serial_SetMask (SERIAL_PORT_STRUCT *serial, DWORD dwEventMask)
{
	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Check if the device is open
	if (serial->hHandle == NULL)
	{
		// Set the internal error code
		serial->lLastError = ERROR_INVALID_HANDLE;

		// Issue an error and quit
		return FALSE;
	}

	// Set the new mask. Note that this will generate an EEventNone
	// if there is an asynchronous WaitCommEvent pending.
	if (!SetCommMask(serial->hHandle, dwEventMask))
	{
		// Obtain the error code
		serial->lLastError = GetLastError();

		// Error
		return FALSE;
	}

	// Save event mask and return successful
	serial->dwEventMask = dwEventMask;
	return TRUE;
}

BOOL Serial_SetupDataFrame (SERIAL_PORT_STRUCT *serial, DWORD BaudRate, BYTE DataBits, BYTE Parity, BYTE StopBits)
{
	DCB dcb;

	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Check if the device is open
	if (serial->hHandle == NULL)
	{
		// Set the internal error code
		serial->lLastError = ERROR_INVALID_HANDLE;

		// Issue an error and quit
		return FALSE;
	}

	// Obtain the DCB structure for the device	
	if (!GetCommState(serial->hHandle, &dcb))
	{
		// Obtain the error code
		serial->lLastError = GetLastError();

		// Issue an error and quit
		return FALSE;
	}

	// Set the new data
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = BaudRate;
	dcb.ByteSize = DataBits;
	dcb.Parity   = Parity;
	dcb.StopBits = StopBits;

	// Determine if parity is used
	dcb.fParity  = (Parity != NOPARITY);

	// Flow control: None
	dcb.fOutxCtsFlow = FALSE;					// Disable CTS monitoring
	dcb.fOutxDsrFlow = FALSE;					// Disable DSR monitoring
	dcb.fDtrControl = DTR_CONTROL_DISABLE;		// Disable DTR monitoring
	dcb.fOutX = FALSE;							// Disable XON/XOFF for transmission
	dcb.fInX = FALSE;							// Disable XON/XOFF for receiving
	dcb.fRtsControl = RTS_CONTROL_DISABLE;		// Disable RTS (Ready To Send)

	// Set the new DCB structure
	if (!SetCommState(serial->hHandle, &dcb))
	{
		// Obtain the error code
		serial->lLastError = GetLastError();

		// Issue an error and quit
		return FALSE;
	}

	// Return successful
	return TRUE;
}

BOOL Serial_SetupReadTimeouts (SERIAL_PORT_STRUCT *serial)
{
	COMMTIMEOUTS cto;

	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Check if the device is open
	if (serial->hHandle == NULL)
	{
		// Set the internal error code
		serial->lLastError = ERROR_INVALID_HANDLE;

		// Issue an error and quit
		return FALSE;
	}

	// Determine the time-outs	
	if (!GetCommTimeouts(serial->hHandle, &cto))
	{
		// Obtain the error code
		serial->lLastError = GetLastError();

		// Issue an error and quit
		return FALSE;
	}

	// Set the new timeouts
	// Always non-blocking
	cto.ReadIntervalTimeout = MAXDWORD; // Set to 0 for blocking
	cto.ReadTotalTimeoutConstant = 0;
	cto.ReadTotalTimeoutMultiplier = 0;

	// Set the new DCB structure
	if (!SetCommTimeouts(serial->hHandle, &cto))
	{
		// Obtain the error code
		serial->lLastError = GetLastError();

		// Issue an error and quit
		return FALSE;
	}

	// Return successful
	return TRUE;
}

BOOL Serial_Close (SERIAL_PORT_STRUCT *serial)
{
	// If the device is already closed,
	// then we don't need to do anything.
	if (serial->hHandle == NULL)
		return TRUE;

	// Free event handle
	if (serial->hEvtOverlapped)
	{
		CloseHandle(serial->hEvtOverlapped);
		serial->hEvtOverlapped = NULL;
	}

	// Close COM port
	CloseHandle(serial->hHandle);
	serial->hHandle = NULL;

	// Return successful
	return TRUE;
}

BOOL Serial_Open (SERIAL_PORT_STRUCT *serial, const char *port, BOOL fOverlapped, DWORD BaudRate, BYTE DataBits, BYTE Parity, BYTE StopBits)
{
	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Check if the port isn't already opened
	if (serial->hHandle) {
		serial->lLastError = ERROR_ALREADY_INITIALIZED;
		return FALSE;
	}

	// Open the device
	serial->hHandle = CreateFile(port,
						   GENERIC_READ|GENERIC_WRITE,
						   0,
						   NULL,
						   OPEN_EXISTING,
						   fOverlapped?FILE_FLAG_OVERLAPPED:0,
						   0);
	if (serial->hHandle == INVALID_HANDLE_VALUE) {
		serial->lLastError = GetLastError();
		return FALSE;
	}

	// Create the event handle for internal overlapped operations (manual reset)
	if (fOverlapped)
	{
		serial->hEvtOverlapped = CreateEvent(0,TRUE,FALSE,0);
		if (serial->hEvtOverlapped == NULL)
		{
			// Obtain the error information
			serial->lLastError = GetLastError();

			// Close the port
			CloseHandle(serial->hHandle);
			serial->hHandle = NULL;

			// Return the error
			return FALSE;
		}
	}

	// Setup the COM-port
	if (serial->dwInQueue || serial->dwOutQueue)
	{
		// Make sure the queue-sizes are reasonable sized. Win9X systems crash
		// if the input queue-size is zero. Both queues need to be at least
		// 16 bytes large.
		//_ASSERTE(dwInQueue >= 16);
		//_ASSERTE(dwOutQueue >= 16);

		if (!SetupComm(serial->hHandle, serial->dwInQueue, serial->dwOutQueue))
		{
			// Display a warning
			serial->lLastError = GetLastError();

			// Close the port
			Serial_Close(serial);

			// Save last error from SetupComm
			return FALSE;
		}
	}

	// Setup the default communication mask
	Serial_SetMask(serial, EV_RXCHAR);

	// Non-blocking reads is default
	Serial_SetupReadTimeouts (serial);

	// Setup the device for default settings
	{
/*
		COMMCONFIG commConfig = {0};
		DWORD dwSize = sizeof(commConfig);
		commConfig.dwSize = dwSize;
		if (!GetDefaultCommConfig(port, &commConfig, &dwSize)) {
			serial->lLastError = GetLastError();
			// Close the port
			Serial_Close(serial);
			return FALSE;
		}
		if (!SetCommConfig(serial->hHandle, &commConfig, dwSize)) {
			serial->lLastError = GetLastError();
			// Close the port
			Serial_Close(serial);
			return FALSE;
		}
*/
	}

	// Setup data frame
	if (!Serial_SetupDataFrame (serial, BaudRate, DataBits, Parity, StopBits)) {
		// Close the port
		Serial_Close(serial);
		return FALSE;
	}

	// Purge
	//Serial_Purge(serial);

	// Return
	return TRUE;
}

BOOL Serial_Purge(SERIAL_PORT_STRUCT *serial)
{
	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Check if the device is open
	if (serial->hHandle == NULL)
	{
		// Set the internal error code
		serial->lLastError = ERROR_INVALID_HANDLE;

		// Issue an error and quit
		return FALSE;
	}

	if (!PurgeComm(serial->hHandle, PURGE_TXCLEAR | PURGE_RXCLEAR))
	{
		// Set the internal error code
		serial->lLastError = GetLastError();
		return FALSE;
	}
	
	// Return successfully
	return TRUE;
}

BOOL Serial_Write (SERIAL_PORT_STRUCT *serial, const char* pData, DWORD dwLen, DWORD* pdwWritten, DWORD dwTimeout)
{
	OVERLAPPED Overlapped;
	DWORD dwWritten;

	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Use our own variable for read count	
	if (pdwWritten == 0) {
		pdwWritten = &dwWritten;
	}

	// Reset the number of bytes written
	*pdwWritten = 0;

	// Check if the device is open
	if (serial->hHandle == NULL) {
		// Set the internal error code
		serial->lLastError = ERROR_INVALID_HANDLE;

		return FALSE;
	}
	
	// Setup our own overlapped structure
	memset(&Overlapped, 0, sizeof(Overlapped));
	Overlapped.hEvent = serial->hEvtOverlapped;

	//
	//

	// Write the data
	if (!WriteFile(serial->hHandle, pData, dwLen, pdwWritten, &Overlapped))
	{
		// Set the internal error code
		long lLastError = GetLastError();

		// Overlapped operation in progress is not an actual error
		if (lLastError != ERROR_IO_PENDING)
		{
			// Save the error
			serial->lLastError = lLastError;

			// Issue an error and quit
			return FALSE;
		}

		// Wait for the overlapped operation to complete
		switch (WaitForSingleObject(Overlapped.hEvent,dwTimeout))
		{
		case WAIT_OBJECT_0:
			// The overlapped operation has completed
			if (!GetOverlappedResult(serial->hHandle,&Overlapped, pdwWritten, FALSE))
			{
				// Set the internal error code
				serial->lLastError = GetLastError();
				return FALSE;
			}
			break;

		case WAIT_TIMEOUT:
			// Cancel the I/O operation
			CancelIo(serial->hHandle);

			// The operation timed out. Set the internal error code and quit
			serial->lLastError = ERROR_TIMEOUT;
			return FALSE;

		default:
			// Set the internal error code
			serial->lLastError = GetLastError();
			
			// Issue an error and quit
			return FALSE;
		}
	}
	else
	{
		// The operation completed immediatly. Just to be sure
		// we'll set the overlapped structure's event handle.
		SetEvent(Overlapped.hEvent);
	}


	// Return successfully
	return TRUE;
}

BOOL Serial_Read (SERIAL_PORT_STRUCT *serial, void* pData, DWORD dwLen, DWORD* pdwRead, DWORD dwTimeout)
{
	OVERLAPPED Overlapped;
	DWORD dwRead;

	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Use our own variable for read count	
	if (pdwRead == 0) {
		pdwRead = &dwRead;
	}

	// Reset the number of bytes read
	*pdwRead = 0;

	// Check if the device is open
	if (serial->hHandle == NULL)
	{
		// Set the internal error code
		serial->lLastError = ERROR_INVALID_HANDLE;

		// Issue an error and quit
		return FALSE;
	}

	// Setup our own overlapped structure
	memset(&Overlapped, 0, sizeof(Overlapped));
	Overlapped.hEvent = serial->hEvtOverlapped;

	ResetEvent(Overlapped.hEvent);

	// Read the data
	if (!ReadFile(serial->hHandle, pData, dwLen, pdwRead, &Overlapped))
	{
		// Set the internal error code
		long lLastError = GetLastError();

		// Overlapped operation in progress is not an actual error
		if (lLastError != ERROR_IO_PENDING)
		{
			// Save the error
			serial->lLastError = lLastError;

			// Issue an error and quit
			return FALSE;
		}

		// Wait for the overlapped operation to complete
		switch (WaitForSingleObject(Overlapped.hEvent, dwTimeout))
		{
		case WAIT_OBJECT_0:
			// The overlapped operation has completed
			if (!GetOverlappedResult(serial->hHandle, &Overlapped, pdwRead, FALSE))
			{
				// Set the internal error code
				serial->lLastError = GetLastError();
				return FALSE;
			}
			break;

		case WAIT_TIMEOUT:
			// Cancel the I/O operation
			CancelIo(serial->hHandle);

			// The operation timed out. Set the internal error code and quit
			serial->lLastError = ERROR_TIMEOUT;
			return FALSE;

		default:
			// Set the internal error code
			serial->lLastError = GetLastError();

			// Issue an error and quit
			return FALSE;
		}		
	}
	else
	{
		// The operation completed immediatly. Just to be sure
		// we'll set the overlapped structure's event handle.
		SetEvent(Overlapped.hEvent);
	}

	// Return successfully
	return TRUE;
}

BOOL Serial_Write_Ex (SERIAL_PORT_STRUCT *serial, const char* pData, DWORD dwLen)
{
	DWORD dwWritten;
	
	// Flush buffer
	
	// Write
	if (!Serial_Write (serial, pData, dwLen, &dwWritten, 1000))
		return FALSE;
	
	return (dwWritten == dwLen);
}

BOOL Serial_Read_Ex (SERIAL_PORT_STRUCT *serial, char* pData, DWORD dwLen, DWORD* pdwRead, DWORD dwTimeout)
{
	BOOL sta = TRUE;
	int start_tick;
	DWORD read_count, reading;
	
	start_tick = GetTickCount();
	read_count = 0;
	while (sta && (read_count < dwLen) && ((GetTickCount() - start_tick) < (int)dwTimeout)) {
		sta = Serial_Read (serial, &pData[read_count], (dwLen - read_count), &reading, dwTimeout);
		if (sta)
			read_count += reading;
	}
	*pdwRead = read_count;
	return sta;	
}

// ========================================================================
// FTDI device
// ========================================================================
#define IAP_LOGGING_HDR "aMG USB Connect: "

BOOL ft231x_validate(FT_HANDLE ftHandle, const char *key_str)
{
	FT_STATUS res;
	char Manufacturer[64];
	char ManufacturerId[64];
	char Description[64];
	char SerialNumber[64];
	FT_EEPROM_HEADER ft_eeprom_header; 
	FT_EEPROM_X_SERIES ft_eeprom_x;

	/* Configuration */
	ft_eeprom_header.deviceType = FT_DEVICE_X_SERIES;
	ft_eeprom_x.common = ft_eeprom_header;
	ft_eeprom_x.common.deviceType = FT_DEVICE_X_SERIES;

	/* Read */
	res = FT_EEPROM_Read(ftHandle, &ft_eeprom_x, sizeof(ft_eeprom_x),
			&Manufacturer[0], &ManufacturerId[0], &Description[0], &SerialNumber[0]);
	/* Check if description is matched */
	if ((res == 0) && !strcmp(key_str, Description/*Manufacturer*/))
		return TRUE;
	else
		return FALSE;
}

BOOL ft231_getportname(FT_HANDLE ftHandle, char *buffer)
{
	LONG lComPortNumber;
	FT_STATUS ftStatus;
	
	ftStatus = FT_GetComPortNumber(ftHandle, &lComPortNumber);
	if ((ftStatus == FT_OK) && (lComPortNumber > 0)) {
		if (lComPortNumber > 9) {
			sprintf_s(buffer, 32, "\\\\.\\COM%d", (int)lComPortNumber);
			//sprintf_s(buffer, 32, "\\\\\\\\.\\\\COM%d", (int)lComPortNumber);
		}
		else {
			sprintf(buffer, "COM%d", lComPortNumber);
		}
		return TRUE;
	}
	
	return FALSE;
}

FT_STATUS ft231_getvalid_deviceindex (int *deviceIndex)
{
	int i;
	FT_STATUS ftStatus;
	FT_DEVICE_LIST_INFO_NODE *devInfo;
	DWORD numDevs;
	
	// Default index as invalid
	*deviceIndex = -1;
	
	// Create the device information list 
	ftStatus = FT_CreateDeviceInfoList(&numDevs);
	if (ftStatus != FT_OK)
		return ftStatus;
	
	// Get device information list
	if (numDevs == 0) {
		*deviceIndex = -1; // Invalid
		return FT_OK;
	}
	
	// List the device list information
	devInfo = (FT_DEVICE_LIST_INFO_NODE *) mxMalloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numDevs);
	ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs);
	if (ftStatus == FT_OK) {
		for (i=0; i<numDevs; i++) {
			if (!strcmp(devInfo[i].Description, "aMG USB Connect")) {
				*deviceIndex = i;
				break;
			}
		}
	}
	mxFree(devInfo);
	
	// Return status
	return ftStatus;
}

FT_HANDLE ft231_open (void)
{
	int deviceNumber;
	FT_HANDLE ftHandle = 0;
	FT_STATUS ftStatus; 
	
	// Get valid number
	ftStatus = ft231_getvalid_deviceindex (&deviceNumber);
	if ((ftStatus != FT_OK) && (deviceNumber >= 0))
		return 0; // NULL	

	/* Open port */
	ftStatus = FT_Open(deviceNumber, &ftHandle);

	/* Return Handle */
	return ftHandle;
}

BOOL ft231_write (FT_HANDLE ftHandle, const char *TxBuffer, ULONG TxBytes)
{
	ULONG ret;
	return (FT_Write(ftHandle, (char *)TxBuffer, TxBytes, &ret) == FT_OK);
}

BOOL ft231_read (FT_HANDLE ftHandle, ULONG RxBytes, char *RxBuffer, ULONG *BytesReceived, ULONG timeout_ms)
{
	DWORD ulRead, ulReading, ulReadIndex;
	FT_STATUS ftStatus;
	DWORD start_tick, current_tick;
	
	/* Get start tick */
	start_tick = GetTickCount();
	ulReadIndex = 0;
	do {
		ftStatus = FT_GetQueueStatus(ftHandle, &ulRead);
		if ((ftStatus == FT_OK) && (ulRead > 0)) {
			if(FT_Read(ftHandle, &RxBuffer[ulReadIndex], ulRead > (RxBytes-ulReadIndex)?(RxBytes-ulReadIndex):ulRead, &ulReading) == FT_OK) {
				ulReadIndex += ulReading;
				/* Update new start tick */
				start_tick = GetTickCount();
			}
		}
		else {
			/* Wait some time */
			Sleep(10);
		}
		
		/* Current Tick */
		current_tick = GetTickCount();
	} while (((current_tick - start_tick) < timeout_ms) && (ulReadIndex < RxBytes));
	
	/* Number of reading */
	*BytesReceived = ulReadIndex;
	
	return (BOOL)(ulReadIndex > 0);	
}

void ft231_close (FT_HANDLE ftHandle)
{
	if (ftHandle)
		FT_Close(ftHandle);
}


// ========================================================================
// BOOT Mode Control
// ========================================================================
/* CBUS0 -> RESET
 * CBUS1 -> BOOT0
 */
#define RESET_H() _reset_cntrl(ftHandle, TRUE) //FT_ClrRts(ftHandle)
#define RESET_L() _reset_cntrl(ftHandle, FALSE ) //FT_SetRts(ftHandle)
#define BOOT0_H() _boot_cntrl (ftHandle, FALSE) //FT_SetDtr(ftHandle)
#define BOOT0_L() _boot_cntrl (ftHandle, TRUE ) //FT_ClrDtr(ftHandle)

static BYTE _bitband_mask = 0x33;
FT_STATUS _reset_cntrl(FT_HANDLE ftHandle, BOOL state)
{
	if(state)
		_bitband_mask |= 0x01;
	else
		_bitband_mask &= 0xFE;
	return FT_SetBitMode(ftHandle, _bitband_mask, 0x20);
}

FT_STATUS _boot_cntrl(FT_HANDLE ftHandle, BOOL state)
{
	if(state)
		_bitband_mask |= 0x02;
	else
		_bitband_mask &= 0xFD;
	return FT_SetBitMode(ftHandle, _bitband_mask, 0x20);	
}

void _enter_iap_mode(FT_HANDLE ftHandle)
{
	FT_STATUS res = 0;

	/* Reset */
	_bitband_mask = 0x33;
	
	res |= RESET_L();
	res |= BOOT0_H();
	Sleep(100);
	res |= RESET_H();
	
	if (res != 0)
		mexPrintf("Error: failed to control Reset/boot pin.\n");
}

void _exit_iap_mode(FT_HANDLE ftHandle)
{
	FT_STATUS res = 0;

	res |= RESET_L();
	res |= BOOT0_L();
	Sleep(100);
	res |= RESET_H();
	
	if (res != 0)
		mexPrintf("Error: failed to control Reset/boot pin.\n");
}

// ========================================================================
// IAP Protocol
// ========================================================================
// Refer to AN3155 Application note: 
// USART protocol used in the STM32? bootloader

// Response
#define IAP_ACK      0x79
#define IAP_NACK     0x1F

// Command
#define IAP_CMD_GET				0x00 /* Gets the version and the allowed commamds 
										supported by the current version of the 
										bootloader */
#define IAP_CMD_GET_VER_RDP		0x01 /* Gets the bootloader version and the Read 
										Protection status of the Flash memory */
#define IAP_CMD_GET_ID			0x02 /* Gets the chip ID */
#define IAP_CMD_READ_MEM		0x11 /* Reads up to 256 bytes of memory starting
										from an address specified by the application */
#define IAP_CMD_GO				0x21 /* Jumps to user applicatin code located in
										the internal Flash or in SRAM */
#define IAP_CMD_WRITE			0x31 /* Writes up to 256 bytes to the RAM or 
										Flash memory starting from an address 
										specified by the application */
#define IAP_CMD_ERASE			0x43 /* Erase from one to all the Flash memory pages */
#define IAP_CMD_EXTND_ERASE		0x44 /* Erase from one to all the Flash memory pages
										using two byte address mode available only 
										for v3.0 usart bootloader version and above). */
#define IAP_CMD_WRITE_PROTECT   0x63 /* Enabled the write protect for some sectors */
#define IAP_CMD_WRITE_UNPROTECT 0x73 /* Disable the write protection for all Flash 
										memory sectors */
#define IAP_CMD_READ_PROTECT	0x82 /* Enables the read protection */
#define IAP_CMD_READ_UNPROTECT  0x92 /* Disables the read protection */

typedef struct {
	unsigned char Cmd_Get;
	unsigned char Cmd_Get_Ver_Rdp;
	unsigned char Cmd_Get_Id;
	unsigned char Cmd_Read_Mem;
	unsigned char Cmd_Go;
	unsigned char Cmd_Write;
	unsigned char Cmd_Erase;
	unsigned char Cmd_Erase_Ext;
	unsigned char Cmd_Write_Protect;
	unsigned char Cmd_Write_UnProtect;
	unsigned char Cmd_Read_Protect;
	unsigned char Cmd_Read_UnProtect;
} IAP_CMD_SUPPORT;

typedef struct {
	unsigned char Byte1;
	unsigned char Byte2;
} OPTION_BYTES;

//BOOL IAP_Exit(FT_HANDLE ftHandle)
//{
//	_exit_iap_mode(ftHandle);
//	return TRUE;
//}

BOOL IAP_DeviceInfo (FT_HANDLE ftHandle, IAP_CMD_SUPPORT *IapCmdSupport, OPTION_BYTES *OptionsByte, DWORD *Pid) 
{
	// Get IAP device information
	int i, CmdCount;
	char TxData[2];
	char RxBuffer[32];
	char tmp[1024];
	ULONG BytesReceived;
	
	/* Reset */
	memset(IapCmdSupport, sizeof(IAP_CMD_SUPPORT), 0);
	memset(OptionsByte, sizeof(OPTION_BYTES), 0);
	
	/* Flush buffer */
	i = 0;
	while(! ft231_read (ftHandle, 1024, &tmp[0], &BytesReceived, 10) && (BytesReceived > 0) && (++i < 100));
	
	
	ft231_write (ftHandle, "\x7F", 1);
	if(! ft231_read (ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
		return FALSE;
	if (RxBuffer[0] != IAP_ACK) {
		return FALSE;
	}
	
	/* === Get supported command === */
	TxData[0] = IAP_CMD_GET;
	TxData[1] = ~IAP_CMD_GET;
	ft231_write (ftHandle, TxData, 2);	
	Sleep(100);
	/* Get list of supported command */
	if (! ft231_read (ftHandle, 32, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived < 4)) {
		mexPrintf("Read failed, %d bytes: %X...\n", BytesReceived, RxBuffer[0]);
		return FALSE;
	}
	if (!((RxBuffer[0] == IAP_ACK) && (RxBuffer[BytesReceived-1] == IAP_ACK))) {
		mexPrintf("NAK.\n");
		return FALSE;
	}

	/* 1: Command support count */
	CmdCount = (int)(RxBuffer[1]);
	if (CmdCount != ((int)BytesReceived-4)) {
		mexPrintf(IAP_LOGGING_HDR);
		mexPrintf("Internal error!\n");
		return FALSE;
	}
	
	/* 2: Boot loader version */
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Bootloader v%d.%d\n", (int)(RxBuffer[2] >> 4), (int)(RxBuffer[2] & 0x0F));
	
	/* 3: .... (3+CmdCount) */
	for (i=0; i<CmdCount; i++) {
		switch (RxBuffer[3+i]) {
			case IAP_CMD_GET: 
				IapCmdSupport->Cmd_Get = 1;
				break;
			case IAP_CMD_GET_VER_RDP: 
				IapCmdSupport->Cmd_Get_Ver_Rdp = 1;
				break;
			case IAP_CMD_GET_ID: 
				IapCmdSupport->Cmd_Get_Id = 1;
				break;
			case IAP_CMD_READ_MEM: 
				IapCmdSupport->Cmd_Read_Mem = 1;
				break;
			case IAP_CMD_GO: 
				IapCmdSupport->Cmd_Go = 1;
				break;
			case IAP_CMD_WRITE: 
				IapCmdSupport->Cmd_Write = 1;
				break;
			case IAP_CMD_ERASE: 
				IapCmdSupport->Cmd_Erase = 1;
				break;
			case IAP_CMD_EXTND_ERASE: 
				IapCmdSupport->Cmd_Erase_Ext = 1;
				break;
			case IAP_CMD_WRITE_PROTECT: 
				IapCmdSupport->Cmd_Write_Protect = 1;
				break;
			case IAP_CMD_WRITE_UNPROTECT: 
				IapCmdSupport->Cmd_Write_UnProtect = 1;
				break;
			case IAP_CMD_READ_PROTECT: 
				IapCmdSupport->Cmd_Read_Protect = 1;
				break;
			case IAP_CMD_READ_UNPROTECT: 
				IapCmdSupport->Cmd_Read_UnProtect = 1;
				break;
			default:
				break;
		}
	}
	
	/* === Get option bytes === */
	TxData[0] = IAP_CMD_GET_VER_RDP;
	TxData[1] = ~IAP_CMD_GET_VER_RDP;
	ft231_write(ftHandle, TxData, 2);
	if (! ft231_read (ftHandle, 32, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 5))
		return FALSE;
	if (!((RxBuffer[0] == IAP_ACK) && (RxBuffer[BytesReceived-1] == IAP_ACK)))
		return FALSE;
	OptionsByte->Byte1 = RxBuffer[2];
	OptionsByte->Byte2 = RxBuffer[3];
	
	/* === Get PID === */
	TxData[0] = IAP_CMD_GET_ID;
	TxData[1] = ~IAP_CMD_GET_ID;
	IAP_Comm_Write(ftHandle, TxData, 2);

	if (! IAP_Comm_Read (ftHandle, 32, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 5))
		return FALSE;
	if (!((RxBuffer[0] == IAP_ACK) && (RxBuffer[BytesReceived-1] == IAP_ACK)))
		return FALSE;
	for (i=1; i<4; i++) {
		//mexPrintf(", %x", (DWORD)RxBuffer[i]);
	}
	*Pid = ((DWORD)RxBuffer[2] << 8) | ((DWORD)RxBuffer[3]);
	
	// No error
	return TRUE;
}

BOOL IAP_ModeInit(char *portname)
{
	IAP_CMD_SUPPORT IapCmdSupport;
	OPTION_BYTES OptionsByte;
	DWORD Pid;
	LONG      lComPortNumber;
	FT_HANDLE ftHandle;
	FT_STATUS ftStatus;
	BOOL sta;
	
	// This function use for replace IAP_Init()
	
	sta = TRUE;
	ftHandle = ft231_open ();	
	if (!ftHandle)
		sta = FALSE;
	
	// Communication
	/* Data chatacteristics: Set 8 data bits, 1 stop bit and even parity  */
	if (sta)
		sta = (FT_OK == FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_EVEN));
	
	/* Baudrate */
	if (sta)
		sta = (FT_OK == FT_SetBaudRate(ftHandle, 115200));
	
	/* Default Timeout */
	if (sta)
		sta = (FT_OK == FT_SetTimeouts(ftHandle, 100, 100));
	
	// Enter IAP mode
	if (sta) {
		_enter_iap_mode(ftHandle);
		Sleep(100); /* Some delay after MCU reset */
	}
	
	// Get device information
	if (sta) {
		sta = IAP_DeviceInfo(ftHandle, &IapCmdSupport, &OptionsByte, &Pid);		
	}

	// Validate "aMG USB Connect" board
	if (sta) {
		sta = ft231x_validate(ftHandle, "aMG USB Connect");
		if (!sta)
			mexPrintf("Could not found the \"aMG USB Connect board.\n\"");
	}
	
	// Get port name
	if (sta) {
		sta = ft231_getportname(ftHandle, portname);
	}
	
	// Verify
	if (sta) {
		sta = (Pid == 0x0448);
		if (!sta) {
			mexPrintf("Expect PID=0x0448, reading=%X\n", Pid);
			sta = FALSE;
		}		
	}
	
	// Close Handle
	if (ftHandle) {
		ft231_close(ftHandle);
	}
	
	if (sta)
		return TRUE;
	else
		return FALSE;
}

BOOL IAP_ModeExit(void)
{
	FT_HANDLE ftHandle;
	BOOL sta;
	
	sta = TRUE;
	ftHandle = ft231_open ();	
	if (!ftHandle)
		sta = FALSE;
	
	// Exit from IAP mode
	_exit_iap_mode(ftHandle);
	
	if (ftHandle) {
		ft231_close(ftHandle);
	}	
	
	// Return status
	return sta;
}


BOOL IAP_Erase_Ext_Flash(SERIAL_PORT_STRUCT *serial, DWORD dwPageIndex)
{
	DWORD dwBytesReceived;
	char TxData[8];
	char RxBuffer[32];
	
	/* --- Erase command --- */
	TxData[0] = IAP_CMD_EXTND_ERASE;
	TxData[1] = ~IAP_CMD_EXTND_ERASE;	
	if (!Serial_Write_Ex (serial, TxData, 2)) {
		return FALSE;
	}
	
	/* Ack? */
	if (! Serial_Read_Ex (serial, &RxBuffer[0], 1, &dwBytesReceived, 500) || (dwBytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	/* N */
	TxData[0] = 0;
	TxData[1] = 0;
	TxData[2] = (char)(dwPageIndex>>8);
	TxData[3] = (char)dwPageIndex;
	TxData[4] = TxData[0] ^ TxData[1] ^ TxData[2] ^ TxData[3];
	if (!Serial_Write_Ex (serial, TxData, 5)) {
		return FALSE;
	}
	Sleep(50);
	/* Ack? */
	if (! Serial_Read_Ex (serial, &RxBuffer[0], 1, &dwBytesReceived, 500) || (dwBytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	
	return TRUE;
}

BOOL IAP_FullErase_Ext_Flash(SERIAL_PORT_STRUCT *serial)
{
	DWORD dwBytesReceived;
	char TxData[8];
	char RxBuffer[32];
	
	/* --- Erase command --- */
	TxData[0] = IAP_CMD_EXTND_ERASE;
	TxData[1] = ~IAP_CMD_EXTND_ERASE;	
	if (!Serial_Write_Ex (serial, TxData, 2)) {
		return FALSE;
	}
	/* Ack? */
	if (! Serial_Read_Ex (serial, &RxBuffer[0], 1, &dwBytesReceived, 500) || (dwBytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	/* N */
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0x00;
	if (!Serial_Write_Ex (serial, TxData, 3)) {
		return FALSE;
	}
	/* Ack? */
	if (! Serial_Read_Ex (serial, &RxBuffer[0], 1, &dwBytesReceived, 500) || (dwBytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	
	return TRUE;
}

#if 0
BOOL IAP_Read_Flash(FT_HANDLE ftHandle, DWORD address, DWORD count, char *Buffer)
{
	DWORD start_tick;
	int i, CmdCount;
	char TxData[8];
	char RxBuffer[32];
	ULONG BytesReceived, PacketDataCount;
	
	DWORD dwReadIndex;
	DWORD dwReadCountRem, dwReadCountPkt;
    
	start_tick = GetTickCount();
	dwReadIndex = 0;
	dwReadCountRem = count;
	do {
		mexPrintf(IAP_LOGGING_HDR);
		mexPrintf(">> [%u] Read addr: %x", GetTickCount()-start_tick, address);

		/* --- Cmd --- */
		TxData[0] = IAP_CMD_READ_MEM;
		TxData[1] = ~IAP_CMD_READ_MEM;
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* --- Address --- */
		TxData[0] = (char)(address >> 24);
		TxData[1] = (char)(address >> 16);
		TxData[2] = (char)(address >> 8);
		TxData[3] = (char)(address >> 0);
		TxData[4] = (char)(TxData[0] ^ TxData[1] ^ TxData[2] ^ TxData[3]);
		IAP_Comm_Write(ftHandle, TxData, 5);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* --- Num of bytes to read --- */
		if(dwReadCountRem > 256) {
			/* Max read packet 256 bytes */
			PacketDataCount = 256;			
			dwReadCountRem -= 256;
		}
		else {
			PacketDataCount = dwReadCountRem;
			dwReadCountRem = 0;
		}
		
		mexPrintf(", %d bytes\n", PacketDataCount);
		
		TxData[0] = (char)(PacketDataCount-1);
		TxData[1] = (char)(~(PacketDataCount-1));
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* --- Data --- */
		if (! IAP_Comm_Read(ftHandle, PacketDataCount, &Buffer[dwReadIndex], &BytesReceived, 500) || (BytesReceived != PacketDataCount)) {
			mexPrintf("\nRead failed, received: %u bytes\n", BytesReceived);
			return FALSE;
		}
		
		/* Next read address */
		address += PacketDataCount;
		dwReadIndex += PacketDataCount;
		
	} while (dwReadIndex < count);
	
	return TRUE;
}
#endif //0

BOOL IAP_Write_Flash(SERIAL_PORT_STRUCT *serial, const char *Buffer, DWORD address, DWORD count)
{	
	char xor_byte;
	DWORD start_tick;
	int i, CmdCount;
	char TxData[8];
	char RxBuffer[32];
	DWORD dwBytesReceived, PacketDataCount;
	
	DWORD dwWriteIndex;
	DWORD dwReadCountRem, dwReadCountPkt;
    
	start_tick = GetTickCount();
	dwWriteIndex = 0;
	dwReadCountRem = count;
	do {		
		/* --- Cmd --- */
		TxData[0] = IAP_CMD_WRITE;
		TxData[1] = ~IAP_CMD_WRITE;
		if (!Serial_Write_Ex(serial, TxData, 2)) {
			mexPrintf("Write failed-1\n");
			return FALSE;
		}
		/* Ack? */
		if (! Serial_Read_Ex(serial, &RxBuffer[0], 1, &dwBytesReceived, 1000) || (dwBytesReceived != 1)) {
			mexPrintf("Read failed-1\n");
			return FALSE;
		}
		if (!(RxBuffer[0] == IAP_ACK)) {
			mexPrintf("NACK-1\n");
			return FALSE;
		}
		/* --- Address --- */
		TxData[0] = (char)(address >> 24);
		TxData[1] = (char)(address >> 16);
		TxData[2] = (char)(address >> 8);
		TxData[3] = (char)(address >> 0);
		TxData[4] = (char)(TxData[0] ^ TxData[1] ^ TxData[2] ^ TxData[3]);
		if (!Serial_Write_Ex(serial, TxData, 5)) {
			mexPrintf("Write failed-2\n");
			return FALSE;
		}
		/* Ack? */
		if (! Serial_Read_Ex(serial, &RxBuffer[0], 1, &dwBytesReceived, 1000) || (dwBytesReceived != 1)) {
			mexPrintf("Read failed-2\n");
			return FALSE;
		}
		if (!(RxBuffer[0] == IAP_ACK)) {
			mexPrintf("NACK-2\n");
			return FALSE;
		}
		/* --- Num of bytes to read --- */
		if(dwReadCountRem > 128) {
			/* Max read packet 256 bytes */
			PacketDataCount = 128;			
			dwReadCountRem -= 128;
		}
		else {
			PacketDataCount = dwReadCountRem;
			dwReadCountRem = 0;
		}
		
		TxData[0] = (char)(PacketDataCount-1);
		if (!Serial_Write_Ex(serial, TxData, 1)) {
			mexPrintf("Write failed-3\n");
			return FALSE;		
		}
		
		xor_byte = (char)(PacketDataCount-1);
		for (i=0; i<PacketDataCount; i++) {
			xor_byte = xor_byte ^ Buffer[dwWriteIndex+i];
		}
		if (!Serial_Write_Ex(serial, &Buffer[dwWriteIndex], PacketDataCount)) {
			mexPrintf("Write failed-4\n");
			return FALSE;
		}
		if (!Serial_Write_Ex(serial, &xor_byte, 1)) {
			mexPrintf("Write failed-5\n");
			return FALSE;
		}
		
		/* Ack? */
		if (! Serial_Read_Ex(serial, &RxBuffer[0], 1, &dwBytesReceived, 1000) || (dwBytesReceived != 1)) {
			mexPrintf("Read failed-3\n");
			return FALSE;
		}
		if (!(RxBuffer[0] == IAP_ACK)) {
			mexPrintf("NACK-3\n");
			return FALSE;
		}
		
		/* Next read address */
		address += PacketDataCount;
		dwWriteIndex += PacketDataCount;
		
	} while (dwWriteIndex < count);

	return TRUE;	
}

// ========================================================================
// IAP Communication
// ========================================================================

BOOL IAP_Comm_Write (FT_HANDLE ftHandle, const char *TxBuffer, ULONG TxBytes)
{
	ULONG ret;
	return (FT_Write(ftHandle, (char *)TxBuffer, TxBytes, &ret) == FT_OK);
}

extern DWORD WINAPI GetTickCount(void);
BOOL IAP_Comm_Read (FT_HANDLE ftHandle, ULONG RxBytes, char *RxBuffer, ULONG *BytesReceived, ULONG timeout_ms)
{
	DWORD ulRead, ulReading, ulReadIndex;
	FT_STATUS ftStatus;
	DWORD start_tick, current_tick;
	
	/* Get start tick */
	start_tick = GetTickCount();
	ulReadIndex = 0;
	do {
		ftStatus = FT_GetQueueStatus(ftHandle, &ulRead);
		if ((ftStatus == FT_OK) && (ulRead > 0)) {
			if(FT_Read(ftHandle, &RxBuffer[ulReadIndex], ulRead > (RxBytes-ulReadIndex)?(RxBytes-ulReadIndex):ulRead, &ulReading) == FT_OK) {
				ulReadIndex += ulReading;
				/* Update new start tick */
				start_tick = GetTickCount();
			}
		}
		else {
			/* Wait some time */
			Sleep(10);
		}
		
		/* Current Tick */
		current_tick = GetTickCount();
	} while (((current_tick - start_tick) < timeout_ms) && (ulReadIndex < RxBytes));
	
	/* Number of reading */
	*BytesReceived = ulReadIndex;
	
	return (BOOL)(ulReadIndex > 0);
}

/* Working buffer */
BOOL IAP_File_Load(const char *filename, char *mem_buffer, int mem_buffer_size, DWORD *file_size)
{
	FILE *fp = NULL;
	BOOL sta = TRUE;
	DWORD read_count;	
	size_t f_idx;
				
	/* Open file */
	fp = fopen(filename, "rb");
	if (fp == NULL) {
		sta = FALSE;
		mexPrintf("Failed to open file: %s\n", filename);
	}
	
	/* Get file size */
	if (sta) {
		fseek(fp , 0 , SEEK_END);
		*file_size = ftell(fp);
		rewind(fp);
		if (*file_size > mem_buffer_size) {
			sta = FALSE;
			mexPrintf("Error: file size: %d\n", *file_size);
		}
	}
	
	/* Read file */
	if (sta) {
		f_idx = 0;
		read_count = 0xFFFFFFFF;
		while (sta && !feof(fp) && (f_idx < *file_size)) {
			read_count = fread(&mem_buffer[f_idx], 1, 512, fp);
			f_idx += read_count;
			if (read_count == 0) {
				sta = FALSE;
				mexPrintf("Error: read file.\n");
			}
		}
	}
	
	/* Close file */
	if (fp)
		fclose(fp);
	
	/* Return error status */ 
	return sta;	
}

// ========================================================================
// Mex function
// ========================================================================
static char Flash_Buffer[1024*1024];
static char Output_Message[1024*1024];
static char PortName[32];
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[])
{	
	int output_sta = 0;
	char input_cmd[64] = {0};
	char *output_buf, *input_buf;
	
	OPTION_BYTES OptionsByte;
	DWORD Pid;
	IAP_CMD_SUPPORT IapCmdSupport;
	double *p, result = 0;
	const mwSize dims[] = {1};
	
	/* Init buffer */
	Output_Message[0] = '\0';
	
	/* Validate */
	if (nrhs < 1)
		mexErrMsgTxt("Invalid number of input parameter."); 
	if (nlhs > 2)
		mexErrMsgTxt("Invalid number of output."); 
	if (!mxIsChar(prhs[0]))
		mexErrMsgTxt("Input parameter 1 must be string"); 
	
	/* Create input buffer, !!! Memory buse be free at end function */
	input_buf = mxArrayToString(prhs[0]);
	if(input_buf) {
		if(strlen(input_buf) >= 64) {
			memcpy(input_cmd, input_buf, 63);
			input_cmd[63] = '\0';			
		}
		else { strcpy(input_cmd, input_buf); }
		mxFree(input_buf);
	}
	
	/* ========================
	 * === Command: connect ===
	 */
	if(!strcmp(input_cmd, "connect")) {
		BOOL sta = TRUE;
		
		/* Enter IAP mode */
		sta = IAP_ModeInit(PortName);
		if (sta) {
			output_sta = 1;
			sprintf(Output_Message, "Connected.\n");
			mexPrintf("Port: %s\n", PortName);
		}
		else {
			sprintf(Output_Message, "Error: connection to target.\n");
		}
		
		// Return output status
		if (sta) {
			output_sta = 1;
		}
	}

	/* === Command: writeflash === */
	else if(!strcmp(input_cmd, "writeflash")) {
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		
		size_t f_idx;
		FT_HANDLE ftHandle = 0;
		char *input_filename;
		int input_filesize;
		DWORD mem_address, mem_size;
		FILE *fp = NULL;
		double* data_in;
		BOOL sta = TRUE;
		
		/* Default memory address */
		memset(Flash_Buffer, 0xFF, 0x20000);		
		
		/* Assume status is 0 */
		output_sta = 0;
		
		/* Number of input parameter */
		if (!((nrhs == 4)
				&& (mxIsDouble(prhs[1]) || mxIsUint32(prhs[1])) /* Address */
				&& (mxIsDouble(prhs[2]) || mxIsUint32(prhs[2])) /* Size */ 
				&& (mxIsChar(prhs[3])) /*File name*/
				)) {
			sta = FALSE;
		}
			
		/* Input parameters */
		if (sta) {
			data_in = mxGetPr(prhs[1]);
			mem_address = (DWORD)data_in[0];
			data_in = mxGetPr(prhs[2]);
			mem_size = (DWORD)data_in[0];
			input_filename = mxArrayToString(prhs[3]);
			
			/* Load file */
			sta = IAP_File_Load(input_filename, &Flash_Buffer[0], sizeof(Flash_Buffer), &input_filesize);
		}
		
		/* Connect to IAP device mode */
		if (sta) {
			sta = IAP_ModeInit(PortName);
			if (!sta) {
				mexPrintf("Error: Enable IAP mode.\n");
			}
		}
		
		/* Open COM port */
		if (sta) {
			sta = Serial_Open ( &serial, (const char *)PortName, TRUE, 115200U, 8U, EVENPARITY, ONESTOPBIT);
			if (!sta) {
				mexPrintf("Error: Open COM port: \"%s\"\n", PortName);
			}
		}
		
		/* Erase Flash */
		if (sta) {
			DWORD flash_size;
			DWORD page_idx;
			
			//page_idx = 0;
			page_idx= (mem_address & 0x1FFFF)>>11;
			flash_size = (DWORD)input_filesize;
			while (sta && (flash_size > 0)) {
				if (!IAP_Erase_Ext_Flash(&serial, page_idx)) {
					sta = FALSE;					
					sprintf(Output_Message, "\nError: Erase flash.\n");
					mexPrintf(Output_Message);
				}
				page_idx ++;
				if (flash_size >= 2048) // 2k
					flash_size -= 2048;
				else
					flash_size = 0;
			}
		}
		
		/* Write Flash */
		if (sta) {
			sta = IAP_Write_Flash(&serial, Flash_Buffer, mem_address, input_filesize);
			if (sta)
				mexPrintf("Success: Write Flash.\n");
			else
				mexPrintf("Error: Write Flash.\n");
		}

		/* --- Cleanup --- */
		Serial_Close(&serial);
		
		if (sta) {
			output_sta = 1;
			sprintf(Output_Message, "Success.\n");
		}
		else {
			sprintf(Output_Message, "Error: Program and Verify.\n");
		}
	}
	
	/* === Command: fullerase === */
	else if(!strcmp(input_cmd, "fullerase")) {
		BOOL sta = TRUE;
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		FT_HANDLE ftHandle = 0;
		
		// Connect to IAP mode
		sta = IAP_ModeInit(PortName);
		
		// Open port
		if (sta) {
			sta = Serial_Open( &serial, PortName, TRUE, 115200U, 8U, EVENPARITY, ONESTOPBIT);
		}
		
		// Erase
		if (sta) {
			sta = IAP_FullErase_Ext_Flash(&serial);
			if (sta) {
				sprintf(Output_Message, "Success.\n");
			}
			else {
				sprintf(Output_Message, "Error: Full Chip Erase\n");
			}
		}
		
		/* --- Cleanup --- */
		Serial_Close(&serial);
		
		if (sta) {
			output_sta = 1;
		}
	}
	
	/* === Command: run === */
	else if(!strcmp(input_cmd, "run")) {
		if (IAP_ModeExit())
			output_sta = 1;		
		else
			output_sta =0;
		
		if (output_sta != 0)
			strcpy(Output_Message, "Success.\n");
		else
			strcpy(Output_Message, "Error: Run.\n");
	}	
	/* === Unknown === */
	else {
		strcpy(Output_Message, "Unknown action.");
	}
	
	/* Return */
	plhs[0] = mxCreateDoubleScalar (output_sta); //mxCreateNumericArray(1,dims,mxINT32_CLASS,mxREAL);	
	if (nlhs > 1) {
		output_buf = mxCalloc(1024, sizeof(char));
		strcpy(output_buf, Output_Message);
		plhs[1] = mxCreateString(output_buf);
	}
	return;
}
