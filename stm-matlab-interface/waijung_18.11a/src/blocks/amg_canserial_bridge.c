
#include "mex.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

// FTDI Device Lib
#include "..\..\utils\devices\aMG_USBConnect\ftd2xx.h"
//#pragma comment(lib,"..\\..\\utils\\devices\\amgprog\\x86\\ftd2xx.lib")
//#pragma comment(lib,"..\\..\\utils\\devices\\amgprog\\x64\\ftd2xx.lib")

#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\i386\\ftd2xx.lib")
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\amd64\\ftd2xx.lib")

		
// ========================================================================
static unsigned int crc_table[256] =
{
	0x00000000,	0x04C11DB7,	0x09823B6E,	0x0D4326D9,
	0x130476DC,	0x17C56B6B,	0x1A864DB2,	0x1E475005,
	0x2608EDB8,	0x22C9F00F,	0x2F8AD6D6,	0x2B4BCB61,
	0x350C9B64,	0x31CD86D3,	0x3C8EA00A,	0x384FBDBD,
	0x4C11DB70,	0x48D0C6C7,	0x4593E01E,	0x4152FDA9,
	0x5F15ADAC,	0x5BD4B01B,	0x569796C2,	0x52568B75,
	0x6A1936C8,	0x6ED82B7F,	0x639B0DA6,	0x675A1011,
	0x791D4014,	0x7DDC5DA3,	0x709F7B7A,	0x745E66CD,
	0x9823B6E0,	0x9CE2AB57,	0x91A18D8E,	0x95609039,
	0x8B27C03C,	0x8FE6DD8B,	0x82A5FB52,	0x8664E6E5,
	0xBE2B5B58,	0xBAEA46EF,	0xB7A96036,	0xB3687D81,
	0xAD2F2D84,	0xA9EE3033,	0xA4AD16EA,	0xA06C0B5D,
	0xD4326D90,	0xD0F37027,	0xDDB056FE,	0xD9714B49,
	0xC7361B4C,	0xC3F706FB,	0xCEB42022,	0xCA753D95,
	0xF23A8028,	0xF6FB9D9F,	0xFBB8BB46,	0xFF79A6F1,
	0xE13EF6F4,	0xE5FFEB43,	0xE8BCCD9A,	0xEC7DD02D,
	0x34867077,	0x30476DC0,	0x3D044B19,	0x39C556AE,
	0x278206AB,	0x23431B1C,	0x2E003DC5,	0x2AC12072,
	0x128E9DCF,	0x164F8078,	0x1B0CA6A1,	0x1FCDBB16,
	0x018AEB13,	0x054BF6A4,	0x0808D07D,	0x0CC9CDCA,
	0x7897AB07,	0x7C56B6B0,	0x71159069,	0x75D48DDE,
	0x6B93DDDB,	0x6F52C06C,	0x6211E6B5,	0x66D0FB02,
	0x5E9F46BF,	0x5A5E5B08,	0x571D7DD1,	0x53DC6066,
	0x4D9B3063,	0x495A2DD4,	0x44190B0D,	0x40D816BA,
	0xACA5C697,	0xA864DB20,	0xA527FDF9,	0xA1E6E04E,
	0xBFA1B04B,	0xBB60ADFC,	0xB6238B25,	0xB2E29692,
	0x8AAD2B2F,	0x8E6C3698,	0x832F1041,	0x87EE0DF6,
	0x99A95DF3,	0x9D684044,	0x902B669D,	0x94EA7B2A,
	0xE0B41DE7,	0xE4750050,	0xE9362689,	0xEDF73B3E,
	0xF3B06B3B,	0xF771768C,	0xFA325055,	0xFEF34DE2,
	0xC6BCF05F,	0xC27DEDE8,	0xCF3ECB31,	0xCBFFD686,
	0xD5B88683,	0xD1799B34,	0xDC3ABDED,	0xD8FBA05A,
	0x690CE0EE,	0x6DCDFD59,	0x608EDB80,	0x644FC637,
	0x7A089632,	0x7EC98B85,	0x738AAD5C,	0x774BB0EB,
	0x4F040D56,	0x4BC510E1,	0x46863638,	0x42472B8F,
	0x5C007B8A,	0x58C1663D,	0x558240E4,	0x51435D53,
	0x251D3B9E,	0x21DC2629,	0x2C9F00F0,	0x285E1D47,
	0x36194D42,	0x32D850F5,	0x3F9B762C,	0x3B5A6B9B,
	0x0315D626,	0x07D4CB91,	0x0A97ED48,	0x0E56F0FF,
	0x1011A0FA,	0x14D0BD4D,	0x19939B94,	0x1D528623,
	0xF12F560E,	0xF5EE4BB9,	0xF8AD6D60,	0xFC6C70D7,
	0xE22B20D2,	0xE6EA3D65,	0xEBA91BBC,	0xEF68060B,
	0xD727BBB6,	0xD3E6A601,	0xDEA580D8,	0xDA649D6F,
	0xC423CD6A,	0xC0E2D0DD,	0xCDA1F604,	0xC960EBB3,
	0xBD3E8D7E,	0xB9FF90C9,	0xB4BCB610,	0xB07DABA7,
	0xAE3AFBA2,	0xAAFBE615,	0xA7B8C0CC,	0xA379DD7B,
	0x9B3660C6,	0x9FF77D71,	0x92B45BA8,	0x9675461F,
	0x8832161A,	0x8CF30BAD,	0x81B02D74,	0x857130C3,
	0x5D8A9099,	0x594B8D2E,	0x5408ABF7,	0x50C9B640,
	0x4E8EE645,	0x4A4FFBF2,	0x470CDD2B,	0x43CDC09C,
	0x7B827D21,	0x7F436096,	0x7200464F,	0x76C15BF8,
	0x68860BFD,	0x6C47164A,	0x61043093,	0x65C52D24,
	0x119B4BE9,	0x155A565E,	0x18197087,	0x1CD86D30,
	0x029F3D35,	0x065E2082,	0x0B1D065B,	0x0FDC1BEC,
	0x3793A651,	0x3352BBE6,	0x3E119D3F,	0x3AD08088,
	0x2497D08D,	0x2056CD3A,	0x2D15EBE3,	0x29D4F654,
	0xC5A92679,	0xC1683BCE,	0xCC2B1D17,	0xC8EA00A0,
	0xD6AD50A5,	0xD26C4D12,	0xDF2F6BCB,	0xDBEE767C,
	0xE3A1CBC1,	0xE760D676,	0xEA23F0AF,	0xEEE2ED18,
	0xF0A5BD1D,	0xF464A0AA,	0xF9278673,	0xFDE69BC4,
	0x89B8FD09,	0x8D79E0BE,	0x803AC667,	0x84FBDBD0,
	0x9ABC8BD5,	0x9E7D9662,	0x933EB0BB,	0x97FFAD0C,
	0xAFB010B1,	0xAB710D06,	0xA6322BDF,	0xA2F33668,
	0xBCB4666D,	0xB8757BDA,	0xB5365D03,	0xB1F740B4
};

unsigned int crc32_update(unsigned int initial, const unsigned int *data, int count)
{
  int i;
  unsigned int accum;

  accum = initial;
  for (i=0; i<count; i++) {
    accum = (accum<< 8)^ crc_table[((accum>> 24)^ (data[i]>> 24))& 0xFF];
    accum = (accum<< 8)^ crc_table[((accum>> 24)^ (data[i]>> 16))& 0xFF];
    accum = (accum<< 8)^ crc_table[((accum>> 24)^ (data[i]>> 8 ))& 0xFF];
    accum = (accum<< 8)^ crc_table[((accum>> 24)^ (data[i]>> 0 ))& 0xFF];
  }
  return accum;
}

BOOL load_file_tobuffer(const char *filename, char *buffer, unsigned int count)
{
	FILE *f;
	size_t reading_count;
	
	f = fopen(filename, "rb");
	if (f) {
		reading_count = fread(buffer, 1, count, f);		
		fclose(f); // Close
		if(reading_count > 0)
			return TRUE; // Success
	}
	return FALSE; // Fail
}

BOOL save_file_frombuffer(const char *filename, const char *buffer, unsigned int count)
{
	FILE *f;
	size_t written_count;
	
	f = fopen((const char *)filename, "wb");
	if (f) {
		written_count = fwrite(buffer, 1, count, f);
		/* Close file */
		fclose(f);
		if (written_count == count)
			return TRUE; // Success
	}
	return FALSE; // Fail
}

extern DWORD WINAPI GetTickCount(void);

// ========================================================================
// COM interface
// ========================================================================
#define SERIAL_BAUD_RATE 115200

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

void Serial_Flush(SERIAL_PORT_STRUCT *serial)
{
	DWORD dwRead;
	char tmp[128];
	
	while (Serial_Read (serial, tmp, 128, &dwRead, 100) && (dwRead > 0));
}

BOOL Serial_Write_Ex (SERIAL_PORT_STRUCT *serial, const char* pData, DWORD dwLen)
{
	DWORD dwWritten;
	
	// Flush buffer
	Serial_Flush(serial);
	
	// Write
	if (!Serial_Write (serial, pData, dwLen, &dwWritten, 2000))
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

BOOL Serial_ReadLine (SERIAL_PORT_STRUCT *serial, char* pData, DWORD dwLen, DWORD* pdwRead, DWORD dwTimeout)
{
	BOOL sta = TRUE;
	char tmp;
	DWORD read_count, dwRead;
	int start_tick;

	// Start time
	start_tick = GetTickCount();
	
	read_count = 0;
	while (sta && (read_count < dwLen) && ((GetTickCount() - start_tick) < dwTimeout)) {
		sta = Serial_Read(serial, &pData[read_count], 1, &dwRead, dwTimeout);
		if (sta && (dwRead == 1)) {
			read_count += dwRead;
			start_tick = GetTickCount();
		}
		if (read_count >= 2) {
			if((pData[read_count-1] == 0x0A) && (pData[read_count-2] == 0x0D)) {
				// Done
				pData[read_count-2] = '\0';
				*pdwRead = read_count;
				return TRUE;
			}
		}
	}
	return FALSE;
}

// ========================================================================
// FTDI device
// ========================================================================
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
		// Search for "aMG_USBConverter-N A"
		if (*deviceIndex < 0) {
			for (i=0; i<numDevs; i++) {
				if (devInfo[i].Type == FT_DEVICE_2232H) {
					*deviceIndex = i;
					break;
				}
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

void ft231_close (FT_HANDLE ftHandle)
{
	if (ftHandle)
		FT_Close(ftHandle);
}

BOOL ft231_getportname(char *buffer)
{
	LONG lComPortNumber;
	FT_STATUS ftStatus;
	FT_HANDLE ftHandle;
	
	// Open
	ftHandle = ft231_open();
	if (ftHandle == 0)
		return FALSE;
	
	// Get port number
	ftStatus = FT_GetComPortNumber(ftHandle, &lComPortNumber);
	if ((ftStatus == FT_OK) && (lComPortNumber > 0)) {
		if (lComPortNumber > 9) {
			sprintf_s(buffer, 32, "\\\\.\\%s%d", "COM", lComPortNumber);
		}
		else {
			sprintf(buffer, "COM%d", lComPortNumber);
		}
	}
	
	// Close
	ft231_close (ftHandle);
	
	// Return
	return (ftStatus == FT_OK);
}

// ========================================================================
// CCP Definition
// ========================================================================
#define CCP_CONNECT                        0x01
#define CCP_GET_CCP_VERSION                0x1B
#define CCP_EXCHANGE_ID                    0x17
#define CCP_GET_SEED                       0x12
#define CCP_UNLOCK                         0x13
#define CCP_SET_MTA                        0x02
#define CCP_DNLOAD                         0x03
#define CCP_DNLOAD_6                       0x23
#define CCP_UPLOAD                         0x04
#define CCP_SHORT_UP                       0x0F
#define CCP_SELECT_CAL_PAGE                0x11
#define CCP_GET_DAQ_SIZE                   0x14
#define CCP_SET_DAQ_PTR                    0x15
#define CCP_WRITE_DAQ                      0x16
#define CCP_START_STOP                     0x06
#define CCP_DISCONNECT                     0x07
#define CCP_SET_S_STATUS                   0x0C
#define CCP_GET_S_STATUS                   0x0D
#define CCP_BUILD_CHKSUM                   0x0E
#define CCP_CLEAR_MEMORY                   0x10
#define CCP_PROGRAM                        0x18
#define CCP_PROGRAM_6                      0x22
#define CCP_MOVE                           0x19
#define CCP_TEST                           0x05
#define CCP_GET_ACTIVE_CAL_PAGE            0x09
#define CCP_START_STOP_ALL                 0x08
#define CCP_DIAG_SERVICE                   0x20
#define CCP_ACTION_SERVICE                 0x21

typedef struct {
	unsigned int idtype; // 0: ext, 1 or otherwise: std
	unsigned int id;
	unsigned char dlc; // 0-8
	unsigned char data[8];
} CAN_MESSAGE;

const char *CAN_ID_TYPE[2] = {"extid", "stdid"};

//#define CAN_SERIAL_BRIDGE_BINARY 1

BOOL cantx(SERIAL_PORT_STRUCT *serial, const CAN_MESSAGE *can_msg, DWORD *stamp, DWORD *code) {
	DWORD dwRead;
	int i;
	unsigned char sum;
	char tx_buffer[256];
	char rx_buffer[256];

	// Write
	tx_buffer[0] = 0x7E;
	tx_buffer[1] = 0x7E;
	tx_buffer[2] = (char)(1 & can_msg->idtype); // 0-ExtID, 1-StdID
	tx_buffer[3] = (char)(can_msg->id >> 0);
	tx_buffer[4] = (char)(can_msg->id >> 8);
	tx_buffer[5] = (char)(can_msg->id >> 16);
	tx_buffer[6] = (char)(can_msg->id >> 24);	
	tx_buffer[7] = (char)can_msg->dlc; // DLC
	tx_buffer[8] = (char)can_msg->data[0];
	tx_buffer[9] = (char)can_msg->data[1];
	tx_buffer[10] = (char)can_msg->data[2];
	tx_buffer[11] = (char)can_msg->data[3];
	tx_buffer[12] = (char)can_msg->data[4];
	tx_buffer[13] = (char)can_msg->data[5];
	tx_buffer[14] = (char)can_msg->data[6];
	tx_buffer[15] = (char)can_msg->data[7];
	
	sum = 0;
	for (i=0; i<16; i++)
		sum += tx_buffer[i];
	tx_buffer[16] = (char)sum;
	
	if (!Serial_Write_Ex(serial, tx_buffer, 17)) {
		mexPrintf("Error: write serial.\n");
		return FALSE;
	}
	
	// Get response
	if (!Serial_Read_Ex (serial, rx_buffer, 4, &dwRead, 1000)) {
		mexPrintf("Error: wait for response.\n");
		return FALSE;
	}
	
	// Verify response
	if ((dwRead == 4) && (rx_buffer[0] == 0x7F) && (rx_buffer[1] == 0x7F) && (rx_buffer[2] == 0x00))
	{
		// Verify checksum
		if ((char)rx_buffer[3] == (char)(rx_buffer[0] + rx_buffer[1] + rx_buffer[2])) {
			*code = (DWORD)rx_buffer[2];
			return TRUE;
		}
		else {
			mexPrintf("Error: checksum\n");
		}
	}
	else {
		mexPrintf("Error: response %d bytes: %X,%X,%X,%X,%X,%X,%X,%X\n", dwRead, (unsigned int)rx_buffer[0], (unsigned int)rx_buffer[1], (unsigned int)rx_buffer[2], (unsigned int)rx_buffer[3], (unsigned int)rx_buffer[4], (unsigned int)rx_buffer[5], (unsigned int)rx_buffer[6], (unsigned int)rx_buffer[7]);
	}
	
	return FALSE;
}

BOOL canrx(SERIAL_PORT_STRUCT *serial, int timeout, CAN_MESSAGE *can_msg_buff, DWORD *stamp) {
	char sum;
	int i;
	DWORD dwRead;
	char tx_buffer[256];
	char rx_buffer[256];
	
	int retry_count = 0;
	
	
___retry:	
	if (!Serial_Read_Ex(serial, rx_buffer, 17, &dwRead, 1000)) {
		mexPrintf("Error: wait for response.\n");
		return FALSE;
	}
	
	if (dwRead == 0) {
		if (retry_count < 1) {
			retry_count ++;
			goto ___retry;
		}
	}
	
	if (dwRead != 17) {
		mexPrintf("Error: invalid number of response bytes: %u\n", dwRead);
		return FALSE;		
	}
	
	if ((rx_buffer[0] != (char)0xFF) || (rx_buffer[1] != (char)0xFF)) {
		mexPrintf("Error: response header bytes: %X,%X,..\n", (unsigned int)rx_buffer[0], (unsigned int)rx_buffer[1]);
		return FALSE;
	}
	
	// Verify checksum
	sum = 0;
	for (i=0; i<16; i++) {
		sum += rx_buffer[i];
	}
	if ((char)sum != (char)rx_buffer[16]) {
		mexPrintf("Error: checksum\n");
		return FALSE;
	}

	// Return data
	can_msg_buff->idtype  = rx_buffer[2]; // 0: ext, 1 or otherwise: std
	can_msg_buff->id      = \
			(unsigned int)rx_buffer[3] \
			| (unsigned int)(rx_buffer[4]<<8) \
			| (unsigned int)(rx_buffer[5]<<16) \
			| (unsigned int)(rx_buffer[6]<<24);	
	can_msg_buff->dlc     = rx_buffer[7];
	can_msg_buff->data[0] = rx_buffer[8];
	can_msg_buff->data[1] = rx_buffer[9];
	can_msg_buff->data[2] = rx_buffer[10];
	can_msg_buff->data[3] = rx_buffer[11];
	can_msg_buff->data[4] = rx_buffer[12];
	can_msg_buff->data[5] = rx_buffer[13];
	can_msg_buff->data[6] = rx_buffer[14];
	can_msg_buff->data[7] = rx_buffer[15];
	
	// Success
	return TRUE;	
}

/* fileter_id: 0xFFFFFFFF - no filter, otherwise address must match.
 */
BOOL setup_can_serial_bridge(SERIAL_PORT_STRUCT *serial, char id_type, char enable, unsigned int filter_id)
{
	int i;
	unsigned char sum;
	DWORD dwRead;
	char tx_buffer[256];
	char rx_buffer[256];
	
	// * Filter Setting Format (13): [AA][55][88][88][88][88] [IDtype] [Enable] [ID0][ID1][ID2][ID3] [SUM]
	// * RESPONSE (3): [AA][55][SUM]
	tx_buffer[0] = 0xAA;
	tx_buffer[1] = 0x55;
	tx_buffer[2] = 0x88;
	tx_buffer[3] = 0x88;
	tx_buffer[4] = 0x88;
	tx_buffer[5] = 0x88;
	tx_buffer[6] = id_type;
	tx_buffer[7] = enable;
	tx_buffer[8] = (char)((filter_id >> 0) & 0xFF);
	tx_buffer[9] = (char)((filter_id >> 8) & 0xFF);
	tx_buffer[10] = (char)((filter_id >> 16) & 0xFF);
	tx_buffer[11] = (char)((filter_id >> 24) & 0xFF);
	
	sum = 0;
	for (i=0; i<12; i++)
		sum += tx_buffer[i];
	tx_buffer[12] = (char)sum;
	
	// Write to serial
	if (!Serial_Write_Ex(serial, tx_buffer, 13)) {
		mexPrintf("Error: write serial.\n");
		return FALSE;
	}
	
	// Get response
	if (!Serial_Read_Ex(serial, rx_buffer, 3, &dwRead, 1000)) {
		mexPrintf("Error: wait for response.\n");
		return FALSE;
	}
	
	// Verify response
	if ((dwRead == 3) && (rx_buffer[0] == (char)0xBB) && (rx_buffer[1] == (char)0x66) && (rx_buffer[2] == (char)0x00))
	{
		// Valid
	}
	else {
		mexPrintf("Error: setup serial bridge response %d bytes: %X,%X,%X\n", \
				dwRead, (unsigned int)rx_buffer[0], (unsigned int)rx_buffer[1], (unsigned int)rx_buffer[2]);
		return FALSE;
	}	
	
	// No error
	return TRUE;
}

// ========================================================================
// Mex function
// ========================================================================
static char Output_Message[1024] = {0};

// CCP slave configuration
static unsigned char ccp_ctr = 0;
static unsigned int ccp_slave_id = 0;
static unsigned int ccp_slave_idtype = 1; // 0-ext, 1-std

BOOL CCP_Execute_Cmd(SERIAL_PORT_STRUCT *serial, unsigned char Cmd, const unsigned char *in_data_6, unsigned char *out_data_5)
{
	DWORD time_stamp, err_code;
	CAN_MESSAGE can_msg, can_rxmsg;
	
	can_msg.idtype = (unsigned int)ccp_slave_idtype;
	can_msg.id     = (unsigned int)ccp_slave_id;
	can_msg.dlc = 8;
	can_msg.data[0] = Cmd; // CCP_CONNECT;
	can_msg.data[1] = ccp_ctr++; // CTR
	can_msg.data[2] = in_data_6[0];
	can_msg.data[3] = in_data_6[1];
	can_msg.data[4] = in_data_6[2];
	can_msg.data[5] = in_data_6[3];
	can_msg.data[6] = in_data_6[4];
	can_msg.data[7] = in_data_6[5];
	
	// Transmit
	if (! cantx(serial, &can_msg, &time_stamp, &err_code)) {
		mexPrintf("Error: execute cmd-transmit.\n");
		return FALSE;
	}
	// Error Code	
	if (err_code != 0) {
		mexPrintf("Error: execute, code: \n", err_code);
		return FALSE;
	}
	// Response
	if (! canrx( serial, 2000, &can_rxmsg, &time_stamp)) {
		mexPrintf("Error: execute, canrx\n");
		return FALSE;
	}
	// CAN Status
	if ( !((can_rxmsg.data[0] == 255) && (can_rxmsg.data[1] == 0) && (can_rxmsg.data[2] == (unsigned char)(ccp_ctr-1)))) {		
		mexPrintf("Error: Can Rx, %2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X\n", 
				can_rxmsg.data[0],
				can_rxmsg.data[1],
				can_rxmsg.data[2],
				can_rxmsg.data[3],
				can_rxmsg.data[4],
				can_rxmsg.data[5],
				can_rxmsg.data[6],
				can_rxmsg.data[7]);
		return FALSE;
	}
	// Return data
	out_data_5[0] = can_rxmsg.data[3];
	out_data_5[1] = can_rxmsg.data[4];
	out_data_5[2] = can_rxmsg.data[5];
	out_data_5[3] = can_rxmsg.data[6];
	out_data_5[4] = can_rxmsg.data[7];
	
	return TRUE;
}
	
static char PortName[32];
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[])
{
	
	
	double p_idtype;
	double p_id;
	double p_dlc;
	double *p_data;
	
	DWORD time_stamp, err_code;
	CAN_MESSAGE can_msg, can_rxmsg;
	//FT_HANDLE ftHandle = 0;
	double output_val = -1.00; // For return value
	int output_sta = 0; // For return status
	char input_cmd[64] = {0};
	char *output_buf, *input_buf;
	double *p, result = 0;
	const mwSize dims[] = { 1 };
	
	/* Init buffer */
	Output_Message[0] = '\0';
	
	/* Validate */
	if (nrhs < 1)
		mexErrMsgTxt("Invalid number of input parameter."); 
	if (nlhs > 3)
		mexErrMsgTxt("Invalid number of output."); 
	if (!mxIsChar(prhs[0]))
		mexErrMsgTxt("Input parameter 1 must be string");
	
	input_buf = mxArrayToString(prhs[0]);
	if(input_buf) {
		if(strlen(input_buf) >= 64) {
			memcpy(input_cmd, input_buf, 63);
			input_cmd[63] = '\0';
		}
		else { strcpy(input_cmd, input_buf); }
		mxFree(input_buf);
	}
	
	// Validate for specific command
	// CONFIG
	if(!strcmp(input_cmd, "CONFIG")) {
		if (nrhs != 3)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('CONFIG', <idtype>, <id>)\"");
	}
	// CONNECT
	else if(!strcmp(input_cmd, "CONNECT")) {
		if (nrhs != 3)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('CONNECT', <idtype>, <id>)\"");
	}
	// SHORT_UP
	else if(!strcmp(input_cmd, "SHORT_UP")) {
		if (nrhs != 4)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('SHORT_UP', '<datatype>', <mem_id>, <address>)\"");
	}
	// DOWNLOAD
	else if(!strcmp(input_cmd, "DOWNLOAD")) {
		if (nrhs != 5)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('DOWNLOAD', '<datatype>', <mem_id>, <address>, <value>)\"");
	}
	// UNLOCK
	else if(!strcmp(input_cmd, "UNLOCK")) {
		if (nrhs != 2)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('UNLOCK', <mask>)\"");
	}
	// PROGRAM
	else if(!strcmp(input_cmd, "PROGRAM")) {
		if (nrhs != 5)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('PROGRAM', '<datatype>', <mem_id>, <address>, <value>)\"");
	}	
	// FW_UPGRADE
	else if(!strcmp(input_cmd, "FW_UPGRADE")) {
		if (nrhs != 4)
			mexErrMsgTxt("Invalid input parameters, \"Usage: "
					"[sta,msg] = amg_canserial_bridge('FW_UPGRADE', <canid>, '<filename>', '<sectorsize>')\"");
	}
	else {
		mexErrMsgTxt("Invalid input command.");
	}
	
	
	/* ================================================================
	 * Slave configuration
	 * ================================================================
	 */
	if(!strcmp(input_cmd, "CONFIG") || !strcmp(input_cmd, "CONNECT")) {
		// Input parameter
		p_idtype = *((double *)mxGetData(prhs[1]));
		p_id = *((double *)mxGetData(prhs[2]));
		
		ccp_slave_id = (unsigned int)p_id;
		ccp_slave_idtype = (unsigned int)p_idtype;
	}
	
	
	// Init CAN Message
	can_msg.idtype  = ccp_slave_idtype;
	can_msg.id      = ccp_slave_id;
	can_msg.dlc     = 8;
	can_msg.data[0] = 0;
	can_msg.data[1] = 0;
	can_msg.data[2] = 0;
	can_msg.data[3] = 0;
	can_msg.data[4] = 0;
	can_msg.data[5] = 0;
	can_msg.data[6] = 0;
	can_msg.data[7] = 0;
	
	if(!strcmp(input_cmd, "CONFIG")) {
		
	}
	
	/* ================================================================
	 * === Command: connect ===
	 * Usage: amg_canserial_bridge('CONNECT', <idtype>, <id>)
	 *  idtype, 0 - ext, 1 - std
	 *  id, 11 or 29 bit
	 * ================================================================
	 */
	else if(!strcmp(input_cmd, "CONNECT")) {		
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};			
		BOOL status = TRUE;

		// Find serial port
		if (status) {
			status = ft231_getportname(&PortName[0]);
			if (!status)
				mexPrintf("Error: get serial port.\n");
		}
		
		// Open serial port
		if (status) {
			status = Serial_Open( &serial, PortName, TRUE, SERIAL_BAUD_RATE, 8U, NOPARITY, ONESTOPBIT);
			if (!status)
				mexPrintf("Error: open port: %s\n", PortName);
		}
		
		// Enable UART-CAN Bridge
		if (status) {
			status = setup_can_serial_bridge(&serial, (char)can_msg.idtype, 1, 0xFFFFFFFF);
		}
		
		// CONNECT
		if (status) {
			status = CCP_Execute_Cmd(&serial, CCP_CONNECT, tx_buffer, rx_buffer);
			if (!status) {
				status = FALSE;
				sprintf(Output_Message, "Error: CCP_CONNECT cmd.\n");
			}
		}
		
		// 
		if (status) {
			output_sta = 1;
			sprintf(Output_Message, "Connected.");
		}
		
		// Close serial port
		Serial_Close(&serial);
		
		if (status) {
			output_sta = 1;
		}
		else {
			sprintf(Output_Message, "Error: CCP_CONNECT.\n");
		}

	}
	/* ================================================================
	 * === Command: SHORT_UP ===
	 * Usage: amg_canserial_bridge('SHORT_UP', '<dtype>', <mem_id>, <address>)
	 *  dtype= 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32'
	 *  mem_id= 0-ID, 1-RO, 2-RW, 3-FLASH, 4-EEPROM
	 *  address= 0... (memory address to store value)
	 * ================================================================
	 */
	else if(!strcmp(input_cmd, "SHORT_UP")) {
		unsigned int address, mem_id;
		char *dtype;
		unsigned char type_id, type_size;
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
		BOOL status = TRUE;

		// Find serial port
		if (status) {
			status = ft231_getportname(&PortName[0]);
			if (!status)
				mexPrintf("Error: get serial port.\n");
		}
		
		// Open serial port
		if (status) {
			status = Serial_Open( &serial, PortName, TRUE, SERIAL_BAUD_RATE, 8U, NOPARITY, ONESTOPBIT);
			if (!status)
				mexPrintf("Error: open port.\n");
		}
		
		// Input parameters
		if (status) {
			if (mxIsChar(prhs[1]) && ((dtype = mxArrayToString(prhs[1])) != 0)) {
				mem_id = (unsigned int)(*((double *)mxGetData(prhs[2])));
				address = (unsigned int)(*((double *)mxGetData(prhs[3])));
			}
			else {
				status = FALSE;
				mexPrintf("Error: invalid parameters.\n");
			}
		}
			
		// Packing
		if (status)	{
			if (!strcmp(dtype, "single")) {
				type_id = 1; type_size = 4;
			}
			else if (!strcmp(dtype, "int8")) {
				type_id = 2; type_size = 1;
			}
			else if (!strcmp(dtype, "uint8")) {
				type_id = 3; type_size = 1;
			}
			else if (!strcmp(dtype, "int16")) {
				type_id = 4; type_size = 2;
			}
			else if (!strcmp(dtype, "uint16")) {
				type_id = 5; type_size = 2;
			}
			else if (!strcmp(dtype, "int32")) {
				type_id = 6; type_size = 4;
			}
			else if (!strcmp(dtype, "uint32")) {
				type_id = 7; type_size = 4;
			}
			else { // Not supported
				type_id = 0; type_size = 8; status = FALSE;
			}
			mxFree(dtype);
			
			// 0: Size
			tx_buffer[0] = type_size;
			// 1: Extension address
			tx_buffer[1] = (unsigned char)(mem_id & 0xFF);
			// 2-5: Address
			tx_buffer[2] = (unsigned char)(address>>24);
			tx_buffer[3] = (unsigned char)(address>>16);
			tx_buffer[4] = (unsigned char)(address>>8 );
			tx_buffer[5] = (unsigned char)(address>>0 );
		}
		
		// Execute command
		if (status) {
			status = CCP_Execute_Cmd(&serial, CCP_SHORT_UP, tx_buffer, rx_buffer);
			if (status) {
				output_sta = 1;
			}
			else {
				mexPrintf("Error: execute command CCP_SHORT_UP\n");
			}
		}
		
		// Response
		if (status) {
			switch (type_id) {
				case 1:  { // single
					float val;
					memcpy(&val, &(rx_buffer[0]), 4);
					output_val = val;
					break;
				}
				case 2: { // int8
					output_val = (double)((int8_T)rx_buffer[0]);
					break;
				}
				case 3: { // uint8
					output_val = (double)((uint8_T)rx_buffer[0]);
					break;
				}
				case 4: { // int16
					int16_T val;
					memcpy(&val, &(rx_buffer[0]), 2);
					output_val = val;
					break;
				}
				case 5: { // uint16
					uint16_T val;
					memcpy(&val, &(rx_buffer[0]), 2);
					output_val = val;
					break;
				}
				case 6: { // int32
					int32_T val;
					memcpy(&val, &(rx_buffer[0]), 4);
					output_val = val;
					break;
				}
				case 7: {// uint32
					uint32_T val;
					memcpy(&val, &(rx_buffer[0]), 4);
					output_val = val;
					break;
				}
			}
		}
		
		// Close serial port
		Serial_Close(&serial);
		
		if (status) {
			output_sta = 1;
		}
		else {
			sprintf(Output_Message, "Error: CCP_SHORT_UP.\n");
		}
	}
	
	/* ================================================================
	 * === Command: DOWNLOAD ===
	 * Usage: amg_canserial_bridge('DOWNLOAD', '<dtype>', <mem_id>, <address>, <value>)
	 *  dtype= 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32'
	 *  mem_id= 0-ID, 1-RO, 2-RW, 3-FLASH, 4-EEPROM
	 *  address= 0... (memory address to store value)
	 *  value = value to upload
	 * ================================================================
	 */
	else if(!strcmp(input_cmd, "DOWNLOAD")) {
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		BOOL status;
		char *dtype;
		unsigned char type_id, type_size;
		unsigned int address;
		unsigned int mem_id;
		double value;
		
		// Initial status
		status = TRUE;
		
		// Find serial port
		if (status) {
			status = ft231_getportname(&PortName[0]);
			if (!status)
				mexPrintf("Error: get serial port.\n");
		}
		
		// Open serial port
		if (status) {
			status = Serial_Open( &serial, PortName, TRUE, SERIAL_BAUD_RATE, 8U, NOPARITY, ONESTOPBIT);
			if (!status)
				mexPrintf("Error: open port.\n");
		}
		
		if (status) {
			// Pack message fro SET_MTA
			if (mxIsChar(prhs[1]) && ((dtype = mxArrayToString(prhs[1])) != 0)) {
				
				mem_id = (unsigned int)(*((double *)mxGetData(prhs[2])));
				address = (unsigned int)(*((double *)mxGetData(prhs[3])));
				value = *((double *)mxGetData(prhs[4]));
				
				// 0: Header
				can_msg.data[0] = CCP_SET_MTA;
				// 1: CTR
				can_msg.data[1] = ccp_ctr++; // CTR
				
				if (!strcmp(dtype, "single")) {
					type_id = 1; type_size = 4;
				}
				else if (!strcmp(dtype, "int8")) {
					type_id = 2; type_size = 1;
				}
				else if (!strcmp(dtype, "uint8")) {
					type_id = 3; type_size = 1;
				}
				else if (!strcmp(dtype, "int16")) {
					type_id = 4; type_size = 2;
				}
				else if (!strcmp(dtype, "uint16")) {
					type_id = 5; type_size = 2;
				}
				else if (!strcmp(dtype, "int32")) {
					type_id = 6; type_size = 4;
				}
				else if (!strcmp(dtype, "uint32")) {
					type_id = 7; type_size = 4;
				}
				else { // Not supported
					type_id = 0;
					type_size = 8;
					status = FALSE;
				}
				// Free memory
				mxFree(dtype);
			} else {
				status = FALSE;
				sprintf(Output_Message, "Error: data type must be a string.");
			}
		}
		
		// SET_MTA
		if (status) {
			unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
			
			// Set MTA
			tx_buffer[0] = 0; // MTA0
			tx_buffer[1] = (unsigned char)(mem_id & 0xFF);
			tx_buffer[2] = (unsigned char)(address>>24); // Add3
			tx_buffer[3] = (unsigned char)(address>>16); // Add2
			tx_buffer[4] = (unsigned char)(address>>8 ); // Add1
			tx_buffer[5] = (unsigned char)(address>>0 ); // Add0
			status = CCP_Execute_Cmd(&serial, CCP_SET_MTA, tx_buffer, rx_buffer);
			if (!status) {
				status = FALSE;
				sprintf(Output_Message, "Error: set MTA.\n");
			}
		}
		
		// DOWNLOAD
		if (status) {
			real32_T val_f;
			int8_T val_i8;
			uint8_T val_u8;
			int16_T val_i16;
			uint16_T val_u16;
			int32_T val_i32;
			uint32_T val_u32;
			unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
			
			tx_buffer[0] = type_size; // Size of data
			switch (type_id) {
				case 1:
					val_f = (real32_T)value;
					memcpy(&tx_buffer[1], &val_f, 4);
					break;
				case 2:
					tx_buffer[1] = (int8_T)value;
					break;
				case 3:
					tx_buffer[1] = (uint8_T)value;
					break;
				case 4:
					val_i16 = (int16_T)value;
					memcpy(&tx_buffer[1], &val_i16, 2);
					break;
				case 5:
					val_u16 = (uint16_T)value;
					memcpy(&tx_buffer[1], &val_u16, 2);
					break;
				case 6:
					val_i32 = (int32_T)value;
					memcpy(&tx_buffer[1], &val_i32, 4);
					break;
				case 7:
					val_u32 = (uint32_T)value;
					memcpy(&tx_buffer[1], &val_u32, 4);
					break;
				default:
					break;
			}
			
			status = CCP_Execute_Cmd(&serial, CCP_DNLOAD, tx_buffer, rx_buffer);
			if (!status) {
				status = FALSE;
				sprintf(Output_Message, "Error: DNLOAD.\n");
			}
		}
		
		// Close serial port
		Serial_Close(&serial);

		
		if (status) {
			output_sta = 1;
		}
	}
	
	/* ================================================================
	 * === Command: PROGRAM ===
	 * Usage: amg_canserial_bridge('PROGRAM', '<dtype>', <mem_id>, <address>, <value>)
	 *  dtype= 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32'
	 *  mem_id= 3-FLASH, 4-EEPROM
	 *  address= 0... (memory address to store value)
	 *  value = value to upload
	 * ================================================================
	 */
	else if(!strcmp(input_cmd, "PROGRAM")) {
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		BOOL status;
		char *dtype;
		unsigned char type_id, type_size;
		unsigned int address;
		unsigned int mem_id;
		double value;
		
		// Initial status
		status = TRUE;

		// Find serial port
		if (status) {
			status = ft231_getportname(&PortName[0]);
			if (!status)
				mexPrintf("Error: get serial port.\n");
		}
		
		// Open serial port
		if (status) {
			status = Serial_Open( &serial, PortName, TRUE, SERIAL_BAUD_RATE, 8U, NOPARITY, ONESTOPBIT);
			if (!status)
				mexPrintf("Error: open port.\n");
		}
		
		if (status) {
			// Pack message fro SET_MTA
			if (mxIsChar(prhs[1]) && ((dtype = mxArrayToString(prhs[1])) != 0)) {
				
				mem_id = (unsigned int)(*((double *)mxGetData(prhs[2])));
				address = (unsigned int)(*((double *)mxGetData(prhs[3])));
				value = *((double *)mxGetData(prhs[4]));
				
				// 0: Header
				can_msg.data[0] = CCP_SET_MTA;
				// 1: CTR
				can_msg.data[1] = ccp_ctr++; // CTR
				
				if (!strcmp(dtype, "single")) {
					type_id = 1; type_size = 4;
				}
				else if (!strcmp(dtype, "int8")) {
					type_id = 2; type_size = 1;
				}
				else if (!strcmp(dtype, "uint8")) {
					type_id = 3; type_size = 1;
				}
				else if (!strcmp(dtype, "int16")) {
					type_id = 4; type_size = 2;
				}
				else if (!strcmp(dtype, "uint16")) {
					type_id = 5; type_size = 2;
				}
				else if (!strcmp(dtype, "int32")) {
					type_id = 6; type_size = 4;
				}
				else if (!strcmp(dtype, "uint32")) {
					type_id = 7; type_size = 4;
				}
				else { // Not supported
					type_id = 0;
					type_size = 8;
					status = FALSE;
				}
				// Free memory
				mxFree(dtype);
			} else {
				status = FALSE;
				sprintf(Output_Message, "Error: data type must be a string.");
			}
		}
		// SET_MTA
		if (status) {
			unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
			
			// Set MTA
			tx_buffer[0] = 0; // MTA0
			tx_buffer[1] = (unsigned char)(mem_id & 0xFF);
			tx_buffer[2] = (unsigned char)(address>>24); // Add3
			tx_buffer[3] = (unsigned char)(address>>16); // Add2
			tx_buffer[4] = (unsigned char)(address>>8 ); // Add1
			tx_buffer[5] = (unsigned char)(address>>0 ); // Add0
			status = CCP_Execute_Cmd(&serial, CCP_SET_MTA, tx_buffer, rx_buffer);
			if (!status) {
				status = FALSE;
				sprintf(Output_Message, "Error: set MTA.\n");
			}
		}
		
		// PROGRAM
		if (status) {
			real32_T val_f;
			int8_T val_i8;
			uint8_T val_u8;
			int16_T val_i16;
			uint16_T val_u16;
			int32_T val_i32;
			uint32_T val_u32;
			unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
			
			tx_buffer[0] = type_size; // Size of data
			switch (type_id) {
				case 1:
					val_f = (real32_T)value;
					memcpy(&tx_buffer[1], &val_f, 4);
					break;
				case 2:
					tx_buffer[1] = (int8_T)value;
					break;
				case 3:
					tx_buffer[1] = (uint8_T)value;
					break;
				case 4:
					val_i16 = (int16_T)value;
					memcpy(&tx_buffer[1], &val_i16, 2);
					break;
				case 5:
					val_u16 = (uint16_T)value;
					memcpy(&tx_buffer[1], &val_u16, 2);
					break;
				case 6:
					val_i32 = (int32_T)value;
					memcpy(&tx_buffer[1], &val_i32, 4);
					break;
				case 7:
					val_u32 = (uint32_T)value;
					memcpy(&tx_buffer[1], &val_u32, 4);
					break;
				default:
					break;
			}
			
			status = CCP_Execute_Cmd(&serial, CCP_PROGRAM, tx_buffer, rx_buffer);
			if (!status) {
				status = FALSE;
				sprintf(Output_Message, "Error: PROGRAM.\n");
			}
		}
		
		// Close serial port
		Serial_Close(&serial);

		if (status) {
			output_sta = 1;
		}
	}
	
	/* ================================================================
	 * === Command: UNLOCK ===
	 * Usage: amg_canserial_bridge('UNLOCK', <mask>)
	 *  mask - 0x40 -> Flash/EEPROM Programing
	 *  mask - 0x02 -> DAQ
	 *  mask - 0x01 -> Calbration
	 * ================================================================
	 */
	else if(!strcmp(input_cmd, "UNLOCK")) {
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		BOOL status;
		uint8_T mask;
		unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
		
		// Initial status
		status = TRUE;
		
		// Find serial port
		if (status) {
			status = ft231_getportname(&PortName[0]);
			if (!status)
				mexPrintf("Error: get serial port.\n");
		}
		
		// Open serial port
		if (status) {
			status = Serial_Open( &serial, PortName, TRUE, SERIAL_BAUD_RATE, 8U, NOPARITY, ONESTOPBIT);
			if (!status)
				mexPrintf("Error: open port.\n");
		}
		
		mask = (uint8_T)(*((double *)mxGetData(prhs[1])));
		
		// Get seed
		if (status) {
			// Get seed
			tx_buffer[0] = mask; // Mask
			status = CCP_Execute_Cmd(&serial, CCP_GET_SEED, tx_buffer, rx_buffer);
			if (!status) {
				sprintf(Output_Message, "Failed to get seed data.\n");
			}
		}
		// Unlock
		if (status) {
			tx_buffer[0] = 0;
			tx_buffer[1] = 0;
			tx_buffer[2] = rx_buffer[1];
			tx_buffer[3] = rx_buffer[2];
			tx_buffer[4] = rx_buffer[3];
			tx_buffer[5] = rx_buffer[4];
			status = CCP_Execute_Cmd(&serial, CCP_UNLOCK, tx_buffer, rx_buffer);
			if (!status || ((rx_buffer[0] & 0x40)==0)) {
				status = FALSE;
				sprintf(Output_Message, "Failed to Unlock PGM.\n");
			}
		}
		
		// Close serial port
		Serial_Close(&serial);

		
		if (status) {
			output_sta = 1;
		}
	}
	
	/* ================================================================
	 * === Command: FW_UPGRADE ===
	 * Usage: [sta,msg] = amg_canserial_bridge('FW_UPGRADE', <canid>, '<filename>', '<sectorsize>')
	 *  filename = 'can_sensor_demo.bin'
	 *  sectorsize = '52k'
	 * ================================================================
	 */
	else if(!strcmp(input_cmd, "FW_UPGRADE")) {
		SERIAL_PORT_STRUCT serial = {NULL, NULL, ERROR_SUCCESS, EV_BREAK|EV_ERR|EV_RXFLAG, 2048, 2048};
		unsigned int sector_size, crc_value;
		int len;
		BOOL status;
		char *filename, *s;
		char input_sector_size[32];
		//unsigned char type_id, type_size;
		char Flash_Buffer[256*1024];
		unsigned char tx_buffer[6] = {0, 0, 0, 0, 0, 0}, rx_buffer[5] = {0, 0, 0, 0, 0};
		
		// Initial status

		status = TRUE;
		
		// Find serial port
		if (status) {
			status = ft231_getportname(&PortName[0]);
			if (!status)
				mexPrintf("Error: get serial port.\n");
			else
				mexPrintf("Port: %s\n", PortName);
		}
		
		// Open serial port
		if (status) {
			status = Serial_Open( &serial, PortName, TRUE, SERIAL_BAUD_RATE, 8U, NOPARITY, ONESTOPBIT);
			if (!status)
				mexPrintf("Error: open port.\n");
		}

		if (status) {
			// CAN ID
			ccp_slave_id = (unsigned int)(*((double *)mxGetData(prhs[1])));
			//ccp_slave_id |= 0x200; // Set Programming ID bit
			
			/* Sector Size */
			s = mxArrayToString(prhs[3]);
			len = strlen(s);
			if (len >= sizeof(input_sector_size))
				len = (sizeof(input_sector_size) - 1);
			strncpy(input_sector_size, s, len);
			input_sector_size[len] = '\0';
			mxFree(s);
			if (!strncmp(input_sector_size, "0x", 2) || !strncmp(input_sector_size, "0X", 2)) { // Input is in hex
				if (sscanf(input_sector_size, "0x%X", &sector_size) != 1) {
					//mexErrMsgTxt("Sector size is invalid (parameter 2)");
					strcpy(Output_Message, "Sector size is invalid (parameter 2)");
					status = FALSE;
				}
			}
			else if (strstr(input_sector_size, "k") || strstr(input_sector_size, "K")) { // Input is in kilo bytes
				if (sscanf(input_sector_size, "%uk", &sector_size) != 1) {
					//mexErrMsgTxt("Sector size is invalid (parameter 2)");
					strcpy(Output_Message, "Sector size is invalid (parameter 2)");
					status = FALSE;
				}
				else {
					sector_size *= 1024;
				}
			}
			else { // Expect a decimal string
				if (sscanf(input_sector_size, "%u", &sector_size) != 1) {
					//mexErrMsgTxt("Sector size is invalid (parameter 2)");
					strcpy(Output_Message, "Sector size is invalid (parameter 2)");
					status = FALSE;
				}
			}
			if ((sector_size < 4) || (sector_size>sizeof(Flash_Buffer))) {
				//mexErrMsgTxt("Sector size is invalid (parameter 2)");
				strcpy(Output_Message, "Sector size is invalid (parameter 2)");
				status = FALSE;
			}
			/* Validate sector size */
			if ((sector_size > sizeof(Flash_Buffer)) || ((sector_size & 0x3FF) != 0)) {
				status =FALSE;
				sprintf(Output_Message, "Invalid sector size, max 256kBytes and must be multiple of 2048.\n");
			}
		}
		// Filename
		if (status && mxIsChar(prhs[2]) && ((filename = mxArrayToString(prhs[2])) != 0)) {
			// Load firmware from file
			memset(Flash_Buffer, 0xFF, sizeof(Flash_Buffer));
			if (!load_file_tobuffer(filename, Flash_Buffer, sizeof(Flash_Buffer))) {
				status = FALSE;
				sprintf(Output_Message, "Failed to load from file: \"%s\"\n", filename);
			}
			if (status) {
				unsigned int embedded_crc;
				// Calculate CRC
				crc_value = crc32_update(0xFFFFFFFF, (unsigned int *)&Flash_Buffer[0], (sector_size>>2)-1);
				
				// Embedded CRC into buffer
				//Flash_Buffer[sector_size-4] = (char)((crc_value>>0 ) & 0xFF);
				//Flash_Buffer[sector_size-3] = (char)((crc_value>>8 ) & 0xFF);
				//Flash_Buffer[sector_size-2] = (char)((crc_value>>16) & 0xFF);
				//Flash_Buffer[sector_size-1] = (char)((crc_value>>24) & 0xFF);
				
				embedded_crc = *((unsigned int *)(&Flash_Buffer[sector_size-4]));
				if (embedded_crc != crc_value) {
					status = 0;
					sprintf(Output_Message, "Invalid firmware CRC.\n");
				}
			}
			
			// Enable CAN-Serial bridge
			if (status) {
				status = setup_can_serial_bridge(&serial, (char)can_msg.idtype, 1, 0xFF); // Slave return ID
			}
			
			// CAN connect
			if (status) {
				status = CCP_Execute_Cmd(&serial, CCP_CONNECT, tx_buffer, rx_buffer);
				if (!status)
					sprintf(Output_Message, "Failed to Connect to CAN slave.\n");
			}
			// Get seed
			if (status) {
				// Get seed
				tx_buffer[0] = 0x40; // Mask
				status = CCP_Execute_Cmd(&serial, CCP_GET_SEED, tx_buffer, rx_buffer);
				if (!status)
					sprintf(Output_Message, "Failed to get seed data.\n");
			}
			// Unlock
			if (status) {
				tx_buffer[0] = 0;
				tx_buffer[1] = 0;
				tx_buffer[2] = rx_buffer[1];
				tx_buffer[3] = rx_buffer[2];
				tx_buffer[4] = rx_buffer[3];
				tx_buffer[5] = rx_buffer[4];
				status = CCP_Execute_Cmd(&serial, CCP_UNLOCK, tx_buffer, rx_buffer);
				if (!status || ((rx_buffer[0] & 0x40)==0)) {
					status = FALSE;
					sprintf(Output_Message, "Failed to Unlock PGM.\n");
				}
			}
			// Erase memory
			if (status) {
				unsigned int mta = 0;
				
				// Set MTA
				tx_buffer[0] = 0; // MTA0
				tx_buffer[1] = 3; // 3-Flash
				tx_buffer[2] = 0; // Add3
				tx_buffer[3] = 0; // Add2
				tx_buffer[4] = 0; // Add1
				tx_buffer[5] = 0; // Add0
				status = CCP_Execute_Cmd(&serial, CCP_SET_MTA, tx_buffer, rx_buffer);
				if (!status)
					sprintf(Output_Message, "Failed to set MTA for flash erase.\n");
				// Erase memory
				tx_buffer[0] = (unsigned char)(sector_size >> 24); // Size 3
				tx_buffer[1] = (unsigned char)(sector_size >> 16); // Size 2
				tx_buffer[2] = (unsigned char)(sector_size >> 8 ); // Size 1
				tx_buffer[3] = (unsigned char)(sector_size >> 0 ); // Size 0
				tx_buffer[4] = 0; // Don't care
				tx_buffer[5] = 0; // Don't care
				status = CCP_Execute_Cmd(&serial, CCP_CLEAR_MEMORY, tx_buffer, rx_buffer);
				if (!status)
					sprintf(Output_Message, "Failed to set erase flash erase.\n");
			}
			// CCP upgrade process
			if (status) {
				unsigned int mta = 0, rem;
				// Set MTA
				tx_buffer[0] = 0; // MTA0
				tx_buffer[1] = 3; // 3-Flash
				tx_buffer[2] = 0; // Add3
				tx_buffer[3] = 0; // Add2
				tx_buffer[4] = 0; // Add1
				tx_buffer[5] = 0; // Add0
				status = CCP_Execute_Cmd(&serial, CCP_SET_MTA, tx_buffer, rx_buffer);
				if (!status)
					sprintf(Output_Message, "Failed to set MTA for Program.\n");
				
				// Program 6 Byte
				rem = sector_size;
				while (status && (rem > 0)) {
					mexPrintf("Address: %X\n", mta);
					
					// Data 6 bytes
					if (rem >= 6) {						
						tx_buffer[0] = Flash_Buffer[mta ++];
						tx_buffer[1] = Flash_Buffer[mta ++];
						tx_buffer[2] = Flash_Buffer[mta ++];
						tx_buffer[3] = Flash_Buffer[mta ++];
						tx_buffer[4] = Flash_Buffer[mta ++];
						tx_buffer[5] = Flash_Buffer[mta ++];
						rem -= 6;
						// Program6
						status = CCP_Execute_Cmd(&serial, CCP_PROGRAM_6, tx_buffer, rx_buffer);
						if (!status)
							sprintf(Output_Message, "Failed to sent data to Program, CCP_PROGRAM_6: %X.\n", mta-6);
					}
					else {
						tx_buffer[0] = (unsigned char)rem;
						memcpy(&(tx_buffer[1]), &Flash_Buffer[mta], rem);
						mta += rem;
						
						
						// Program6
						status = CCP_Execute_Cmd(&serial, CCP_PROGRAM, tx_buffer, rx_buffer);
						if (!status)
							sprintf(Output_Message, "Failed to sent data to Program, CCP_PROGRAM: %X.\n", mta-rem);
						
						rem = 0;
					}
					
					if (status) {
						unsigned int postincaddr;
						
						postincaddr = ((unsigned int)rx_buffer[1] << 24) \
								| ((unsigned int)rx_buffer[2] << 16) \
								| ((unsigned int)rx_buffer[3] << 8) \
								| ((unsigned int)rx_buffer[4] << 0);
						if (!((rx_buffer[0] == 3 /* Flash */) && (postincaddr == mta))) {
							status = FALSE;
							sprintf(Output_Message, "Failed to Program 6 byte.\n");
						}
					}
				}
			}
			
			// Disconnect
			if (status) {
				mexPrintf("Disconnect CCP from CCP device.\n");
				status = CCP_Execute_Cmd(&serial, CCP_DISCONNECT, tx_buffer, rx_buffer);
				if (!status)
					sprintf(Output_Message, "Failed to send Disconnect command.\n");
			}
			else {
				
			}
			
			// Release CAN ID filter
			// Enable CAN-Serial bridge
			if (! setup_can_serial_bridge(&serial, (char)can_msg.idtype, 1, 0xFFFFFFFF)) {
				sprintf(&Output_Message[strlen(Output_Message)], "\nFailed to reset CAN filter ID.\n");
			}
		}
		else {
			//status = FALSE;
			//sprintf(Output_Message, "Invalid filename.\n");
		}
		
		// Close serial port
		Serial_Close(&serial);
		
		// Finallize
		if (status) {
			output_sta = 1;
			sprintf(&Output_Message[strlen(Output_Message)], "Success.\n");
		}
	}
	/* === Unknown === */
	else {
		strcpy(Output_Message, "Error: Unknown command.");
	}

	mexPrintf("%s\n", Output_Message);
	/* ========================
	 * Close port
	 */
	
	/* Return */
	plhs[0] = mxCreateDoubleScalar (output_sta);
	if (nlhs > 1) {
		output_buf = mxCalloc(1024, sizeof(char));
		strcpy(output_buf, Output_Message);
		plhs[1] = mxCreateString(output_buf);
	}
	if (nlhs > 2) {
		plhs[2] = mxCreateDoubleScalar (output_val);
	}
	return;
}
