#define S_FUNCTION_NAME  amg_usbconverter_n_connect
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
	ARGC_CONF = 0,

	ARGC_AUTOPORT,
	ARGC_BAUDRATE,
	ARGC_DATABITS,
	ARGC_PARITY,
	ARGC_STOPBIT,
	ARGC_FLOWCONTROL,
	ARGC_TRANSFER,
	ARGC_TIMEOUT,
	ARGC_PACKETMODE,
	ARGC_INITIALVALUES,
	ARGC_BINHEADER,
	ARGC_BINTERMINATOR,
	ARGC_ASCIIFORMAT,
	ARGC_TX_TERMINATOR,
	ARGC_RX_TERMINATOR,
    
    ARGC_INPUT_PORTTYPE,
    ARGC_INPUT_PORTWIDTH,
    ARGC_OUTPUT_PORTTYPE,
    ARGC_OUTPUT_PORTWIDTH,
    
    ARGC_OPTIONSTRING,
    
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
	
	ARGC_RESERVED1,
	ARGC_RESERVED2,
	ARGC_RESERVED3,
	ARGC_RESERVED4,
	ARGC_RESERVED5,
    
    __PARAM_COUNT
};

#define ENABLE_ISR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_ISR_ENABLE))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int input_count;
    int output_count;
    int input_width_count;
    int output_width_count;
	int width;
    
    int priority = 0;
    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* Port count */
    input_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE));
    output_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE));
    input_width_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH));
    output_width_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_PORTWIDTH));
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
    
    /* Port */
    for(k=0; k<output_count; k++) {		
        if(k<output_width_count) {
			width = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTWIDTH)))[k]);
            ssSetOutputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetOutputPortWidth(S, k, 1);
		}
        ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE)))[k]));
    }
    
    for(k=0; k<input_count; k++) {		
        ssSetInputPortDirectFeedThrough(S, k, 1);
		ssSetInputPortComplexSignal(S,  k, COMPLEX_NO);
		ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
        if(k<input_width_count) {
			width = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH)))[k]);
            ssSetInputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetInputPortWidth(S, k, 1);
		}
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE)))[k]));
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLETIME(S));
} /* end mdlInitializeSampleTimes */

/*
#define MDL_ENABLE
#if defined(MDL_ENABLE) && defined(MATLAB_MEX_FILE)
void mdlEnable(SimStruct *S){
}
#endif

#define MDL_DISABLE
#if defined(MDL_DISABLE) && defined(MATLAB_MEX_FILE)
static void mdlDisable(SimStruct *S){
}
#endif
*/

static int get_list_count(char *s)
{
    char *p;
    int count;
    
    count = 1;
    p = s;
    while((p=strstr(p, ",")) != (void*)0) {
        count ++;
        p++;
    }
    return count;
}

#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
	int NOutputPara = 3; /* Number of parameters to output to model.rtw */
	
	char *conf; // ARCG_CONF
	char* optionstring;
	char *sampletimestr;
	char *blockid;
	
	conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONF));
	optionstring = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_OPTIONSTRING));
	blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
	
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "conf", conf,
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", blockid
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	
	/* Write configuration string */
	if (!ssWriteRTWStrVectParam(S, "optionstring", optionstring, get_list_count(optionstring))){
		return;
	}
	
	mxFree(conf);
	mxFree(optionstring);
	mxFree(blockid);
}

/* ========================================================================
 * Serial library
 * ========================================================================
 */

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
	if (serial == NULL)
		return TRUE;

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
	char portname[32];
	int portnumber;
	// Reset error state
	serial->lLastError = ERROR_SUCCESS;

	// Check if the port isn't already opened
	if (serial->hHandle) {
		serial->lLastError = ERROR_ALREADY_INITIALIZED;
		return FALSE;
	}
	
	// Get port number
	if (sscanf(port, "COM%d", &portnumber) != 1)
		return FALSE;
	if (portnumber > 9) {
		sprintf_s(portname, 32, "\\\\.\\COM%d", portnumber);
	}
	else {
		sprintf_s(portname, 32, "COM%d", portnumber);
	}

	// Open the device
	serial->hHandle = CreateFile(portname,
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

/*
BOOL Serial_Write_Ex (SERIAL_PORT_STRUCT *serial, const char* pData, DWORD dwLen)
{
	DWORD dwWritten;
	
	// Flush buffer
	
	// Write
	if (!Serial_Write (serial, pData, dwLen, &dwWritten, 1000))
		return FALSE;
	
	return (dwWritten == dwLen);
}
*/

/*
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
*/

/* ========================================================================
 * Serial management
 * ========================================================================
 */
#define SERIAL_BUFFER_SIZE 4096 // Must be 2^N
#define SERIAL_PORT_COUNT 16
// List
typedef struct {
	char port[32]; // COMx
	SERIAL_PORT_STRUCT object; // Serial port object
	char txbuffer [SERIAL_BUFFER_SIZE];
	char rxbuffer [SERIAL_BUFFER_SIZE];
	unsigned int rx_index;
	unsigned int rx_count;	
	unsigned int tx_count;	
} SERIAL_OBJECT;

SERIAL_OBJECT SerialObjectList[SERIAL_PORT_COUNT] =
{
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
	{{0}, {0,0,0,0,0,0}, {0}, {0}, 0, 0, 0},
};

void SerialObjectList_CloseAll(void)
{
	int i;
	for (i=0; i<SERIAL_PORT_COUNT; i++) {
		Serial_Close(&(SerialObjectList[i].object));
		SerialObjectList[i].port[0] = '\0';
		SerialObjectList[i].rx_index = 0;
		SerialObjectList[i].rx_count = 0;
		SerialObjectList[i].tx_count = 0;
	}
}

SERIAL_OBJECT *SerialObjectList_Open(const char *port, DWORD BaudRate, BYTE DataBits, BYTE Parity, BYTE StopBits)
{
	int i, available_idx;
	
	available_idx = -1;
	for (i=0; i<SERIAL_PORT_COUNT; i++) {
		if ((available_idx < 0) && (SerialObjectList[i].port[0] == '\0'))
			available_idx = i;
		if (!strcmp(port, SerialObjectList[i].port)) {
			return &(SerialObjectList[i]);
		}
	}
	if (available_idx < 0) {
		mexPrintf("Unavailable serial port slot!\n");
		return NULL;
	}
	// Add and open new port
	if (Serial_Open (&(SerialObjectList[available_idx].object), port, TRUE, BaudRate, DataBits, Parity, StopBits)) {
		strcpy(&(SerialObjectList[available_idx].port[0]), port);
		SerialObjectList[available_idx].rx_index = 0;
		SerialObjectList[available_idx].rx_count = 0;		
		SerialObjectList[available_idx].tx_count = 0;		
		return &(SerialObjectList[available_idx]);
	}
	
	// Error
	return NULL;
}

BOOL SerialObjectList_Transmit(SERIAL_OBJECT *serial_object)
{
	DWORD dwWritten;
	if (!Serial_Write(&(serial_object->object), &(serial_object->txbuffer[0]), serial_object->tx_count, &dwWritten, 1000))
		return FALSE;
	
	if (dwWritten != serial_object->tx_count)
		return FALSE;
	
	return TRUE;
}

BOOL SerialObjectList_Receive(SERIAL_OBJECT *serial_object)
{
	DWORD dwRead;
	BYTE *rxbuffer;
	DWORD rxbuffer_len;
	
	rxbuffer = (BYTE *)&(serial_object->rxbuffer[serial_object->rx_count]);
	rxbuffer_len = SERIAL_BUFFER_SIZE-serial_object->rx_count;
	
    // ADDED:
    if (rxbuffer_len <= 0)
        return FALSE;
        
	// Read from Serial port
	if (!Serial_Read (&(serial_object->object), rxbuffer, rxbuffer_len, &dwRead, 1000))
		return FALSE;
	serial_object->rx_count += dwRead;
	serial_object->rx_count &= (SERIAL_BUFFER_SIZE-1);

	return TRUE;	
}
/* ========================================================================
 * Packet management
 * ========================================================================
 */
/* Data read structure */
typedef struct {
	DWORD firststep; /* Detect first step for initial value */
	DWORD index; /* Index of data in buffer */
	DWORD count; /* Return data count */
	char *buffer; /* Return buffer pointer of valid data */
	
	// Link-list
	void *next;
	char *blockid;
} BUFFER_READ_STRUCT;

static BUFFER_READ_STRUCT *global_read_struct = NULL; //{0, 0, 0,(BYTE *)0, (void *)0};
BUFFER_READ_STRUCT *get_read_struct_by_id (const char *blockid)
{
	BUFFER_READ_STRUCT *p;
	
	if (!blockid) {
		mexPrintf("NULL value of \"blockid\".");
		return NULL;
	}
	// Search list
	p = global_read_struct;
	while (p!= NULL) {
		if (!(p->blockid)) {
			mexPrintf("NULL value of \"p->blockid\".");
			return NULL;
		}
		if (!strcmp(p->blockid, blockid))
			return p;
		p = (BUFFER_READ_STRUCT *)(p->next);
	}
	return NULL;
}

void read_struct_add_list(BUFFER_READ_STRUCT *read_struct)
{
	BUFFER_READ_STRUCT *p;
	p = global_read_struct;
	if (p == NULL) {
		// First item in the list
		global_read_struct = read_struct;
	}
	else {
		while (p->next != NULL)
			p = (BUFFER_READ_STRUCT *)(p->next);
		p->next = read_struct;
	}
}

void read_struct_clear_list(void)
{
	// Fixed:
	// Crash error, change mxFree to free
	// Make sure list is clear
	BUFFER_READ_STRUCT *p;
	p = global_read_struct;	
	while (p != NULL) {
		if (p->blockid) {
			free((p->blockid));
		}
		global_read_struct = p;
		p = (BUFFER_READ_STRUCT *)(p->next);
		free(global_read_struct);
	}
	global_read_struct = NULL;
}

BUFFER_READ_STRUCT *SerialRx_GetReadStruct (SimStruct *S)
 {
	BUFFER_READ_STRUCT *p;
	char *blockid;	
	
	// Get blcokid
	blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));	
	
	p = get_read_struct_by_id (blockid);
	if (p == NULL) {
		p = (BUFFER_READ_STRUCT *)malloc(sizeof(BUFFER_READ_STRUCT)); //mxMalloc
		memset(p, 0, sizeof(BUFFER_READ_STRUCT));
		p->blockid = (char *)malloc(strlen(blockid)+1);// blockid; //mxMalloc
		strcpy(p->blockid, blockid);
		p->firststep = 1; // Activate first step
		// Add to list
		read_struct_add_list(p);
	}
	//else {
	mxFree(blockid);
	//}
	return p;
	
/*
	mxArray *struct_array;
	struct_array = mexGetVariable("global", blockid);
	if (struct_array == NULL) {
		global_read_struct.index = 0;
		global_read_struct.count = 0;
		global_read_struct.firststep = 1;
		global_read_struct.buffer = (BYTE *)0;
	}
	else {
		global_read_struct.index = (DWORD)(mxGetPr(struct_array)[0]);
		global_read_struct.count = (DWORD)(mxGetPr(struct_array)[1]);
		global_read_struct.firststep = (DWORD)(mxGetPr(struct_array)[2]);
		global_read_struct.buffer = (BYTE *)0;
		
		//
		mxDestroyArray(struct_array);
	}

	return &global_read_struct;
 */
}
/*

BOOL SerialRx_SetReadStruct(SimStruct *S, BUFFER_READ_STRUCT *read_struct)
 {
	int status;
	mxArray *struct_array;
	
	struct_array = mexGetVariable("global", blockid);
	if (struct_array == NULL) {
		struct_array = mxCreateDoubleMatrix(1, 3, mxREAL);
	}
	mxGetPr(struct_array)[0] = (double)read_struct->index;
	mxGetPr(struct_array)[1] = (double)read_struct->count;
	mxGetPr(struct_array)[2] = (double)read_struct->firststep;
	status=mexPutVariable("global", blockid, struct_array);
	if (status==1){
		mexPrintf("Error: storing variable %s at global workspace.\n", blockid);
		//mexErrMsgIdAndTxt( "MATLAB:mexgetarray:errorSettingGlobal",
		//		"Could not put variable in base workspace.\n");
	}
	
	// Free resource
	mxDestroyArray(struct_array);
	
	if (status == 0)
		return TRUE;
	else
		return FALSE;
}
*/

void SerialRx_RestoreBytes(SERIAL_OBJECT *serial_object, BUFFER_READ_STRUCT *read_struct, DWORD count) {
	DWORD roll_count = count;
	
	/* Remove overflow buffer */
	while(roll_count > SERIAL_BUFFER_SIZE)
		roll_count -= SERIAL_BUFFER_SIZE;
	
	/* Return bytes back into buffer */
	if(roll_count > read_struct->index)
		read_struct->index = SERIAL_BUFFER_SIZE - roll_count + read_struct->index;
	else
		read_struct->index -= roll_count;
}

/* Read buffer from DMA
 ** Return value: Number of bytes vaiable.
 */
void SerialRx_Read(SERIAL_OBJECT *serial_object, BUFFER_READ_STRUCT *read_struct) {
	volatile DWORD data_int_curr_count;
	
	DWORD data_index = 0;
	DWORD data_count = 0;
	
	/* Current data received count of Rx-buffer */
	data_int_curr_count = serial_object->rx_count; //UART%<uartmodule>_Rx_Count;
	
	/* Read single part of data buffer */
	if(read_struct->index < data_int_curr_count) { /* Data is available */
		data_index = read_struct->index;
		data_count = data_int_curr_count - read_struct->index;
		read_struct->index += data_count;
		read_struct->index &= (SERIAL_BUFFER_SIZE-1);
	}
	else if(read_struct->index > data_int_curr_count) { /* Data is available with overlap */
		data_index = read_struct->index;
		data_count = SERIAL_BUFFER_SIZE-read_struct->index;
		read_struct->index = 0;
	}
	else { /* No new data */
	}
	
	/* Return the reading */
	if(data_count > 0) {
		read_struct->buffer = (char *)&(serial_object->rxbuffer[data_index]);
		read_struct->count = data_count;
	}
	else { read_struct->count = 0; }
}

void SerialRx_ReadEx(SERIAL_OBJECT *serial_object, \
		BUFFER_READ_STRUCT *read_struct, \
		char *buffer, DWORD buffer_size, DWORD*reading_count) {
	DWORD bytes_to_read, data_read_index;
	
	bytes_to_read = buffer_size; /* Tracking count of data readings */
	data_read_index = 0; /* Increment buffer index */
	do {
		SerialRx_Read(serial_object, read_struct);
		if(read_struct->count <= bytes_to_read) {
			memcpy(&buffer[data_read_index], read_struct->buffer, read_struct->count);
			data_read_index += read_struct->count;
			bytes_to_read -= read_struct->count;
		}
		else {
			/* Return some byte back to buffer */
			//read_struct->index -= (read_struct->count - bytes_to_read);
			SerialRx_RestoreBytes(serial_object, read_struct, (read_struct->count - bytes_to_read));
			
			/* Return reading data */
			memcpy(&buffer[data_read_index], read_struct->buffer, bytes_to_read);
			bytes_to_read = 0;
		}
	} while ((bytes_to_read > 0) && (read_struct->count > 0));
	
	/* Number of reading bytes */
	*reading_count = buffer_size - bytes_to_read;
}

/* Read Ascii packet
 * Return char count, exclude NULL
 * Terminator: "\n", "\r", "\r\n"
 */
DWORD SerialRx_ReadLine(SERIAL_OBJECT *serial_object, \
		BUFFER_READ_STRUCT *read_struct, \
		const char *terminator, DWORD terminator_count, \
		char *buffer, DWORD buffer_size) {
	DWORD count, packet_len = 0, receive_count = 0;
	DWORD i;
	BYTE terminator_found = 0;
	
	/* Determine maximum number of bytes to read */
	count = buffer_size - 1;
	if(count >= SERIAL_BUFFER_SIZE)
		count = SERIAL_BUFFER_SIZE-1;
	
	/* Ignore terminator is invalid */
	if(terminator_count < 1)
		return 0;
	
	/* Read packet */
	do {
		SerialRx_Read(serial_object, read_struct); /* Check DMA buffer */
		receive_count += read_struct->count; /* Total number of data received */
		
		/* Search terminator */
		i = 0;
		while(!terminator_found && (i < read_struct->count)) {
			if(read_struct->buffer[i] == (char)terminator[terminator_count - 1])
				terminator_found = 1;
			i++;
		}
		packet_len += i;
		if(terminator_found) {
			terminator_found = 0;
			
			/* Roll-back buffer index */
			if ((packet_len > count) || (packet_len < terminator_count)) { /* Packet count is invalid, drop it */
				SerialRx_RestoreBytes(serial_object, read_struct, (receive_count-packet_len));
				/* Reset */
				packet_len       = 0;
				receive_count    = 0;
			}
			else {
				SerialRx_RestoreBytes(serial_object, read_struct, receive_count);
				
				/* Load data into buffer */
				SerialRx_ReadEx(serial_object, read_struct, buffer, packet_len, &i);
				buffer[packet_len] = '\0'; /* Append NULL */
				
				/* Validate terminator */
				if(!strncmp((char *)&buffer[packet_len-terminator_count], terminator, terminator_count)) {
					return packet_len; /* packet reading success, return number of received bytes */
				}
				else {
					/* Invalid terminator */
					packet_len       = 0;
					receive_count    = 0;
				}
			}
		}
	} while (read_struct->count > 0);
	
	/* Could not find the packet terminator, reset reading struct to its original position */
	if(receive_count > 0) {
		SerialRx_RestoreBytes(serial_object, read_struct, receive_count);
	}
	
	/* No byte receive */
	return 0;
}

/* Read Binary packet
 * 0: Not ready, 1: Data is ready
 */
BOOL SerialRx_ReadBinary(SERIAL_OBJECT *serial_object, \
		BUFFER_READ_STRUCT *read_struct, \
		const char *header, DWORD header_count, \
		const char *terminator, DWORD terminator_count, \
		char *buffer, DWORD data_count) {
	DWORD receive_count = 0, drop_count = 0, binary_state = 0, binary_index = 0;
	DWORD i;
	
	do {
		SerialRx_Read(serial_object, read_struct); /* Check DMA buffer */
		receive_count += read_struct->count; /* Total number of data received */
		
		/* Binary packet processing */
		for(i=0; i<read_struct->count; i++) {
			switch( binary_state ) {
				case 0: /* Search for header */
					if(binary_index < header_count) {
						if(read_struct->buffer[i] == header[binary_index]) {
							binary_index ++;
						}
						else {
							binary_index = 0;
							drop_count = receive_count - (read_struct->count - i - 1); /* Drop packet */
						}
						break;
					}
					else { /* Change to DATA state */
						binary_index = 0;
						binary_state ++;
					}
					
				case 1: /* Wait for data */
					/* Wait for DATA */
					if(binary_index < data_count) {
						buffer[binary_index] = read_struct->buffer[i];
						binary_index ++;
						
						/* Check if ready (No terminator) */
						if ((binary_index >= data_count) && (terminator_count == 0)) {
							SerialRx_RestoreBytes(serial_object, read_struct, (read_struct->count - i - 1)); /* Restore some bytes */
							return TRUE; /* Return success status */
						}
						break;
					}
					else { /* Change to Terminator state */
						binary_index = 0;
						binary_state ++;
					}
					
				case 2: /* Scan for terminator */
					if(binary_index < terminator_count) {
						if(read_struct->buffer[i] == terminator[binary_index]) {
							binary_index ++;
						}
						else {
							binary_state = 0;
							binary_index = 0;
							drop_count = receive_count - (read_struct->count - i - 1); /* Drop packet */
						}
					}
					
					if(binary_index >= terminator_count) { /* Success */
						/* Restore some bytes */
						SerialRx_RestoreBytes(serial_object, read_struct, (read_struct->count - i - 1));
						return TRUE; /* Return success status */
					}
					break;
			}
		}
	} while (read_struct->count > 0);
	
	/* Restore bytes */
	SerialRx_RestoreBytes(serial_object, read_struct, (receive_count - drop_count));
	return FALSE;
}

/* ========================================================================
 * Simulation
 * ========================================================================
 */

// Packet mode
#define PACKET_MODE_ASCII        0
#define PACKET_MODE_BINARY       1
#define PACKET_MODE_BINARYVECTOR 2

#define MAX_HEADER_TERMINATOR_SIZE 32			
typedef struct {
	/* Block configuration */
	char conf[32];
	
	/* UART setup */
	char port[32];
	ULONG baudrate;
	ULONG databits;
	ULONG parity;
	ULONG stopbit;
	ULONG flowcontrol;
	
	BOOL transfer_blocking;
	unsigned int timeout;
	
	/* Packet mode */
	unsigned int packetmode;
	
	/* Binary format */
	char binheader[MAX_HEADER_TERMINATOR_SIZE];
	unsigned int binheader_count;
	char binterminator[MAX_HEADER_TERMINATOR_SIZE];
	unsigned int binterminator_count;
	
	/* Ascii format */
	//char *asciiformat; // Must be Free at mdlTerminate
	//unsigned char ascii_terminator[MAX_HEADER_TERMINATOR_SIZE];
	//unsigned int ascii_terminator_count;	
} BLOCK_PORT_CONFIGURATION;

BOOL get_block_configuration(SimStruct *S, BLOCK_PORT_CONFIGURATION *port_configuration)
{
	int i;
	char *transfer, *port, *stop, *conf, *packetmode;
	char *binheader_str, *binterminator_str, *s;
	unsigned int h, binheader_count, binterminator_count;
	BOOL sta;
	
	// conf
	conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONF));
	strcpy(port_configuration->conf, conf);
	mxFree(conf);
	
	// port
	port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_AUTOPORT));
	if (strlen(port) < 32) {
		strcpy(port_configuration->port, port);
	}
	else {
		memset(port_configuration->port, 0, 32);
		strncpy(port_configuration->port, port, 31);
	}
	mxFree(port);
	
	// baudrate
	port_configuration->baudrate = (unsigned int)mxGetScalar(ssGetSFcnParam(S, ARGC_BAUDRATE));
		
	//unsigned char databits;
	port_configuration->databits = 8;
	
	//unsigned char parity;
	port_configuration->parity = NOPARITY; // None
	
	//unsigned char stopbit;
	stop = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_STOPBIT));
	if(!strcmp(stop, "1")) {
		port_configuration->stopbit = ONESTOPBIT;
	}
	else if(!strcmp(stop, "1.5")) {
		port_configuration->stopbit = ONE5STOPBITS;
	}
	else if(!strcmp(stop, "2")) {
		port_configuration->stopbit = TWOSTOPBITS;
	}
	else {
		ssSetErrorStatus(S, "Invalid stop bits!.\n");
		return FALSE;
	}
	mxFree(stop);
	
	//unsigned char flowcontrol;
	port_configuration->flowcontrol = 0; // None
	
	//unsigned int transfer_blocking;
	transfer = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TRANSFER));
	if (!strcmp(transfer,"Blocking"))
		port_configuration->transfer_blocking = TRUE;
	else
		port_configuration->transfer_blocking = FALSE;	
	mxFree(transfer);
	
	//unsigned int timeout;
	port_configuration->timeout = (unsigned int)(1000.0*mxGetScalar(ssGetSFcnParam(S, ARGC_TIMEOUT)));
	
	// packetmode ( Ascii | Binary | BinaryVector )
	packetmode = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PACKETMODE));
	if (!strcmp(packetmode, "Ascii")) {
		port_configuration->packetmode = PACKET_MODE_ASCII;
	}
	else if (!strcmp(packetmode, "Binary")) {
		port_configuration->packetmode = PACKET_MODE_BINARY;
	}
	else if (!strcmp(packetmode, "BinaryVector")) {
		port_configuration->packetmode = PACKET_MODE_BINARYVECTOR;
	}
	else {
		ssSetErrorStatus(S, "Invalid stop bits!.\n");
		return FALSE;
	}
	mxFree(packetmode);
	
	// binheader ('7E 7E')
	binheader_str = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BINHEADER));
	binheader_count = 0;
	s = strtok(binheader_str, " ,.-");
	sta = TRUE;
	while (s != NULL) {
		if (s[0] != '\0') {
			if((binheader_count < MAX_HEADER_TERMINATOR_SIZE) && \
					(sscanf(s, "%x", &h) == 1) && \
					(h <= 255)) {
				port_configuration->binheader[binheader_count] = (char)((unsigned char)h);
				binheader_count ++;
			}
			else {
				ssSetErrorStatus(S, "Invalid binary header format!\n Example correct format: \"'7E 7E'\"\nEach element can not be larger than 0xFF.\nSeparate each element by a space.");
				sta = FALSE;
			}
		}
		s = strtok(NULL, " ,.-");
	}
	mxFree(binheader_str);
	if (!sta)
		return FALSE;
	
	// binheader_count
	port_configuration->binheader_count = binheader_count;
			
	// binterminator
	binterminator_str = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BINTERMINATOR));
	binterminator_count = 0;
	s = strtok(binterminator_str, " ,.-");
	sta = TRUE;
	while (s != NULL) {
		if (s[0] != '\0') {
			if((binterminator_count < MAX_HEADER_TERMINATOR_SIZE) && \
					(sscanf(s, "%x", &h) == 1) && \
					(h <= 255)) {
				port_configuration->binterminator[binterminator_count] = (char)((unsigned char)h);
				binterminator_count ++;
			}
			else {
				ssSetErrorStatus(S, "Invalid binary terminator format!\n Example correct format: \"'7E 7E'\"\nEach element can not be larger than 0xFF.\nSeparate each element by a space.");
				sta = FALSE;
			}
		}
		s = strtok(NULL, " ,.-");
	}
	mxFree(binterminator_str);
	if (!sta)
		return FALSE;
	
	// binterminator_count
	port_configuration->binterminator_count = binterminator_count;
	
	// Ascii
	
/*
	// DEBUG:
	mexPrintf("Header %d bytes: ", port_configuration->binheader_count);
	for (i=0; i<port_configuration->binheader_count; i++) {
		mexPrintf("%X ", port_configuration->binheader[i]);
	}
	mexPrintf("\n");
*/
	
	// No Error finally
	return TRUE;
}

static int StrPos(char* s, const char* sub_str) {
    char* pos = strstr(s, sub_str);
    if(pos)
        return (pos-s);
    
    /* Sub string not present in s */
    return -1;
}

static int GetFormattedPos(char* s, int offset) {
    char* pos = &s[offset];
    int index = 0;
    
    while(*pos) {
        /* Check existing */
        if(StrPos(pos, "%") < 0)
            return -1;
        pos+= (StrPos(pos, "%"));
        if(StrPos(pos, "%") == StrPos(pos, "%%"))
            pos+= 2;
        else
            return (pos - s);
    }
    return -1;
}

int GetFormattedSegment(char* s) {
    int index1 = GetFormattedPos(s, 0);
    int index2;
    if(index1 < 0)
        return strlen(s);
    index2 = GetFormattedPos(s, index1+1);
    if(index2 < 0)
        return strlen(s);
    return index2;
}

#define MDL_START
static void mdlStart(SimStruct *S) {
	//char *blockid;
	//BUFFER_READ_STRUCT read_struct;
	
	// Close all serial port
    SerialObjectList_CloseAll();	
	
	// Read struct
	read_struct_clear_list();
}

// Tx Ascii processing buffer
BYTE ascii_tmp_formatted_string[SERIAL_BUFFER_SIZE];
BYTE ascii_tmp_segmented_string[SERIAL_BUFFER_SIZE];
BYTE rx_temp_scanf_result[512]; /* 8 x 64 */

static char global_rx_buffer[SERIAL_BUFFER_SIZE];
static BLOCK_PORT_CONFIGURATION block_port_configuration;
static void mdlOutputs(SimStruct *S, int_T tid) {
	int i;
	//char *blockid;
	SERIAL_OBJECT *serial_object = NULL;
	
	// Get block configuration
	if (!get_block_configuration(S, &block_port_configuration)) {
		ssSetErrorStatus(S, "Error: mask input.\n");
		return;
	}
	
	// Open port for Tx or Rx
	if (!strcmp(block_port_configuration.conf, "tx") || !strcmp(block_port_configuration.conf, "rx")) {
		serial_object = SerialObjectList_Open(\
				block_port_configuration.port, \
				block_port_configuration.baudrate, \
				(BYTE) block_port_configuration.databits, \
				(BYTE) block_port_configuration.parity, \
				(BYTE) block_port_configuration.stopbit);
		if (serial_object == NULL) {
			mexPrintf("Failed to open COM port: %s\n", block_port_configuration.port);
			ssSetErrorStatus(S, "Error: failed to open COM port.\n");
			return;
		}
		
		// Read bytes from port
		if (!SerialObjectList_Receive(serial_object)) {
			mexPrintf("Failed to read data from port: %s\n", block_port_configuration.port);
			ssSetErrorStatus(S, "Error: failed to read data from port.\n");
			return;
		}
	}
	
	// ====================================================================
	// = "Host Serial Setup" block
	// ====================================================================
	if (!strcmp(block_port_configuration.conf, "setup")) {
		// Do nothing
	}
	
	// ====================================================================
	// = "Host Serial Send" block
	// ====================================================================
	else if (!strcmp(block_port_configuration.conf, "tx")) {
		// === (Binary | BinaryVector) mode ===
		if ((block_port_configuration.packetmode == PACKET_MODE_BINARY) \
				|| (block_port_configuration.packetmode == PACKET_MODE_BINARYVECTOR))
		{
			int input_count, width;
			serial_object->tx_count = 0;
			
			// Header
			if (block_port_configuration.binheader_count > 0) {
				memcpy(&(serial_object->txbuffer[serial_object->tx_count]), \
						&(block_port_configuration.binheader[0]), block_port_configuration.binheader_count);
				serial_object->tx_count += block_port_configuration.binheader_count;
			}
			// Data
			input_count = ssGetNumInputPorts(S);
			for(i=0; i<input_count; i++) {
				width = ssGetInputPortWidth(S, i);
				switch((BYTE)ssGetInputPortDataType(S, i)) {
					case 0: /* Double */
						if ((serial_object->tx_count + width*sizeof(real_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), \
									(real_T*) ssGetInputPortSignal(S, i), width*sizeof(real_T));
							serial_object->tx_count += width*sizeof(real_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 1: /* Single */
						if ((serial_object->tx_count + width*sizeof(real32_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (real32_T*) ssGetInputPortSignal(S, i), width*sizeof(real32_T));
							serial_object->tx_count += width*sizeof(real32_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 2: /* int8 */
						if ((serial_object->tx_count + width*sizeof(int8_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (int8_T*) ssGetInputPortSignal(S, i), width*sizeof(int8_T));
							serial_object->tx_count += width*sizeof(int8_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 3: /* uint8 */
						if ((serial_object->tx_count + width*sizeof(uint8_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (uint8_T*) ssGetInputPortSignal(S, i), width*sizeof(uint8_T));
							serial_object->tx_count += width*sizeof(uint8_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 4: /* int16 */
						if ((serial_object->tx_count + width*sizeof(int16_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (int16_T*) ssGetInputPortSignal(S, i), width*sizeof(int16_T));
							serial_object->tx_count += width*sizeof(int16_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 5: /* uint16 */
						if ((serial_object->tx_count + width*sizeof(uint16_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (uint16_T*) ssGetInputPortSignal(S, i), width*sizeof(uint16_T));
							serial_object->tx_count += width*sizeof(uint16_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 6: /* int32 */
						if ((serial_object->tx_count + width*sizeof(int32_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (int32_T*) ssGetInputPortSignal(S, i), width*sizeof(int32_T));
							serial_object->tx_count += width*sizeof(int32_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					case 7: /* uint32 */
						if ((serial_object->tx_count + width*sizeof(uint32_T)) < SERIAL_BUFFER_SIZE) {
							memcpy(&(serial_object->txbuffer[serial_object->tx_count]), (uint32_T*) ssGetInputPortSignal(S, i), width*sizeof(uint32_T));
							serial_object->tx_count += width*sizeof(uint32_T);
						}
						else {
							ssSetErrorStatus(S, "Error: number of bytes exceed maximum buffer size.\n");
						}
						break;
					default:
						ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
						break;
				}
			}
			
			// Terminator
			if (block_port_configuration.binterminator_count > 0) {
				memcpy(&(serial_object->txbuffer[serial_object->tx_count]), &(block_port_configuration.binterminator[0]), block_port_configuration.binterminator_count);
				serial_object->tx_count += block_port_configuration.binterminator_count;
			}
			
			// Write to port
			if (!SerialObjectList_Transmit(serial_object)) {
				ssSetErrorStatus(S, "Error: failed to write binary packet to COM port.\n");
			}
		}
		
		// === Ascii mode ===
		else if (block_port_configuration.packetmode == PACKET_MODE_ASCII)
		{
			int input_count, bytes_index;
			char* ascii_pformat_str, *ascii_format, *ascii_terminator;
			int ascii_segment_index = 0;
			
			ascii_format = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIFORMAT));
			ascii_pformat_str = ascii_format;
			
			input_count = ssGetNumInputPorts(S);
			bytes_index = 0;
			memset(&(serial_object->txbuffer[bytes_index]), 0, SERIAL_BUFFER_SIZE);
			
			if (input_count == 0) {
				strcpy(&(serial_object->txbuffer[bytes_index]), ascii_format);
				bytes_index += strlen(ascii_format);
			}
			else {
				for(i=0; i<input_count; i++) {
					// Get first formatted
					if(*ascii_pformat_str) {
						ascii_segment_index = GetFormattedSegment(ascii_pformat_str);
						strncpy_s(ascii_tmp_segmented_string,
								sizeof(ascii_tmp_segmented_string),
								ascii_pformat_str,
								ascii_segment_index);
						ascii_tmp_segmented_string[ascii_segment_index+1] = '\0';
						
						switch((BYTE)ssGetInputPortDataType(S, i)) {
							case 0: /* Double */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const real_T*) ssGetInputPortSignal(S, i));
								break;
							case 1: /* Single */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const real32_T*) ssGetInputPortSignal(S, i));
								break;
							case 2: /* int8 */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const int8_T*) ssGetInputPortSignal(S, i));
								break;
							case 3: /* uint8 */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const uint8_T*) ssGetInputPortSignal(S, i));
								break;
							case 4: /* int16 */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const int16_T*) ssGetInputPortSignal(S, i));
								break;
							case 5: /* uint16 */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const uint16_T*) ssGetInputPortSignal(S, i));
								break;
							case 6: /* int32 */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const int32_T*) ssGetInputPortSignal(S, i));
								break;
							case 7: /* uint32 */
								bytes_index += sprintf_s((char*)&(serial_object->txbuffer[bytes_index]),
										(SERIAL_BUFFER_SIZE-bytes_index),
										(char*)ascii_tmp_segmented_string, *(const uint32_T*) ssGetInputPortSignal(S, i));
								break;
						}
						ascii_pformat_str = &ascii_pformat_str[ascii_segment_index];
					}
				}
			}

			/* Ascii terminator */
			ascii_terminator = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TX_TERMINATOR));
			//mexPrintf("Terminator: %s", ascii_terminator);
			// CR (0x0D - "\r")
			if (!strcmp(ascii_terminator, "CR (0x0D - \"\\r\")")) {
				serial_object->txbuffer[bytes_index ++] = 0x0D;
			}
			// LF (0x0A - "\n")
			else if (!strcmp(ascii_terminator, "LF (0x0A - \"\\n\")")) {
				serial_object->txbuffer[bytes_index ++] = 0x0A;
			}
			// CRLF (0x0D 0x0A - "\r\n")
			else if (!strcmp(ascii_terminator, "CRLF (0x0D 0x0A - \"\\r\\n\")")) {
				serial_object->txbuffer[bytes_index ++] = 0x0D;
				serial_object->txbuffer[bytes_index ++] = 0x0A;
			}
			// None
			else if (!strcmp(ascii_terminator, "None")) {
			}
			else {
				ssSetErrorStatus(S, "Error: invalid Ascii packet terminator.\n");
			}
			serial_object->txbuffer[bytes_index] = 0x00; // NULL
			serial_object->tx_count = bytes_index;
			
			/* Free */
			mxFree(ascii_format);
			mxFree(ascii_terminator);
			
			/* Write to Port */
			//mexPrintf("Str: %s\n", serial_object->txbuffer);
			// Write to port
			if (!SerialObjectList_Transmit(serial_object)) {
				ssSetErrorStatus(S, "Error: failed to write binary packet to COM port.\n");
			}

			return;
		}
		// === Invalid mode ===
		else {
			ssSetErrorStatus(S, "Error: invalid Tx packet mode.\n");
			return;
		}		
	}
	
	// ====================================================================
	// = "Host Serial Receive" block
	// ====================================================================
	else if (!strcmp(block_port_configuration.conf, "rx")) {
		// === (Binary | BinaryVector) mode ===
		if ((block_port_configuration.packetmode == PACKET_MODE_BINARY) \
				|| (block_port_configuration.packetmode == PACKET_MODE_BINARYVECTOR))
        {
			int output_count, width;
			int output_port_start;
			int output_data_count, output_data_index;
			BUFFER_READ_STRUCT *read_struct;
			//BOOL sta;
			BOOL rx_ready, rx_timeout, rx_error;

			BOOL    initial_values_enable;
			int     initial_values_count;
			int     initial_values_index;
			real_T *initial_values;

			DWORD start_tickcount;
		
			// Determine output port 0 is READY port
			if (block_port_configuration.transfer_blocking)
				output_port_start = 0;
			else
				output_port_start = 1;
			
			// Number of data
			output_data_count = 0;
			
			//Collect Rx data bytes count
			output_count = ssGetNumOutputPorts(S);
			for(i=output_port_start; i<output_count; i++) {
				width = ssGetOutputPortWidth(S, i);
				switch((BYTE)ssGetOutputPortDataType(S, i)) {
					case 0: /* Double */ output_data_count += width*sizeof(real_T);
						break;
					case 1: /* Single */ output_data_count += width*sizeof(real32_T);
						break;
					case 2: /* int8 */   output_data_count += width*sizeof(int8_T);
						break;
					case 3: /* uint8 */  output_data_count += width*sizeof(uint8_T);
						break;
					case 4: /* int16 */  output_data_count += width*sizeof(int16_T);
						break;
					case 5: /* uint16 */ output_data_count += width*sizeof(uint16_T);
						break;
					case 6: /* int32 */  output_data_count += width*sizeof(int32_T);
						break;
					case 7: /* uint32 */ output_data_count += width*sizeof(uint32_T);
						break;
					default:
						ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
						break;
				}
			}
			
			// Load read struct
			read_struct = SerialRx_GetReadStruct(S);
			
			// Determine Rx block need return inialvalues
			initial_values_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INITIALVALUES));
			initial_values = (double*)mxGetPr(ssGetSFcnParam(S, ARGC_INITIALVALUES));
			if ((read_struct->firststep != 0) && (initial_values_count > 0))
				initial_values_enable = TRUE;
			else
				initial_values_enable = FALSE;
			
			// Return initial values to port
			if (initial_values_enable) {
				rx_ready = TRUE;
				
				// Store read struct back
				read_struct->firststep = 0;
				//sta = SerialRx_SetReadStruct(S, read_struct);
			}
			// Read from port
			else {
				rx_ready = FALSE;
				rx_timeout = FALSE;
				rx_error = FALSE;
				start_tickcount = GetTickCount();
				do {
					// Match packet
					rx_ready = SerialRx_ReadBinary( \
							serial_object, \
							read_struct, \
							block_port_configuration.binheader, \
							block_port_configuration.binheader_count, \
							block_port_configuration.binterminator, \
							block_port_configuration.binterminator_count, \
							global_rx_buffer, output_data_count);
					// Poll Rx
					if (block_port_configuration.transfer_blocking) {
						rx_error = !SerialObjectList_Receive(serial_object);
					}
					// Timeout check
					if ((GetTickCount() - start_tickcount) > block_port_configuration.timeout) {
						rx_timeout = TRUE;
					}
				} while (!rx_ready && block_port_configuration.transfer_blocking && !rx_timeout && !rx_error);
				
				// Store read struct back
				read_struct->firststep = 0;
				//sta = SerialRx_SetReadStruct(S, read_struct);
				
				// Error
				if (rx_error) {
					ssSetErrorStatus(S, "Error: error occur while read data from port.\n");
					return;
				}
				// Timeout check
				if (block_port_configuration.transfer_blocking && rx_timeout) {
					ssSetErrorStatus(S, "Error: timeout occur while waiting for Rx data.\n");
					return;
				}
			}
			
			// Non-Blocking
			if (!block_port_configuration.transfer_blocking) {
				*(uint8_T*) ssGetOutputPortSignal(S, 0) = (uint8_T)rx_ready; /* Ready? */
			}
			// Set data to port if it ready, otherwise left it with the previous value
			if (rx_ready) {				
				output_data_index = 0;
				initial_values_index = 0;
				for(i=output_port_start; i<output_count; i++) {
					width = ssGetOutputPortWidth(S, i);					
					switch((BYTE)ssGetOutputPortDataType(S, i))
                    {
						case 0: /* double */
							if (initial_values_enable) {
								*((real_T*) ssGetOutputPortSignal(S, i)) = initial_values[initial_values_index];
							}
							else {
								memcpy((real_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(real_T));
								output_data_index += width*sizeof(real_T);
							}
							break;
						case 1: /* single */
							if (initial_values_enable) {
								*((real32_T*) ssGetOutputPortSignal(S, i)) = (real32_T)initial_values[initial_values_index];
							}
							else {
								memcpy((real32_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(real32_T));
								output_data_index += width*sizeof(real32_T);
							}
							break;
						case 2: /* int8 */
							if (initial_values_enable) {
								*((int8_T*) ssGetOutputPortSignal(S, i)) = (int8_T)initial_values[initial_values_index];
							}
							else {
								memcpy((int8_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(int8_T));
								output_data_index += width*sizeof(int8_T);
							}
							break;
						case 3: /* uint8 */
							if (initial_values_enable) {
								*((uint8_T*) ssGetOutputPortSignal(S, i)) = (uint8_T)initial_values[initial_values_index];
							}
							else {
								memcpy((uint8_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(uint8_T));
								output_data_index += width*sizeof(uint8_T);
							}
							break;
						case 4: /* int16 */
							if (initial_values_enable) {
								*((int16_T*) ssGetOutputPortSignal(S, i)) = (int16_T)initial_values[initial_values_index];
							}
							else {
								memcpy((int16_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(int16_T));
								output_data_index += width*sizeof(int16_T);
							}
							break;
						case 5: /* uint16 */
							if (initial_values_enable) {
								*((uint16_T*) ssGetOutputPortSignal(S, i)) = (uint16_T)initial_values[initial_values_index];
							}
							else {
								memcpy((uint16_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(uint16_T));
								output_data_index += width*sizeof(uint16_T);
							}
							break;
						case 6: /* int32 */
							if (initial_values_enable) {
								*((int32_T*) ssGetOutputPortSignal(S, i)) = (int32_T)initial_values[initial_values_index];
							}
							else {
								memcpy((int32_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(int32_T));
								output_data_index += width*sizeof(int32_T);
							}
							break;
						case 7: /* uint32 */
							if (initial_values_enable) {
								*((uint32_T*) ssGetOutputPortSignal(S, i)) = (uint32_T)initial_values[initial_values_index];
							}
							else {
								memcpy((uint32_T*) ssGetOutputPortSignal(S, i), &global_rx_buffer[output_data_index], width*sizeof(uint32_T));
								output_data_index += width*sizeof(uint32_T);
							}
							break;
						default:
							ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
							break;
					}
					
					// Initial value index
					if ((initial_values_index + 1) < initial_values_count) {
						initial_values_index ++;
					}
				}
			}
		}
		// === Ascii mode ===
		else if (block_port_configuration.packetmode == PACKET_MODE_ASCII)
		{
			DWORD ascii_rx_terminator_count;
			char  ascii_rx_terminator[3];
			char *ascii_terminator, *ascii_format;
			int output_count, width;
			int output_port_start;
			int output_data_count, output_data_index;
			BUFFER_READ_STRUCT *read_struct;
			//BOOL sta;
			BOOL rx_ready, rx_timeout, rx_error;

			BOOL    initial_values_enable;
			int     initial_values_count;
			int     initial_values_index;
			real_T *initial_values;

			DWORD start_tickcount;
		
			// Determine output port 0 is READY port
			if (block_port_configuration.transfer_blocking)
				output_port_start = 0;
			else
				output_port_start = 1;
			
			//Collect Rx data bytes count
			output_count = ssGetNumOutputPorts(S);
			
			// Load read struct
			read_struct = SerialRx_GetReadStruct(S);
			
			// Determine Rx block need return inialvalues
			initial_values_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INITIALVALUES));
			initial_values = (double*)mxGetPr(ssGetSFcnParam(S, ARGC_INITIALVALUES));
			if ((read_struct->firststep != 0) && (initial_values_count > 0))
				initial_values_enable = TRUE;
			else
				initial_values_enable = FALSE;
			
			// Ascii packet terminator
			ascii_format = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIFORMAT));
			ascii_terminator = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_RX_TERMINATOR));
			
			// CR (0x0D - "\r")
			if (!strcmp(ascii_terminator, "CR (0x0D - \"\\r\")")) {
				ascii_rx_terminator[0] = 0x0D;
				ascii_rx_terminator[1] = 0x00;
				ascii_rx_terminator_count = 1;
			}
			// LF (0x0A - "\n")
			else if (!strcmp(ascii_terminator, "LF (0x0A - \"\\n\")")) {
				ascii_rx_terminator[0] = 0x0A;
				ascii_rx_terminator[1] = 0x00;
				ascii_rx_terminator_count = 1;
			}
			// CRLF (0x0D 0x0A - "\r\n")
			else if (!strcmp(ascii_terminator, "CRLF (0x0D 0x0A - \"\\r\\n\")")) {
				ascii_rx_terminator[0] = 0x0D;
				ascii_rx_terminator[1] = 0x0A;
				ascii_rx_terminator[2] = 0x00;
				ascii_rx_terminator_count = 2;
			}
			else {
				ascii_rx_terminator_count = 0;
				ssSetErrorStatus(S, "Error: invalid Ascii packet terminator.\n");
			}

			rx_timeout = FALSE;
			rx_error = FALSE;

			// Return initial values to port
			if (initial_values_enable) {
				rx_ready = TRUE;
				
				// Store read struct back
				read_struct->firststep = 0;
			}
			// Read from port
			else {
				rx_ready = FALSE;
				start_tickcount = GetTickCount();
				do {
					// Match packet
					global_rx_buffer[0] = '\0';
					rx_ready = (SerialRx_ReadLine(serial_object, read_struct, \
							ascii_rx_terminator, ascii_rx_terminator_count, \
							global_rx_buffer, SERIAL_BUFFER_SIZE) > 0);
					if (rx_ready) {
						if ((output_count - output_port_start) <= 0) {
							int rx_len_z = strlen(global_rx_buffer);
							rx_len_z -= ascii_rx_terminator_count;
							// Compare message
							global_rx_buffer[rx_len_z] = '\0';
							if (!strcmp(global_rx_buffer, ascii_format)) {
								rx_ready = TRUE;
							}
							else {
								rx_ready = FALSE;
							}
						}
						else {
							// Scan message
							if(sscanf((char*)global_rx_buffer, ascii_format, \
									// 0 - 7
									(void*)&rx_temp_scanf_result[0*8],
									(void*)&rx_temp_scanf_result[1*8],
									(void*)&rx_temp_scanf_result[2*8],
									(void*)&rx_temp_scanf_result[3*8],
									(void*)&rx_temp_scanf_result[4*8],
									(void*)&rx_temp_scanf_result[5*8],
									(void*)&rx_temp_scanf_result[6*8],
									(void*)&rx_temp_scanf_result[7*8],
									// 8 - 15
									(void*)&rx_temp_scanf_result[8*8],
									(void*)&rx_temp_scanf_result[9*8],
									(void*)&rx_temp_scanf_result[10*8],
									(void*)&rx_temp_scanf_result[11*8],
									(void*)&rx_temp_scanf_result[12*8],
									(void*)&rx_temp_scanf_result[13*8],
									(void*)&rx_temp_scanf_result[14*8],
									(void*)&rx_temp_scanf_result[15*8],
									// 16 - 23
									(void*)&rx_temp_scanf_result[16*8],
									(void*)&rx_temp_scanf_result[17*8],
									(void*)&rx_temp_scanf_result[18*8],
									(void*)&rx_temp_scanf_result[19*8],
									(void*)&rx_temp_scanf_result[20*8],
									(void*)&rx_temp_scanf_result[21*8],
									(void*)&rx_temp_scanf_result[22*8],
									(void*)&rx_temp_scanf_result[23*8],
									// 24 - 31
									(void*)&rx_temp_scanf_result[24*8],
									(void*)&rx_temp_scanf_result[25*8],
									(void*)&rx_temp_scanf_result[26*8],
									(void*)&rx_temp_scanf_result[27*8],
									(void*)&rx_temp_scanf_result[28*8],
									(void*)&rx_temp_scanf_result[29*8],
									(void*)&rx_temp_scanf_result[30*8],
									(void*)&rx_temp_scanf_result[31*8],
									// 32 - 39
									(void*)&rx_temp_scanf_result[32*8],
									(void*)&rx_temp_scanf_result[33*8],
									(void*)&rx_temp_scanf_result[34*8],
									(void*)&rx_temp_scanf_result[35*8],
									(void*)&rx_temp_scanf_result[36*8],
									(void*)&rx_temp_scanf_result[37*8],
									(void*)&rx_temp_scanf_result[38*8],
									(void*)&rx_temp_scanf_result[39*8],
									// 40 - 47
									(void*)&rx_temp_scanf_result[40*8],
									(void*)&rx_temp_scanf_result[41*8],
									(void*)&rx_temp_scanf_result[42*8],
									(void*)&rx_temp_scanf_result[43*8],
									(void*)&rx_temp_scanf_result[44*8],
									(void*)&rx_temp_scanf_result[45*8],
									(void*)&rx_temp_scanf_result[46*8],
									(void*)&rx_temp_scanf_result[47*8],
									// 48 - 55
									(void*)&rx_temp_scanf_result[48*8],
									(void*)&rx_temp_scanf_result[49*8],
									(void*)&rx_temp_scanf_result[50*8],
									(void*)&rx_temp_scanf_result[51*8],
									(void*)&rx_temp_scanf_result[52*8],
									(void*)&rx_temp_scanf_result[53*8],
									(void*)&rx_temp_scanf_result[54*8],
									(void*)&rx_temp_scanf_result[55*8],
									// 56 - 63
									(void*)&rx_temp_scanf_result[56*8],
									(void*)&rx_temp_scanf_result[57*8],
									(void*)&rx_temp_scanf_result[58*8],
									(void*)&rx_temp_scanf_result[59*8],
									(void*)&rx_temp_scanf_result[60*8],
									(void*)&rx_temp_scanf_result[61*8],
									(void*)&rx_temp_scanf_result[62*8],
									(void*)&rx_temp_scanf_result[63*8]
									) == (output_count - output_port_start)) {
								// Success
								rx_ready = TRUE;
							}
							else {
								// Scan Fail
								rx_ready = FALSE;
							}
						}
					}
					
					// Poll Rx
					if (block_port_configuration.transfer_blocking) {
						rx_error = !SerialObjectList_Receive(serial_object);
					}
					// Timeout check
					if ((GetTickCount() - start_tickcount) > block_port_configuration.timeout) {
						rx_timeout = TRUE;
					}
				} while (!rx_ready && block_port_configuration.transfer_blocking && !rx_timeout && !rx_error);
				
				// Store read struct back
				read_struct->firststep = 0;
			}
			
			// Free
			mxFree(ascii_terminator);
			mxFree(ascii_format);
			
			// Error
			if (rx_error) {
				ssSetErrorStatus(S, "Error: error occur while read data from port.\n");
				return;
			}
			// Timeout check
			if (block_port_configuration.transfer_blocking && rx_timeout) {
				ssSetErrorStatus(S, "Error: timeout occur while waiting for Rx data.\n");
				return;
			}
			
			// Non-Blocking
			if (!block_port_configuration.transfer_blocking) {
				*((uint8_T*) ssGetOutputPortSignal(S, 0)) = (uint8_T)rx_ready; /* Ready? */
			}
			
			// Set data to port if it ready, otherwise left it with the previous value
			if (rx_ready) {
				output_data_index = 0;
				initial_values_index = 0;
				for(i=output_port_start; i<output_count; i++) {
					width = ssGetOutputPortWidth(S, i);
					switch((BYTE)ssGetOutputPortDataType(S, i)) {
						case 0: /* double */
							if (initial_values_enable) {
								*((real_T*) ssGetOutputPortSignal(S, i)) = initial_values[initial_values_index];
							}
							else {
								memcpy((real_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(real_T));
							}
							break;
						case 1: /* single */
							if (initial_values_enable) {
								*((real32_T*) ssGetOutputPortSignal(S, i)) = (real32_T)initial_values[initial_values_index];
							}
							else {
								memcpy((real32_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(real32_T));
							}
							break;
						case 2: /* int8 */
							if (initial_values_enable) {
								*((int8_T*) ssGetOutputPortSignal(S, i)) = (int8_T)initial_values[initial_values_index];
							}
							else {
								memcpy((int8_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(int8_T));
							}
							break;
						case 3: /* uint8 */
							if (initial_values_enable) {
								*((uint8_T*) ssGetOutputPortSignal(S, i)) = (uint8_T)initial_values[initial_values_index];
							}
							else {
								memcpy((uint8_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(uint8_T));
							}
							break;
						case 4: /* int16 */
							if (initial_values_enable) {
								*((int16_T*) ssGetOutputPortSignal(S, i)) = (int16_T)initial_values[initial_values_index];
							}
							else {
								memcpy((int16_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(int16_T));
							}
							break;
						case 5: /* uint16 */
							if (initial_values_enable) {
								*((uint16_T*) ssGetOutputPortSignal(S, i)) = (uint16_T)initial_values[initial_values_index];
							}
							else {
								memcpy((uint16_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(uint16_T));
							}
							break;
						case 6: /* int32 */
							if (initial_values_enable) {
								*((int32_T*) ssGetOutputPortSignal(S, i)) = (int32_T)initial_values[initial_values_index];
							}
							else {
								memcpy((int32_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(int32_T));
							}
							break;
						case 7: /* uint32 */
							if (initial_values_enable) {
								*((uint32_T*) ssGetOutputPortSignal(S, i)) = (uint32_T)initial_values[initial_values_index];
							}
							else {
								memcpy((uint32_T*) ssGetOutputPortSignal(S, i), &rx_temp_scanf_result[output_data_index], width*sizeof(uint32_T));
							}
							break;
						default:
							ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
							break;
					}
					output_data_index += 8;
					
					// Initial value index
					if ((initial_values_index + 1) < initial_values_count) {
						initial_values_index ++;
					}
				}
			}

			return;
		}
		// === Invalid mode ===
		else {
			ssSetErrorStatus(S, "Error: invalid Rx packet mode.\n");
			return;
		}			
	}
	
	// ====================================================================
	// = Invalid configuration
	// ====================================================================
	else {
		mexPrintf("Invalid conf: %s\n", block_port_configuration.conf);
		ssSetErrorStatus(S, "Error: invalid conf.\n");
	}
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
	// Close all serial port
    SerialObjectList_CloseAll();
	//
	read_struct_clear_list();
} /* end mdlTerminate */

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file amg_usbconverter_n_connect.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function amg_usbconverter_n_connect.c"
#endif

