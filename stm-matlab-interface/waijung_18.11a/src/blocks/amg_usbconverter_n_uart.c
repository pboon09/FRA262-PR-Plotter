#define S_FUNCTION_NAME  amg_usbconverter_n_uart
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#include "windows.h"

#define NPARAMS 20 /* Total number of block parameters */

#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

enum {
    UART_ID = 0,    /* "setup", "rx", "tx" */
    UART_PORT,      /* "setup", "rx", "tx" */
    UART_BAUDRATE,  /* 9600,57600,115200,... */
    UART_DATABITS,  /* 8, 9 */
    UART_PARITY,    /* "None", "Odd", "Even" */
    UART_STOPBITS,  /* "1", "1.5", "2" */
    UART_TIMEOUT,   /* 10000 */
    UART_PACKETMODE, /* "Ascii", "Binary" */
    UART_ASCII_FORMAT, /* "Value=%d\n" */
    UART_ASCII_TYPEARRAY, /* ["double", "single", "..."] */
    
};

#define ID(S)                  (char*)mxArrayToString(ssGetSFcnParam(S, 0)) /* "setup", "rx", "tx" */
#define PORT(S)                (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* "COM1", "COM2",... */
#define BAUDRATE(S)            (int)mxGetScalar(ssGetSFcnParam(S, 2)) /* 9600,57600,115200,... */
#define DATABITS(S)            (int)mxGetScalar(ssGetSFcnParam(S, 3)) /* 8, 9 */
#define PARITY(S)              (char*)mxArrayToString(ssGetSFcnParam(S, 4)) /* "None", "Odd", "Even" */
#define STOPBITS(S)            (char*)mxArrayToString(ssGetSFcnParam(S, 5)) /* "1", "1.5", "2" */
#define TIMEOUT(S)             (int)mxGetScalar(ssGetSFcnParam(S, 6)) /* 10000 */
#define PACKETMODE(S)          (char*)mxArrayToString(ssGetSFcnParam(S, 7)) /* "Ascii", "Binary" */
#define ASCII_FORMAT(S)        (char*)mxArrayToString(ssGetSFcnParam(S, 8)) /* "Value=%d\n" */
#define ASCII_TYPEARRAY(S)     (char*)mxArrayToString(ssGetSFcnParam(S, 9)) /*  */
#define ASCII_TYPECOUNT(S)     mxGetScalar(ssGetSFcnParam(S, 10)) /*  */
#define BIN_HEADERARRAY(S)     ssGetSFcnParam(S, 11) /* Header array */
#define BIN_HEADERCOUNT(S)     (int)mxGetScalar(ssGetSFcnParam(S, 12)) /* Header count */
#define BIN_TERMINATORARRAY(S) ssGetSFcnParam(S, 13) /* Used pin id array */
#define BIN_TERMINATORCOUNT(S) (int)mxGetScalar(ssGetSFcnParam(S, 14)) /* Used pin count */
#define BIN_DATATYPEARRAY(S)   ssGetSFcnParam(S, 15) /* Data type array */
#define BIN_DATATYPECOUNT(S)   (int)mxGetScalar(ssGetSFcnParam(S, 16)) /* Data type count */
#define SAMPLETIME(S)          mxGetScalar(ssGetSFcnParam(S, 17)) /* Sample time (sec) */
#define SAMPLETIMESTR(S)       (char*)mxArrayToString(ssGetSFcnParam(S, 18)) /* Compiled sample time (sec) in string */
#define BLOCKID(S)             (char*)mxArrayToString(ssGetSFcnParam(S, 19)) /* BlockID */


#define MAX_IN_OUT_PORT         128 /* Maximum number allowed for port signal  */
#define MAX_DATA_TRANS_RECEIVE       1024 /* Maximum bytes a packet of transmit and receive  */

#define PORT_HANDLE_COUNT         128
#define MAX_PORTNAME_COUNT      32
typedef struct _PORT_INFO {
    char m_PortName[MAX_PORTNAME_COUNT]; /* Port Name max 32 byte */
    ULONG m_Timeout;
    ULONG m_BaudRate;
    BYTE m_DataBits;
    BYTE m_StopBits;
    BYTE m_Parity;
    
    /* Debug */
    BOOL m_FirstRead;
    ULONG m_ForceOutCounter;
    /* Handle */
    HANDLE	m_hComm;
    HANDLE	m_hEvent;
} PORT_HANDLE;

union _Conv_Val {
    BYTE buffer[8];
    double val_double;
    float val_single;
    signed char val_int8;
    unsigned char val_uint8;
    signed short val_int16;
    unsigned short val_uint16;
    int val_int32;
    unsigned int val_uint32;
} Conv_Val;

/* Port info */
static PORT_HANDLE PortHandleList[PORT_HANDLE_COUNT];
static UINT PortInfo_Counter = 0;

/*====================*
 * COM *
 *====================*/
/*
 * void Serial_Close(PORT_HANDLE* port);
 *
 * static void Serial_Init(PORT_HANDLE* port) {
 * port->m_hEvent = NULL;
 * // automatically close our serial port
 * Serial_Close(port);
 * }
 */

static void Serial_Close(PORT_HANDLE* port) {
    if( port->m_hComm ) {
        CloseHandle( port->m_hComm );
        port->m_hComm = NULL;
    }
}

static bool Serial_Open(PORT_HANDLE* port, LPCTSTR lpszComm, DWORD dwDesiredAccess,
        LPSECURITY_ATTRIBUTES lpSecurityAttributes, DWORD dwFlagsAndAttributes ) {
    Serial_Close(port);
    port->m_hComm = CreateFile( lpszComm, dwDesiredAccess, 0,
            lpSecurityAttributes, OPEN_EXISTING, dwFlagsAndAttributes, NULL );
    if( port->m_hComm == INVALID_HANDLE_VALUE )
        return FALSE;
    return TRUE;
}

static bool Serial_Read(PORT_HANDLE* port, LPVOID lpBuffer, DWORD nNumberOfBytesToRead,
        LPDWORD lpNumberOfBytesRead, LPOVERLAPPED lpOverlapped) {
    return ReadFile(port->m_hComm, lpBuffer, nNumberOfBytesToRead,
            lpNumberOfBytesRead, lpOverlapped );
}

static bool Serial_Write(PORT_HANDLE* port, LPCVOID lpBuffer, DWORD nNumberOfBytesToWrite,
        LPDWORD lpNumberOfBytesWritten, LPOVERLAPPED lpOverlapped) {
    return WriteFile( port->m_hComm, lpBuffer, nNumberOfBytesToWrite,
            lpNumberOfBytesWritten, lpOverlapped );
}

static bool Serial_IsOpen(PORT_HANDLE* port) {
    return (BOOL)(port->m_hComm != NULL);
}

static bool Serial_GetComNumber(LPCSTR com_str, BYTE* com_no) {
    DWORD tmp = 0;
    if(sscanf_s(com_str, (char*)"COM%d", &tmp)) {
        *com_no = (BYTE)tmp;
        return TRUE;
    }
    else
        return FALSE;
}

// See:
// http://msdn.microsoft.com/en-us/library/windows/desktop/aa363214(v=vs.85).aspx
static bool Serial_OpenPort(PORT_HANDLE* port) {
    DCB dcb;
    COMMTIMEOUTS timeouts;
    BYTE com_no;
    char PortForOpen[32];
    /* Check if port already open */
    if(!Serial_IsOpen(port)) {
        if(Serial_GetComNumber(port->m_PortName, &com_no) == FALSE)
            return FALSE;
        if(com_no > 9)
            sprintf_s(PortForOpen, 32, ("\\\\.\\%s"), (LPCSTR)port->m_PortName);
        else
            sprintf_s(PortForOpen, 32, ("%s"), (LPCSTR)port->m_PortName);
        
        /* Open port */
        if(Serial_Open(port,
                (LPCTSTR)PortForOpen,
                GENERIC_READ|GENERIC_WRITE,
                NULL,
                FILE_FLAG_OVERLAPPED) == FALSE)
            return FALSE;
        
        /* Setup serial state */
        memset(&dcb, 0, sizeof(DCB));
        GetCommState( port->m_hComm, &dcb );
        dcb.BaudRate = port->m_BaudRate;//CBR_38400;
        dcb.ByteSize = port->m_DataBits;//8;//
        dcb.StopBits = port->m_StopBits; //ONESTOPBIT;
        dcb.fParity = FALSE;
        dcb.Parity = NOPARITY;
        
        dcb.fOutxCtsFlow = FALSE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fAbortOnError = FALSE;
        dcb.fNull = FALSE;
        if( !SetCommState( port->m_hComm, &dcb ))
            return FALSE;
        SetupComm(port->m_hComm, 4096, 4096 ); // 4K buffer in-out
        // purge any data in serial port
        if(!PurgeComm( port->m_hComm, PURGE_RXCLEAR|PURGE_TXCLEAR|PURGE_TXABORT|PURGE_RXABORT))
            return FALSE;
        
        //EscapeCommFunction( port->m_hComm, SETRTS );
        //EscapeCommFunction( port->m_hComm, SETDTR );
        
        // set comms timeouts
        GetCommTimeouts( port->m_hComm, &timeouts );
        timeouts.ReadIntervalTimeout = port->m_Timeout;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        timeouts.ReadTotalTimeoutConstant = port->m_Timeout;
        timeouts.WriteTotalTimeoutMultiplier = 1;
        timeouts.WriteTotalTimeoutConstant = 5000;
        
        if( !SetCommTimeouts( port->m_hComm, &timeouts ))
            return FALSE;
    }
    
    /* Result */
    return TRUE;
}



/*====================*
 * Helper function *
 *====================*/
static int GetPortIndex(char* PortName) {
    UINT i = 0;
    
    /* Search port index */
    while(i < PortInfo_Counter) {
        /* Found */
        if(!strcmp((char*)PortName, (char*)(PortHandleList[i].m_PortName))) {
            return i;
        }
        i++;
    }
    /* Port is not found in list */
    return -1;
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

static bool WriteToPort(char* PortName, BYTE* buffer, DWORD count) {
    OVERLAPPED ovl;
    DWORD dwBytesWritten;
    DWORD dwRet;
    DWORD dwTimeout = 1000;
    int port_index = -1;
    
    /* Get port, check if port is open */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return FALSE;
    if(!Serial_IsOpen(&PortHandleList[port_index]))
        return FALSE;
    
    /* Init overlap mode */
    memset(&ovl, 0, sizeof(ovl));
    ovl.hEvent = PortHandleList[port_index].m_hEvent;
    
    /* Write to port */
    if( !Serial_Write(&PortHandleList[port_index], (LPCVOID)&buffer[0], count, &dwBytesWritten, &ovl )) {
        if( GetLastError() != ERROR_IO_PENDING )
            return FALSE;
        dwRet = WaitForSingleObject( ovl.hEvent, dwTimeout );
        switch( dwRet ) {
            case WAIT_OBJECT_0:
                if( !GetOverlappedResult( (PortHandleList[port_index].m_hComm), &ovl, &dwBytesWritten, FALSE ))
                    return FALSE;
                break;
                
            case WAIT_TIMEOUT:
                PurgeComm( (PortHandleList[port_index].m_hComm), PURGE_TXABORT|PURGE_TXCLEAR );
                return FALSE;
                
            default:
                return FALSE;
        }
    }
    
    /* Result */
    return TRUE;
}

static bool ReadByte(char* PortName, DWORD timeout, BYTE* data) {
    OVERLAPPED ovl;
    DWORD dwRet;
    DWORD dwBytesRead = 0;
    BYTE bff[16];
    int port_index = -1;
    
    /* Get port, check if port is open */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return FALSE;
    if(!Serial_IsOpen(&PortHandleList[port_index]))
        return FALSE;
    
    /* Init overlap mode */
    memset(&ovl, 0, sizeof(ovl));
    ovl.hEvent = PortHandleList[port_index].m_hEvent;
    
    /* Read */
    if( !Serial_Read(&PortHandleList[port_index], (LPVOID)&bff[0], 1, &dwBytesRead, &ovl)) {
        if( GetLastError() != ERROR_IO_PENDING )
            return FALSE;
        dwRet = WaitForSingleObject(ovl.hEvent, timeout);
        switch(dwRet) {
            case WAIT_OBJECT_0:
                if(!GetOverlappedResult( PortHandleList[port_index].m_hComm, &ovl, &dwBytesRead, FALSE))
                    return FALSE;
                if(dwBytesRead < 1)
                    return FALSE;
                break;
                
            case WAIT_TIMEOUT:
                PurgeComm(PortHandleList[port_index].m_hComm, PURGE_RXABORT|PURGE_RXCLEAR);
                return FALSE;
                
            default:
                //CancelIO();
                return FALSE;
        }
    }
    *data = bff[0];
    return TRUE;
}

static bool ReadBytes(char* PortName, DWORD timeout, UINT count, BYTE* data) {
    static OVERLAPPED ovl;
    static DWORD dwRet;
    static DWORD dwBytesRead = 0;
    static int port_index = -1;
    
    /* Get port, check if port is open */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return FALSE;
    if(!Serial_IsOpen(&PortHandleList[port_index]))
        return FALSE;
    
    /* Init overlap mode */
    memset(&ovl, 0, sizeof(ovl));
    ovl.hEvent = PortHandleList[port_index].m_hEvent;
    
    /* Read */
    if( !Serial_Read(&PortHandleList[port_index], (LPVOID)data, count, &dwBytesRead, &ovl)) {
        if( GetLastError() != ERROR_IO_PENDING )
            return FALSE;
        dwRet = WaitForSingleObject(ovl.hEvent, timeout);
        switch(dwRet) {
            case WAIT_OBJECT_0:
                if(!GetOverlappedResult( PortHandleList[port_index].m_hComm, &ovl, &dwBytesRead, FALSE))
                    return FALSE;
                if(dwBytesRead < 1)
                    return FALSE;
                break;
                
            case WAIT_TIMEOUT:
                PurgeComm(PortHandleList[port_index].m_hComm, PURGE_RXABORT|PURGE_RXCLEAR);
                return FALSE;
                
            default:
                //CancelIO();
                return FALSE;
        }
    }
    return TRUE;
}

static bool IfFirstRead(char* PortName) {
    int port_index;
    
    /* Get port */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return FALSE;
    
    if(PortHandleList[port_index].m_FirstRead) {
        if(PortHandleList[port_index].m_ForceOutCounter > 0) {
            PortHandleList[port_index].m_ForceOutCounter--;
            return TRUE;
        }
        else {
            PortHandleList[port_index].m_FirstRead = FALSE;
        }
    }
    else {
        return FALSE;
    }
    
    /* No error finally */
    return TRUE;
}

static bool ClosePort(char* PortName) {
    int port_index;
    
    /* Get port */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return FALSE;
    
    /* Close */
    Serial_Close(&PortHandleList[port_index]);
    
    /* No error finally */
    return TRUE;
}

static bool OpenPort(char* PortName, ULONG BaudRate, BYTE DataBits, BYTE StopBits, ULONG Timeout_ms) {
    int port_index = -1;
    
    /* Get port */
    port_index = GetPortIndex(PortName);
    
    /* Check if port is already existing and open */
    if(port_index >= 0) {
        if(Serial_IsOpen(&PortHandleList[port_index]))
            return TRUE;
    }
    /* Open new port */
    else {
        port_index = PortInfo_Counter;
        strcpy(PortHandleList[port_index].m_PortName, PortName);
        PortInfo_Counter ++;
    }
    
    /* Limit the port name */
    if(strlen(PortName) > (MAX_PORTNAME_COUNT-1))
        return FALSE;
    
    /* COM port */
    PortHandleList[port_index].m_hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );;
    PortHandleList[port_index].m_hComm = NULL;
    PortHandleList[port_index].m_BaudRate = BaudRate;
    PortHandleList[port_index].m_DataBits = DataBits;
    PortHandleList[port_index].m_StopBits = StopBits;
    PortHandleList[port_index].m_FirstRead = TRUE;
    PortHandleList[port_index].m_Timeout = Timeout_ms;
    PortHandleList[port_index].m_ForceOutCounter = 2;
    if(!Serial_OpenPort(&PortHandleList[port_index])) {
        return FALSE;
    }
    
    /* No error finally */
    return TRUE;
}

static bool PortIsOpen(char* PortName) {
    int port_index = -1;
    
    /* Get port, check if port is open */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return FALSE;
    return Serial_IsOpen(&PortHandleList[port_index]);
}

static void mdlInitializeSizes(SimStruct *S) {
    char *strID;
    
    unsigned int i;
    //unsigned int host_com_datatype_id[MAX_IN_OUT_PORT];
    unsigned int host_com_datatype_id_count = 0;
    unsigned int host_com_datatype_id_rx_count = 0;
    unsigned int host_com_datatype_id_tx_count = 0;
    
    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        /* mdlCheckParameters(S);
         */
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    #endif
            
            ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);
    
    strID = mxArrayToString(ssGetSFcnParam(S, UART_ID));
    
    /* Determine number of port */
    if(!strncmp(strID, "tx", 2)) {
        host_com_datatype_id_rx_count = 0;
        host_com_datatype_id_tx_count = BIN_DATATYPECOUNT(S);
    }
    else if(!strncmp(strID, "rx", 2)) {
        host_com_datatype_id_rx_count = BIN_DATATYPECOUNT(S);
        host_com_datatype_id_tx_count = 0;
    }
    else if(!strncmp(strID, "setup", 5)) {
        host_com_datatype_id_rx_count = 0;
        host_com_datatype_id_tx_count = 0;
    }
    else {
        ssSetErrorStatus(S, "Invalid block: allowed rx, tx and cf only\n.");
    }
    
    /* Free ID*/
    mxFree(strID);
    
    /*Input Port */
    if (ssSetNumInputPorts(S, host_com_datatype_id_tx_count)) {
        for(i=0; i<host_com_datatype_id_tx_count; i++) {
            ssSetInputPortComplexSignal(S,  i, COMPLEX_NO);
            ssSetInputPortDirectFeedThrough(S, i, true);
            ssSetInputPortRequiredContiguous(S, i, 1); //direct input signal access
            ssSetInputPortWidth(S,  i, 1);
            ssSetInputPortDataType(S, i, (int)(((double*)mxGetData(BIN_DATATYPEARRAY(S)))[i]));
        }
    }
    else {
        ssSetErrorStatus(S, "Failed to set num of input port.");
    }
    
    /* Output Port*/
    if (ssSetNumOutputPorts(S, host_com_datatype_id_rx_count)) {
        for(i=0; i<host_com_datatype_id_rx_count; i++) {
            ssSetOutputPortComplexSignal(S,  i, COMPLEX_NO);
            ssSetOutputPortWidth(S,  i, 1);
            ssSetOutputPortComplexSignal(S, i, COMPLEX_NO);
            ssSetOutputPortWidth(S,  i, 1);
            ssSetOutputPortDataType(S, i, (int)(((double*)mxGetData(BIN_DATATYPEARRAY(S)))[i]));
        }
    }
    else {
        ssSetErrorStatus(S, "Failed to set num of output port.");
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_WORKS_WITH_CODE_REUSE));
    
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLETIME(S));
    ssSetOffsetTime(S, 0, 0.0);
} /* end mdlInitializeSampleTimes */

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

# define MDL_SET_INPUT_PORT_FRAME_DATA
static void mdlSetInputPortFrameData(SimStruct  *S,
        int_T      port,
        Frame_T    frameData) {
    ssSetInputPortFrameData(S, port, frameData);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType) {
    ssSetInputPortDataType( S, 0, dType);
}
#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType) {
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S) {
    ssSetInputPortDataType( S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    
    int i;
    int byte_index;
    real_T reading;
    
    real_T tmp_double = 0; /* double */
    real32_T ss; /* Single */
    uint8_T u8;
    uint16_T u16;
    uint32_T u32;
    int8_T s8;
    int16_T s16;
    int32_T s32;
    
    real_T *ptmp_double = 0; /* double */
    real32_T *pss; /* Single */
    uint8_T *pu8;
    uint16_T *pu16;
    uint32_T *pu32;
    int8_T *ps8;
    int16_T *ps16;
    int32_T *ps32;
    
    /* Used by Ascii mode */
    uint8_T ascii_tmp_formatted_string[MAX_DATA_TRANS_RECEIVE];
    uint8_T ascii_tmp_segmented_string[MAX_DATA_TRANS_RECEIVE];
    char* ascii_pformat_str;
    int ascii_segment_index = 0;
    uint32_T tmp_read_values[MAX_IN_OUT_PORT];
    real32_T tmp_write_values[MAX_IN_OUT_PORT];
    int tmp_read_values_count;
    
    
    uint16_T tmp_buffer_count = 0;
    uint8_T write_buffer[MAX_DATA_TRANS_RECEIVE];
    uint32_T write_buffer_count = 0;
    
    uint8_T read_buffer[MAX_DATA_TRANS_RECEIVE];
    uint32_T read_buffer_count = 0;
    
    BYTE host_com_writedatatype_id[MAX_IN_OUT_PORT];
    int host_com_writedatatype_id_count = 0;
    BYTE host_com_readdatatype_id[MAX_IN_OUT_PORT];
    int host_com_readdatatype_id_count = 0;
    /* Check port open */
    char error_msg[1024];
    
    int rx_error_count;
    
    /* ID type */
    char* strID;
    char* strPort;
    char* strPacketmode;
    int id;
    char* strAsciiformat;
    
    char port[64];
    char packetmode[64];
    char ascii_format[MAX_DATA_TRANS_RECEIVE]; //ASCII_FORMAT(S)
    char ascii_read_buffer[MAX_DATA_TRANS_RECEIVE+1];
    int ascii_readcount;
    int ascii_terminator_index;
    
   
    uint8_T buffer1[8];
    uint8_T buffer2[8];
    uint8_T buffer3[8];
    uint8_T buffer4[8];
    uint8_T buffer5[8];
    uint8_T buffer6[8];
    uint8_T buffer7[8];
    uint8_T buffer8[8];
    uint8_T buffer9[8];
    uint8_T buffer10[8];
    uint8_T buffer11[8];
    uint8_T buffer12[8];
    uint8_T buffer13[8];
    uint8_T buffer14[8];
    uint8_T buffer15[8];
    uint8_T buffer16[8];
    
    uint8_T*bufferlist[16] =
    {
        buffer1, buffer2, buffer3, buffer4, buffer5, buffer6, buffer7,
        buffer8, buffer9, buffer10, buffer11, buffer12, buffer13, buffer14,
        buffer15, buffer16        
    };
    
    tmp_buffer_count = 0;
    host_com_writedatatype_id_count = 0;
    host_com_readdatatype_id_count = 0;
    
    /* Find ID type of block */
    strID = mxArrayToString(ssGetSFcnParam(S, UART_ID));
    if(!strcmp(strID, "setup")) { id = 0; }
    else if(!strcmp(strID, "tx")) { id = 1; }
    else if(!strcmp(strID, "rx")) { id = 2; }
    else { id = -1; }
    mxFree(strID);
    
    if(id < 0) {
        ssSetErrorStatus(S, "Invalid block\n");
        return;
    }
    
    /* Port */
    strPort = mxArrayToString(ssGetSFcnParam(S, UART_PORT));
    strcpy(port, strPort);
    mxFree(strPort);
    
    /* Packet mode */
    strPacketmode = mxArrayToString(ssGetSFcnParam(S, UART_PACKETMODE));
    strcpy(packetmode, strPacketmode);
    mxFree(strPacketmode);
    
    /* Ascii format */
    strAsciiformat = (char*)mxArrayToString(ssGetSFcnParam(S, UART_ASCII_FORMAT));
    strcpy(ascii_format, strAsciiformat);
    mxFree(strAsciiformat);   
    
    
    /* Check port is opened */
    if(!PortIsOpen(port)) {
        sprintf(error_msg, "Missing setup for \"%s\".", port);
        ssSetErrorStatus(S, (char*)error_msg);
        return;
    }
    
    
    
    /* ********************************************************************
     * Tx block
     * ********************************************************************/
    switch(id) {
        case 1: /* Tx */
            /* Get input type id list */
            host_com_writedatatype_id_count = ssGetNumInputPorts(S);
            for(i=0; i<host_com_writedatatype_id_count; i++)
                host_com_writedatatype_id[i] = (BYTE)ssGetInputPortDataType(S, i);
            
            write_buffer_count = 0;
            /* Collect data buffer */
            switch(strcmp(packetmode, "Binary")) {
                case 0: /* Binary */
                    /* Header */
                    tmp_buffer_count = 0;
                    while(tmp_buffer_count < BIN_HEADERCOUNT(S)) {
                        write_buffer[write_buffer_count+tmp_buffer_count] = (uint8_T)\
                                (((double*)mxGetPr(BIN_HEADERARRAY(S)))[tmp_buffer_count]);
                        tmp_buffer_count++;
                    }
                    write_buffer_count += tmp_buffer_count;
                    /* Bin data */
                    for(i=0; i<host_com_writedatatype_id_count; i++) {
                        switch(host_com_writedatatype_id[i]) {
                            case 0: /* Double */
                                tmp_double = *(const real_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_double = tmp_double;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 8);
                                write_buffer_count += 8;
                                break;
                            case 1: /* Single */
                                ss = *(const real32_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_single = ss;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 4);
                                write_buffer_count += 4;
                                break;
                            case 2: /* int8 */
                                s8 = *(const int8_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_int8 = s8;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 1);
                                write_buffer_count += 1;
                                break;
                            case 3: /* uint8 */
                                u8 = *(const uint8_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_uint8 = u8;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 1);
                                write_buffer_count += 1;
                                break;
                            case 4: /* int16 */
                                s16 = *(const int16_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_int16 = s16;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 2);
                                write_buffer_count += 2;
                                break;
                            case 5: /* uint16 */
                                u16 = *(const uint16_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_uint16 = u16;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 2);
                                write_buffer_count += 2;
                                break;
                            case 6: /* int32 */
                                s32 = *(const int32_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_int32 = s32;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 4);
                                write_buffer_count += 4;
                                break;
                            case 7: /* uint32 */
                                u32 = *(const uint32_T*) ssGetInputPortSignal(S, i);
                                Conv_Val.val_uint32 = u32;
                                memcpy(&(write_buffer[write_buffer_count]), Conv_Val.buffer, 4);
                                write_buffer_count += 4;
                                break;
                            default:
                                ssSetErrorStatus(S, "Invalid Bin data type id.");
                                break;
                        }
                    }
                    /* Terminator */
                    tmp_buffer_count = 0;
                    while(tmp_buffer_count < BIN_TERMINATORCOUNT(S)) {
                        write_buffer[write_buffer_count+tmp_buffer_count] = (uint8_T)\
                                (((double*)mxGetPr(BIN_TERMINATORARRAY(S)))[tmp_buffer_count]);
                        tmp_buffer_count++;
                    }
                    write_buffer_count += tmp_buffer_count;
                    break;
                    
                default: /* Ascii */
                    if(strcmp(packetmode, "Ascii")) {
                        ssSetErrorStatus(S, "Invalid mode!\n");
                        return;
                    }
                    
                    if(host_com_writedatatype_id_count > 16) {
                        ssSetErrorStatus(S, "Maximum number of port is 16.\n");
                        return;
                    }
                    
                    ascii_pformat_str = ascii_format;
                    for(i=0; i<host_com_writedatatype_id_count; i++) {
                        // Get first formatted
                        if(*ascii_pformat_str) {
                            ascii_segment_index = GetFormattedSegment(ascii_pformat_str);
                            strncpy_s(ascii_tmp_segmented_string,
                                    sizeof(ascii_tmp_segmented_string),
                                    ascii_pformat_str,
                                    ascii_segment_index);
                            ascii_tmp_segmented_string[ascii_segment_index+1] = '\0';
                            
                            switch(host_com_writedatatype_id[i]) {
                                case 0: /* Double */
                                    tmp_double = *(const real_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_double = tmp_double;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(double*)&(Conv_Val.buffer[0]));
                                    break;
                                case 1: /* Single */
                                    ss = *(const real32_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_single = ss;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(float*)&(Conv_Val.buffer[0]));
                                    break;
                                case 2: /* int8 */
                                    s8 = *(const int8_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_int8 = s8;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(char*)&(Conv_Val.buffer[0]));
                                    break;
                                case 3: /* uint8 */
                                    u8 = *(const uint8_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_uint8 = u8;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(BYTE*)&(Conv_Val.buffer[0]));
                                    break;
                                case 4: /* int16 */
                                    s16 = *(const int16_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_int16 = s16;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(short*)&(Conv_Val.buffer[0]));
                                    break;
                                case 5: /* uint16 */
                                    u16 = *(const uint16_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_uint16 = u16;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(WORD*)&(Conv_Val.buffer[0]));
                                    break;
                                case 6: /* int32 */
                                    s32 = *(const int32_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_int32 = s32;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(INT*)&(Conv_Val.buffer[0]));
                                    break;
                                case 7: /* uint32 */
                                    u32 = *(const uint32_T*) ssGetInputPortSignal(S, i);
                                    Conv_Val.val_uint32 = u32;
                                    write_buffer_count += sprintf_s((char*)&write_buffer[write_buffer_count],
                                            (MAX_DATA_TRANS_RECEIVE-write_buffer_count),
                                            (char*)ascii_tmp_segmented_string, *(UINT*)&(Conv_Val.buffer[0]));
                                    break;
                            }
                            ascii_pformat_str = &ascii_pformat_str[ascii_segment_index];
                        }
                    }
            }
            
            /* Write data to port */
            if(!WriteToPort(port, (BYTE*)write_buffer, write_buffer_count)) {
                ssSetErrorStatus(S, "Failed to write buffer to port!\n");
                return;
            }
            break;
            
            /* ********************************************************************
             * Rx block
             * ********************************************************************/
        case 2: /* Rx */
            /* Get output type id list */
            host_com_readdatatype_id_count = ssGetNumOutputPorts(S);
            for(i=0; i<host_com_readdatatype_id_count; i++)
                host_com_readdatatype_id[i] = (BYTE)ssGetOutputPortDataType(S, i);
            
            /* Read mode */
            switch(strcmp(packetmode, "Binary")) {
                case 0: /* Binary */
                    /* ####################################################
                     * Header
                     * ####################################################
                     */
                    
                    tmp_buffer_count = 0;
                    while(tmp_buffer_count < BIN_HEADERCOUNT(S)) {
                        write_buffer[write_buffer_count+tmp_buffer_count] = (uint8_T)\
                                (((double*)mxGetPr(BIN_HEADERARRAY(S)))[tmp_buffer_count]);
                        tmp_buffer_count++;
                    }
                    read_buffer_count = tmp_buffer_count;
                    
                    /* Search for header */
                    if(ReadBytes(port,
                            (DWORD)(TIMEOUT(S)),
                            read_buffer_count,
                            read_buffer)) {
                        if(memcmp(write_buffer, read_buffer, read_buffer_count)) {
                            goto __search;
                        }
                    }
                    else {
                        __search:
                            rx_error_count = 0;
                            tmp_buffer_count = 0;
                            while(tmp_buffer_count <read_buffer_count) {
                                if(ReadByte(port, (DWORD)(TIMEOUT(S)), &read_buffer[0])) {
                                    if(read_buffer[0] == write_buffer[tmp_buffer_count]) {
                                        tmp_buffer_count++;
                                    }
                                    else {
                                        if(++rx_error_count >= 128) {
                                            ssSetErrorStatus(S, "The receive packet header did not matched!\n");
                                            return;
                                        }
                                        tmp_buffer_count = 0;
                                    }
                                }
                                else {
                                    printf("Re-try\n");
                                    if(IfFirstRead(port))
                                        return;
                                    else {
                                        ssSetErrorStatus(S, "Failed to receive header from port!");
                                        return;
                                    }
                                }
                            }
                    }
                    
                    /* Data */
                    read_buffer_count = 0;
                    for(i=0; i<host_com_readdatatype_id_count; i++) {
                        switch(host_com_readdatatype_id[i]) {
                            case 0: read_buffer_count += 8; /* Double */ break;
                            case 1: read_buffer_count += 4; /* Single */ break;
                            case 2: read_buffer_count += 1; /* int8 */ break;
                            case 3: read_buffer_count += 1; /* uint8 */ break;
                            case 4: read_buffer_count += 2; /* int16 */ break;
                            case 5: read_buffer_count += 2; /* uint16 */ break;
                            case 6: read_buffer_count += 4; /* int32 */ break;
                            case 7: read_buffer_count += 4; /* uint32 */ break;
                            default:
                                ssSetErrorStatus(S, "Invalid Bin data type id.");
                                break;
                        }
                    }
                    if(read_buffer_count>0) {
                        if(!ReadBytes(port,
                                (DWORD)(TIMEOUT(S)),
                                read_buffer_count,
                                read_buffer)) {
                            if(IfFirstRead(port))
                                return;
                            else
                                ssSetErrorStatus(S, "Failed to read data from port!");
                            return;
                        }
                        byte_index = 0;
                        
                        for(i=0; i<host_com_readdatatype_id_count; i++) {
                            switch(host_com_readdatatype_id[i]) { /* Convert and set to port */
                                case 0: /* Double */
                                    ptmp_double = (real_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(real_T*)&(read_buffer[byte_index]);
                                    byte_index+=8;
                                    *ptmp_double = (real_T)reading;
                                    break;
                                case 1: /* Single */
                                    pss = (real32_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(real32_T*)&(read_buffer[byte_index]);
                                    byte_index+=4;
                                    *pss = (real32_T)reading;
                                    break;
                                case 2: /* int8 */
                                    ps8 = (int8_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(int8_T*)&(read_buffer[byte_index]);
                                    byte_index+=1;
                                    *ps8 = (int8_T)reading;
                                    break;
                                case 3: /* uint8 */
                                    pu8 = (uint8_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(uint8_T*)&(read_buffer[byte_index]);
                                    byte_index+=1;
                                    *pu8 = (uint8_T)reading;
                                    break;
                                case 4: /* int16 */
                                    ps16 = (int16_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(int16_T*)&(read_buffer[byte_index]);
                                    byte_index+=2;
                                    *ps16 = (int16_T)reading;
                                    break;
                                case 5: /* uint16 */
                                    pu16 = (uint16_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(uint16_T*)&(read_buffer[byte_index]);
                                    byte_index+=2;
                                    *pu16 = (uint16_T)reading;
                                    break;
                                case 6: /* int32 */
                                    ps32 = (int32_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(int32_T*)&(read_buffer[byte_index]);
                                    byte_index+=4;
                                    *ps32 = (int32_T)reading;
                                    break;
                                case 7: /* uint32 */
                                    pu32 = (uint32_T*) ssGetOutputPortSignal(S, i);
                                    reading = *(uint32_T*)&(read_buffer[byte_index]);
                                    byte_index+=4;
                                    *pu32 = (uint32_T)reading;
                                    break;
                                default:
                                    ssSetErrorStatus(S, "Invalid Bin data type id.");
                                    break;
                            }
                        }
                        
                    }
                    /* Terminator */                    
                    tmp_buffer_count = 0;
                    while(tmp_buffer_count < BIN_TERMINATORCOUNT(S)) {
                        write_buffer[write_buffer_count+tmp_buffer_count] = (uint8_T)\
                                (((double*)mxGetPr(BIN_TERMINATORARRAY(S)))[tmp_buffer_count]);
                        tmp_buffer_count++;
                    }
                    read_buffer_count = tmp_buffer_count;
                    if(read_buffer_count > 0) {
                        if(!ReadBytes(port,
                                (DWORD)(TIMEOUT(S)),
                                read_buffer_count,
                                read_buffer)) {
                            if(IfFirstRead(port))
                                return;
                            else
                                ssSetErrorStatus(S, "Failed to read terminator from port!");
                            return;
                        }
                        if(memcmp(write_buffer, read_buffer, read_buffer_count)) {
                            ssSetErrorStatus(S, "Received terminator mismatch!");
                            return;
                        }
                    }
                    break;
                    
                default: /* Ascii */
                    if(strcmp(packetmode, "Ascii") != 0) {
                        ssSetErrorStatus(S, "Invalid mode.");
                        return;
                    }
                    
                    /* Validat number of terminator */
                    if(BIN_TERMINATORCOUNT(S) < 1) {
                        ssSetErrorStatus(S, "Terminator must be at lease 1 charactor.\n");
                        return;
                    }
                    
                    rx_error_count = 0;
                    __retry_ascii_rx:
                    
                    /* Get input */
                    ascii_readcount = 0;
                    ascii_terminator_index = 0;
                    while (ascii_terminator_index < BIN_TERMINATORCOUNT(S)) {
                        if(ReadByte(port, (DWORD)(TIMEOUT(S)), &read_buffer[0])) {
                                                        
                            /* Store the reading byte into buffer */
                            ascii_read_buffer[ascii_readcount] = read_buffer[0];
                            ascii_readcount++;
                            
                            /* Check if bytes received over flow */
                            if(ascii_readcount>=MAX_DATA_TRANS_RECEIVE) {
                                ssSetErrorStatus(S, "Could not found terminator!\n");
                                return;
                            }
                            
                            /* Check terminator */
                            if((uint8_T)(((double*)mxGetPr(BIN_TERMINATORARRAY(S)))[ascii_terminator_index]) == read_buffer[0]) {
                                ascii_terminator_index++;
                            }
                            else {
                                ascii_terminator_index = 0;
                            }
                        }
                        else {
                            ascii_readcount = 0;
                            if(IfFirstRead(port))
                                return;
                            else
                                ssSetErrorStatus(S, "Failed to read terminator from port!");
                            
                            return;
                        }
                    }
                    
                    /* Append NULL as string terminator */
                    ascii_read_buffer[ascii_readcount] = 0;
                    
                    /* Scanf */
                    if(sscanf(ascii_read_buffer, ascii_format, 
                            buffer1, buffer2, buffer3, buffer4, buffer5,
                            buffer6, buffer7, buffer8, buffer9, buffer10,
                            buffer11, buffer12, buffer13, buffer14, buffer15,
                            buffer16) != host_com_readdatatype_id_count) {
                        
                        if(++rx_error_count < 16) {                            
                            printf("Re-try to receive Rx ascii packet.\n");
                            goto __retry_ascii_rx;
                        }
                        else {                        
                            ssSetErrorStatus(S, "Failed to scan data");
                            return;
                        }
                    }
                    
                    /* Set data to port */
                    for(i=0; i<host_com_readdatatype_id_count; i++) {
                        switch(host_com_readdatatype_id[i]) { /* Convert and set to port */
                            case 0: /* Double */
                                ptmp_double = (real_T*) ssGetOutputPortSignal(S, i);
                                *ptmp_double = *(real_T*)bufferlist[i];
                                break;
                            case 1: /* Single */
                                pss = (real32_T*) ssGetOutputPortSignal(S, i);
                                *pss = *(real32_T*)bufferlist[i];
                                break;
                            case 2: /* int8 */
                                ps8 = (int8_T*) ssGetOutputPortSignal(S, i);
                                *ps8 = *(int8_T*)bufferlist[i];
                                break;
                            case 3: /* uint8 */
                                pu8 = (uint8_T*) ssGetOutputPortSignal(S, i);
                                *pu8 = *(uint8_T*)bufferlist[i];
                                break;
                            case 4: /* int16 */
                                ps16 = (int16_T*) ssGetOutputPortSignal(S, i);
                                *ps16 = *(int16_T*)bufferlist[i];
                                break;
                            case 5: /* uint16 */
                                pu16 = (uint16_T*) ssGetOutputPortSignal(S, i);
                                *pu16 = *(uint16_T*)bufferlist[i];
                                break;
                            case 6: /* int32 */
                                ps32 = (int32_T*) ssGetOutputPortSignal(S, i);
                                *ps32 = *(int32_T*)bufferlist[i];
                                break;
                            case 7: /* uint32 */
                                pu32 = (uint32_T*) ssGetOutputPortSignal(S, i);
                                *pu32 = *(uint32_T*)bufferlist[i];
                                break;
                            default:
                                ssSetErrorStatus(S, "Invalid Bin data type id.");
                                break;
                        }
                    }
                    
                    break;
            }
            break;
            
            /* ********************************************************************
             * Configuration block
             * ********************************************************************/
        case 0:
            break;
    }
} /* end mdlOutputs */

/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
#define MDL_START
static void mdlStart(SimStruct *S) {
    uint8_T data_bits;
    ULONG stop_bits;
    ULONG tmout;
    char error_msg[1024];
    char* strID;
    int id;
    
    char *strPort;
    char port[64];
    
    char *strStopbits;
    char stopbits[64];
    
    
    /* ID */
    strID = mxArrayToString(ssGetSFcnParam(S, UART_ID));
    if(!strcmp(strID, "setup")) { id = 0; }
    else { id = -1; }
    mxFree(strID);
    
    /* Port */
    strPort = mxArrayToString(ssGetSFcnParam(S, UART_PORT));
    strcpy(port, strPort);
    mxFree(strPort);
    
    /* Stop bits */
    strStopbits = mxArrayToString(ssGetSFcnParam(S, UART_STOPBITS));
    strcpy(stopbits, strStopbits);
    mxFree(strStopbits);
    
    /* Open if block is configuration */
    if(id == 0) {
        if(PortIsOpen(port)) {
            sprintf(error_msg, "Detect %s has multiple setup block!\n", port);
            ssSetErrorStatus(S, (char*)error_msg);
        }
        
        /* Data bits*/
        switch(DATABITS(S)) {
            case 8:
                data_bits = 8;
                break;
            default:
                ssSetErrorStatus(S, "Support 8bits data only!.\n");
                break;
        }
        
        /* Stop bits */
        if(!strcmp(stopbits, "1")) {
            stop_bits = ONESTOPBIT;
        }
        else if(!strcmp(stopbits, "1.5")) {
            stop_bits = ONE5STOPBITS;
        }
        else if(!strcmp(stopbits, "2")) {
            stop_bits = TWOSTOPBITS;
        }
        else {
            ssSetErrorStatus(S, "Invalid stop bits!.\n");
            return;
        }
        
        tmout = TIMEOUT(S);
        
        /*
         * printf("---\n");
         * printf("Port: %s\n", port);
         * printf("Baud: %d\n", (ULONG)BAUDRATE(S));
         * printf("Data bits: %d\n", data_bits);
         * printf("Stop bits: %d\n", stop_bits);
         * printf("Timeout: %d\n", (ULONG)tmout);
         */
        
        if(OpenPort(port, (ULONG)BAUDRATE(S), data_bits, stop_bits, (ULONG)tmout) == FALSE) {
            /* Close port */
            ClosePort(port);

			sprintf(error_msg, "Failed to open COM port: \"%s\".\n", port);
            ssSetErrorStatus(S, (char*)error_msg);

            return;
        }
    }
    /* Tx/Rx */
    else {
        
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S) {
    char* strPort;
    char port[64];
    
    strPort = mxArrayToString(ssGetSFcnParam(S, UART_PORT));
    strcpy(port, strPort);
    mxFree(strPort);
    
    if(ClosePort(port) == FALSE) {
        printf("Failed to close port: ");
        printf(port);
        printf("\n");
    }
} /* end mdlTerminate */

//#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
    
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_canrx.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function amg_usbconverter_n_uart.c"
#endif


/* ########################################################################
 * Serial port communication routine
 * - 
 * ########################################################################
 */
