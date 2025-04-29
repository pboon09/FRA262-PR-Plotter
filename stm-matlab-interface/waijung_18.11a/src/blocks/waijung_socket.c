#define S_FUNCTION_NAME  waijung_socket
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include <winsock2.h> /* Must be include first (before windows.h) */
#include <stdlib.h>
#include <tchar.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include "windows.h"

#pragma comment(lib, "Ws2_32.lib")

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
	ARCG_CONF = 0,
	
	ARGC_INPUT_PORTTYPE,
	ARGC_INPUT_PORTWIDTH,
	ARGC_OUTPUT_PORTTYPE,
	ARGC_OUTPUT_PORTWIDTH,
	
	ARGC_ADDRESS,
	ARGC_PORT,
	
	ARGC_OPTIONSTRING,
	
	ARGC_SAMPLETIME,
	ARGC_HOSTADDRESS,//ARGC_BLOCKID,
	
	ARGC_SIM_OPTIONS,
	
	__PARAM_COUNT
};

#define ENABLE_ISR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_ISR_ENABLE))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */

/* Prototype */
void Socket_Initialize(SimStruct *S);
void Socket_Finallize(SimStruct *S);
void Socket_Write(SimStruct *S, unsigned char *Buffer, int count);
BOOL Socket_Read(SimStruct *S, unsigned char *Buffer, int BufferCount, int *reading_count, BYTE *remote_ip, WORD *remote_port);
DWORD WINAPI GetTickCount(void);

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
		else
			ssSetOutputPortWidth(S, k, 1);
		ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE)))[k]));
	}
	
	for(k=0; k<input_count; k++) {
		ssSetInputPortDirectFeedThrough(S, k, 1);
		if(k<input_width_count) {
			width = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH)))[k]);
			ssSetInputPortWidth(S, k, (width>0)?width:1);
		}
		else
			ssSetInputPortWidth(S, k, 1);
		ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
		ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE)))[k]));
	}
	
	ssSetNumSampleTimes(S, 1);
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_CALL_TERMINATE_ON_EXIT);
	
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S, 0, SAMPLETIME(S));
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

/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
#define MDL_START
static void mdlStart(SimStruct *S) {
    BOOL host;
    char *conf;
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
    if(!strncmp(conf, "Host", 4)) {
        Socket_Initialize(S);
    }
    mxFree(conf);    
}

enum OPTIONS_INDEX {
	IDX_PORTNUM = 0,
	IDX_SOCKTYPE,
	IDX_DIRECTION, /* 0-Write,1-Read */
    IDX_TIMEOUT,
    IDX_STREAMMING
};

static void mdlOutputs(SimStruct *S, int_T tid) {
	struct {
		int input_count;
		int input_width;
		int output_count;
		int output_width;
		unsigned int Status;
	} Port;
	
	union _Conv_Val {
		BYTE           buffer[8];
		double         val_double;
		float          val_single;
		signed char    val_int8;
		unsigned char  val_uint8;
		signed short   val_int16;
		unsigned short val_uint16;
		int            val_int32;
		unsigned int   val_uint32;
	} Conv_Val;
	
	real_T   dd;
	real32_T ss;
	uint8_T  u8;
	uint16_T u16;
	uint32_T u32;
	int8_T   s8;
	int16_T  s16;
	int32_T  s32;
	
	static unsigned char buffer[128];
	
	/* Init */
	Port.input_count = 0;
	Port.input_width = 0;
	Port.output_count = 0;
	Port.output_width = 0;
	
	Port.Status = 0;
	
	/* Check input port, if empty means it is receive */
	Port.input_count = ssGetNumInputPorts(S);
	if(Port.input_count == 0) {
		/* Do nothing */
	}
	else {
		Port.input_width = ssGetInputPortWidth(S, 0);
		if(Port.input_width == 1) { /* Normal port */
			int i;
			int data_index;
			int bytes_count;
			unsigned char *write_buffer;
			
			/* Get data bytes count*/
			bytes_count = 0;
			for(i=0; i<Port.input_count; i++) {
				switch(ssGetInputPortDataType(S, i)) {
					case 0: // Double
						bytes_count += 8;
						break;
					case 1: case 6: case 7:
						bytes_count += 4;
						break;
					case 2: case 3:
						bytes_count += 1;
						break;
					case 4:	case 5:
						bytes_count += 2;
						break;
					default:
						break;
				}
			}
			
			/* Allocate buffer */
			if((write_buffer = malloc(bytes_count)) == NULL) {
				/* Out of memory */
				ssSetErrorStatus(S, (char*)"Out of memory.\n");
				return;
			}
			
			/* Collect data from input port */
			bytes_count = 0;
			for(i=0; i<Port.input_count; i++) {
				switch(ssGetInputPortDataType(S, i)) {
					case 0: // Double
						dd = *(const real_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_double = dd;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 8);
						bytes_count += 8;
						break;
					case 1:
						ss = *(const real32_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_single = ss;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 4);
						bytes_count += 4;
						break;
					case 2:
						s8 = *(const int8_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_int8 = s8;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 1);
						bytes_count += 1;
						break;
					case 3:
						u8 = *(uint8_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_uint8 = u8;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 1);
						bytes_count += 1;
						break;
					case 4:
						s16 = *(const int16_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_int16 = s16;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 2);
						bytes_count += 2;
						break;
					case 5:
						u16 = *(const uint16_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_uint16 = u16;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 2);
						bytes_count += 2;
						break;
					case 6:
						s32 = *(const int32_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_int32 = s32;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 4);
						bytes_count += 4;
						break;
					case 7:
						u32 = *(const uint32_T*) ssGetInputPortSignal(S, i);
						Conv_Val.val_uint32 = u32;
						memcpy(&(write_buffer[bytes_count]), Conv_Val.buffer, 4);
						bytes_count += 4;
						break;
					default:
						break;
				}
			}
			
			/* Write to port */
			Socket_Write(S, write_buffer, bytes_count);
			
			/* Free mem */
			free(write_buffer);
		}
		else { /* Vector */
			int_T nu;
			const void *u;
			DTypeId typeid;
			typeid = ssGetInputPortDataType(S, 0);
			
			/* Assume number of input is 1 */
			if(Port.input_count > 1) {
				ssSetErrorStatus(S, "Number of input port must be 1 for vector data.\n");
                return;
			}
			
			nu = ssGetInputPortWidth(S, 0);
			u = ssGetInputPortSignal(S, 0);
			
			/* If it is single byte */
			if((typeid == 2) || (typeid == 3)) { /* int8 or uint8 */
				/* Write to port */
				Socket_Write(S, (unsigned char*)u, nu);
			}
			else {
				unsigned char *write_buffer;
				int size;
				int i;
				
				if(typeid == 0) { /* Double */
					size = 8;
				}
				else if((typeid == 1)||(typeid==6)||(typeid==7)) { /* Single, Int32, UInt32 */
					size = 4;
				}
				else if((typeid == 4)||(typeid==5)) { /* Int16, UInt16 */
					size = 2;
				}
				else {
					ssSetErrorStatus(S, "Invalid type id.\n");
				}
				
				/* Allocate mem */
				write_buffer = malloc(nu*size);
				
				/* Collect data */
				for(i=0; i<nu; i++) {
					memcpy(&write_buffer[i*size], &((char*)u)[i*size], size);
				}
				
				/* Write to port */
				Socket_Write(S, (unsigned char*)write_buffer, nu*size);
				
				/* Free */
				free(write_buffer);
			}
		}
	}
	
    /* ====================================================================
     */
	/* Output port */
	Port.output_count = ssGetNumOutputPorts(S);
    if(Port.output_count >= 3) {
        int data_bytes;
        int data_size;
        char *read_buffer;
        BYTE remote_ip[4];
        WORD remote_port;
        int actual_count;
        int data_size_array[8] = {8, 4, 1, 1, 2, 2, 4, 4};
        
        int num;
        double *options;
        
        /* Get Sim options */
        num = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_SIM_OPTIONS));
        options = (double*)mxGetPr(ssGetSFcnParam(S, ARGC_SIM_OPTIONS));
        
        /* Port 0: Remote port */
        /* Port 1: Remote IP */
        /* Port 2... Data */        
        if(Port.output_count == 3) { /* Vector, or single data port */
            int dtype = ssGetOutputPortDataType(S, 2);
            if((dtype < 0) || (dtype >= 8)) {
                ssSetErrorStatus(S, "Invalid type id.\n");
            }
            data_size = (int)data_size_array[dtype];
            Port.input_width = ssGetOutputPortWidth(S, 2);
            data_bytes = Port.input_width * data_size;
        }
        else { /* Normal data port */
            int i;
            int dtype;
            
            data_bytes = 0;
            for(i=2; i<Port.output_count; i++) {
                dtype = ssGetOutputPortDataType(S, i);
                if((dtype < 0) || (dtype >= 8)) {
                    ssSetErrorStatus(S, "Invalid type id.\n");
                }
                data_bytes += (int)data_size_array[dtype];
            }
        }
        
        /* Allocate buffer */
        read_buffer = mxMalloc(data_bytes);
        
        
        /* Check if data is streaming */
        if((int)options[IDX_STREAMMING] == 0) { /* Normal */
            /* Read UDP packet */
            Socket_Read(S, (unsigned char *)read_buffer, data_bytes, &actual_count, remote_ip, &remote_port);
			if (actual_count == 0)
			{
				mexPrintf("No data received!\n");
				ssSetErrorStatus(S, "No data received!\n");
				return;
			}
			else if(data_bytes != actual_count) {
                char msg[256];
                sprintf(msg, "Expected data packet length: %d, but the reciving packet length is:%d\n", data_bytes, actual_count);
				mexPrintf(msg);
                ssSetErrorStatus(S, msg);
                return;
            }
        }
        else { /* Stream */
            int packet_counter;
            int data_index;
            int packet_len;
            BOOL start_detect;
            BYTE *packet_buffer = mxMalloc(10*1024);
            if(!packet_buffer) {
              ssSetErrorStatus(S, "No memory for packet processing.\n");
              return;
            }
            
            memset(packet_buffer, 0, 10*1024);
            
            /* Get packet */
            packet_counter = 0;
            data_index = 0;
            start_detect = FALSE;
            do {
                if(++packet_counter >= 1000) {
                    ssSetErrorStatus(S, "Exceed the maximum number of packet in a stream, 1000.\n");
                    return;
                }
                /* Read UDP packet */
                if(!Socket_Read(S, (unsigned char *)packet_buffer, 10*1024, &actual_count, remote_ip, &remote_port)) {
                    ssSetErrorStatus(S, "Failed to read data packet.\n");
                    return;
                }
                
                /* Update read_buffer */
                if(actual_count> 4) {
                    memcpy(&data_index, packet_buffer, 4);                    
                    if(data_index == 0) {
                        start_detect = TRUE;
                    }
                    
                    packet_len = actual_count - 4;
                    /*  */
                    if(data_index < data_bytes) { /* New idx */
                        memcpy(&read_buffer[data_index], &packet_buffer[4], ((data_index+packet_len)<=data_bytes)?packet_len:(data_bytes-data_index));
                        data_index += packet_len;
                    }
                    
                   // mexPrintf("Index: %d/%d (%8X)->len: %d\n", data_index, data_bytes, data_index, packet_len);
                }
                
            } while((data_index < data_bytes) || (!start_detect));
            
            //mexPrintf("====Index: %d (%X)=====\n", data_index, data_index);
            
            if(packet_buffer)
                mxFree(packet_buffer);
        }
        
        
        /* Port0: remote_port */
        {
            uint16_T *port = (uint16_T *)ssGetOutputPortSignal(S,0);
            *port = remote_port;
        }
        
        /* Port1: remote_address */
        {
            uint8_T *addr = (uint8_T *)ssGetOutputPortSignal(S,1);
            memcpy(addr,remote_ip,4);
        }
        
        /* Port2: data */
        if(Port.output_count == 3) { /* Vector, or single data port */
            void *y = (void *)ssGetOutputPortSignal(S,2);
            memcpy(y,read_buffer,data_bytes);
        }
        else { /* Normal data port */
            int i, dat_idx, dtype; 
            void *y;
            
            dat_idx = 0;
            for (i=2; i<Port.output_count; i++) {
                dtype = ssGetOutputPortDataType(S, i);
                y = (void *)ssGetOutputPortSignal(S,i);
                memcpy(y, &read_buffer[dat_idx], (int)data_size_array[dtype]);
                dat_idx += (int)data_size_array[dtype];
            }
        }
        
        /* Free buffer */
        mxFree(read_buffer);
    }
	
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
    BOOL host;
    char *conf;
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
    if(!strncmp(conf, "Host", 4)) {
        Socket_Finallize(S);
    }
    mxFree(conf);
} /* end mdlTerminate */

static int get_list_count(char *s) {
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
	int NOutputPara = 2; /* Number of parameters to output to model.rtw */
	
	char *conf; // ARCG_CONF
	char* optionstring;
	
	conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
	optionstring = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_OPTIONSTRING));
	
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "conf", conf,
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	
	/* Write configuration string */
	if (!ssWriteRTWStrVectParam(S, "optionstring", optionstring, get_list_count(optionstring))){
		return;
	}
	
	mxFree(conf);
	mxFree(optionstring);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file waijung_socket.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_socket.c"
#endif

/*=======================================================================*/
/* Socket Manager */
/*=======================================================================*/
#include <stdlib.h>
#include <tchar.h>
#pragma comment(lib, "Ws2_32.lib")
#define WSA_VERSION  MAKEWORD(2, 0)

#include <stdio.h>

#define HOSTNAME_SIZE   MAX_PATH
#define STRING_LENGTH   40


typedef enum {
	CON_TCP,
	CON_UDP
} PROTOCOL_ID;

#define MAX_UDP_PACKET_SIZE (8*1024+20)

typedef struct {
    /* Data (Receive) */
	BOOL data_ready;
    DWORD data_count; /* Data receive count */
    BYTE data[MAX_UDP_PACKET_SIZE];
	BYTE tmpdata[MAX_UDP_PACKET_SIZE]; /* Access by monitor thread only */
} DATA_BUFFER;

#define DEEP_BUFFER_COUNT 512

typedef struct {
	/* Handle */
	HANDLE m_hComm;
	/* Thread */
	HANDLE hThread;
	/* Event */
	HANDLE hStopEvent;
	/* Mutex */
	HANDLE hUpdateMutex;
	
	/* Address & Port */
	char Address[128];
	char ServiceName[128]; /* Name, or port */
    
    /* Timeout */
    int timeout;
    
    BOOL streamming; /* Indicate data is streamming? */
	
	/* Data (Receive) */
//	BOOL data_ready;
//	DWORD data_count; /* Data receive count */
//	BYTE data[MAX_UDP_PACKET_SIZE];	
//	BYTE tmpdata[MAX_UDP_PACKET_SIZE]; /* Access by monitor thread only */
    
    DATA_BUFFER buffer_list[DEEP_BUFFER_COUNT];
    DWORD buffer_list_index;
    DWORD buffer_list_count;
	
	/* Object link */
	void *next; /* For link list */
} SOCKET_STRUCT;

ULONG SOCKET_GetIPAddress(LPCTSTR strHostName);
USHORT SOCKET_GetPortNumber(LPCTSTR strServiceName);
void SOCKET_CloseComm(SOCKET_STRUCT *com);
DWORD WINAPI ThreadProc(LPVOID lpParam);
BOOL SOCKET_GetPeerName(SOCKET_STRUCT *com, SOCKADDR_IN *saddr_in);
DWORD SOCKET_ReadComm(SOCKET_STRUCT *com, LPBYTE lpBuffer, DWORD dwSize, DWORD dwTimeout);

/* ========================================================================
 */

SOCKET_STRUCT *SocketStruct_Receive_list = NULL;

SOCKET_STRUCT *get_receive_socket_from_port(char *port) {
	SOCKET_STRUCT *sock = NULL;
	
	/* List is empty */
	if(SocketStruct_Receive_list == NULL)
		return NULL;
	
	/* Search through the list, ...
	 * to find object corresponding to specified port */
	sock = SocketStruct_Receive_list;
	while(sock != NULL) {
		if(!strcmp(sock->ServiceName, port))
			return sock;
		sock = (SOCKET_STRUCT*)(sock->next);
	}
	
	/* Could not found the object for specified port */
	return NULL;
}

/* CreateSocketEx */
BOOL SOCKET_CreateSocketEx(SOCKET_STRUCT *com, LPCTSTR strHost, LPCTSTR strServiceName, int nFamily, int nType, UINT uOptions /* = 0 */) {
	SOCKET sock;
	
	// Create a Socket that is bound to a specific service provide
	// nFamily: (AF_INET)
	// nType: (SOCK_STREAM, SOCK_DGRAM)
	sock = socket(nFamily, nType, IPPROTO_IP);
	if (INVALID_SOCKET != sock) {
		if (uOptions & SO_REUSEADDR) {
			// Inform Windows Sockets provider that a bind on a socket should not be disallowed
			// because the desired address is already in use by another socket
			BOOL optval = TRUE;
			if ( SOCKET_ERROR == setsockopt( sock, SOL_SOCKET, SO_REUSEADDR, (char *) &optval, sizeof( BOOL ) ) ) {
				closesocket( sock );
				return false;
			}
		}
		
		if (nType == SOCK_DGRAM) {
			if (uOptions & SO_BROADCAST) {
				// Inform Windows Sockets provider that broadcast messages are allowed
				BOOL optval = TRUE;
                if ( SOCKET_ERROR == setsockopt( sock, SOL_SOCKET, SO_BROADCAST, (char *) &optval, sizeof( BOOL ) ) ) {
					closesocket( sock );
					return false;
				}
			}
		}
        
        // Navy
        {
            int a = 65535;
            if (SOCKET_ERROR == setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &a, sizeof(int))) {
                closesocket( sock );
                return false;
            }
        }
        
		// Associate a local address with the socket
		//SockAddrIn sockAddr;
		//sockAddr.CreateFrom(strHost, strServiceName, nFamily);
		{
			SOCKADDR_IN sockAddr;
			memset(&sockAddr, 0, sizeof(SOCKADDR_IN));
			sockAddr.sin_addr.s_addr = htonl(SOCKET_GetIPAddress(strHost));
			sockAddr.sin_port = htons(SOCKET_GetPortNumber(strServiceName));
			sockAddr.sin_family = nFamily;
			if((sockAddr.sin_addr.s_addr==0L)&&(sockAddr.sin_port==0)) {
				closesocket( sock );
				return FALSE;
			}
			if (bind(sock, (SOCKADDR *)&sockAddr, sizeof(SOCKADDR_IN)) == SOCKET_ERROR) {
				closesocket(sock);
				return FALSE;
			}
            /* Inform IP address */
            mexPrintf("UDP Server (%d.%d.%d.%d:%d)\n",
                    (BYTE)(sockAddr.sin_addr.s_addr>>0),
                    (BYTE)(sockAddr.sin_addr.s_addr>>8),
                    (BYTE)(sockAddr.sin_addr.s_addr>>16),
                    (BYTE)(sockAddr.sin_addr.s_addr>>24),
                    SOCKET_GetPortNumber(strServiceName));
		}
		
		// Listen to the socket, only valid for connection socket
		if (SOCK_STREAM == nType) {
			if ( SOCKET_ERROR == listen(sock, SOMAXCONN)) {
				closesocket( sock );
				return false;
			}
		}
		
		// Success, now we may save this socket
		com->m_hComm = (HANDLE)sock;
        

	}
	
	return (INVALID_SOCKET != sock);
}

BOOL OpenPort_Receive(char *addr, char *port, int sock_type, int timeout) {
	SOCKET_STRUCT *socket;
	
	/* Get socket */
	socket = get_receive_socket_from_port(port);
	
	/* Open port if not openning */
	if(socket == NULL) {
		/* Allocate mem */
		socket = (SOCKET_STRUCT*)malloc(sizeof(SOCKET_STRUCT));
		if(socket == NULL) { /* Out of memory */
			return -1;
		}
		memset(socket, 0, sizeof(SOCKET_STRUCT));
		
		/* Copy information */
		strcpy(socket->ServiceName, port);
		strcpy(socket->Address, addr);
        socket->timeout = timeout;
        
        
        socket->buffer_list_index = 0;
        socket->buffer_list_count = 0;
		
		/* Add to list */
		if(SocketStruct_Receive_list == NULL) {
			/* 1st item in list */
			SocketStruct_Receive_list = socket;
		}
		else {
			SOCKET_STRUCT *last;
			/* Find last item in list */
			last = SocketStruct_Receive_list;
			while(last->next != NULL) {
				last = (SOCKET_STRUCT*)last->next;
			}
			/* Add */
			last->next = socket;
		}
		
		/* Open connection */
		if(SOCKET_CreateSocketEx(socket, (LPCTSTR)addr/* Local */, (LPCTSTR)port, AF_INET, sock_type, SO_BROADCAST)) {
			DWORD dwThreadId;
			
			/* Setup Stop event */
			socket->hStopEvent = CreateEvent(
					NULL,               // default security attributes
					TRUE,               // manual-reset event
					FALSE,              // initial state is nonsignaled
					TEXT("StopEvent")  // object name
					);
			ResetEvent(socket->hStopEvent);
			
			/* Setup Mutex for access buffer */
			socket->hUpdateMutex = CreateMutex(
					NULL,              // default security attributes
					FALSE,             // initially not owned
					NULL);             // unnamed mutex
			
			/* Run thread */
			socket->hThread = CreateThread(NULL, 0, ThreadProc, socket, CREATE_SUSPENDED, &dwThreadId);
			if(socket->hThread != NULL) {
				ResumeThread(socket->hThread);
			}
			
			/* No error detect */
			return TRUE;
		}
	}
	
	/* Return status */
	return FALSE;
}

/* Thread process */
DWORD WINAPI ThreadProc(LPVOID lpParam) {
	DWORD dwWaitResult;
	DWORD   dwBytes  = 0L;
	DWORD   dwTimeout = INFINITE;
	LPBYTE  lpData;
	DWORD   dwSize;
	SOCKET_STRUCT *com;
	
	SOCKADDR_IN sockAddr;
	memset(&sockAddr, 0, sizeof(SOCKADDR_IN));
	
	/* Init */
	com = (SOCKET_STRUCT*)lpParam;
	//lpData = (LPBYTE)&(com->tmpdata[0]);//socket->buffer_list_index
    //lpData = (LPBYTE)&(com->buffer_list[com->buffer_list_count].tmpdata[0]);
	dwSize = MAX_UDP_PACKET_SIZE;
	
    
	/* Get peer */
	SOCKET_GetPeerName(com, (SOCKADDR_IN *)&sockAddr);
	
	/* Monitor incomming bytes */
	while((dwWaitResult = WaitForSingleObject(com->hStopEvent, 0)) != WAIT_OBJECT_0) {
        
        lpData = (LPBYTE)&(com->buffer_list[com->buffer_list_count].tmpdata[0]);
        
		/* Blocking mode: Wait for event */
		dwBytes = SOCKET_ReadComm(com, lpData, dwSize, dwTimeout);
		
		/* Error? - need to signal error */
		if (dwBytes == (DWORD)-1L) {
			/* special case for UDP, alert about the event but do not stop */
		}
		else if (dwBytes > 0L) { /* Chars received? */
			/* Synchronize data */
			dwWaitResult = WaitForSingleObject(
					com->hUpdateMutex,    // handle to mutex
					INFINITE);  // no time-out interval
			if(dwWaitResult == WAIT_OBJECT_0) {
				_try {
					/* Copy data */
					memcpy(&(com->buffer_list[com->buffer_list_count].data[0]), \
                            &(com->buffer_list[com->buffer_list_count].tmpdata[0]), \
                            dwBytes);
					com->buffer_list[com->buffer_list_count].data_count = dwBytes;
					
					/* Update Ready Status */
					com->buffer_list[com->buffer_list_count].data_ready = 1;
                    
                    com->buffer_list_count += 1;
                    if(com->buffer_list_count>= DEEP_BUFFER_COUNT) {
                        com->buffer_list_count = 0;
                    }
				}
				_finally { /* Release Mutex */
					ReleaseMutex(com->hUpdateMutex);
				}
			}
		}
	}
	return 0;
}

BOOL Socket_Read(SimStruct *S, unsigned char *Buffer, int BufferCount, int *reading_count, BYTE *remote_ip, WORD *remote_port) {
	char *port;
	SOCKET_STRUCT *socket;
	DWORD dwWaitResult;
    BOOL data_rdy;
    int timeout;
	
    DWORD timeout_start;
	SOCKADDR_IN sockAddr;
    
    /* Get socket */
    _try {
        port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
        socket = get_receive_socket_from_port(port);
        if(socket == NULL) {
            ssSetErrorStatus(S, (char*)"Invalid Socket.\n");
            return FALSE;
        }
    }
    _finally { /* Free */
        mxFree(port);
    }
    
    timeout = 10000; /* Default */
    timeout_start = GetTickCount();
    data_rdy = FALSE;
    do {        
        /* Synchronize data */
        dwWaitResult = WaitForSingleObject(
                socket->hUpdateMutex,    // handle to mutex
                INFINITE);  // no time-out interval
        
        if(dwWaitResult == WAIT_OBJECT_0) {
            timeout = socket->timeout; /* Update timeout */
            _try {
                //if(socket->data_ready) {
                if(socket->buffer_list_count != socket->buffer_list_index)
                {
                    //mexPrintf("Deep index: %d\n", socket->buffer_list_index);
					//socket->data_ready = 0;/* clear ready status */

					/* Dispatch receiving packet */
					//if(BufferCount >= (socket->data_count - sizeof(SOCKADDR_IN))) {
                    if(BufferCount >= ((socket->buffer_list[socket->buffer_list_index]).data_count - sizeof(SOCKADDR_IN))) {
                        /* Return number of receive bytes */
                        *reading_count = ((socket->buffer_list[socket->buffer_list_index]).data_count - sizeof(SOCKADDR_IN));
                        /* Get remote address */
						memcpy(&sockAddr, (socket->buffer_list[socket->buffer_list_index]).data, sizeof(SOCKADDR_IN));
						memcpy(remote_ip, &sockAddr.sin_addr.s_addr, 4); /* IP addres */
                        /* Get remote port */
                        *remote_port = sockAddr.sin_port; /* Port */
                        /* Get data */
                        memcpy(Buffer, &(socket->buffer_list[socket->buffer_list_index].data[sizeof(SOCKADDR_IN)]), BufferCount); /* Data */
                        /* Activate ready status */
                        data_rdy = TRUE;
                    }
                    
                    socket->buffer_list_index += 1;
                    if(socket->buffer_list_index >= DEEP_BUFFER_COUNT) {
                        socket->buffer_list_index = 0;
                    }
                }
            }
            _finally { /* Release Mutex */
                ReleaseMutex(socket->hUpdateMutex);
            }
        }
        
        /* Timeout check */
        if(!data_rdy) {
            if((GetTickCount() - timeout_start) > timeout) {
                ssSetErrorStatus(S, (char*)"Timeout occure while UDP waiting for data.");
                return FALSE;
            }
        }
    } while(!data_rdy); /* Repeat while data is not ready */
	
    /* Receive data success */
	return TRUE;
}

/* ========================================================================
 */

/* Prototypes */
BOOL SOCKET_Connect(SOCKET_STRUCT *com, char *hostaddr, char *strDestination, char *strServiceName, int nFamily, int nType);
DWORD SOCKET_WriteComm(SOCKET_STRUCT *com, const LPBYTE lpBuffer, DWORD dwCount, DWORD dwTimeout);

SOCKET_STRUCT *SocketStruct_Send_list = NULL;

void CloseAllPorts(void) {
    SOCKET_STRUCT *sock = NULL;
    
    /* List is empty */
    if(SocketStruct_Send_list != NULL) {
        /* Free all in the list */
        while(SocketStruct_Send_list != NULL) {
            sock = SocketStruct_Send_list;
            SocketStruct_Send_list = (SOCKET_STRUCT *)(SocketStruct_Send_list->next);
            
            /* Close */
            SOCKET_CloseComm(sock);
            
            /* Free mem */
            free(sock);
        }
        
        /* Empty list */
        SocketStruct_Send_list = NULL;
    }
	
	/* ==================================== */
    if(SocketStruct_Receive_list != NULL) {
        
        /* Free all in the list */
        while(SocketStruct_Receive_list != NULL) {
            sock = SocketStruct_Receive_list;
            SocketStruct_Receive_list = (SOCKET_STRUCT *)(SocketStruct_Receive_list->next);
            
            /* Stop */
            SetEvent(sock->hStopEvent);
            
            /* Close */
            SOCKET_CloseComm(sock);
            
            /* Wait for thread to terminated */
            if(WaitForSingleObject(sock->hThread, 5000) != WAIT_OBJECT_0) {
                mexPrintf("Error: Thread did not terminated correctly.\n");
            }
            
            /* Free mem */
            free(sock);
        }
        
        /* Empty list */
        SocketStruct_Receive_list = NULL;
    }
}

SOCKET_STRUCT *get_send_socket_from_port(char *addr, char *port) {
	SOCKET_STRUCT *sock = NULL;
	
	/* List is empty */
	if(SocketStruct_Send_list == NULL)
		return NULL;
	
	/* Search through the list, ...
	 * to find object corresponding to specified port */
	sock = SocketStruct_Send_list;
	while(sock != NULL) {
		if(!strcmp(sock->ServiceName, port) && !strcmp(sock->Address, addr))
			return sock;
		sock = (SOCKET_STRUCT*)(sock->next);
	}
	
	/* Could not found the object for specified port */
	return NULL;
}

BOOL OpenPort_Send(char *hostaddr, char *addr, char *port, int sock_type) {
	SOCKET_STRUCT *socket;
	
	/* Get socket */
	socket = get_send_socket_from_port(addr, port);
	
	/* Open port if not openning */
	if(socket == NULL) {
		/* Allocate mem */
		socket = (SOCKET_STRUCT*)malloc(sizeof(SOCKET_STRUCT));
		if(socket == NULL) { /* Out of memory */
			return -1;
		}
		memset(socket, 0, sizeof(SOCKET_STRUCT));
		
		/* Copy information */
		strcpy(socket->ServiceName, port);
		strcpy(socket->Address, addr);
        
        socket->buffer_list_index = 0;
        socket->buffer_list_count = 0;
		
		/* Add to list */
		if(SocketStruct_Send_list == NULL) {
			/* 1st item in list */
			SocketStruct_Send_list = socket;
		}
		else {
			SOCKET_STRUCT *last;
			/* Find last item in list */
			last = SocketStruct_Send_list;
			while(last->next != NULL) {
				last = (SOCKET_STRUCT*)last->next;
			}
			/* Add */
			last->next = socket;
		}
		
		/* Open connection */
		if(SOCKET_Connect(socket, hostaddr, addr, (char *)port, AF_INET, sock_type)) {
			return TRUE;
		}
	}
	
	/* Return status */
	return FALSE;
}

#if 0
BOOL OpenPort(char *addr, char *port, int sock_type) {
	SOCKET_STRUCT *socket;
	
	/* Get socket */
	socket = get_send_socket_from_port(addr, port);
	
	/* Open port if not openning */
	if(socket == NULL) {
		/* Allocate mem */
		socket = (SOCKET_STRUCT*)malloc(sizeof(SOCKET_STRUCT));
		if(socket == NULL) { /* Out of memory */
			return -1;
		}
		memset(socket, 0, sizeof(SOCKET_STRUCT));
		
		/* Copy information */
		strcpy(socket->ServiceName, port);
		strcpy(socket->Address, addr);
        
        socket->buffer_list_index = 0;
        socket->buffer_list_count = 0;
		
		/* Add to list */
		if(SocketStruct_Send_list == NULL) {
			/* 1st item in list */
			SocketStruct_Send_list = socket;
		}
		else {
			SOCKET_STRUCT *last;
			/* Find last item in list */
			last = SocketStruct_Send_list;
			while(last->next != NULL) {
				last = (SOCKET_STRUCT*)last->next;
			}
			/* Add */
			last->next = socket;
		}
		
		/* Open connection */
		if(SOCKET_Connect(socket, addr, (char *)port, AF_INET, sock_type)) {
			return TRUE;
		}
	}
	
	/* Return status */
	return FALSE;
}
#endif //0

/* ========================================================================
 */

int wsadata_init = 0;
WSADATA	WSAData = { 0 };

/* Init */
void Socket_Initialize(SimStruct *S) {
	int i;
	int num;
	double *options;
	int sock_type;
	
	char *port;
	char *addr;
	char *hostaddr; // For HostSend block
	
	if ((!wsadata_init) && (0 != WSAStartup( WSA_VERSION, &WSAData))) {
		WSACleanup( );
		ssSetErrorStatus(S, (char*)"WSAStartup failed.\n");
	}
	else {
		wsadata_init = 1;
	}
	
	/* Get Sim options */
	num = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_SIM_OPTIONS));
	options = (double*)mxGetPr(ssGetSFcnParam(S, ARGC_SIM_OPTIONS));
	
	/* Socket type */
	sock_type = (int)options[IDX_SOCKTYPE]; /* 1-TCP, 2-UDP */
	if((sock_type != 1) && (sock_type != 2)) {
		ssSetErrorStatus(S, (char*)"Invalid socket type.\n");
	}
	
	_try {
		/* Allocate string */
		port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
		addr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ADDRESS));
		hostaddr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_HOSTADDRESS));
		
		/* Send */
		if((int)options[IDX_DIRECTION] == 0) { /* Write */
			if(!OpenPort_Send(hostaddr, addr, port, sock_type)) {
				ssSetErrorStatus(S, (char*)"Failed to open connection.\n");
			}
		}
		else { /* Receive */
			if(!OpenPort_Receive(addr, port, sock_type, (int)options[IDX_TIMEOUT])) {
				ssSetErrorStatus(S, (char*)"Failed to open port.\n");
			}
		}

        /* Stream */
        {
            /* Stream */
            SOCKET_STRUCT *socket;
            
            /* Get socket */
            socket = get_receive_socket_from_port(port);
            if(socket) {            
                if((int)options[IDX_DIRECTION] == 0) { /* Not a stream */
                    socket->streamming = FALSE;
                }
                else {
                    socket->streamming = TRUE;
                }
            }
        }
	}
	_finally { /* Free */
		mxFree(port);
		mxFree(addr);
		mxFree(hostaddr);
	}
}

/* Terminate */
void Socket_Finallize(SimStruct *S) {
	CloseAllPorts();
}

/* Output */
void Socket_Write(SimStruct *S, unsigned char *Buffer, int count) {
	char *port;
	char *addr;
	SOCKET_STRUCT *socket;
	
	/* Get socket */
	_try {
		port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
		addr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ADDRESS));
		socket = get_send_socket_from_port(addr, port);
		if(socket == NULL) {
			ssSetErrorStatus(S, (char*)"Invalid Socket.\n");
		}
		else {
			if(SOCKET_WriteComm(socket, (const LPBYTE)Buffer, count, 1000) != count) {
				ssSetErrorStatus(S, (char*)"Failed to write data to port.\n");
			}
		}
	}
	_finally { /* Free */
		mxFree(port);
		mxFree(addr);
	}
}

/* ===================================================================== */

/* GetLocalName */
BOOL SOCKET_GetLocalName(LPTSTR strName, UINT nSize) {
	if (strName != NULL && nSize > 0) {
		char strHost[HOSTNAME_SIZE] = { 0 };
		
		// get host name, if fail, SetLastError is set
		if (SOCKET_ERROR != gethostname(strHost, sizeof(strHost))) {
			struct hostent* hp;
			hp = gethostbyname(strHost);
			if (hp != NULL) {
				strncpy(strHost, hp->h_name, HOSTNAME_SIZE);
			}
			
			// check if user provide enough buffer
			if (strlen(strHost) > nSize) {
				SetLastError(ERROR_INSUFFICIENT_BUFFER);
				return false;
			}
			
			// Unicode conversion
			_tcscpy(strName, strHost);
			return TRUE;
		}
	}
	else
		SetLastError(ERROR_INVALID_PARAMETER);
	return FALSE;
}

/* GetIPAddress */
ULONG SOCKET_GetIPAddress(LPCTSTR strHostName) {
	LPHOSTENT   lphostent;
	ULONG       uAddr = INADDR_NONE;
	TCHAR       strLocal[HOSTNAME_SIZE] = { 0 };
	LPCTSTR strHost;
	
	// if no name specified, get local
	if ( NULL == strHostName ) {
		SOCKET_GetLocalName(strLocal, sizeof(strLocal));
		strHostName = strLocal;
	}
	
	strHost = strHostName;
	
	// Check for an Internet Protocol dotted address string
	uAddr = inet_addr(strHost);
	
	if((INADDR_NONE == uAddr) && (strcmp( strHost, "255.255.255.255" ))) {
		// It's not an address, then try to resolve it as a hostname
		if ( lphostent = gethostbyname( strHost ) )
			uAddr = *((ULONG *) lphostent->h_addr_list[0]);
	}
	
	return ntohl( uAddr );
}

/* GetPortNumber */
USHORT SOCKET_GetPortNumber(LPCTSTR strServiceName) {
	LPSERVENT   lpservent;
	USHORT      nPortNumber = 0;
	
	if ( _istdigit( strServiceName[0] ) ) {
		nPortNumber = (USHORT) _ttoi( strServiceName );
	}
	else {
		LPCTSTR pstrService = strServiceName;
		// Convert network byte order to host byte order
		if ( (lpservent = getservbyname( pstrService, NULL )) != NULL )
			nPortNumber = ntohs( lpservent->s_port );
	}
	
	return nPortNumber;
}

/* GetPeerName */
BOOL SOCKET_GetPeerName(SOCKET_STRUCT *com, SOCKADDR_IN *saddr_in) {
	if (com->m_hComm != NULL) {
		int namelen = sizeof(SOCKADDR_IN);
		return (SOCKET_ERROR != getpeername(com->m_hComm, saddr_in, &namelen));
	}
	return FALSE;
}

/* Connect */
BOOL SOCKET_Connect(SOCKET_STRUCT *com, char *hostaddr, char *strDestination, char *strServiceName, int nFamily, int nType) {
	SOCKET sock;
	// Socket is already opened
	//if ( com->m_hComm != NULL )
	//    return FALSE;
	
	// Create a Socket that is bound to a specific service provide
	// nFamily: (AF_INET)
	// nType: (SOCK_STREAM, SOCK_DGRAM)
	sock = socket(nFamily, nType, 0);
	if (sock != INVALID_SOCKET) {
		/* Associate a local address with the socket */
		SOCKADDR_IN sockAddr;
		memset(&sockAddr, 0, sizeof(SOCKADDR_IN));
		
		// DEBUG
		// ================================================================
		sockAddr.sin_addr.s_addr = htonl(SOCKET_GetIPAddress(hostaddr));
		sockAddr.sin_port = htons(SOCKET_GetPortNumber("0"));
		sockAddr.sin_family = nFamily;
		if((sockAddr.sin_addr.s_addr==0L)&&(sockAddr.sin_port==0)) {
		    closesocket( sock );
		    return FALSE;
		}
		if (bind(sock, (SOCKADDR *)&sockAddr, sizeof(SOCKADDR_IN)) == SOCKET_ERROR) {
		    closesocket(sock);
		    return FALSE;
		}
		/* Inform IP address */
		mexPrintf("UDP Send (%d.%d.%d.%d)\n",
				(BYTE)(sockAddr.sin_addr.s_addr>>0),
				(BYTE)(sockAddr.sin_addr.s_addr>>8),
				(BYTE)(sockAddr.sin_addr.s_addr>>16),
				(BYTE)(sockAddr.sin_addr.s_addr>>24));
		// ================================================================
		// END-DEBUG
		
		/* Now get destination address & port */
		sockAddr.sin_addr.s_addr = htonl(SOCKET_GetIPAddress(strDestination));
		sockAddr.sin_port = htons(SOCKET_GetPortNumber(strServiceName));
		sockAddr.sin_family = nFamily;
		
		/* try to connect - if fail, server not ready */
		if (connect(sock, (SOCKADDR *)&sockAddr, sizeof(SOCKADDR_IN)) == SOCKET_ERROR) {
			closesocket(sock);
			return FALSE;
		}
		
		/* Success, now we may save this socket */
		com->m_hComm = (HANDLE)sock;
	}
	
	if(sock != INVALID_SOCKET) {
		return TRUE; /* Success */
	}
	
	/* Fail */
	return FALSE;
}

/* WriteComm */
DWORD SOCKET_WriteComm(SOCKET_STRUCT *com, const LPBYTE lpBuffer, DWORD dwCount, DWORD dwTimeout) {
	fd_set  fdWrite  = { 0 };
	TIMEVAL stTime;
	TIMEVAL *pstTime = NULL;
	
	SOCKET s;
	DWORD dwBytesWritten = 0L;
	int res;
	
	// Accept 0 bytes message
	if ((com->m_hComm == NULL) || (NULL == lpBuffer))
		return 0L;
	
	if ( INFINITE != dwTimeout ) {
		stTime.tv_sec = dwTimeout/1000;
		stTime.tv_usec = (dwTimeout%1000)*1000;
		pstTime = &stTime;
	}
	
	s = (SOCKET) com->m_hComm;
	// Set Descriptor
	if ( !FD_ISSET( s, &fdWrite ) )
		FD_SET( s, &fdWrite );
	
	// Select function set write timeout
	dwBytesWritten = 0L;
	res = select(s+1, NULL, &fdWrite, NULL, pstTime);
	if ( res > 0) {
		res = send( s, (LPCSTR)lpBuffer, dwCount, 0);
	}
	dwBytesWritten = (DWORD)((res >= 0)?(res) : (-1L));
	
	return dwBytesWritten;
}

/* ReadComm */
DWORD SOCKET_ReadComm(SOCKET_STRUCT *com, LPBYTE lpBuffer, DWORD dwSize, DWORD dwTimeout) {
	SOCKET s;
	DWORD dwBytesRead;
	int res;
	
	fd_set  fdRead  = { 0 };
	TIMEVAL stTime;
	TIMEVAL *pstTime = NULL;
	
	if (lpBuffer == NULL || dwSize < 1L)
		return 0L;
	
	if ( INFINITE != dwTimeout ) {
		stTime.tv_sec = dwTimeout/1000;
		stTime.tv_usec = (dwTimeout%1000)*1000;
		pstTime = &stTime;
	}
	
	s = (SOCKET)com->m_hComm;
	/* Set Descriptor */
	if(!FD_ISSET(s, &fdRead))
		FD_SET(s, &fdRead);
	
	/* Select function set read timeout */
	dwBytesRead = 0L;
	res = select( s+1, &fdRead, NULL, NULL, pstTime );
	if (res > 0) {
		if(1) { /* Broadcast */
			SOCKADDR_IN sockAddr;
			int nLen;
			int nOffset;
			LPSTR lpszData;
			
			sockAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
			nLen = sizeof(SOCKADDR_IN);
			nOffset = nLen; // use offset for Smart addressing
			if ( dwSize < (DWORD) nOffset) {  // error - buffer to small
				SetLastError( ERROR_INVALID_USER_BUFFER );
				return -1L;
			}
			lpszData = (LPSTR)(lpBuffer + nOffset);
			res = recvfrom( s, lpszData, dwSize-nOffset, 0, (SOCKADDR *)&sockAddr, &nLen);
			
			// clear 'sin_zero', we will ignore them with 'SockAddrIn' anyway!
			memset(&sockAddr.sin_zero, 0, sizeof(sockAddr.sin_zero));
			
			if ( res >= 0) {
				memcpy(lpBuffer, &sockAddr, sizeof(SOCKADDR_IN));
				res += sizeof(SOCKADDR_IN);
			}
		}
		else {
			res = recv( s, (LPSTR)lpBuffer, dwSize, 0);
		}
	}
	dwBytesRead = (DWORD)((res > 0)?(res) : (-1L));
	
	/* Return number of data reading */
	return dwBytesRead;
}

/* CloseComm */
void SOCKET_CloseComm(SOCKET_STRUCT *com) {
	if (com->m_hComm != NULL) {
		shutdown((SOCKET)com->m_hComm, SD_BOTH);
		closesocket((SOCKET)com->m_hComm);
	}
}

/* === End === */
