#define S_FUNCTION_NAME  amg_usbconverter_n_serial
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

#include "windows.h"
#include "stdio.h"

enum {
    ARGC_CONFIGURATION = 0,
    ARGC_PORT,
    ARGC_TRANSFER,
    ARGC_BAUDRATE,
    ARGC_STOPBITS,
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    
    ARGC_BINHEADER, /* Vector */
    ARGC_BINTERMINATOR, /* Vector */
    ARGC_ASCIIHEADER, /* String */
    ARGC_ASCIITERMINATOR, /* Vector */
    
    ARGC_BINDATALENGTH,
    ARGC_PACKETMODE,
    ARGC_TIMEOUT,
    
    ARGC_SAMPLETIME,
    ARGC_INTIAL_VALUES, //ARGC_SAMPLETIMESTR,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define SAMPLETIME(S)    mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */
#define BLOCKID(S)      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID)) /* BlockID */

typedef enum {
    packetBinary = 0,
    packetAscii
} PACKET_MODE;

typedef enum {
    transferBlocking = 0,
    transferNonBlocking
} TRANSFER_MODE;

/* Prototypes */
typedef struct {
	int FirstStep;
	double *InitialValues;
	int InitialValuesSize;
	
    char *id; /* Unique ID for block. */
    
    PACKET_MODE packet_mode;
    TRANSFER_MODE transfer_mode;
    
    DWORD global_data_index; /* NDTR reference to main input port buffer. */
    DWORD global_search_index;
    DWORD global_search_state;

    /* Packet header, valid for both Ascii and Binary */    
    DWORD packet_header_index;
    DWORD packet_header_count;
    BYTE *packet_header;
    
    /* Binary data length */
    DWORD packet_data_len; /* Packet Binary only */
    
    /* Packet terminator, valid for both Ascii and Binary */
    DWORD packet_terminator_index;
    DWORD packet_terminator_count;
    BYTE *packet_terminator;
    
    DWORD data_buffer_index;
    DWORD data_buffer_size;
    BYTE *data_buffer;
    
    void* Next; /* Next PORT_READ_INFO in list */
    
    /* Packet processing */
    
} PORT_READ_INFO;

static bool ClosePort(char* PortName);
static bool PortIsOpen(char* PortName);
static bool OpenPort(char* PortName, ULONG BaudRate, BYTE DataBits, BYTE StopBits, ULONG Timeout_ms);
BOOL ReadByteNonBlocking(char* PortName, BOOL* ready);
void FreeReadInfo(void);
PORT_READ_INFO* CreateReadInfo(char*id, DWORD buffer_size);
PORT_READ_INFO* GetReadInfo(char* id);
int ReadBuffer(char* PortName, PORT_READ_INFO* read_info, BYTE *buffer, DWORD read_count);
BOOL ProcessReadAsciiPacket(BYTE b, PORT_READ_INFO* read_info, char *buffer, DWORD buffer_size);
static bool WriteToPort(char* PortName, BYTE* buffer, DWORD count);
BOOL ReadByte(char* PortName, DWORD timeout, BYTE* data);
BOOL ReadBytes(char* PortName, DWORD timeout, UINT count, BYTE* data);


typedef struct {
	char port[32];
    ULONG Timeout;
    ULONG BaudRate;
    BYTE DataBits;
    BYTE StopBits;
    BYTE Parity;	
} PORT_CONFIGURATION;

void PortCOnfiguration_Clear(void);
int PortConfiguration_EmptyIndex(void);
int PortConfiguration_Put (const char *port, ULONG Timeout, ULONG BaudRate,
		BYTE DataBits, BYTE StopBits, BYTE Parity) ;
PORT_CONFIGURATION *PortCongiguration_Get (const char *port);

/*
 * 
 */
static void mdlInitializeSizes(SimStruct *S) {
    int k;
    
    int input_count;
    int output_count;
    
    /* Parameter validatone */    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    input_count = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[0]);
    output_count = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[0]);
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
   
    /* Port */
    for(k=0; k<output_count; k++) {
        ssSetOutputPortWidth(S, k, 1);
        ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[k+1]));
    }

    for(k=0; k<input_count; k++) {
        ssSetInputPortComplexSignal(S,  k, COMPLEX_NO);
        ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortWidth(S, k, 1);
        ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k+1]));
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

PACKET_MODE GetpacketMode(SimStruct *S)
{
    PACKET_MODE res;
    char *s;
    
    /* Packet mode, Ascii | Binary*/
    s = mxArrayToString(ssGetSFcnParam(S, ARGC_PACKETMODE));
    if(strcmp(s, "Binary") == 0)
        res = packetBinary;
    else
        res = packetAscii;
    mxFree(s);
    
    return res;
}

/* Temporary buffer for Rx processing */
BYTE rx_temp_buffer[2048+1];
BYTE rx_temp_scanf_result[512]; /* 8 x 64 */
BYTE tx_temp_buffer[2048];

BYTE ascii_tmp_formatted_string[2048];
BYTE ascii_tmp_segmented_string[2048];

#define MAX_DATA_TRANS_RECEIVE 2048

char ascii_format[2048];

extern DWORD WINAPI GetTickCount(void);

static void mdlOutputs(SimStruct *S, int_T tid) {
	char error_msg[1024];
    char* ascii_pformat_str;
    int ascii_segment_index = 0;
    
    double timeout_source;
    
    
    PORT_READ_INFO* read_info;
    char *id;
    char *s;
    char conf[12];
    char port[12];
    BYTE b;
    BOOL ready;
    int i;
    int output_idx;
    int output_count;
    
    int input_idx;
    int input_count;
    
    int bytes_index;
    
    DWORD rx_timer_start;
    BOOL read_result;
    
    //char error_msg[1024];
    
    /* Configuration */
    s = mxArrayToString(ssGetSFcnParam(S, ARGC_CONFIGURATION));
    strcpy(conf, s);
    mxFree(s);
    
    /* Port */
    s = mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
    strcpy(port, s);
    mxFree(s);
	
	/* Check if port is not Openning */
	if(!PortIsOpen(port)) {
		PORT_CONFIGURATION *port_setup = PortCongiguration_Get (port);		
		if (port_setup == NULL) {
			sprintf(error_msg, "Require \"Host Serial Setup\" block for: \"%s\".\n", port);
			ssSetErrorStatus(S, (char*)error_msg);
		}
		else {	
			if(OpenPort(port, port_setup->BaudRate, port_setup->DataBits, 
					port_setup->StopBits, port_setup->Timeout) == FALSE) {
				// Close port
				ClosePort(port);
				
				sprintf(error_msg, "Failed to open COM port: \"%s\".\n", port);
				ssSetErrorStatus(S, (char*)error_msg);
				return;
			}
			mexPrintf("Port open: %s\n", port);
		}
	}
    

    //ready = TRUE;
    //while (ready) {
    //    if(ReadByteNonBlocking(port, &ready) == FALSE) {
    //        ssSetErrorStatus(S, (char*)"Failed to Read data from port!\n");
    //        return;
    //    }
    //}
    
    /* ********************************************************************
     * Setup block
     * ********************************************************************
     */
    if(!strcmp(conf, "Setup")) {
    }
    /* ********************************************************************
     * Tx block
     * ********************************************************************
     */
    else if(!strcmp(conf, "Tx")) {
        /* ############################################################
         * Binary Tx
         * ############################################################
         */
        input_count = ssGetNumInputPorts(S);
        
        if(GetpacketMode(S) == packetBinary) { /* Rx packet is Binary */
            bytes_index = 0;
            /* Header */
            for(i=0; i<(int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINHEADER)))[0]); i++){
                tx_temp_buffer[bytes_index] = (BYTE)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINHEADER)))[i+1]);
                bytes_index ++;
            }
            
            /* Data */
            for(i=0; i<input_count; i++) {
                switch((BYTE)ssGetInputPortDataType(S, i)) {
                    case 0: /* Double */
                        memcpy(&tx_temp_buffer[bytes_index], (real_T*) ssGetInputPortSignal(S, i), 8);
                        bytes_index += 8;
                        break;
                    case 1: /* Single */                    
                        memcpy(&tx_temp_buffer[bytes_index], (real32_T*) ssGetInputPortSignal(S, i), 4);
                        bytes_index += 4;
                        break;                    
                    case 2: /* int8 */
                        memcpy(&tx_temp_buffer[bytes_index], (int8_T*) ssGetInputPortSignal(S, i), 1);
                        bytes_index += 1;
                        break;
                    case 3: /* uint8 */
                        memcpy(&tx_temp_buffer[bytes_index], (uint8_T*) ssGetInputPortSignal(S, i), 1);
                        bytes_index += 1;
                        break;                    
                    case 4: /* int16 */
                        memcpy(&tx_temp_buffer[bytes_index], (int16_T*) ssGetInputPortSignal(S, i), 2);
                        bytes_index += 2;
                        break;
                    case 5: /* uint16 */
                        memcpy(&tx_temp_buffer[bytes_index], (uint16_T*) ssGetInputPortSignal(S, i), 2);
                        bytes_index += 2;
                        break;
                    case 6: /* int32 */
                        memcpy(&tx_temp_buffer[bytes_index], (int32_T*) ssGetInputPortSignal(S, i), 4);
                        bytes_index += 4;
                        break;
                    case 7: /* uint32 */
                        memcpy(&tx_temp_buffer[bytes_index], (uint32_T*) ssGetInputPortSignal(S, i), 4);
                        bytes_index += 4;
                        break;
                    default:
                        ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
                        break;
                }
            }
            
            /* Terminator */
            for(i=0; i<(int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINTERMINATOR)))[0]); i++){
                tx_temp_buffer[bytes_index] = (BYTE)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINTERMINATOR)))[i+1]);
                bytes_index ++;
            }
            
            /* Write to Port */
            if(WriteToPort(port, tx_temp_buffer, (DWORD)bytes_index) == FALSE) {
                ssSetErrorStatus(S, (char*)"Failed to write data to port.\n");
                return;
            }
        }
        /* ############################################################
         * Ascii Tx
         * ############################################################
         */
        else { /* Tx packet is Ascii */
            s = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIHEADER));
            strcpy(ascii_format, s);
            mxFree(s);            
            ascii_pformat_str = ascii_format;
            
            bytes_index = 0;
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
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const real_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 1: /* Single */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const real32_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 2: /* int8 */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const int8_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 3: /* uint8 */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const uint8_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 4: /* int16 */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const int16_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 5: /* uint16 */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const uint16_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 6: /* int32 */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const int32_T*) ssGetInputPortSignal(S, i));
                            break;
                        case 7: /* uint32 */
                            bytes_index += sprintf_s((char*)&tx_temp_buffer[bytes_index],
                                    (MAX_DATA_TRANS_RECEIVE-bytes_index),
                                    (char*)ascii_tmp_segmented_string, *(const uint32_T*) ssGetInputPortSignal(S, i));
                            break;
                    }
                    ascii_pformat_str = &ascii_pformat_str[ascii_segment_index];
                }
            }
            
            /* Write to Port */
            if(WriteToPort(port, tx_temp_buffer, (DWORD)bytes_index) == FALSE) {
                ssSetErrorStatus(S, (char*)"Failed to write data to port.\n");
                return;
            }            
        }
    }
    /* ********************************************************************
     * Rx block
     * ********************************************************************
     */
    else if(!strcmp(conf, "Rx")) {
        
        id = mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
        read_info = GetReadInfo(id);        
        mxFree(id);
        
        if(read_info == NULL) {
            ssSetErrorStatus(S, (char*)"Internal error, failed to get Read information for specific block id.\n");
            return;
        }
		
		/* Check if first step */
		if(read_info->FirstStep) {
			read_info->FirstStep = 0; /* Clear first step flag */
			if(read_info->InitialValuesSize > 0) {			
				/* Number of port, expect it is blocking mode */
				output_count = ssGetNumOutputPorts(S);
				if(read_info->InitialValuesSize != output_count) {
					ssSetErrorStatus(S, (char*)"Invalid number of initial value.\n");
					return;
				}
				
				/* Set value to port */
				i = 0;
				while(i < output_count) {
					switch((BYTE)ssGetOutputPortDataType(S, i)) {
						case 0: /* Double */
							*(real_T*) ssGetOutputPortSignal(S, i) = read_info->InitialValues[i];
							break;
						case 1: /* Single */
							*(real32_T*) ssGetOutputPortSignal(S, i) = (real32_T)read_info->InitialValues[i];
							break;
						case 2: /* int8 */
							*(int8_T*) ssGetOutputPortSignal(S, i) = (int8_T)read_info->InitialValues[i];
							break;
						case 3: /* uint8 */
							*(uint8_T*) ssGetOutputPortSignal(S, i) = (uint8_T)read_info->InitialValues[i];
							break;
						case 4: /* int16 */
							*(int16_T*) ssGetOutputPortSignal(S, i) = (int16_T)read_info->InitialValues[i];
							break;
						case 5: /* uint16 */
							*(uint16_T*) ssGetOutputPortSignal(S, i) = (uint16_T)read_info->InitialValues[i];
							break;
						case 6: /* int32 */
							*(int32_T*) ssGetOutputPortSignal(S, i) = (int32_T)read_info->InitialValues[i];
							break;
						case 7: /* uint32 */
							*(uint32_T*) ssGetOutputPortSignal(S, i) = (uint32_T)read_info->InitialValues[i];
							break;
						default:
							ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
							break;
					}
					i++;
				}
				
				/* Success, return */
				return;
			}
		}
    
        /* Start Timer */
        timeout_source = (double)mxGetScalar(ssGetSFcnParam(S, ARGC_TIMEOUT));
        timeout_source *= 1000;
        
        rx_timer_start = GetTickCount();
        
        /* Init Read info, if it is Blocking */
        if(read_info->transfer_mode == transferBlocking) {
            read_info->global_search_state = 0;
        }
__read_packet_start:
    
        /* Check timeout */
        if(read_info->transfer_mode == transferBlocking) {            
            if((GetTickCount() - rx_timer_start) >= (DWORD)timeout_source) {
                ssSetErrorStatus(S, (char*)"Timeout occurs while Rx Blocking wait for data.\n");
                return;
            }
        }
        
        /* Read a byte */
        if(read_info->transfer_mode == transferBlocking) {
            read_result = ReadBytes(port, (UINT)timeout_source, 1, &b);
        }
        else {
            ready = TRUE;
            while (ready) {
                if(ReadByteNonBlocking(port, &ready) == FALSE) {
                    ssSetErrorStatus(S, (char*)"Failed to Read data from port!\n");
                }
            }
            read_result = ReadBuffer(port, read_info, &b, 1);
        }
        
        if(read_result) {            
            /* Calculate output count */
            output_count = ssGetNumOutputPorts(S);
            if(read_info->transfer_mode == transferNonBlocking) {
                output_count --;
            }
            
            //printf("Receive: %x\n", b);
            /* ############################################################
             * Binary Rx
             * ############################################################
             */
            if(read_info->packet_mode == packetBinary) { /* Rx packet is Binary */
                //printf("Process: %d\n", read_info->global_search_state);
                if(ProcessReadBinaryPacket(b, read_info, rx_temp_buffer, sizeof(rx_temp_buffer))) {
                    /* Write to output */
                    output_idx = 0;
                    if(read_info->transfer_mode == transferNonBlocking) {
                        *(uint8_T*) ssGetOutputPortSignal(S, output_idx) = 1; /* Ready */
                        output_idx ++;
                    }
                    i = 0;
                    bytes_index = 0;
                    while(i < output_count) {
                        switch((BYTE)ssGetOutputPortDataType(S, i+output_idx)) {
                            case 0: /* Double */
                                *(real_T*) ssGetOutputPortSignal(S, i+output_idx) = *((real_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 8;
                                break;
                            case 1: /* Single */
                                *(real32_T*) ssGetOutputPortSignal(S, i+output_idx) = *((real32_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 4;
                                break;
                            case 2: /* int8 */
                                *(int8_T*) ssGetOutputPortSignal(S, i+output_idx) = *((int8_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 1;
                                break;
                            case 3: /* uint8 */
                                *(uint8_T*) ssGetOutputPortSignal(S, i+output_idx) = *((uint8_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 1;
                                break;
                            case 4: /* int16 */
                                *(int16_T*) ssGetOutputPortSignal(S, i+output_idx) = *((int16_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 2;
                                break;
                            case 5: /* uint16 */
                                *(uint16_T*) ssGetOutputPortSignal(S, i+output_idx) = *((uint16_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 2;
                                break;
                            case 6: /* int32 */
                                *(int32_T*) ssGetOutputPortSignal(S, i+output_idx) = *((int32_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 4;
                                break;
                            case 7: /* uint32 */
                                *(uint32_T*) ssGetOutputPortSignal(S, i+output_idx) = *((uint32_T*)&rx_temp_buffer[bytes_index]);
                                bytes_index += 4;
                                break;
                            default:
                                ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
                                break;
                        }
                        i++;
                    }
                }
                else { /* Packet is not ready */
                    if(read_info->transfer_mode == transferNonBlocking) {
                        *(int8_T*) ssGetOutputPortSignal(S, 0) = 0; /* Not ready */
                    }
                    else {
                        goto __read_packet_start;
                    }                    
                }
            }
            /* ############################################################
             * Ascii Rx
             * ############################################################
             */
            else { /* Rx packet is Ascii */
                if(ProcessReadAsciiPacket(b, read_info, (char*)rx_temp_buffer, sizeof(rx_temp_buffer))) { /* Packet is ready */
                    if(sscanf((char*)rx_temp_buffer, (char*)read_info->packet_header, \
                            /* 0 - 7 */
                            (void*)&rx_temp_scanf_result[0*8],
                            (void*)&rx_temp_scanf_result[1*8],
                            (void*)&rx_temp_scanf_result[2*8],
                            (void*)&rx_temp_scanf_result[3*8],
                            (void*)&rx_temp_scanf_result[4*8],
                            (void*)&rx_temp_scanf_result[5*8],
                            (void*)&rx_temp_scanf_result[6*8],
                            (void*)&rx_temp_scanf_result[7*8],
                            /* 8 - 15 */
                            (void*)&rx_temp_scanf_result[8*8],
                            (void*)&rx_temp_scanf_result[9*8],
                            (void*)&rx_temp_scanf_result[10*8],
                            (void*)&rx_temp_scanf_result[11*8],
                            (void*)&rx_temp_scanf_result[12*8],
                            (void*)&rx_temp_scanf_result[13*8],
                            (void*)&rx_temp_scanf_result[14*8],
                            (void*)&rx_temp_scanf_result[15*8],
                            /* 16 - 23 */
                            (void*)&rx_temp_scanf_result[16*8],
                            (void*)&rx_temp_scanf_result[17*8],
                            (void*)&rx_temp_scanf_result[18*8],
                            (void*)&rx_temp_scanf_result[19*8],
                            (void*)&rx_temp_scanf_result[20*8],
                            (void*)&rx_temp_scanf_result[21*8],
                            (void*)&rx_temp_scanf_result[22*8],
                            (void*)&rx_temp_scanf_result[23*8],
                            /* 24 - 31 */
                            (void*)&rx_temp_scanf_result[24*8],
                            (void*)&rx_temp_scanf_result[25*8],
                            (void*)&rx_temp_scanf_result[26*8],
                            (void*)&rx_temp_scanf_result[27*8],
                            (void*)&rx_temp_scanf_result[28*8],
                            (void*)&rx_temp_scanf_result[29*8],
                            (void*)&rx_temp_scanf_result[30*8],
                            (void*)&rx_temp_scanf_result[31*8],
                            /* 32 - 39 */
                            (void*)&rx_temp_scanf_result[32*8],
                            (void*)&rx_temp_scanf_result[33*8],
                            (void*)&rx_temp_scanf_result[34*8],
                            (void*)&rx_temp_scanf_result[35*8],
                            (void*)&rx_temp_scanf_result[36*8],
                            (void*)&rx_temp_scanf_result[37*8],
                            (void*)&rx_temp_scanf_result[38*8],
                            (void*)&rx_temp_scanf_result[39*8],
                            /* 40 - 47 */
                            (void*)&rx_temp_scanf_result[40*8],
                            (void*)&rx_temp_scanf_result[41*8],
                            (void*)&rx_temp_scanf_result[42*8],
                            (void*)&rx_temp_scanf_result[43*8],
                            (void*)&rx_temp_scanf_result[44*8],
                            (void*)&rx_temp_scanf_result[45*8],
                            (void*)&rx_temp_scanf_result[46*8],
                            (void*)&rx_temp_scanf_result[47*8],
                            /* 48 - 55 */
                            (void*)&rx_temp_scanf_result[48*8],
                            (void*)&rx_temp_scanf_result[49*8],
                            (void*)&rx_temp_scanf_result[50*8],
                            (void*)&rx_temp_scanf_result[51*8],
                            (void*)&rx_temp_scanf_result[52*8],
                            (void*)&rx_temp_scanf_result[53*8],
                            (void*)&rx_temp_scanf_result[54*8],
                            (void*)&rx_temp_scanf_result[55*8],
                            /* 56 - 63 */
                            (void*)&rx_temp_scanf_result[56*8],
                            (void*)&rx_temp_scanf_result[57*8],
                            (void*)&rx_temp_scanf_result[58*8],
                            (void*)&rx_temp_scanf_result[59*8],
                            (void*)&rx_temp_scanf_result[60*8],
                            (void*)&rx_temp_scanf_result[61*8],
                            (void*)&rx_temp_scanf_result[62*8],
                            (void*)&rx_temp_scanf_result[63*8]                            
                            ) == output_count) {
                        /* Write to output */
                        output_idx = 0;
                        if(read_info->transfer_mode == transferNonBlocking) {
                            *(uint8_T*) ssGetOutputPortSignal(S, output_idx) = 1; /* Ready */
                            output_idx ++;
                        }
                        i = 0;
                        while(i < output_count) {
                            switch((BYTE)ssGetOutputPortDataType(S, i+output_idx)) {
                                case 0: /* Double */
                                    *(real_T*) ssGetOutputPortSignal(S, i+output_idx) = *((real_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 1: /* Single */
                                    *(real32_T*) ssGetOutputPortSignal(S, i+output_idx) = *((real32_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 2: /* int8 */
                                    *(int8_T*) ssGetOutputPortSignal(S, i+output_idx) = *((int8_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 3: /* uint8 */
                                    *(uint8_T*) ssGetOutputPortSignal(S, i+output_idx) = *((uint8_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 4: /* int16 */
                                    *(int16_T*) ssGetOutputPortSignal(S, i+output_idx) = *((int16_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 5: /* uint16 */
                                    *(uint16_T*) ssGetOutputPortSignal(S, i+output_idx) = *((uint16_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 6: /* int32 */
                                    *(int32_T*) ssGetOutputPortSignal(S, i+output_idx) = *((int32_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                case 7: /* uint32 */
                                    *(uint32_T*) ssGetOutputPortSignal(S, i+output_idx) = *((uint32_T*)&rx_temp_scanf_result[i*8]);
                                    break;
                                default:
                                    ssSetErrorStatus(S, (char*)"Internal error, invalid port data type.\n");
                                    break;
                            }
                            i++;
                        }
                    }
                    else {
                        if(read_info->transfer_mode == transferNonBlocking) {
                            *(uint8_T*) ssGetOutputPortSignal(S, 0) = 0; /* Not ready */
                        }
                        else {
                            goto __read_packet_start;
                        }
                    }
                }
                else { /* Packet is not ready */
                    if(read_info->transfer_mode == transferNonBlocking) {
                        *(uint8_T*) ssGetOutputPortSignal(S, 0) = 0; /* Not ready */
                    }
                    else {
                        goto __read_packet_start;
                    }
                }
            }
        }
        else {
            
            if(read_info->transfer_mode == transferNonBlocking) {
                *(int8_T*) ssGetOutputPortSignal(S, 0) = 0; /* Not ready */
            }
            else {
                goto __read_packet_start;
            }
        }
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
    int i;
    char *s;
    char *id;
    char *ascii_header;
    char conf[12];
    char port[12];
    char stop[12];
    char error_msg[1024];
    ULONG baud;
    ULONG stop_bits;
    BYTE databits;
    
    BYTE *packet_terminator;
    PORT_READ_INFO *read_info;
    
    
    
    /* Configuration */
    s = mxArrayToString(ssGetSFcnParam(S, ARGC_CONFIGURATION));
    strcpy(conf, s);
    mxFree(s);
    
    /* ********************************************************************
     * Setup block
     * ********************************************************************
     */
    if(!strcmp(conf, "Setup")) {
        
        /* Port */
        s = mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
        strcpy(port, s);
        mxFree(s);
    
        /* Stop */
        s = mxArrayToString(ssGetSFcnParam(S, ARGC_STOPBITS));
        strcpy(stop, s);
        mxFree(s);
        
        /* Baud */
        baud = (ULONG)mxGetScalar(ssGetSFcnParam(S, ARGC_BAUDRATE));
        
        /* Check if the port is already opened */
/*
        if(PortIsOpen(port)) {
            sprintf(error_msg, "Detect %s has multiple setup block!\n", port);
            ssSetErrorStatus(S, (char*)error_msg);
        }
*/
        
        /* Stop bits */
        if(!strcmp(stop, "1")) {
            stop_bits = ONESTOPBIT;
        }
        else if(!strcmp(stop, "1.5")) {
            stop_bits = ONE5STOPBITS;
        }
        else if(!strcmp(stop, "2")) {
            stop_bits = TWOSTOPBITS;
        }
        else {
            ssSetErrorStatus(S, "Invalid stop bits!.\n");
            return;
        }        
        
		/* Store Port configuration */
		if (PortCongiguration_Get (port) == NULL) {
			if (PortConfiguration_Put (port, (ULONG)2000, baud, 8, (BYTE)stop_bits, (BYTE)0) < 0) {
				sprintf(error_msg, "Failed to setup port: \"%s\"\n", port);
				ssSetErrorStatus(S, (char*)error_msg);
			}
		}
		else {
            sprintf(error_msg, "Detect multiple Host Serial Setup block for \"%s\"!\n", port);
            ssSetErrorStatus(S, (char*)error_msg);			
		}
		
        /* Open port */
/*
        if(OpenPort(port, baud, 8, (BYTE)stop_bits, (ULONG)2000) == FALSE) {
            // Close port
            ClosePort(port);
            
            sprintf(error_msg, "Failed to open COM port: \"%s\".\n", port);
            ssSetErrorStatus(S, (char*)error_msg);

            return;
        }
*/
    }
    /* ********************************************************************
     * Rx block
     * ********************************************************************
     */
    else if(!strcmp(conf, "Rx")) {
        /* blockid */
        id = mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
        
        /* Create read info */
        if((read_info = CreateReadInfo(id, 2048)) == NULL) { // id will Free at terminate
            ssSetErrorStatus(S, (char*)"Failed to create read info.\n");
        }
        
        /* Packet mode, Ascii | Binary*/
        s = mxArrayToString(ssGetSFcnParam(S, ARGC_PACKETMODE));
        if(strcmp(s, "Binary") == 0)
            read_info->packet_mode = packetBinary;
        else
            read_info->packet_mode = packetAscii;
        mxFree(s);
        
        /* Transfer mode, Blocking | Non-Blocking */
        s = mxArrayToString(ssGetSFcnParam(S, ARGC_TRANSFER));
        if(strcmp(s, "Blocking") == 0)
            read_info->transfer_mode = transferBlocking;
        else
            read_info->transfer_mode = transferNonBlocking;
        mxFree(s);
        
        /* Header */
        if(read_info->packet_mode == packetBinary) { /* Binary */            
            read_info->packet_header_index = 0;
            read_info->packet_header_count = (DWORD)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINHEADER)))[0]);
            read_info->packet_header = (BYTE*)malloc(read_info->packet_header_count);
            for(i=0; i<(int)read_info->packet_header_count; i++) {
                read_info->packet_header[i] = (BYTE)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINHEADER)))[i+1]);
            }
        }
        else { /* Ascii */
            ascii_header = mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIHEADER));            
            read_info->packet_header_index = 0;
            read_info->packet_header_count = strlen(ascii_header);
            read_info->packet_header = (BYTE*)malloc(read_info->packet_header_count+1);
            memcpy((char*)read_info->packet_header, ascii_header, (read_info->packet_header_count+1));
            mxFree(ascii_header);
        }
        
        /* Data length */
        if(read_info->packet_mode == packetBinary) { /* Binary */
            read_info->packet_data_len = (DWORD)mxGetScalar(ssGetSFcnParam(S, ARGC_BINDATALENGTH));
        }
        else { /* Ascii */
            read_info->packet_data_len = 0;
        }
        
        /* Terminator */
        if(read_info->packet_mode == packetBinary) { /* Binary */            
            read_info->packet_terminator_index = 0;
            read_info->packet_terminator_count = (DWORD)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINTERMINATOR)))[0]);
            read_info->packet_terminator = (BYTE*)malloc(read_info->packet_terminator_count);
            for(i=0; i<(int)read_info->packet_terminator_count; i++) {
                read_info->packet_terminator[i] = (BYTE)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_BINTERMINATOR)))[i+1]);
            }
        }
        else { /* Ascii */
            read_info->packet_terminator_index = 0;
            read_info->packet_terminator_count = (DWORD)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_ASCIITERMINATOR)))[0]);
            read_info->packet_terminator = (BYTE*)malloc(read_info->packet_terminator_count);
            for(i=0; i<(int)read_info->packet_terminator_count; i++) {
                read_info->packet_terminator[i] = (BYTE)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_ASCIITERMINATOR)))[i+1]);
            }
        }
		
		/* Initial value */
		read_info->FirstStep = 1;
		if(read_info->transfer_mode == transferBlocking) {		
			read_info->InitialValuesSize = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INTIAL_VALUES));
			if(read_info->InitialValuesSize > 0) {
				read_info->InitialValues = (double *)mxMalloc(read_info->InitialValuesSize * sizeof(double));
				if(read_info->InitialValues == NULL) {
					ssSetErrorStatus(S, (char*)"Internal error, out of memory.\n");
				}
				for(i=0; i<read_info->InitialValuesSize; i++) {
					read_info->InitialValues[i] = ((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INTIAL_VALUES)))[i];
				}
			}
			else {
				read_info->InitialValues = NULL;
			}
		}
		else {			
			read_info->InitialValuesSize = 0;
			read_info->InitialValues = NULL;
		}		
	}
    /* ********************************************************************
     * Tx block
     * ********************************************************************
	 */
	else {
        /* id */
        //id = mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
        //printf("ID: %s\n", id);
		
        
	}
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S) {
    int i;
    char *s;
    char port[12];

    /* Port */
    s = mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
    strcpy(port, s);
    mxFree(s);
	
    if(ClosePort(port) == FALSE) {
        printf("Failed to close port: ");
        printf(port);
        printf("\n");
    }
    
    /* Free read info */
    FreeReadInfo();
	/* Clear port setup */
	PortCOnfiguration_Clear();
} /* end mdlTerminate */

#define MDL_RTW

/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
    int NOutputPara = 2; /* Number of parameters to output to model.rtw */
    
    char *conf; // ARCG_CONF
    char *blockid;
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFIGURATION));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "conf", conf,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid            
            )) {
        return; /* An error occurred which will be reported by SL */
    }

    mxFree(conf);
    mxFree(blockid);    
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function amg_usbconverter_n_serial.c"
#endif



/* ########################################################################
 * Serial port communication routine
 * - 
 * ########################################################################
 */

#define MAX_IN_OUT_PORT         128 /* Maximum number allowed for port signal  */
#define MAX_DATA_TRANS_RECEIVE       1024 /* Maximum bytes a packet of transmit and receive  */

#define PORT_HANDLE_COUNT         128
#define MAX_PORTNAME_SIZE      32
typedef struct _PORT_INFO {
    char m_PortName[MAX_PORTNAME_SIZE]; /* Port Name max 32 byte */
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
    
    /* Port Read handling */
    //BOOL bResult;
    //DWORD dwError; /* Last error status */
    DWORD dwNDTR; /* Number of data to read */
    OVERLAPPED m_ovl; /* Asynchronious read for COM Receiving */
    
    /* Read buffer */
    BYTE* read_buffer; /* Dynamic buffer allocation */
    BYTE* write_buffer; /* Dynamic buffer allocation */
    WORD read_buffer_size;
    WORD write_buffer_size;
    WORD read_index;
    WORD write_index;
} PORT_HANDLE;


#define MAX_INFO_LISTCOUNT   256

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
static PORT_READ_INFO *port_list_node = NULL;


/* --------------------------------------------------------------------- */
/* Store port configuration list */
static PORT_CONFIGURATION PortConfigurationList[PORT_HANDLE_COUNT] =
{
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0},
	{"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}, {"\0", 0,0,0,0,0}
};

void PortCOnfiguration_Clear(void)
{
	int i;
	for (i=0; i<PORT_HANDLE_COUNT; i++) {
		memset(&PortConfigurationList[i].port[0], 0, 32);
	}
}

int PortConfiguration_EmptyIndex(void)
{
	int i;
	for (i=0; i<PORT_HANDLE_COUNT; i++) {
		if (PortConfigurationList[i].port[0] == '\0')
			return i;
	}
	return -1;
}

int PortConfiguration_Put (const char *port, ULONG Timeout, ULONG BaudRate,
		BYTE DataBits, BYTE StopBits, BYTE Parity) 
{
	int index;
	
	/* Get index */
	index = PortConfiguration_EmptyIndex();
	if (index < 0)
		return index;
	
	/* Set data to port */
	strcpy(&(PortConfigurationList[index].port[0]), port);
	PortConfigurationList[index].Timeout = Timeout;
	PortConfigurationList[index].BaudRate = BaudRate;
	PortConfigurationList[index].DataBits = DataBits;
	PortConfigurationList[index].StopBits = StopBits;
	PortConfigurationList[index].Parity = Parity;
	
	/* Return */
	return index;
}

PORT_CONFIGURATION *PortCongiguration_Get (const char *port)
{
	int i;
	for (i=0; i<PORT_HANDLE_COUNT; i++) {
		if (!strcmp(PortConfigurationList[i].port, port))
			return &PortConfigurationList[i];
	}
	return NULL;
}

/* --------------------------------------------------------------------- */
/*====================*
 * Link list *
 *====================*/
void InitReadInfo(PORT_READ_INFO* info, char *id, DWORD buffer_size)
{
    info->id = id;
    info->global_data_index = 0;
    info->global_search_index = 0;
    info->global_search_state = 0;
    info->data_buffer_index = 0;
    info->data_buffer_size = buffer_size;
    info->data_buffer = malloc(buffer_size);
    info->Next = NULL;
}

void ClearReadInfo(PORT_READ_INFO* info)
{
    mxFree(info->id);
	if(info->InitialValues) {
		mxFree(info->InitialValues);
	}	
    free(info->data_buffer);
    free(info->packet_terminator);
    free(info->packet_header);
}

PORT_READ_INFO* GetReadInfoLast(void)
{
    PORT_READ_INFO* node;
    if(port_list_node == NULL) {
        return NULL;
    }
    else {
        node = port_list_node;
        while(node->Next != NULL)
            node = node->Next;
        return node;
    }
}

PORT_READ_INFO* CreateReadInfo(char*id, DWORD buffer_size) {
    PORT_READ_INFO* node;
    
    /* Check if it is first node */
    if(port_list_node == NULL) { /* Create head */
        port_list_node = (PORT_READ_INFO*)malloc(sizeof(PORT_READ_INFO));
        InitReadInfo(port_list_node, id, buffer_size);
        return port_list_node;
    }
    /* Find last */
    else {
        node = GetReadInfoLast();
        node->Next = (PORT_READ_INFO*)malloc(sizeof(PORT_READ_INFO));
        InitReadInfo(node->Next, id, buffer_size);
        return node->Next;
    }
}

PORT_READ_INFO* GetReadInfo(char* id) {
    PORT_READ_INFO* node;
    
    node = port_list_node;
    while(node != NULL) {
        if(strcmp(node->id, id) == 0)
            return node;
        else
            node = (PORT_READ_INFO*)node->Next;
    }
    return NULL;
}

void FreeReadInfo(void) {
    PORT_READ_INFO* node;
    PORT_READ_INFO* free_node;
    
    node = port_list_node; /* Head */
    while(node != NULL) {
        free_node = node;
        node = (PORT_READ_INFO*)node->Next;
        
        ClearReadInfo(free_node);
        free(free_node);
    }
    
    /* Free head node */
    port_list_node = NULL;
}

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
        CancelIo(port->m_hComm); // !!!!
        CloseHandle( port->m_hComm );
        port->m_hComm = NULL;
    }
    if(port->read_buffer) {
        free(port->read_buffer);
        port->read_buffer = NULL;
    }
    if(port->write_buffer) {
        free(port->write_buffer);
        port->write_buffer = NULL;
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
        
        /* Init overlap mode */
        memset(&(port->m_ovl), 0, sizeof(OVERLAPPED));
        port->m_ovl.hEvent = port->m_hEvent;   
        
        
        port->dwNDTR = 0; /* Number of data to read */
        
        /* Buffer allocation */
        port->read_index = 0;
        port->write_index = 0;
        port->read_buffer_size = 2048;
        port->write_buffer_size = 2048;
        port->read_buffer = malloc(port->read_buffer_size);        
        port->write_buffer = malloc(port->write_buffer_size);
        if((port->read_buffer == NULL) || (port->write_buffer == NULL)) {
            if(port->read_buffer) {
                free(port->read_buffer);
                port->read_buffer = NULL;
            }
            if(port->write_buffer) {
                free(port->write_buffer);
                port->write_buffer = NULL;
            }
            printf("Failed to allocate buffer for Tx or Rx.\n");
            return FALSE;
        }
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

BOOL ReadRxBuffer(char *PortName, PORT_READ_INFO *read_info)
{
    int port_index;
    
    /* Validate port index */
    if((port_index = GetPortIndex(PortName)) < 0) {
        return FALSE;
    }
    
    /* Processing bytes */
    
    
    
    /* No error */
    return TRUE;
}
/*
 * Add new implementation read byte function.
 * See: http://msdn.microsoft.com/en-us/library/aa365690(v=vs.85).aspx */
BOOL ReadByteNonBlocking(char* PortName, BOOL* ready) {
    BOOL bResult;
    DWORD dwError; 
    DWORD dwRet;
    DWORD dwBytesRead = 0;
    BYTE bff[16];
    int port_index = -1;
    
    /* Init */
    *ready = FALSE;
    
    /* Get port, check if port is open */
    port_index = GetPortIndex(PortName);
    if(port_index < 0) {
        return FALSE;
    }
    if(!Serial_IsOpen(&PortHandleList[port_index])) {
        return FALSE;
    }
    
    /* ====================================================================
     * Start to receive data.
     */
    dwBytesRead = 0;
    if(PortHandleList[port_index].dwNDTR == 0) {
        /* Check read buffer index */
        if(PortHandleList[port_index].read_index >= PortHandleList[port_index].read_buffer_size) {
            PortHandleList[port_index].read_index = 0;
        }
        
        /* Initial overlapse struct */
        memset(&(PortHandleList[port_index].m_ovl), 0, sizeof(OVERLAPPED));
        PortHandleList[port_index].m_ovl.hEvent = PortHandleList[port_index].m_hEvent;    
        
        /* Initial read */
        PortHandleList[port_index].dwNDTR = 1;
        bResult = Serial_Read(&PortHandleList[port_index], \
                (LPVOID)&(PortHandleList[port_index].read_buffer[PortHandleList[port_index].read_index]), 
                PortHandleList[port_index].dwNDTR, &dwBytesRead, &(PortHandleList[port_index].m_ovl));
        dwError = GetLastError();
        
        /* Store into Global variable */
        if(bResult) { /* Success. */
            if(dwBytesRead == PortHandleList[port_index].dwNDTR) { /* All bytes received */
                /* Reset number of bytes to read. */
                PortHandleList[port_index].dwNDTR = 0;
                *ready = TRUE;
                
                /* Increment data index */
                PortHandleList[port_index].read_index++;
            }
            else { /* Error */
                PortHandleList[port_index].dwNDTR = 0;
            }
        }
        else {
            switch(dwError) {
                case ERROR_IO_INCOMPLETE:
				case ERROR_IO_PENDING:
                    /* No use Overlap */
                    //CancelIoEx(PortHandleList[port_index].m_hComm, &(PortHandleList[port_index].m_ovl));
                    //PortHandleList[port_index].dwNDTR = 0;
                    break;                    
                default: /* Failed to read from port! */
                    return FALSE;
            }
        }
    }
    
    /* ====================================================================
     * Get overlap
     */
    else {        
        bResult = GetOverlappedResult( PortHandleList[port_index].m_hComm, 
                &(PortHandleList[port_index].m_ovl), &dwBytesRead, FALSE);
        dwError = GetLastError();
        if(bResult) { /* Success */
            if(dwBytesRead == PortHandleList[port_index].dwNDTR) { /* All bytes received */
                /* Reset number of bytes to read. */
                PortHandleList[port_index].dwNDTR = 0;
                *ready = TRUE;
                
                /* Increment data index */
                PortHandleList[port_index].read_index++;                
            }
            else { /* Success but no byte received */
                PortHandleList[port_index].dwNDTR = 0;
            }            
        }
        else {
            switch(dwError) {
                case ERROR_IO_INCOMPLETE:
				case ERROR_IO_PENDING:
                    break;
                    
                default: /* Failed to read from port! */
                    return FALSE;
            }
        }
    }
    
    /* No Error */
    return TRUE;
}

BOOL ReadByte(char* PortName, DWORD timeout, BYTE* data) {
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
    memset(&ovl, 0, sizeof(OVERLAPPED));
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

BOOL ReadBytes(char* PortName, DWORD timeout, UINT count, BYTE* data) {
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
                if(dwBytesRead != count)
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
    if(strlen(PortName) > (MAX_PORTNAME_SIZE-1))
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

/* -1: Error
 *  0: Success, no byte available
 *  1: Success, 1 byte available
 *  N: Success, N bytes available
 */
int ReadBuffer(char* PortName, PORT_READ_INFO* read_info, BYTE *buffer, DWORD read_count)
{
    int port_index = -1;
    DWORD count;
    
    /* Get port, check if port is open */
    port_index = GetPortIndex(PortName);
    if(port_index < 0)
        return -1;
    
    count = 0;
    /* Read while data available, until buffer full */
    while((count < read_count) && (read_info->global_data_index != PortHandleList[port_index].read_index)) {
        /* Get a byte */
        buffer[count] = (BYTE)(PortHandleList[port_index].read_buffer[read_info->global_data_index]);
        /* Update index */
        read_info->global_data_index ++;
        read_info->global_data_index &= (2048-1); // TODO:
        /* Update read count */
        count ++;
    }
    return (int)count;
}

BOOL ProcessReadAsciiPacket(BYTE b, PORT_READ_INFO* read_info, char *buffer, DWORD buffer_size)
{
    /* Search state */
    switch(read_info->global_search_state) {
        case 0: /* Ascii format/ body */
            if((b == (BYTE)'\r') || (b == (BYTE)'\n')) {
                read_info->global_search_state ++; /* It is terminator, next state */
                read_info->packet_terminator_index = 0;
            }
            else {
                read_info->data_buffer[read_info->data_buffer_index] = b;
                read_info->data_buffer_index ++;
                break;            
            }
        default: /* Terminator */
            if(read_info->packet_terminator[read_info->packet_terminator_index] == b) {
                /* Also store into data buffer */
                read_info->data_buffer[read_info->data_buffer_index] = b;
                read_info->data_buffer_index ++;
                read_info->data_buffer[read_info->data_buffer_index] = 0; /* String terminator */

                /* Check terminator */
                read_info->packet_terminator_index ++;
                if(read_info->packet_terminator_index >= read_info->packet_terminator_count) {
                    /* Done, return data */
                    strcpy(buffer, (char*)read_info->data_buffer);
                    
                    /* Reset */
                    read_info->global_search_state = 0;
                    read_info->packet_terminator_index = 0;
                    read_info->data_buffer_index = 0;
                    
                    return TRUE;
                }
            }
            else { /* Terminator not match, reset all */
                read_info->global_search_state = 0;
                read_info->packet_terminator_index = 0;
                read_info->data_buffer_index = 0;
            }
            break;
    }
    /* Packet is not ready */
    return FALSE;
}

BOOL ProcessReadBinaryPacket(BYTE b, PORT_READ_INFO* read_info, BYTE *buffer, DWORD buffer_size)
{
    
    /* Search state */
    switch(read_info->global_search_state) {
        case 0: /* Binary header */
__search_retry:            
            if(read_info->packet_header_index >= read_info->packet_header_count) { /* Allow empty header */
                read_info->global_search_state ++;
                read_info->data_buffer_index = 0;
            }
            else {
                if(read_info->packet_header[read_info->packet_header_index] == b) {
                    read_info->packet_header_index ++;
                }
                else { /* Invalid header, reset */
                    read_info->packet_header_index = 0;
                }
                break;
            }

        case 1: /* Binary data */
            if(read_info->data_buffer_index >= read_info->packet_data_len) {
                read_info->global_search_state ++;
                read_info->packet_terminator_index = 0;
            }
            else {
                read_info->data_buffer[read_info->data_buffer_index] = b;
                read_info->data_buffer_index ++;
                break;
            }
            
        default: /* Terminator */
            if(read_info->packet_terminator_index >= read_info->packet_terminator_count) { /* No terminator */
                /* Reset */
                read_info->packet_header_index = 0;
                read_info->global_search_state = 0;
                        
                /* Return data */
                memcpy(buffer, read_info->data_buffer, read_info->data_buffer_index);
                        
                return TRUE;
            }
            else {
                if(read_info->packet_terminator[read_info->packet_terminator_index] == b) {
                    read_info->packet_terminator_index++;
                    /* Check */
                    if(read_info->packet_terminator_index >= read_info->packet_terminator_count) {
                        /* Reset */
                        read_info->packet_header_index = 0;
                        read_info->global_search_state = 0;
                        
                        /* Return data */
                        memcpy(buffer, read_info->data_buffer, read_info->data_buffer_index);
                        
                        return TRUE;
                    }
                }
                else { /* Invalid terminator */
                    read_info->packet_header_index = 0;
                    read_info->global_search_state = 0;
                    goto __search_retry;
                }
            }
            break;
    }
    
    /* Packet is not ready */
    return FALSE;
}

/* EOF */
