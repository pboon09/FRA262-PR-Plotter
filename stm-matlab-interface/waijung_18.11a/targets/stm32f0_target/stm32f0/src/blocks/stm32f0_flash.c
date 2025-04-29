#define S_FUNCTION_NAME  stm32f0_flash
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_CONF = 0,    
	
	ARGC_BASE_ADDRESS,
	ARGC_FILENAME,
	ARGC_PAGESIZE,
    
    ARGC_INPUT_PORTTYPE,
    ARGC_INPUT_PORTWIDTH,
    ARGC_OUTPUT_PORTTYPE,
    ARGC_OUTPUT_PORTWIDTH,
    
    ARGC_OPTIONSTRING,
    
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
    
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
        if(k<input_width_count) {
			width = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH)))[k]);
            ssSetInputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetInputPortWidth(S, k, 1);
		}
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE)))[k]));
		ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

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

#define FLASH_DATA_BUFFER_SIZE (128*1024) // 128k
static uint8_T Flash_Data_Buffer[FLASH_DATA_BUFFER_SIZE];
static uint8_T Flash_Data_Initialized = 0;

// Load data from file into buffer (flash)
static void Flash_LoadFile(SimStruct *S)
{
	FILE *f;
	size_t reading_count;
	char *filename;
	
	// Default
	memset(&Flash_Data_Buffer[0], 0xFF, FLASH_DATA_BUFFER_SIZE);
	//
	filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
	f = fopen((const char *)filename, "rb");
	if (f) {
		reading_count = fread(&Flash_Data_Buffer[0], 1, FLASH_DATA_BUFFER_SIZE, f);
		if(reading_count != FLASH_DATA_BUFFER_SIZE) { /* Reset the reading value */
		}
		/* Close file */
		fclose(f);
	}
	mxFree(filename);
}

// Save data to file
static void Flash_SaveFile(SimStruct *S)
{
	FILE *f;	
	char *filename;
	
	filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
	f = fopen((const char *)filename, "wb");
	if (f) {
		if(fwrite(&Flash_Data_Buffer[0], 1, FLASH_DATA_BUFFER_SIZE, f) != FLASH_DATA_BUFFER_SIZE) {
			/* Failed to write into file. */
		}
		/* Close file */
		fclose(f);
	}
	mxFree(filename);
}

#define MDL_START
static void mdlStart(SimStruct *S) {	
	// Load flash data from file
	if (Flash_Data_Initialized == 0) {
		Flash_Data_Initialized = 1;
		
		Flash_LoadFile(S);
	}
} /* mdlStart */

static uint32_T FLASH_GetBaseAddress(SimStruct *S)
{
	uint32_T value = 0x8000000;
	char *s;
	s = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BASE_ADDRESS));
	if (sscanf(s, "0x%x", &value) != 1) {
		ssSetErrorStatus(S, "\nInvalid flash Base address.\n");
	}	
	mxFree(s);
	
	return value;
}

static uint32_T FLASH_GetPageSize(SimStruct *S)
{
	uint32_T value;
	char *s;
	
	s = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PAGESIZE));
	if (!strcmp(s, "2k"))
		value = 2048;
	else if (!strcmp(s, "1k"))
		value = 1024;
	else
		ssSetErrorStatus(S, "\nInvalid Page size.\n");
	mxFree(s);
	
	return value;
}

static void mdlOutputs(SimStruct *S, int_T tid) {
	char conf[64]; // ARCG_CONF
	char *s;
	uint32_T Offset, MemBase, Address, BufferPointer;
	
	// conf
    s = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
	strcpy(&conf[0], s);
	mxFree(s);
	
	// Offset
	Offset = *((const uint32_T*) ssGetInputPortSignal(S, 0));
	// MemBase
	MemBase = FLASH_GetBaseAddress(S);
	// Address
	Address = MemBase + Offset;
	if ((Address >= (0x8000000 | FLASH_DATA_BUFFER_SIZE)) || (Address < 0x8000000)) {
		ssSetErrorStatus(S, "\nInvalid memory Offset/ Address.\n");
		return;
	}
	// BufferPointer
	BufferPointer = Address - 0x8000000;
	
	// Erase
	if (!strcmp(conf, "Erase")) {
		int32_T Output_Count;
		uint8_T Status;
		uint32_T pagesize = FLASH_GetPageSize(S);
		
		// Output port count
		Output_Count = ssGetNumOutputPorts(S);
		Status = 0;
		
		// Check page address
		if ((Address & (pagesize-1)) != 0) {
			if (Output_Count > 0) Status = 0xFF;
			else ssSetErrorStatus(S, "\nPage Offset value must be a multiple of Page size.\n");
		}
		// Check range
		if ((BufferPointer + pagesize) > FLASH_DATA_BUFFER_SIZE) {
			if (Output_Count > 0) Status = 0xFF;
			else ssSetErrorStatus(S, "\nInvalid page address to Erase.\n");
		}
		// Erase
		if (Status == 0) {
			//mexPrintf("\nAddr: %X", Address);
			//mexPrintf("\nErase: %X\n", BufferPointer);
			memset(&Flash_Data_Buffer[BufferPointer], 0xFF, pagesize);
		}
		// Status
		if (Output_Count > 0)
			*(int8_T*) ssGetOutputPortSignal(S, 0) = Status;
	}	
	// Write
	else if (!strcmp(conf, "Write")) {
		int32_T Output_Count;
		uint8_T Status;
		int data_bytes;
		const uint8_T free_data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		
		// Output port count
		Output_Count = ssGetNumOutputPorts(S);
		Status = 0;		
		
		switch((unsigned char)ssGetInputPortDataType(S, 0)) {
			// double
			case 0: {
				const real_T value = *((const real_T*) ssGetInputPortSignal(S, 1));				
				if ((BufferPointer + 8) <= FLASH_DATA_BUFFER_SIZE) {
					if (memcmp(&Flash_Data_Buffer[BufferPointer], free_data, 8) == 0)
						memcpy(&Flash_Data_Buffer[BufferPointer], &value, 8);
					else {
						if (Output_Count > 0) Status = 0xFF;
						else ssSetErrorStatus(S, "\nFlash must be erased before Write.\n");
					}
				}
				else
					if (Output_Count > 0) Status = 0xFF;
					else ssSetErrorStatus(S, "\nOffset value for Flash write is out of range.\n");
				break;
			}
			// single
			case 1: {
				const real32_T value = *((const real32_T*) ssGetInputPortSignal(S, 1));
				if ((BufferPointer + 4) <= FLASH_DATA_BUFFER_SIZE) {
					if (memcmp(&Flash_Data_Buffer[BufferPointer], free_data, 4) == 0)
						memcpy(&Flash_Data_Buffer[BufferPointer], &value, 4);
					else {
						if (Output_Count > 0) Status = 0xFF;
						else ssSetErrorStatus(S, "\nFlash must be erased before Write.\n");
					}
				}
				else
					if (Output_Count > 0) Status = 0xFF;
					else ssSetErrorStatus(S, "\nOffset value for Flash write is out of range.\n");
				break;
			}
			// int16
			case 4: {
				const int16_T value = *((const int16_T*) ssGetInputPortSignal(S, 1));
				if ((BufferPointer + 2) <= FLASH_DATA_BUFFER_SIZE) {
					if (memcmp(&Flash_Data_Buffer[BufferPointer], free_data, 2) == 0)
						memcpy(&Flash_Data_Buffer[BufferPointer], &value, 2);
					else {
						if (Output_Count > 0) Status = 0xFF;
						else ssSetErrorStatus(S, "\nFlash must be erased before Write.\n");
					}
				}
				else {
					if (Output_Count > 0) Status = 0xFF;
					else ssSetErrorStatus(S, "\nOffset value for Flash write is out of range.\n");
				}
				break;
			}
			// uint16
			case 5: {
				const uint16_T value = *((const uint16_T*) ssGetInputPortSignal(S, 1));				
				if ((BufferPointer + 2) <= FLASH_DATA_BUFFER_SIZE) {
					if (memcmp(&Flash_Data_Buffer[BufferPointer], free_data, 2) == 0)
						memcpy(&Flash_Data_Buffer[BufferPointer], &value, 2);
					else {
						if (Output_Count > 0) Status = 0xFF;
						else ssSetErrorStatus(S, "\nFlash must be erased before Write.\n");
					}
				}
				else {
					if (Output_Count > 0) Status = 0xFF;
					else ssSetErrorStatus(S, "\nOffset value for Flash write is out of range.\n");					
				}
				break;
			}
			// int32
			case 6: {
				const int32_T value = *((const int32_T*) ssGetInputPortSignal(S, 1));				
				if ((BufferPointer + 4) <= FLASH_DATA_BUFFER_SIZE) {
					if (memcmp(&Flash_Data_Buffer[BufferPointer], free_data, 4) == 0)
						memcpy(&Flash_Data_Buffer[BufferPointer], &value, 4);
					else {
						if (Output_Count > 0) Status = 0xFF;
						else ssSetErrorStatus(S, "\nFlash must be erased before Write.\n");					
					}
				}
				else {
					if (Output_Count > 0) Status = 0xFF;
					else ssSetErrorStatus(S, "\nOffset value for Flash write is out of range.\n");				
				}
				break;
			}
			// uint32
			case 7: {
				const uint32_T value = *((const uint32_T*) ssGetInputPortSignal(S, 1));
				if ((BufferPointer + 4) <= FLASH_DATA_BUFFER_SIZE) {
					if (memcmp(&Flash_Data_Buffer[BufferPointer], free_data, 4) == 0)
						memcpy(&Flash_Data_Buffer[BufferPointer], &value, 4);
					else {
						if (Output_Count > 0) Status = 0xFF;
						else ssSetErrorStatus(S, "\nFlash must be erased before Write.\n");
					}
				}
				else {
					if (Output_Count > 0) Status = 0xFF;
					else ssSetErrorStatus(S, "\nOffset value for Flash write is out of range.\n");
				}
				break;
			}
			// Default
			default:
				ssSetErrorStatus(S, "\nInvalid or not supported data type.\n");
				break;
		}
		// Status
		if (Output_Count > 0)
			*(int8_T*) ssGetOutputPortSignal(S, 0) = Status;		
	}
	// Read
	else if (!strcmp(conf, "Read")) {
		switch((unsigned char)ssGetInputPortDataType(S, 0)) {
			// double
			case 0: {
				real_T value;
				if ((BufferPointer + 8) <= FLASH_DATA_BUFFER_SIZE) {					
					memcpy(&value, &Flash_Data_Buffer[BufferPointer], 8);
					*(real_T*) ssGetOutputPortSignal(S, 0) = value;
				}
				else
					ssSetErrorStatus(S, "\nAddress is out of range.\n");
				break;
			}
			// single
			case 1: {
				real32_T value;
				if ((BufferPointer + 4) <= FLASH_DATA_BUFFER_SIZE) {					
					memcpy(&value, &Flash_Data_Buffer[BufferPointer], 4);
					*(real32_T*) ssGetOutputPortSignal(S, 0) = value;
				}
				else
					ssSetErrorStatus(S, "\nAddress is out of range.\n");
				break;
			}
			// int16
			case 4: {
				int16_T value;
				if ((BufferPointer + 2) <= FLASH_DATA_BUFFER_SIZE) {
					memcpy(&value, &Flash_Data_Buffer[BufferPointer], 2);
					*(int16_T*) ssGetOutputPortSignal(S, 0) = value;
				}
				else
					ssSetErrorStatus(S, "\nAddress is out of range.\n");
				break;
			}
			// uint16
			case 5: {
				uint16_T value;
				if ((BufferPointer + 2) <= FLASH_DATA_BUFFER_SIZE) {
					memcpy(&value, &Flash_Data_Buffer[BufferPointer], 2);
					*(uint16_T*) ssGetOutputPortSignal(S, 0) = value;
				}
				else
					ssSetErrorStatus(S, "\nAddress is out of range.\n");
				break;
			}
			// int32
			case 6: {
				int32_T value;
				if ((BufferPointer + 4) <= FLASH_DATA_BUFFER_SIZE) {
					memcpy(&value, &Flash_Data_Buffer[BufferPointer], 4);
					*(int32_T*) ssGetOutputPortSignal(S, 0) = value;
				}
				else
					ssSetErrorStatus(S, "\nAddress is out of range.\n");
				break;
			}
			// uint32
			case 7: {
				uint32_T value;
				if ((BufferPointer + 4) <= FLASH_DATA_BUFFER_SIZE) {
					memcpy(&value, &Flash_Data_Buffer[BufferPointer], 4);
					*(uint32_T*) ssGetOutputPortSignal(S, 0) = value;
				}
				else
					ssSetErrorStatus(S, "\nAddress is out of range.\n");				
				break;
			}
			// Invalid
			default:
				ssSetErrorStatus(S, "\nInvalid or not supported data type.\n");
				break;
		}
	}
	else {
		ssSetErrorStatus(S, "\nInvalid conf.\n");
	}
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
	// Save data to file
	if (Flash_Data_Initialized != 0) {
		Flash_Data_Initialized = 0;
		
		Flash_SaveFile(S);
	}	
} /* end mdlTerminate */

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
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
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

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f0_flash.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f0_flash.c"
#endif

