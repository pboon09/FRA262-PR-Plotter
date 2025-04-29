#define S_FUNCTION_NAME  waijung_data_stream
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_CONF = 0,    
    
    ARGC_INPUT_PORTTYPE,
    ARGC_INPUT_PORTWIDTH,
    ARGC_OUTPUT_PORTTYPE,
    ARGC_OUTPUT_PORTWIDTH,
    
    ARGC_OPTIONSTRING,
	ARGC_SIM_PARAMS,
    
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
		ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE)))[k]));
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
	//ssPrintf("-mdlEnable-\n");
}
#endif

#define MDL_DISABLE
#if defined(MDL_DISABLE) && defined(MATLAB_MEX_FILE)
static void mdlDisable(SimStruct *S){
	//ssPrintf("-mdlDisable-\n");
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
	//ssPrintf("-mdlStart-\n");
}

static void mdlOutputs(SimStruct *S, int_T tid) {
	int num;
	double *options;
	
	/* Simulate options */
	int configuration; /* Packet | Buffer */
	
	/* Get Sim options */
	num = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_SIM_PARAMS));
	options = (double*)mxGetPr(ssGetSFcnParam(S, ARGC_SIM_PARAMS));
	
	/* Check num of options */
	if(num <1) {
		ssSetErrorStatus(S, "Invalid simulation parameters");
		return;
	}
	configuration = (int)(options[0]);
	
	/* === Packet === */
	if(configuration == 0) {
		int data_size_array[8] = {8, 4, 1, 1, 2, 2, 4, 4};
		uint32_T offset;
		DTypeId typeid;
		int i;
		int datasize;
		int datacount;
		const uint32_T *u;
		uint8_T *y;
		
		/* Output port: Data type must be uint8 */
		y = (uint8_T*)ssGetOutputPortSignal(S, 0);
		u = ssGetInputPortSignal(S, 1);
		
		/* Offset */
		offset = *(const uint32_T*)ssGetInputPortSignal(S, 0);
		memcpy(y, &offset, 4);
		
		/* Data */
		typeid = ssGetInputPortDataType(S, 1);
		datasize = data_size_array[typeid];
		datacount = ssGetInputPortWidth(S, 1);
		memcpy(y+4, u, datacount*datasize);
	}
	/* === Buffer === */
	else {
		int data_size_array[8] = {8, 4, 1, 1, 2, 2, 4, 4};
		uint32_T offset;
		DTypeId typeid;
		int i;
		int input_datasize;
		int input_datacount;
		int output_datasize;
		int output_datacount;
		const uint8_T *u;
		uint8_T *y;		
		uint8_T *raw;
		int raw_size;
		
		/* Offset */
		raw = (uint8_T*)ssGetInputPortSignal(S, 0);
		if(!raw) {
			ssSetErrorStatus(S, "Invalid input.\n");
			return;
		}
		raw_size = ssGetInputPortWidth(S, 0);
		
		if(raw_size <= 4) {
			ssSetErrorStatus(S, "Invalid input size.\n");
			return;
		}
		memcpy(&offset, raw, 4);
		input_datacount = raw_size-4;
				
		/* Output data */
		y = (uint8_T*)ssGetOutputPortSignal(S, 1);
		if(!y) {
			ssSetErrorStatus(S, "Invalid output.\n");
			return;
		}
		typeid = ssGetOutputPortDataType(S, 1);
		output_datasize = data_size_array[typeid];
		output_datacount = ssGetOutputPortWidth(S, 1);
		
		if((offset + input_datacount) <= (output_datacount*output_datasize)) {
			uint32_T *y0;	
			
			y0 = (uint32_T*)ssGetOutputPortSignal(S, 0);
			*y0 = (offset + input_datacount);
			memcpy(&y[offset], &raw[4], input_datacount);
			/* Update offset */
			
		}
		else {
			ssSetErrorStatus(S, "Invalid offset.\n");			
		}		
	}	
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
    /* do nothing */
	//ssPrintf("-mdlTerminate-\n");
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
 * (e.g. must have TLC file waijung_data_stream.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_data_stream.c"
#endif

