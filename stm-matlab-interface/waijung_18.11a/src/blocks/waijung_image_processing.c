#define S_FUNCTION_NAME  waijung_image_processing
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_CONF = 0,    
    
    ARGC_INPUT_PORTTYPE,
    ARGC_INPUT_PORTWIDTH,
    ARGC_OUTPUT_PORTTYPE,
    ARGC_OUTPUT_PORTWIDTH,
	
	ARGC_OUTPUT_ROW,
	ARGC_OUTPUT_COLUMN,
    
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
		ssSetOutputPortMatrixDimensions(S, k, \
				(int)mxGetScalar(ssGetSFcnParam(S, ARGC_OUTPUT_ROW)), \
				(int)mxGetScalar(ssGetSFcnParam(S, ARGC_OUTPUT_COLUMN)));
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
}
#endif

#define MDL_DISABLE
#if defined(MDL_DISABLE) && defined(MATLAB_MEX_FILE)
static void mdlDisable(SimStruct *S){
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid) {	
	int num;
	double *options;
	
	/* Parameters */	
	int conf;
	int imagesize;
	int inputalignment; /* 0-RGB, 1:BGR */
	int outputalignment; /* 0-RGB, 1:BGR */
	
	num = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_SIM_PARAMS));
	options = (double*)mxGetPr(ssGetSFcnParam(S, ARGC_SIM_PARAMS));	
	
	if (num != 4) {
		ssPrintf("Param count: %d\n", num);
		ssSetErrorStatus(S, "Invalid parameters count!\n");
		return;
	}
	
	conf = (int)options[0];
	imagesize = (int)options[1];
	inputalignment = (int)options[2];
	outputalignment = (int)options[3];
	
	/* ------------------------------------------------------------
	 */
	
	
	
	
	
	
	/*
	 **********************************************************************
	 */
	
	/* === Action for configuration === */
	switch(conf){
		/* === RGB16ToRGB24 === */
		case 0: {
			int_T i, j;			
			int_T outputcount;
			int_T loopcount;
			uint16_T in;
			
			/* Determine loop count */
			loopcount = (int_T)ssGetInputPortWidth(S, 0);
			outputcount = (int_T)ssGetNumOutputPorts(S);
			
			if(outputcount == 3) {
				uint16_T *src = (uint16_T*)ssGetInputPortSignal(S, 0);
				uint8_T  *out0 = (uint8_T*)ssGetOutputPortSignal(S, 0);
				uint8_T  *out1 = (uint8_T*)ssGetOutputPortSignal(S, 1);
				uint8_T  *out2 = (uint8_T*)ssGetOutputPortSignal(S, 2);
				
				for(i=0; i<loopcount; i++) {
					in = src[i]; /* 16 bits pixel */
					if(inputalignment == 0) { /* RGB */
						out0[i] = (((in >> 11) & 0x1F) << 3);
						out1[i] = (((in >> 5) & 0x3F) << 2);
						out2[i] = ((in & 0x1F) << 3);
					}
					else { /* BGR */
						out0[i] = ((in & 0x1F) << 3);
						out1[i] = (((in >> 5) & 0x3F) << 2);
						out2[i] = (((in >> 11) & 0x1F) << 3);
					}
				}
			}
			else {
				ssSetErrorStatus(S, "Invalid output count.\n");
			}
			
			
			/* Input: uint16_t */
			
			/* Output: can be uint8_T or uint8_T(3) */
		}
		break;
				
		default:
			ssSetErrorStatus(S, "Invalid configuration.\n");
			break;
	}
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
    /* do nothing */
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
 * (e.g. must have TLC file waijung_image_processing.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_image_processing.c"
#endif

