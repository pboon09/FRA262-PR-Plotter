#define S_FUNCTION_NAME  stm32f4_usart
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_CONFIGURATION = 0,
    ARGC_MODULE,
    ARGC_TRANSFER,
    ARGC_BAUDRATE,
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    ARGC_CONFSTR,
    
    ARGC_BINHEADER, /* Vector */
    ARGC_BINTERMINATOR, /* Vector */
    ARGC_ASCIIHEADER, /* String */
    ARGC_ASCIITERMINATOR, /* Vector */
    
    ARGC_BINDATALENGTH,
    ARGC_PACKETMODE,
    ARGC_VARNAME,
    
    ARGC_SAMPLETIME,
    ARGC_SAMPLETIMESTR,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define SAMPLETIME(S)    mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR)) /* Compiled sample time (sec) in string */
#define BLOCKID(S)      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID)) /* BlockID */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    
	unsigned int tmp, tmp_width;
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
   
    /* Output Port */
    for(k=0; k<output_count; k++) {
		tmp = (unsigned int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[k+1]);
		tmp_width = (tmp >> 8);
		if (tmp_width > 2048)
			tmp_width = 2048;
		if (tmp_width < 1)
			tmp_width = 1;
		ssSetOutputPortWidth(S, k, tmp_width);
		ssSetOutputPortDataType(S, k, (tmp & 0xF));
    }

	/* Input Port */
    for(k=0; k<input_count; k++) {
		tmp = (unsigned int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k+1]);
		tmp_width = (tmp >> 8);
		if (tmp_width > 2048)
			tmp_width = 2048;
		if (tmp_width < 1)
			tmp_width = 1;
        ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortWidth(S, k, tmp_width);
        ssSetInputPortDataType(S, k, (tmp & 0xF));
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
    /* do nothing */
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
    /* do nothing */
} /* end mdlTerminate */

#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
    int NOutputPara;

    char* configuration;
    char* module;
    char* transfer;
    char* confstr;
    
    char* asciiheader;
    char* packetmode;
    char* varname;
    
    char* sampletimestr;
    char* blockid;
    
    configuration = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFIGURATION));
    module = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_MODULE));
    transfer = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TRANSFER));  
    confstr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFSTR));
    asciiheader = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIHEADER));
    
    packetmode = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PACKETMODE));
    varname = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_VARNAME));
    
    sampletimestr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    NOutputPara = 15; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "configuration", configuration,
            SSWRITE_VALUE_QSTR, "module", module,
            SSWRITE_VALUE_QSTR, "transfer", transfer,
            SSWRITE_VALUE_VECT, "inputtypearray", mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)), mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)),
            SSWRITE_VALUE_VECT, "outputtypearray", mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)), mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)),
            SSWRITE_VALUE_VECT, "binheader", mxGetPr(ssGetSFcnParam(S, ARGC_BINHEADER)), mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_BINHEADER)),
            SSWRITE_VALUE_VECT, "binterminator", mxGetPr(ssGetSFcnParam(S, ARGC_BINTERMINATOR)), mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_BINTERMINATOR)),
            SSWRITE_VALUE_VECT, "asciiterminator", mxGetPr(ssGetSFcnParam(S, ARGC_ASCIITERMINATOR)), mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_ASCIITERMINATOR)),
            SSWRITE_VALUE_QSTR, "asciiheader", asciiheader,
            SSWRITE_VALUE_NUM, "bindatalength", mxGetScalar(ssGetSFcnParam(S, ARGC_BINDATALENGTH)),
            SSWRITE_VALUE_QSTR, "packetmode", packetmode,
            SSWRITE_VALUE_QSTR, "varname", varname,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "sampletimestr", sampletimestr,
            SSWRITE_VALUE_QSTR, "blockid", blockid
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    
    if (!ssWriteRTWStrVectParam(S, "typestr", "[\"double\", \"float\", \"int8_t\", \"uint8_t\", \"int16_t\", \"uint16_t\", \"int32_t\", \"uint32_t\"]", 8)) {
        return;
    }
    
    if (!ssWriteRTWStrVectParam(S, "confstr", confstr, 29)){
        return;
    }    
    
    mxFree(configuration);
    mxFree(module);
    mxFree(transfer); 
    mxFree(confstr);
    mxFree(asciiheader);
    mxFree(packetmode);
    mxFree(varname);
    mxFree(sampletimestr);
    mxFree(blockid);   
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_usart.c"
#endif

