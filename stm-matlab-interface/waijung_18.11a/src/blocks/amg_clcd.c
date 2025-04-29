#define S_FUNCTION_NAME  amg_clcd
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_MODULE = 0,
    ARGC_CONFIGURATION,
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    ARGC_CONFSTR,
    
    /* Min required */
    ARGC_SAMPLETIME,
    ARGC_SAMPLETIMESTR,
    ARGC_BLOCKID, 
    
    /* Param count */
    __PARAM_COUNT
};

#define SAMPLETIME(S)    mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR)) /* Compiled sample time (sec) in string */
#define BLOCKID(S)      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID)) /* BlockID */

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
        ssSetInputPortDirectFeedThrough(S, k, 1);
        //ssSetInputPortWidth(S, k, -1);
		if(!ssSetInputPortDimensionInfo(S, k, DYNAMIC_DIMENSION)) return;
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k+1]));
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLETIME(S));
} /* end mdlInitializeSampleTimes */

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo) {
    
    /* Set input port dimension */
    int_T inWidth = ssGetInputPortWidth(S, port);
    
    if (inWidth == DYNAMICALLY_SIZED){
        if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
    }    
} /* end mdlSetInputPortDimensionInfo */

# define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo) {
    int_T outWidth = ssGetOutputPortWidth(S, port);
    
    if (outWidth == DYNAMICALLY_SIZED){
        if(!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
    }
} /* end mdlSetOutputPortDimensionInfo */

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
static void mdlSetDefaultPortDimensionInfo(SimStruct *S) {
    int k;
	int input_count;
    int output_count;
	
	input_count = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[0]);
    output_count = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[0]);
	
    
    for (k = 0; k < input_count; k++){
        if(!ssSetInputPortMatrixDimensions(S, k, 1, 1)) return;
    }
} /* end mdlSetDefaultPortDimensionInfo */
#endif

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
    int NOutputPara;

    char* module;
    char* configuration;
    char* confstr;
    char* sampletimestr;
    char* blockid;
    
    module = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_MODULE));
    configuration = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFIGURATION));
    confstr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFSTR));  
    sampletimestr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    NOutputPara = 5; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "module", module,
            SSWRITE_VALUE_QSTR, "configuration", configuration,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "sampletimestr", sampletimestr,
            SSWRITE_VALUE_QSTR, "blockid", blockid
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    
    if (!ssWriteRTWStrVectParam(S, "confstr", confstr, get_list_count(confstr))){
        return;
	}
    
    mxFree(module);
    mxFree(configuration);
    mxFree(confstr);
    mxFree(sampletimestr);
    mxFree(blockid);   
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file amg_clcd.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function amg_clcd.c"
#endif

