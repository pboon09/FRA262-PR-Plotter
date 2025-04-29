#define S_FUNCTION_NAME  waijung_sqlite_setup
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_PORT,
    ARGC_TIMEOUT,
    ARGC_BUFFER_SIZE,
    
    ARGC_RESET_CNTRL,
    ARGC_RESET_PORT,
    ARGC_RESET_PIN,
    
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define TYPEID_COUNT(S)  mxGetScalar(ssGetSFcnParam(S, ARGC_TYPEID_COUNT))
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
        ssSetInputPortWidth(S, k, 1);
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k+1]));
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
    char *blockid;
    char *buffersize;
    char *resetcntrl;
    char *resetport;
    char *resetpin;
    
    /* Collect string */
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    buffersize = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BUFFER_SIZE));
    resetcntrl = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_RESET_CNTRL));
    resetport = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_RESET_PORT));
    resetpin = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_RESET_PIN));
    
    NOutputPara = 8; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_NUM, "port", mxGetScalar(ssGetSFcnParam(S, ARGC_PORT)),
            SSWRITE_VALUE_NUM, "timeout", mxGetScalar(ssGetSFcnParam(S, ARGC_TIMEOUT)),
            SSWRITE_VALUE_QSTR, "buffersize", buffersize,
            SSWRITE_VALUE_QSTR, "resetcntrl", resetcntrl,
            SSWRITE_VALUE_QSTR, "resetport", resetport,
            SSWRITE_VALUE_QSTR, "resetpin", resetpin,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }              
        
    mxFree(blockid);
    mxFree(buffersize);
    mxFree(resetcntrl);
    mxFree(resetport);
    mxFree(resetpin);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_sqlite_setup.c"
#endif

