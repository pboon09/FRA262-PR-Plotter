#define S_FUNCTION_NAME  waijung_stringbuffer_processing
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_CONFIGURATION = 0,
    ARGC_VARNAME,
    ARGC_FORMAT,
    ARGC_DATATYPES,
    ARGC_DATACOUNT,
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    
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
    bool status_ok;
    int NOutputPara;
                    
    char* configuration;
    char* varname;
    char* format;
    char* datatypes;
    char* blockid;
    
    /* Assume no error status */
    status_ok = true;
    
    /* Collect string */
    configuration = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFIGURATION));
    varname = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_VARNAME));
    format = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FORMAT));    
    datatypes = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_DATATYPES));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    NOutputPara = 6; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "configuration", configuration,
            SSWRITE_VALUE_QSTR, "varname", varname,
            SSWRITE_VALUE_QSTR, "format", format,
            SSWRITE_VALUE_NUM, "datacount", mxGetScalar(ssGetSFcnParam(S, ARGC_DATACOUNT)),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid
            )) {
        /* De-activate status */
        status_ok = false;
    }

    /* Write port type list */
    if (status_ok && !ssWriteRTWStrVectParam(S, "datatypes", datatypes, (int)mxGetScalar(ssGetSFcnParam(S, ARGC_DATACOUNT))))
        status_ok = false;    
        
    /* Free memory */
    mxFree(configuration);
    mxFree(varname);
    mxFree(format);
    mxFree(datatypes);
    mxFree(blockid);   
    
    if (!status_ok) {
        ssSetErrorStatus(S, "\nFailed to write RTW parameters.\n");
        return; /* An error occurred which will be reported by SL */
    }
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_stringbuffer_processing.c"
#endif

