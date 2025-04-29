#define S_FUNCTION_NAME  waijung_webserver_vdata_mapping
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_CALLABCK_ENABLE,
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define ARGC_CALLABCK_ENABLE(S) (bool)mxGetScalar(ssGetSFcnParam(S, ARGC_CALLABCK_ENABLE))
#define SAMPLETIME(S)    mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */
#define BLOCKID(S)      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID)) /* BlockID */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int priority = 0;
    int input_count;
    int output_count;
    
    /* Parameter validatone */    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
        
    if(!ARGC_CALLABCK_ENABLE(S)) {
        
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
    }
    else {
        
        /* Configure Input Port */
        if (!ssSetNumInputPorts(S, 0)) return; /* Number of input ports */
        
        if (!ssSetNumOutputPorts(S, 1)) return; /* Number of output ports */
        /* Port: INT */
        ssSetOutputPortWidth(S, 0, 1);
        ssSetOutputPortDataType(S, 0, SS_UINT32);
        
        ssSetNumSampleTimes(S, 1);
        
        /* options */
        ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                SS_OPTION_ASYNCHRONOUS_INTERRUPT));
        
        /* Set up asynchronous timer attributes */
        ssSetTimeSource(S, SS_TIMESOURCE_BASERATE);
        
        /* Set up asynchronous task priority */
        ssSetAsyncTaskPriorities(S, 1, &priority);
    }
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLETIME(S));
    
    if(ARGC_CALLABCK_ENABLE(S)) {
        ssSetOffsetTime(S, 0, 0.0);
    
        /* Used for set port to a function call output */
        ssSetCallSystemOutput(S, 0);  /* call on first element */
    }
    
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
    char* blockid;
    
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));    
    NOutputPara = 3; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,            
            SSWRITE_VALUE_NUM, "callbackenable", mxGetScalar(ssGetSFcnParam(S, ARGC_CALLABCK_ENABLE)),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    
    mxFree(blockid);   
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_webserver_vdata_mapping.c"
#endif

