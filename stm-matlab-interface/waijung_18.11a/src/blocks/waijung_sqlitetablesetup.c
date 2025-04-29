#define S_FUNCTION_NAME  waijung_sqlitetablesetup
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_PORT = 0,
    ARGC_FILENAME,
    ARGC_TABLENAME,    
    ARGC_COLUMNS,
    
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
    /* Parameter validatone */    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, 0)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, 0)) return; /* Number of output ports */
    
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
    char* port;
    char* filename;
    char* tablename;
    char* columns;
    char* blockid;
    
    /* Collect string */
    port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));
    filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
    tablename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TABLENAME));    
    columns = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_COLUMNS));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    NOutputPara = 7; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "port", port,
            SSWRITE_VALUE_QSTR, "filename", filename,
            SSWRITE_VALUE_QSTR, "tablename", tablename,
            SSWRITE_VALUE_QSTR, "columns", columns,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    
    mxFree(port);
    mxFree(filename);
    mxFree(tablename);
    mxFree(columns);
    mxFree(blockid);   
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_sqlitetablesetup.c"
#endif

