#define S_FUNCTION_NAME  stm32f0_encoderread
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 14 /* Total number of block parameters */

#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 0))
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, 1))
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 2))
#define TIMER(S) (char*)mxArrayToString(ssGetSFcnParam(S, 3))
#define APB(S) (char*)mxArrayToString(ssGetSFcnParam(S, 4))
#define PORTSTR(S) ssGetSFcnParam(S, 5)
#define PINSTR(S) ssGetSFcnParam(S, 6)
#define PINMAT(S) ssGetSFcnParam(S, 7)
#define CHMAT(S) ssGetSFcnParam(S, 8)
#define FILTERSTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 9))
#define FILTER(S) mxGetScalar(ssGetSFcnParam(S, 10))
#define FACTOR(S) mxGetScalar(ssGetSFcnParam(S, 11))
#define RSTCNT(S) mxGetScalar(ssGetSFcnParam(S, 12))
#define PPR(S) mxGetScalar(ssGetSFcnParam(S, 13))

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int NPorts, portCnt;
    
    ssSetNumSFcnParams(S, NPAR);
    
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* Configure Input Port */
    NPorts = 0;
    if (!ssSetNumInputPorts(S, NPorts)) return; /* Number of input ports */
    
    /* Configure Output Port */
    NPorts = 2;
    if (!ssSetNumOutputPorts(S, NPorts)) return; /* Number of output ports */
    for (portCnt = 0; portCnt < NPorts; portCnt++) {
        ssSetOutputPortDataType(S, portCnt, SS_UINT32);
        ssSetOutputPortWidth(S, portCnt, 1);
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
    int NOutputPara = 13; /* Number of parameters to output to model.rtw */
    const char * portstr = mxArrayToString(PORTSTR(S));
    const char * pinstr = mxArrayToString(PINSTR(S));
    const real_T * pinmat = (real_T *) mxGetData(PINMAT(S));
    const real_T * chmat = (real_T *) mxGetData(CHMAT(S));
    int portstr_cnt = 1, maxpin_cnt, maxch_cnt;
    char *p;
	real_T period;
    
    // count number of port
    p = strchr(portstr, ',');
    while (p != NULL) {
        p = strchr(p + 1, ',');
        portstr_cnt++;
    }
    maxpin_cnt = (int) mxGetN(PINMAT(S));
    maxch_cnt = (int) mxGetN(CHMAT(S));
    
	period = FACTOR(S)*PPR(S)-1; // (factor * Pulse per Period) - 1

    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
            SSWRITE_VALUE_QSTR, "timer", TIMER(S),
            SSWRITE_VALUE_QSTR, "apb", APB(S),
            SSWRITE_VALUE_VECT_STR, "portstr", portstr, portstr_cnt,
            SSWRITE_VALUE_VECT_STR, "pinstr", pinstr, portstr_cnt,
            SSWRITE_VALUE_2DMAT, "pinmat", pinmat, portstr_cnt, maxpin_cnt,
            SSWRITE_VALUE_2DMAT, "chmat", chmat, 1, maxch_cnt,
            SSWRITE_VALUE_QSTR, "filterstr", FILTERSTR(S),
            SSWRITE_VALUE_NUM, "filter", FILTER(S),
            SSWRITE_VALUE_NUM, "period", period,
            SSWRITE_VALUE_NUM, "rstcnt", RSTCNT(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    mxFree(portstr);
    mxFree(pinstr);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f0_encoderread.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f0_encoderread.c"
#endif

