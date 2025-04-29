#define S_FUNCTION_NAME  stm32f4_basicpwm
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 17 /* Total number of block parameters */

#define TIMER(S) (char*)mxArrayToString(ssGetSFcnParam(S, 0)) /* Timer */
#define PWMPERIOD(S) mxGetScalar(ssGetSFcnParam(S, 1)) /* PWM Period (seconds) */
#define CH1PIN(S) (char*)mxArrayToString(ssGetSFcnParam(S, 2)) /* Channel 1 */
#define CH2PIN(S) (char*)mxArrayToString(ssGetSFcnParam(S, 3)) /* Channel 2 */
#define CH3PIN(S) (char*)mxArrayToString(ssGetSFcnParam(S, 4)) /* Channel 3 */
#define CH4PIN(S) (char*)mxArrayToString(ssGetSFcnParam(S, 5)) /* Channel 4 */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 6)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, 7)) /* Compiled sample time (sec) in string */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 8)) /* BlockID */
#define APB(S) (char*)mxArrayToString(ssGetSFcnParam(S, 9)) /* APB */
#define PORTSTR(S) ssGetSFcnParam(S, 10) /* PORTSTR */
#define PINSTR(S) ssGetSFcnParam(S, 11) /* PINSTR */
#define PINMAT(S) ssGetSFcnParam(S, 12) /* PINMAT */
#define TIMARR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 13)) /* TIMARR */
#define TIMPRESCALE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 14)) /* TIMPRESCALE */
#define CHMAT(S) ssGetSFcnParam(S, 15) /* CHMAT */
#define POLARITYSTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 16)) /* POLARITYSTR */


static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int NPorts = 4, portCnt;
    
    ssSetNumSFcnParams(S, NPAR);
    
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* Configure Input Port */
    if (!strcmp(CH1PIN(S), "Not available - Do not use")){
        NPorts--;
    }
    if (!strcmp(CH2PIN(S), "Not available - Do not use")){
        NPorts--;
    }
    if (!strcmp(CH3PIN(S), "Not available - Do not use")){
        NPorts--;
    }
    if (!strcmp(CH4PIN(S), "Not available - Do not use")){
        NPorts--;
    }
    
    if (!ssSetNumInputPorts(S, NPorts)) return; /* Number of input ports */
    
    for (portCnt = 0; portCnt < NPorts; portCnt++) {
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, SS_DOUBLE);
        ssSetInputPortWidth(S, portCnt, 1);
    }
    
    /* Configure Output Port */
    NPorts = 0;
    if (!ssSetNumOutputPorts(S, NPorts)) return; /* Number of output ports */
    
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
    
    // count number of port
    p = strchr(portstr, ',');
    while (p != NULL) {
        p = strchr(p + 1, ',');
        portstr_cnt++;
    }
    
    maxpin_cnt = (int) mxGetN(PINMAT(S));
    maxch_cnt = (int) mxGetN(CHMAT(S));
    
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "timer", TIMER(S),
            SSWRITE_VALUE_NUM, "pwmperiod", PWMPERIOD(S),
            SSWRITE_VALUE_QSTR, "timarr", TIMARR(S),
            SSWRITE_VALUE_QSTR, "timprescale", TIMPRESCALE(S),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
            SSWRITE_VALUE_QSTR, "apb", APB(S),
            SSWRITE_VALUE_VECT_STR, "portstr", portstr, portstr_cnt,
            SSWRITE_VALUE_VECT_STR, "pinstr", pinstr, portstr_cnt,
            SSWRITE_VALUE_2DMAT, "pinmat", pinmat, portstr_cnt, maxpin_cnt,
            SSWRITE_VALUE_2DMAT, "chmat", chmat, 1, maxch_cnt,
            SSWRITE_VALUE_QSTR, "polaritystr", POLARITYSTR(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    mxFree(portstr);
    mxFree(pinstr);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_basicpwm.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_basicpwm.c"
#endif

