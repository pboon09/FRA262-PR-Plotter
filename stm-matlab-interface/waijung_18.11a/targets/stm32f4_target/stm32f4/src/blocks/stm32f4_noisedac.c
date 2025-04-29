#define S_FUNCTION_NAME  stm32f4_noisedac
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 17 /* Total number of block parameters */

#define DAC1ON(S) mxGetScalar(ssGetSFcnParam(S, 0))
#define DAC2ON(S) mxGetScalar(ssGetSFcnParam(S, 1))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 2))
#define BLOCKID(S) ssGetSFcnParam(S, 3)
#define APB(S) ssGetSFcnParam(S, 4)
#define PORTSTR(S) ssGetSFcnParam(S, 5)
#define PINSTR(S) ssGetSFcnParam(S, 6)
#define PINMAT(S) ssGetSFcnParam(S, 7)
#define CHMAT(S) ssGetSFcnParam(S, 8)
#define DACBUFFERSTR(S) ssGetSFcnParam(S, 9)
#define TIMER(S) ssGetSFcnParam(S, 10)
#define TIMARR(S) ssGetSFcnParam(S, 11)
#define TIMPRESCALE(S) ssGetSFcnParam(S, 12)
#define DAC1MASKSELSTR(S) ssGetSFcnParam(S, 13)
#define DAC2MASKSELSTR(S) ssGetSFcnParam(S, 14)
#define DAC1SEED(S) ssGetSFcnParam(S, 15)
#define DAC2SEED(S) ssGetSFcnParam(S, 16)

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
    int NOutputPara = 16;
    const char * apb = mxArrayToString(APB(S));
    const char * blockid = mxArrayToString(BLOCKID(S));
    const char * portstr = mxArrayToString(PORTSTR(S));
    const char * pinstr = mxArrayToString(PINSTR(S));
    const char * dacbufferstr = mxArrayToString(DACBUFFERSTR(S));
    const char * dac1maskselstr = mxArrayToString(DAC1MASKSELSTR(S));
    const char * dac2maskselstr = mxArrayToString(DAC2MASKSELSTR(S));
    const char * dac1seed = mxArrayToString(DAC1SEED(S));
    const char * dac2seed = mxArrayToString(DAC2SEED(S));
    const char * timer = mxArrayToString(TIMER(S));
    const char * timarr = mxArrayToString(TIMARR(S));
    const char * timprescale = mxArrayToString(TIMPRESCALE(S));
    const real_T * pinmat = (real_T *) mxGetPr(PINMAT(S));
    int portstr_cnt = 1, maxpin_cnt;
    char *p;
    
    // count number of port
    p = strchr(portstr, ',');
    while (p != NULL) {
        p = strchr(p + 1, ',');
        portstr_cnt++;
    }
    
    maxpin_cnt = (int) mxGetN(PINMAT(S));
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_NUM, "dac1on", DAC1ON(S),
            SSWRITE_VALUE_NUM, "dac2on", DAC2ON(S),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid,
            SSWRITE_VALUE_QSTR, "apb", apb,
            SSWRITE_VALUE_VECT_STR, "portstr", portstr, portstr_cnt,
            SSWRITE_VALUE_VECT_STR, "pinstr", pinstr, portstr_cnt,
            SSWRITE_VALUE_2DMAT, "pinmat", pinmat, portstr_cnt, maxpin_cnt,
            SSWRITE_VALUE_QSTR, "dacbufferstr", dacbufferstr,
            SSWRITE_VALUE_QSTR, "dac1maskselstr", dac1maskselstr,
            SSWRITE_VALUE_QSTR, "dac2maskselstr", dac2maskselstr,
            SSWRITE_VALUE_QSTR, "dac1seed", dac1seed,
            SSWRITE_VALUE_QSTR, "dac2seed", dac2seed,
            SSWRITE_VALUE_QSTR, "timer", timer,
            SSWRITE_VALUE_QSTR, "timarr", timarr,
            SSWRITE_VALUE_QSTR, "timprescale", timprescale
            )) {
        return;
    }
    mxFree(apb);
    mxFree(blockid);
    mxFree(portstr);
    mxFree(pinstr);
    mxFree(dacbufferstr);
    mxFree(dac1maskselstr);
    mxFree(dac2maskselstr);
    mxFree(dac1seed);
    mxFree(dac2seed);
    mxFree(timer);
    mxFree(timarr);
    mxFree(timprescale);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_dac.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_noisedac.c"
#endif

