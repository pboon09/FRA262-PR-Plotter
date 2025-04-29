#define S_FUNCTION_NAME  stm32f4_regular_adc
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 30 /* Total number of block parameters */

#define ADCMODULE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 0)) /* ADC Module */
#define OUTPUTDATATYPE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* Output Data Type */
#define PRESCALERSTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 2)) /* ADC Prescaler String */
#define CHSAMPLINGTIMESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 3)) /* Channel Sampling Time Selection (cycles) String */
#define READ_ANx(S, i) mxGetScalar(ssGetSFcnParam(S, 4+i)) /* Read ANx */
#define READ_AN0(S) mxGetScalar(ssGetSFcnParam(S, 4)) /* Read AN0 */
#define READ_AN1(S) mxGetScalar(ssGetSFcnParam(S, 5)) /* Read AN1 */
#define READ_AN2(S) mxGetScalar(ssGetSFcnParam(S, 6)) /* Read AN2 */
#define READ_AN3(S) mxGetScalar(ssGetSFcnParam(S, 7)) /* Read AN3 */
#define READ_AN4(S) mxGetScalar(ssGetSFcnParam(S, 8)) /* Read AN4 */
#define READ_AN5(S) mxGetScalar(ssGetSFcnParam(S, 9)) /* Read AN5 */
#define READ_AN6(S) mxGetScalar(ssGetSFcnParam(S, 10)) /* Read AN6 */
#define READ_AN7(S) mxGetScalar(ssGetSFcnParam(S, 11)) /* Read AN7 */
#define READ_AN8(S) mxGetScalar(ssGetSFcnParam(S, 12)) /* Read AN8 */
#define READ_AN9(S) mxGetScalar(ssGetSFcnParam(S, 13)) /* Read AN9 */
#define READ_AN10(S) mxGetScalar(ssGetSFcnParam(S, 14)) /* Read AN10 */
#define READ_AN11(S) mxGetScalar(ssGetSFcnParam(S, 15)) /* Read AN11 */
#define READ_AN12(S) mxGetScalar(ssGetSFcnParam(S, 16)) /* Read AN12 */
#define READ_AN13(S) mxGetScalar(ssGetSFcnParam(S, 17)) /* Read AN13 */
#define READ_AN14(S) mxGetScalar(ssGetSFcnParam(S, 18)) /* Read AN14 */
#define READ_AN15(S) mxGetScalar(ssGetSFcnParam(S, 19)) /* Read AN15 */
#define READ_AN16(S) mxGetScalar(ssGetSFcnParam(S, 20)) /* Read Temperature Sensor (Internal Pin) */
#define READ_AN17(S) mxGetScalar(ssGetSFcnParam(S, 21)) /* Read VREFINT (Internal Pin) */
#define READ_AN18(S) mxGetScalar(ssGetSFcnParam(S, 22)) /* Read VBAT (Internal Pin) */
#define PINSTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 23)) /* Pin string */
#define USEDPINARRAY(S) ssGetSFcnParam(S, 24) /* Used pin array */
#define USEDPORTIDARRAY(S) ssGetSFcnParam(S, 25) /* Used port id array */
#define USEDPINIDARRAY(S) ssGetSFcnParam(S, 26) /* Used pin id array */
#define USEDPINIDCOUNT(S) mxGetScalar(ssGetSFcnParam(S, 27)) /* Used pin count */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 28)) /* Sample time (sec) */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 29)) /* BlockID */

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
    for(k=0; k< 19; k++) {
        if (READ_ANx(S, k))
            NPorts++;
    }
    
    if (!ssSetNumOutputPorts(S, NPorts)) return; /* Number of output ports */
    
    portCnt = 0;
    for(k=0; k< 19; k++) {
        if (READ_ANx(S, k)) {
            if(strcmp(OUTPUTDATATYPE(S), "Double")==0)
                ssSetOutputPortDataType(S, portCnt, SS_DOUBLE);
            else if(strcmp(OUTPUTDATATYPE(S), "Single") == 0)
                ssSetOutputPortDataType(S, portCnt, SS_SINGLE);
            else
                ssSetOutputPortDataType(S, portCnt, SS_UINT16);
            ssSetOutputPortWidth(S, portCnt, 1);
            portCnt++;
        }
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
    int NOutputPara = NPAR; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "adcmodule", ADCMODULE(S),
            SSWRITE_VALUE_QSTR, "outputdatatype", OUTPUTDATATYPE(S),
            SSWRITE_VALUE_QSTR, "prescalerstr", PRESCALERSTR(S),
            SSWRITE_VALUE_QSTR, "chsamplingtimestr", CHSAMPLINGTIMESTR(S),
            SSWRITE_VALUE_NUM, "read_an0", READ_AN0(S),
            SSWRITE_VALUE_NUM, "read_an1", READ_AN1(S),
            SSWRITE_VALUE_NUM, "read_an2", READ_AN2(S),
            SSWRITE_VALUE_NUM, "read_an3", READ_AN3(S),
            SSWRITE_VALUE_NUM, "read_an4", READ_AN4(S),
            SSWRITE_VALUE_NUM, "read_an5", READ_AN5(S),
            SSWRITE_VALUE_NUM, "read_an6", READ_AN6(S),
            SSWRITE_VALUE_NUM, "read_an7", READ_AN7(S),
            SSWRITE_VALUE_NUM, "read_an8", READ_AN8(S),
            SSWRITE_VALUE_NUM, "read_an9", READ_AN9(S),
            SSWRITE_VALUE_NUM, "read_an10", READ_AN10(S),
            SSWRITE_VALUE_NUM, "read_an11", READ_AN11(S),
            SSWRITE_VALUE_NUM, "read_an12", READ_AN12(S),
            SSWRITE_VALUE_NUM, "read_an13", READ_AN13(S),
            SSWRITE_VALUE_NUM, "read_an14", READ_AN14(S),
            SSWRITE_VALUE_NUM, "read_an15", READ_AN15(S),
            SSWRITE_VALUE_NUM, "read_an16", READ_AN16(S),
            SSWRITE_VALUE_NUM, "read_an17", READ_AN17(S),
            SSWRITE_VALUE_NUM, "read_an18", READ_AN18(S),
            SSWRITE_VALUE_QSTR, "pinstr", PINSTR(S),
            SSWRITE_VALUE_VECT, "usedpinarray", mxGetData(USEDPINARRAY(S)), mxGetNumberOfElements(USEDPINARRAY(S)) ,
            SSWRITE_VALUE_VECT, "usedportidarray", mxGetData(USEDPORTIDARRAY(S)), mxGetNumberOfElements(USEDPORTIDARRAY(S)) ,
            SSWRITE_VALUE_VECT, "usedpinidarray", mxGetData(USEDPINIDARRAY(S)), mxGetNumberOfElements(USEDPINIDARRAY(S)) ,
            SSWRITE_VALUE_NUM, "usedpinidcount", USEDPINIDCOUNT(S),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_regular_adc.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_regular_adc.c"
#endif

