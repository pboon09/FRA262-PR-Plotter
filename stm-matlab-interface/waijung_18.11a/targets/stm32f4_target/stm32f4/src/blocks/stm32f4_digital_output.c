#define S_FUNCTION_NAME  stm32f4_digital_output
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 28 /* Total number of block parameters */

#define USE_BITBAND(S) mxGetScalar(ssGetSFcnParam(S, 0)) /* Use Bit band */
#define PORT(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* Port */
#define SPEED(S) (char*)mxArrayToString(ssGetSFcnParam(S, 2)) /* Speed (MHz) */
#define PORTTYPE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 3)) /* Type (Push-Pull/Open-Drain) */
#define PORTTYPESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 4)) /* Type (Push-Pull/Open-Drain) String */
#define USE_PIN0(S) mxGetScalar(ssGetSFcnParam(S, 5)) /* Use Pin 0 */
#define USE_PIN1(S) mxGetScalar(ssGetSFcnParam(S, 6)) /* Use Pin 1 */
#define USE_PIN2(S) mxGetScalar(ssGetSFcnParam(S, 7)) /* Use Pin 2 */
#define USE_PIN3(S) mxGetScalar(ssGetSFcnParam(S, 8)) /* Use Pin 3 */
#define USE_PIN4(S) mxGetScalar(ssGetSFcnParam(S, 9)) /* Use Pin 4 */
#define USE_PIN5(S) mxGetScalar(ssGetSFcnParam(S, 10)) /* Use Pin 5 */
#define USE_PIN6(S) mxGetScalar(ssGetSFcnParam(S, 11)) /* Use Pin 6 */
#define USE_PIN7(S) mxGetScalar(ssGetSFcnParam(S, 12)) /* Use Pin 7 */
#define USE_PIN8(S) mxGetScalar(ssGetSFcnParam(S, 13)) /* Use Pin 8 */
#define USE_PIN9(S) mxGetScalar(ssGetSFcnParam(S, 14)) /* Use Pin 9 */
#define USE_PIN10(S) mxGetScalar(ssGetSFcnParam(S, 15)) /* Use Pin 10 */
#define USE_PIN11(S) mxGetScalar(ssGetSFcnParam(S, 16)) /* Use Pin 11 */
#define USE_PIN12(S) mxGetScalar(ssGetSFcnParam(S, 17)) /* Use Pin 12 */
#define USE_PIN13(S) mxGetScalar(ssGetSFcnParam(S, 18)) /* Use Pin 13 */
#define USE_PIN14(S) mxGetScalar(ssGetSFcnParam(S, 19)) /* Use Pin 14 */
#define USE_PIN15(S) mxGetScalar(ssGetSFcnParam(S, 20)) /* Use Pin 15 */
#define PINSTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 21)) /* Pin string */
#define USEDPINARRAY(S) ssGetSFcnParam(S, 22) /* Used pin array */
#define BITBANDINGSTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 23)) /* Bit banding string */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 24)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, 25)) /* Compiled sample time (sec) in string */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 26)) /* BlockID */
#define USE_GLOBALINIT(S) mxGetScalar(ssGetSFcnParam(S, 27)) /* Use Pin 15 */

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
    NPorts = 0+ USE_PIN0(S) + USE_PIN1(S) + USE_PIN2(S) + USE_PIN3(S) + USE_PIN4(S) + USE_PIN5(S) + USE_PIN6(S) + USE_PIN7(S) + USE_PIN8(S) + USE_PIN9(S) + USE_PIN10(S) + USE_PIN11(S) + USE_PIN12(S) + USE_PIN13(S) + USE_PIN14(S) + USE_PIN15(S);
    if (!ssSetNumInputPorts(S, NPorts)) return; /* Number of input ports */
    portCnt = 0;
    /* Port: P0 */
    if (USE_PIN0(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P1 */
    if (USE_PIN1(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P2 */
    if (USE_PIN2(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P3 */
    if (USE_PIN3(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P4 */
    if (USE_PIN4(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P5 */
    if (USE_PIN5(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P6 */
    if (USE_PIN6(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P7 */
    if (USE_PIN7(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P8 */
    if (USE_PIN8(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P9 */
    if (USE_PIN9(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P10 */
    if (USE_PIN10(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P11 */
    if (USE_PIN11(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P12 */
    if (USE_PIN12(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P13 */
    if (USE_PIN13(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P14 */
    if (USE_PIN14(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
    }
    /* Port: P15 */
    if (USE_PIN15(S)){
        ssSetInputPortDirectFeedThrough(S, portCnt, 1);
        ssSetInputPortDataType(S, portCnt, DYNAMICALLY_TYPED);
        ssSetInputPortWidth(S, portCnt, 1);
        portCnt++;
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
    int NOutputPara = 28; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            
            SSWRITE_VALUE_NUM, "use_bitband", USE_BITBAND(S),
            SSWRITE_VALUE_QSTR, "port", PORT(S),
            SSWRITE_VALUE_QSTR, "speed", SPEED(S),
            SSWRITE_VALUE_QSTR, "porttype", PORTTYPE(S),
            SSWRITE_VALUE_QSTR, "porttypestr", PORTTYPESTR(S),
            SSWRITE_VALUE_NUM, "use_pin0", USE_PIN0(S),
            SSWRITE_VALUE_NUM, "use_pin1", USE_PIN1(S),
            SSWRITE_VALUE_NUM, "use_pin2", USE_PIN2(S),
            SSWRITE_VALUE_NUM, "use_pin3", USE_PIN3(S),
            SSWRITE_VALUE_NUM, "use_pin4", USE_PIN4(S),
            SSWRITE_VALUE_NUM, "use_pin5", USE_PIN5(S),
            SSWRITE_VALUE_NUM, "use_pin6", USE_PIN6(S),
            SSWRITE_VALUE_NUM, "use_pin7", USE_PIN7(S),
            SSWRITE_VALUE_NUM, "use_pin8", USE_PIN8(S),
            SSWRITE_VALUE_NUM, "use_pin9", USE_PIN9(S),
            SSWRITE_VALUE_NUM, "use_pin10", USE_PIN10(S),
            SSWRITE_VALUE_NUM, "use_pin11", USE_PIN11(S),
            SSWRITE_VALUE_NUM, "use_pin12", USE_PIN12(S),
            SSWRITE_VALUE_NUM, "use_pin13", USE_PIN13(S),
            SSWRITE_VALUE_NUM, "use_pin14", USE_PIN14(S),
            SSWRITE_VALUE_NUM, "use_pin15", USE_PIN15(S),
            SSWRITE_VALUE_QSTR, "pinstr", PINSTR(S),
            SSWRITE_VALUE_VECT, "usedpinarray", mxGetData(USEDPINARRAY(S)), mxGetNumberOfElements(USEDPINARRAY(S)) ,
            SSWRITE_VALUE_QSTR, "bitbandingstr", BITBANDINGSTR(S),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_NUM, "use_globalinit", USE_GLOBALINIT(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_digital_output.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_digital_output.c"
#endif

