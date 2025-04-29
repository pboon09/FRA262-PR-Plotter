#define S_FUNCTION_NAME  stm32f4_can
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_CONF = 0,    
    ARGC_MODULE,
    ARGC_ISR_ENABLE,
    
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    ARGC_INPUTLABEL_ARRAY,
    ARGC_OUTPUTLABEL_ARRAY,
    
    ARGC_CONFSTR,
    
    ARGC_SAMPLETIME,
    ARGC_SAMPLETIMESTR,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define ENABLE_ISR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_ISR_ENABLE))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int input_count;
    int output_count;
    int priority = 0;
    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    if(ENABLE_ISR(S) == 0) {
        /* Port count */
        input_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_ARRAY));
        output_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY));
        
        /* Configure Input Port */
        if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
        
        /* Configure Output Port */
        if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
        
        /* Port */
        for(k=0; k<output_count; k++) {
            ssSetOutputPortWidth(S, k, 1);
            ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[k]));
        }
        
        for(k=0; k<input_count; k++) {
            ssSetInputPortDirectFeedThrough(S, k, 1);
            ssSetInputPortWidth(S, k, 1);
            ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k]));
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
    if(ENABLE_ISR(S) != 0) {
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

/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
//#define MDL_START
//static void mdlStart(SimStruct *S) {
//}

static void mdlOutputs(SimStruct *S, int_T tid) {

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
    int NOutputPara = 6; /* Number of parameters to output to model.rtw */
    
    char *conf; // ARCG_CONF
    char *module; //ARGC_MODULE
    char* confstr;                
    char *sampletimestr;
    char *blockid;
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
    module = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_MODULE));    
    confstr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFSTR));    
    sampletimestr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "conf", conf,
            SSWRITE_VALUE_QSTR, "canmodule", module,
            SSWRITE_VALUE_NUM, "enableisr", ENABLE_ISR(S),            
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "sampletimestr", sampletimestr,
            SSWRITE_VALUE_QSTR, "blockid", blockid            
            )) {
        return; /* An error occurred which will be reported by SL */
    }

    /* Write configuration string */
    if (!ssWriteRTWStrVectParam(S, "confstr", confstr, get_list_count(confstr))){
        return;
    }    
        
    mxFree(conf);
    mxFree(module);
    mxFree(confstr);    
    mxFree(sampletimestr);
    mxFree(blockid);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_can.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_can.c"
#endif

