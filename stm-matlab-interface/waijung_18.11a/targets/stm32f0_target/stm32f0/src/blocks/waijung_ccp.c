#define S_FUNCTION_NAME  waijung_ccp
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_CONF = 0,    
    
    ARGC_INPUT_PORTTYPE,
    ARGC_INPUT_PORTWIDTH,
    ARGC_OUTPUT_PORTTYPE,
    ARGC_OUTPUT_PORTWIDTH,
    
    ARGC_OPTIONSTRING,
    
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define ENABLE_ISR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_ISR_ENABLE))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int input_count;
    int output_count;
    int input_width_count;
    int output_width_count;
	int width;
    
    int priority = 0;
    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* Port count */
    input_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE));
    output_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE));
    input_width_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH));
    output_width_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_PORTWIDTH));
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
    
    /* Port */
    for(k=0; k<output_count; k++) {		
        if(k<output_width_count) {
			width = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTWIDTH)))[k]);
            ssSetOutputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetOutputPortWidth(S, k, 1);
		}
        ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE)))[k]));
    }
    
    for(k=0; k<input_count; k++) {		
        ssSetInputPortDirectFeedThrough(S, k, 1);
        if(k<input_width_count) {
			width = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH)))[k]);
            ssSetInputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetInputPortWidth(S, k, 1);
		}
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE)))[k]));
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
    int NOutputPara = 3; /* Number of parameters to output to model.rtw */
    
    char *conf; // ARCG_CONF
    char* optionstring;
    char *sampletimestr;
    char *blockid;
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
    optionstring = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_OPTIONSTRING));    
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "conf", conf,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid            
            )) {
        return; /* An error occurred which will be reported by SL */
    }

    /* Write configuration string */
    if (!ssWriteRTWStrVectParam(S, "optionstring", optionstring, get_list_count(optionstring))){
        return;
    }    
        
    mxFree(conf);
    mxFree(optionstring);    
    mxFree(blockid);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file waijung_ccp.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_ccp.c"
#endif

