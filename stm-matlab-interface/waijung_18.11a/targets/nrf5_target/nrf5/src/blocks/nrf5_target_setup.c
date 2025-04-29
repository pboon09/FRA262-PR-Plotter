#define S_FUNCTION_NAME  nrf5_target_setup
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_COMPILER = 0,
    ARCG_STACKSIZE,
    ARCG_HEAPSIZE,
    ARCG_SAMPLETIME,
    ARCG_BLOCKID,
    ARCG_SYSTICKRELOADVALUE,
    ARCG_HSEVAL,
    ARCG_HCLK,
    ARCG_SOFTDEVICE_VER,
    ARGC_OPTIONSTRING,
    __PARAM_COUNT
};

#define COMPILER(S) (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_COMPILER))
#define STACKSIZE(S) (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_STACKSIZE))
#define HEAPSIZE(S) (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_HEAPSIZE))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARCG_SAMPLETIME))
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_BLOCKID))
#define SYSTICKRELOADVALUE(S) (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_SYSTICKRELOADVALUE))
#define HSEVAL(S) (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_HSEVAL))
#define HCLK(S) mxGetScalar(ssGetSFcnParam(S, ARCG_HCLK))

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
	/* do nothing */
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
	int NOutputPara = 9; /* Number of parameters to output to model.rtw */
	char *s;
	char* optionstring;
    
	optionstring = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_OPTIONSTRING)); 
	s = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_SOFTDEVICE_VER)); // Soft device version
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "compiler", COMPILER(S),
			SSWRITE_VALUE_QSTR, "stacksize", STACKSIZE(S),
			SSWRITE_VALUE_QSTR, "heapsize", HEAPSIZE(S),
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_QSTR, "systickreloadvalue", SYSTICKRELOADVALUE(S),
			SSWRITE_VALUE_QSTR, "hseval", HSEVAL(S),
			SSWRITE_VALUE_NUM, "hclk", HCLK(S),
			SSWRITE_VALUE_QSTR, "softdeviceversion", s
			)) {
		return; /* An error occurred which will be reported by SL */
	}
    
    /* Write configuration string */
    if (!ssWriteRTWStrVectParam(S, "optionstring", optionstring, get_list_count(optionstring))){
        return;
    }
    mxFree(optionstring);
	mxFree(s);
	
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file nrf5_target_setup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function nrf5_target_setup.c"
#endif

