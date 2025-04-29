#define S_FUNCTION_NAME  stm32f0_target_setup
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_COMPILER = 0,    
	ARGC_STACKSIZE,
	ARGC_HEAPSIZE,
	ARGC_SAMPLETIME,
	ARGC_BLOCKID,
	ARGC_SYSTICKRELOADVALUE,
	ARGC_HSEVAL,
	ARGC_HCLK,
	ARGC_STDLIB,
    
    __PARAM_COUNT
};

#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME))
#define HCLK(S) mxGetScalar(ssGetSFcnParam(S, ARGC_HCLK))

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

#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
	char *compiler           = (char*) mxArrayToString(ssGetSFcnParam(S, ARCG_COMPILER));
	char *stacksize          = (char*) mxArrayToString(ssGetSFcnParam(S, ARGC_STACKSIZE));
	char *heapsize           = (char*) mxArrayToString(ssGetSFcnParam(S, ARGC_HEAPSIZE));
	char *blockid            = (char*) mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
	char *systickreloadvalue = (char*) mxArrayToString(ssGetSFcnParam(S, ARGC_SYSTICKRELOADVALUE));
  	char *hseval             = (char*) mxArrayToString(ssGetSFcnParam(S, ARGC_HSEVAL));	
	char *stdlib             = (char*) mxArrayToString(ssGetSFcnParam(S, ARGC_STDLIB));	
	
	int NOutputPara = 9; /* Number of parameters to output to model.rtw */
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "compiler", compiler,
			SSWRITE_VALUE_QSTR, "stacksize", stacksize,
			SSWRITE_VALUE_QSTR, "heapsize", heapsize,
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", blockid,
			SSWRITE_VALUE_QSTR, "systickreloadvalue", systickreloadvalue,
			SSWRITE_VALUE_QSTR, "hseval", hseval,
			SSWRITE_VALUE_QSTR, "stdlib", stdlib,
			SSWRITE_VALUE_NUM, "hclk", HCLK(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	mxFree (compiler);
	mxFree (stacksize);
	mxFree (heapsize);
	mxFree (blockid);
	mxFree (systickreloadvalue);
	mxFree (hseval);
	mxFree (stdlib);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f0_target_setup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f0_target_setup.c"
#endif

