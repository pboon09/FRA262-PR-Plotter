#define S_FUNCTION_NAME  stm32f4_target_setup
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 8 /* Total number of block parameters */

#define COMPILER(S) (char*)mxArrayToString(ssGetSFcnParam(S, 0))
#define STACKSIZE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1))
#define HEAPSIZE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 2))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 3))
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 4))
#define SYSTICKRELOADVALUE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 5))
#define HSEVAL(S) (char*)mxArrayToString(ssGetSFcnParam(S, 6))
#define HCLK(S) mxGetScalar(ssGetSFcnParam(S, 7))

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
	int NOutputPara = 8; /* Number of parameters to output to model.rtw */
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "compiler", COMPILER(S),
			SSWRITE_VALUE_QSTR, "stacksize", STACKSIZE(S),
			SSWRITE_VALUE_QSTR, "heapsize", HEAPSIZE(S),
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_QSTR, "systickreloadvalue", SYSTICKRELOADVALUE(S),
			SSWRITE_VALUE_QSTR, "hseval", HSEVAL(S),
			SSWRITE_VALUE_NUM, "hclk", HCLK(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_target_setup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_target_setup.c"
#endif

