#define S_FUNCTION_NAME  waijung_capturesystickcounter
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 4 /* Total number of block parameters */

#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 0))
#define BLOCKID(S) ssGetSFcnParam(S, 1)
#define ENABLEINPUT(S) mxGetScalar(ssGetSFcnParam(S, 2))
#define LABEL(S) ssGetSFcnParam(S, 3)

static void mdlInitializeSizes(SimStruct *S) {
	int k;
	
	ssSetNumSFcnParams(S, NPAR);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	for (k = 0; k < NPAR; k++) {
		ssSetSFcnParamNotTunable(S, k);
	}
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	
	/* Configure Input Port */
	if (!ssSetNumInputPorts(S, (int) ENABLEINPUT(S))) return;
	if (ENABLEINPUT(S)) {
		ssSetInputPortDirectFeedThrough(S, 0, 1);
		ssSetInputPortDataType(S, 0, DYNAMICALLY_TYPED);
		ssSetInputPortWidth(S, 0, 1);
	}
	
	/* Configure Output Port */
	if (!ssSetNumOutputPorts(S, 0)) return;
	
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
	int NOutputPara = 2; /* Number of parameters to output to model.rtw */
	const char * blockid = mxArrayToString(BLOCKID(S));
	const char * label = mxArrayToString(LABEL(S));
	
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "blockid", blockid,
			SSWRITE_VALUE_QSTR, "label", label
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	
	mxFree(blockid);
	mxFree(label);
}

/* Enforce use of inlined S-function   *
 * (e.g. must have TLC file stm32f4_digital_output.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_capturesystickcounter.c"
#endif

