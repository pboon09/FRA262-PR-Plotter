#define S_FUNCTION_NAME  stm32f4_uidread
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 3 /* Total number of block parameters */

#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 0)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, 1)) /* Compiled sample time (sec) in string */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 2)) /* BlockID */

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
	NPorts = 1;
	if (!ssSetNumOutputPorts(S, NPorts)) return; /* Number of output ports */
	portCnt = 0;
	/* Port: UID */
	ssSetOutputPortDataType(S, portCnt, SS_UINT32);
	ssSetOutputPortWidth(S, portCnt, 3);
	portCnt++;
	
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
	int NOutputPara = 3; /* Number of parameters to output to model.rtw */
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_uidread.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_uidread.c"
#endif

