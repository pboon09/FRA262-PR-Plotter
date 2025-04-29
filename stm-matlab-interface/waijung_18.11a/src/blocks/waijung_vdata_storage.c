#define S_FUNCTION_NAME  waijung_vdata_storage
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 10 /* Total number of block parameters */

#define STORAGETYPE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 0)) /* Storage Type */
#define STORAGENAME(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* Storage Name (must be a valid variable name) */
#define INITVAL(S) mxGetScalar(ssGetSFcnParam(S, 2)) /* Initial value(s) */
#define VARNAME(S) (char*)mxArrayToString(ssGetSFcnParam(S, 3)) /* Variable name */
#define BUFFERSIZE(S) mxGetScalar(ssGetSFcnParam(S, 4)) /* Buffer size */
#define INITVALSTRING(S) (char*)mxArrayToString(ssGetSFcnParam(S, 5)) /* Initial value string */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 6)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 7)) /* Compiled sample time (sec) in string */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 8)) /* BlockID */
#define STORAGETYPESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 9)) /* Storage Type */

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

#define MDL_START                      /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S) {
	
}
#endif /*  MDL_START */


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
	int NOutputPara = 10; /* Number of parameters to output to model.rtw */
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "storagetype", STORAGETYPE(S),
			SSWRITE_VALUE_QSTR, "storagename", STORAGENAME(S),
			SSWRITE_VALUE_NUM, "initval", INITVAL(S),
			SSWRITE_VALUE_QSTR, "varname", VARNAME(S),
            SSWRITE_VALUE_NUM, "buffersize", BUFFERSIZE(S),
            SSWRITE_VALUE_QSTR, "initvalstring",INITVALSTRING(S),            
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "sampletimestr", SAMPLETIMESTR(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_QSTR, "storagetypestr", STORAGETYPESTR(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file waijung_vdata_storage.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_vdata_storage.c"
#endif

