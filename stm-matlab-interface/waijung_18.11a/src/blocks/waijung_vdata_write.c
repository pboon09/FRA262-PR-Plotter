#define S_FUNCTION_NAME  waijung_vdata_write
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 7 /* Total number of block parameters */

#define STORAGENAME(S) (char*)mxArrayToString(ssGetSFcnParam(S, 0)) /* Storage Name */
#define VARNAME(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* Variable name */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 2)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 3)) /* Compiled sample time (sec) in string */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 4)) /* BlockID */
#define STORAGETYPE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 5)) /* Storage type */
#define STORAGETYPESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 6)) /* Storage type */


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
	NPorts = 1;
	if (!ssSetNumInputPorts(S, NPorts)) return; /* Number of input ports */
	portCnt = 0;
    ssSetInputPortDirectFeedThrough(S, portCnt, 1);
    if(!strcmp(STORAGETYPE(S), "string")) {
        ssSetInputPortDataType(S, portCnt, SS_UINT32);
    }
    else if(!strcmp(STORAGETYPE(S), "double")) {
        ssSetInputPortDataType(S, portCnt, SS_DOUBLE);
    }
    else if(!strcmp(STORAGETYPE(S), "single")) {
        ssSetInputPortDataType(S, portCnt, SS_SINGLE);
    }
    else if(!strcmp(STORAGETYPE(S), "uint32")) {
        ssSetInputPortDataType(S, portCnt, SS_UINT32);
    }
    else if(!strcmp(STORAGETYPE(S), "int32")) {
        ssSetInputPortDataType(S, portCnt, SS_INT32);
    }
    else if(!strcmp(STORAGETYPE(S), "uint16")) {
        ssSetInputPortDataType(S, portCnt, SS_UINT16);
    }
    else if(!strcmp(STORAGETYPE(S), "int16")) {
        ssSetInputPortDataType(S, portCnt, SS_INT16);
    }
    else if(!strcmp(STORAGETYPE(S), "uint8")) {
        ssSetInputPortDataType(S, portCnt, SS_UINT8);
    }
    else if(!strcmp(STORAGETYPE(S), "int8")) {
        ssSetInputPortDataType(S, portCnt, SS_INT8);
    }
    else {
        printf("Write Invalid data type\n");
        return;
    }
    
	ssSetInputPortWidth(S, portCnt, 1);
	//}
	
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
	int NOutputPara = 7; /* Number of parameters to output to model.rtw */
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "storagename", STORAGENAME(S),
			SSWRITE_VALUE_QSTR, "varname", VARNAME(S),
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "sampletimestr", SAMPLETIMESTR(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_QSTR, "storagetype", STORAGETYPE(S),
			SSWRITE_VALUE_QSTR, "storagetypestr", STORAGETYPESTR(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file waijung_printf.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_printf.c"
#endif

