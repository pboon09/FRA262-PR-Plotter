#define S_FUNCTION_NAME  stm32f0_regular_dac
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 16 /* Total number of block parameters */

#define INPUTTYPE(S) (int) mxGetScalar(ssGetSFcnParam(S, 0)) /* Input Type */
#define INPUTTYPESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* Input Type String */
#define DAC1ON(S) mxGetScalar(ssGetSFcnParam(S, 2)) /* Channel 1 */
#define DAC2ON(S) mxGetScalar(ssGetSFcnParam(S, 3)) /* Channel 2 */
#define ADVANCEDSETTINGS(S) mxGetScalar(ssGetSFcnParam(S, 4)) /* Advanced settings */
#define VREF(S) (char*)mxArrayToString(ssGetSFcnParam(S, 5)) /* Input Vref */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 6)) /* Sample time (sec) */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 7)) /* BlockID */
#define APB(S) (char*)mxArrayToString(ssGetSFcnParam(S, 8)) /* APB */
#define PORTSTR(S) ssGetSFcnParam(S, 9) /* PORTSTR */
#define PINSTR(S) ssGetSFcnParam(S, 10) /* PINSTR */
#define PINMAT(S) ssGetSFcnParam(S, 11) /* PINMAT */
#define CHMAT(S) ssGetSFcnParam(S, 12) /* CHMAT */
#define DACMODE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 13))
#define DACALIGNMENT(S) (char*)mxArrayToString(ssGetSFcnParam(S, 14))
#define DACBUFFERSTR(S) ssGetSFcnParam(S, 15)

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
	NPorts = DAC1ON(S) + DAC2ON(S);
	if (!ssSetNumInputPorts(S, NPorts)) return; /* Number of input ports */
	portCnt = 0;
	/* Port: D1 */    
	if (DAC1ON(S)){
		ssSetInputPortDirectFeedThrough(S, portCnt, 1);
		switch(INPUTTYPE(S)) {
			case 1:
				ssSetInputPortDataType(S, portCnt, SS_DOUBLE);
				break;
			case 2:
				ssSetInputPortDataType(S, portCnt, SS_SINGLE);
				break;
			case 3: case 4:
				ssSetInputPortDataType(S, portCnt, SS_UINT16);
				break;
			case 5:
				ssSetInputPortDataType(S, portCnt, SS_UINT8);
				break;
			default:
				break;
		}
		ssSetInputPortWidth(S, portCnt, 1);
		portCnt++;
	}
	/* Port: D2 */
	if (DAC2ON(S)){
		ssSetInputPortDirectFeedThrough(S, portCnt, 1);
		switch(INPUTTYPE(S)) {
			case 1:
				ssSetInputPortDataType(S, portCnt, SS_DOUBLE);
				break;
			case 2:
				ssSetInputPortDataType(S, portCnt, SS_SINGLE);
				break;
			case 3: case 4:
				ssSetInputPortDataType(S, portCnt, SS_UINT16);
				break;
			case 5:
				ssSetInputPortDataType(S, portCnt, SS_UINT8);
				break;
			default:
				break;
		}
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
	int NOutputPara = 14; /* Number of parameters to output to model.rtw */
	const char * portstr = mxArrayToString(PORTSTR(S));
	const char * pinstr = mxArrayToString(PINSTR(S));
	const char * dacbufferstr = mxArrayToString(DACBUFFERSTR(S));
	const real_T * pinmat = (real_T *) mxGetPr(PINMAT(S));
	int portstr_cnt = 1, maxpin_cnt;
	char *p;
	
	// count number of port
	p = strchr(portstr, ',');
	while (p != NULL) {
		p = strchr(p + 1, ',');
		portstr_cnt++;
	}
	
	maxpin_cnt = (int) mxGetN(PINMAT(S));
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "inputtypestr", INPUTTYPESTR(S),
			SSWRITE_VALUE_NUM, "dac1on", DAC1ON(S),
			SSWRITE_VALUE_NUM, "dac2on", DAC2ON(S),
			SSWRITE_VALUE_NUM, "advancedsettings", ADVANCEDSETTINGS(S),
			SSWRITE_VALUE_QSTR, "vref", VREF(S),
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_QSTR, "apb", APB(S),
			SSWRITE_VALUE_VECT_STR, "portstr", portstr, portstr_cnt,
			SSWRITE_VALUE_VECT_STR, "pinstr", pinstr, portstr_cnt,
			SSWRITE_VALUE_2DMAT, "pinmat", pinmat, portstr_cnt, maxpin_cnt,
            SSWRITE_VALUE_QSTR, "dacmode", DACMODE(S),
            SSWRITE_VALUE_QSTR, "dacalignment", DACALIGNMENT(S),
            SSWRITE_VALUE_QSTR, "dacbufferstr", dacbufferstr
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	mxFree(portstr);
	mxFree(pinstr);
    mxFree(dacbufferstr);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f0_regular_dac.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f0_regular_dac.c"
#endif

