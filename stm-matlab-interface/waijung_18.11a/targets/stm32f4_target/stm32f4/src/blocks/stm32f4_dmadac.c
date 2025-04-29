#define S_FUNCTION_NAME  stm32f4_dmadac
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 28 /* Total number of block parameters */

#define INPUTTYPE(S) (int) mxGetScalar(ssGetSFcnParam(S, 0)) /* Input Type */
#define INPUTTYPESTR(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* Input Type String */
#define DAC1ON(S) mxGetScalar(ssGetSFcnParam(S, 2)) /* Channel 1 */
#define DAC2ON(S) mxGetScalar(ssGetSFcnParam(S, 3)) /* Channel 2 */
#define ADVANCEDSETTINGS(S) mxGetScalar(ssGetSFcnParam(S, 4)) /* Advanced settings */
#define VREF(S) (char*)mxArrayToString(ssGetSFcnParam(S, 5)) /* Input Vref */
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 6)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, 7)) /* Compiled sample time (sec) in string */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 8)) /* BlockID */
#define APB(S) (char*)mxArrayToString(ssGetSFcnParam(S, 9)) /* APB */
#define PORTSTR(S) ssGetSFcnParam(S, 10) /* PORTSTR */
#define PINSTR(S) ssGetSFcnParam(S, 11) /* PINSTR */
#define PINMAT(S) ssGetSFcnParam(S, 12) /* PINMAT */
#define CHMAT(S) ssGetSFcnParam(S, 13) /* CHMAT */
#define DACMODE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 14))
#define DACALIGNMENT(S) (char*)mxArrayToString(ssGetSFcnParam(S, 15))
#define DACBUFFERSTR(S) ssGetSFcnParam(S, 16)
#define SIMULATION(S) (int) mxGetScalar(ssGetSFcnParam(S, 17))
#define DAC1VAL(S) ssGetSFcnParam(S, 18)
#define DAC2VAL(S) ssGetSFcnParam(S, 19)
#define DAC1ADDRESS(S) ssGetSFcnParam(S, 20)
#define DMAMODESTR(S) ssGetSFcnParam(S, 21)
#define DAC1DOR(S) ssGetSFcnParam(S, 22)
#define TIMER(S) ssGetSFcnParam(S, 23)
#define TIMARR(S) ssGetSFcnParam(S, 24)
#define TIMPRESCALE(S) ssGetSFcnParam(S, 25)
#define DAC2DOR(S) ssGetSFcnParam(S, 26)
#define DAC2ADDRESS(S) ssGetSFcnParam(S, 27)

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
	// printf("Simulation = %d\n", SIMULATION(S));
	if (SIMULATION(S)) {
		NPorts = DAC1ON(S) + DAC2ON(S);
		if (!ssSetNumOutputPorts(S, NPorts)) return; /* Number of output ports */
		portCnt = 0;
		/* Port: D1 */
		if (DAC1ON(S)){
			ssSetOutputPortDataType(S, portCnt, SS_DOUBLE);
			ssSetOutputPortWidth(S, portCnt, 1);
			portCnt++;
		}
		/* Port: D2 */
		if (DAC2ON(S)){
			ssSetOutputPortDataType(S, portCnt, SS_DOUBLE);
			ssSetOutputPortWidth(S, portCnt, 1);
		}
	}
	else {
		NPorts = 0;
		if (!ssSetNumOutputPorts(S, NPorts)) return; /* Number of output ports */
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
    static int dac1cnt = 0;
    static int dac2cnt = 0;    
    int dac1_length = (int) mxGetN(DAC1VAL(S));
    int dac2_length = (int) mxGetN(DAC2VAL(S));
    const real_T * dac1val = (real_T *) mxGetData(DAC1VAL(S));
	const real_T * dac2val = (real_T *) mxGetData(DAC2VAL(S));
    real_T * p_double_val;
    
    if (SIMULATION(S)) {
        if (DAC1ON(S)) {
            p_double_val = (real_T*) ssGetOutputPortSignal(S, 0);           
            *p_double_val = dac1val[dac1cnt++];
            if(dac1cnt >= dac1_length) {
                dac1cnt = 0;
            }
        }
        
        if (DAC2ON(S)) {
            p_double_val = (real_T*) ssGetOutputPortSignal(S, 1);
            *p_double_val = dac2val[dac2cnt++];
            if(dac2cnt >= dac2_length) {
                dac2cnt = 0;
            }            
        }       
    }                        
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
	int NOutputPara = 27; /* Number of parameters to output to model.rtw */
	const char * portstr = mxArrayToString(PORTSTR(S));
	const char * pinstr = mxArrayToString(PINSTR(S));
	const char * dacbufferstr = mxArrayToString(DACBUFFERSTR(S));
	const char * dac1address = mxArrayToString(DAC1ADDRESS(S));
	const char * dac2address = mxArrayToString(DAC2ADDRESS(S));
	const char * dmamodestr = mxArrayToString(DMAMODESTR(S));
	const char * dac1dor = mxArrayToString(DAC1DOR(S));
	const char * dac2dor = mxArrayToString(DAC2DOR(S));
	const char * timer = mxArrayToString(TIMER(S));
	const char * timarr = mxArrayToString(TIMARR(S));
	const char * timprescale = mxArrayToString(TIMPRESCALE(S));
	const real_T * pinmat = (real_T *) mxGetPr(PINMAT(S));
	const real_T * dac1val = (real_T *) mxGetData(DAC1VAL(S));
	const real_T * dac2val = (real_T *) mxGetData(DAC2VAL(S));
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
			SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S),
			SSWRITE_VALUE_QSTR, "apb", APB(S),
			SSWRITE_VALUE_VECT_STR, "portstr", portstr, portstr_cnt,
			SSWRITE_VALUE_VECT_STR, "pinstr", pinstr, portstr_cnt,
			SSWRITE_VALUE_2DMAT, "pinmat", pinmat, portstr_cnt, maxpin_cnt,
			SSWRITE_VALUE_QSTR, "dacmode", DACMODE(S),
			SSWRITE_VALUE_QSTR, "dacalignment", DACALIGNMENT(S),
			SSWRITE_VALUE_QSTR, "dacbufferstr", dacbufferstr,
			SSWRITE_VALUE_VECT, "dac1val", dac1val, (int) mxGetN(DAC1VAL(S)),
			SSWRITE_VALUE_VECT, "dac2val", dac2val, (int) mxGetN(DAC2VAL(S)),
			SSWRITE_VALUE_NUM, "dac1val_length", (double) mxGetN(DAC1VAL(S)),
			SSWRITE_VALUE_NUM, "dac2val_length", (double) mxGetN(DAC2VAL(S)),
			SSWRITE_VALUE_QSTR, "dac1address", dac1address,
			SSWRITE_VALUE_QSTR, "dac2address", dac2address,
			SSWRITE_VALUE_QSTR, "dmamodestr", dmamodestr,
			SSWRITE_VALUE_QSTR, "timer", timer,
			SSWRITE_VALUE_QSTR, "timarr", timarr,
			SSWRITE_VALUE_QSTR, "timprescale", timprescale,
			SSWRITE_VALUE_QSTR, "dac1dor", dac1dor,
			SSWRITE_VALUE_QSTR, "dac2dor", dac2dor
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	mxFree(portstr);
	mxFree(pinstr);
	mxFree(dacbufferstr);
	mxFree(dac1address);
	mxFree(dac2address);
	mxFree(dmamodestr);
	mxFree(dac1dor);
	mxFree(dac2dor);
	mxFree(timer);
	mxFree(timarr);
	mxFree(timprescale);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_dmadac.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_dmadac.c"
#endif

