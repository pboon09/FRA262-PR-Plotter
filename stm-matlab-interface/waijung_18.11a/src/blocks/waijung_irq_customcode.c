#define S_FUNCTION_NAME  waijung_irq_customcode
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 15 /* Total number of block parameters */

#define INPUTPORTCOUNT(S) (int) mxGetScalar(ssGetSFcnParam(S, 0))
#define INPUTPORTTYPDID(S) ssGetSFcnParam(S, 1)
#define OUTPUTPORTCOUNT(S) (int) mxGetScalar(ssGetSFcnParam(S, 2))
#define OUTPUTPORTTYPDID(S) ssGetSFcnParam(S, 3)
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 4))
#define BLOCKID(S) ssGetSFcnParam(S, 5)
#define CUSTOMINPORTLABEL(S) ssGetSFcnParam(S, 6)
#define CUSTOMOUTPORTLABEL(S) ssGetSFcnParam(S, 7)
#define INCFILESTRARRAY(S) ssGetSFcnParam(S, 8)
#define INPUTPORTTYPE(S) ssGetSFcnParam(S, 9)
#define OUTPUTPORTTYPE(S) ssGetSFcnParam(S, 10)
#define ENABLEFUNCALLSTR(S)  ssGetSFcnParam(S, 11)
#define OUTPUTFUNCALLSTR(S)  ssGetSFcnParam(S, 12)
#define DISABLEFUNCALLSTR(S)  ssGetSFcnParam(S, 13)
#define INITIALFUNCALLSTR(S)  ssGetSFcnParam(S, 14)

static void mdlInitializeSizes(SimStruct *S) {
	int k;
    int priority = 0;
	
	ssSetNumSFcnParams(S, NPAR);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	for (k = 0; k < NPAR; k++) {
		ssSetSFcnParamNotTunable(S, k);
	}
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
         /* Configure Input Port */
            if (!ssSetNumInputPorts(S, 0)) return; /* Number of input ports */
        
        if (!ssSetNumOutputPorts(S, 1)) return; /* Number of output ports */
    /* Port: INT */
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_UINT32);
    
    ssSetNumSampleTimes(S, 1);
    
    /* options */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
            SS_OPTION_ASYNCHRONOUS_INTERRUPT));
    
    /* Set up asynchronous timer attributes */
    ssSetTimeSource(S, SS_TIMESOURCE_BASERATE);
    
    /* Set up asynchronous task priority */
    ssSetAsyncTaskPriorities(S, 1, &priority);
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLETIME(S));
    ssSetOffsetTime(S, 0, 0.0);
    
    /* Used for set port to a function call output */
    ssSetCallSystemOutput(S, 0);  /* call on first element */
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
	int NOutputPara = 11; /* Number of parameters to output to model.rtw */
	const char * blockid = mxArrayToString(BLOCKID(S));
	const char * custominportlabel = mxArrayToString(CUSTOMINPORTLABEL(S));
	const char * customoutportlabel = mxArrayToString(CUSTOMOUTPORTLABEL(S));
	const char * incfilestrarray = mxArrayToString(INCFILESTRARRAY(S));
	const char * enablefuncallstr = mxArrayToString(ENABLEFUNCALLSTR(S));
	const char * outputfuncallstr = mxArrayToString(OUTPUTFUNCALLSTR(S));
	const char * disablefuncallstr = mxArrayToString(DISABLEFUNCALLSTR(S));
	const char * initialfuncallstr = mxArrayToString(INITIALFUNCALLSTR(S));
	int incfilestrarraycnt = 0;
	char *p;
	
	// count number of ','
	//p = strchr(incfilestrarray, ',');    
	//while (p != NULL) {
	//	p = strchr(p + 1, ',');
	//	incfilestrarraycnt++;
	//}
    incfilestrarraycnt = get_list_count(incfilestrarray);
	
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", blockid,
			SSWRITE_VALUE_VECT_STR, "custominportlabel", custominportlabel, INPUTPORTCOUNT(S),
			SSWRITE_VALUE_VECT_STR, "customoutportlabel", customoutportlabel, OUTPUTPORTCOUNT(S),
			SSWRITE_VALUE_VECT_STR, "incfilestrarray", incfilestrarray, incfilestrarraycnt,
			SSWRITE_VALUE_NUM, "outputportcnt", (double) OUTPUTPORTCOUNT(S),
			SSWRITE_VALUE_NUM, "inputportcnt", (double) INPUTPORTCOUNT(S),
			SSWRITE_VALUE_QSTR, "enablefuncallstr", enablefuncallstr,
			SSWRITE_VALUE_QSTR, "outputfuncallstr", outputfuncallstr,
			SSWRITE_VALUE_QSTR, "disablefuncallstr", disablefuncallstr,
			SSWRITE_VALUE_QSTR, "initialfuncallstr", initialfuncallstr
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	
	mxFree(blockid);
	mxFree(custominportlabel);
	mxFree(customoutportlabel);
	mxFree(incfilestrarray);
	mxFree(enablefuncallstr);
	mxFree(outputfuncallstr);
	mxFree(disablefuncallstr);	
	mxFree(initialfuncallstr);	
}

/* Enforce use of inlined S-function   *
 * (e.g. must have TLC file stm32f4_digital_output.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_irq_customcode.c"
#endif

