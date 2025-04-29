#define S_FUNCTION_NAME  waijung_basic_customcode
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_INPUT_PORTCOUNT = 0,
    ARGC_INPUT_PORTID,
    ARGC_OUTPUT_PORTCOUNT,
    ARGC_OUTPUT_PORTID,
    
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
	
	ARGC_CUSTOMINPORTLABEL,
	ARGC_CUSTOMOUTPORTLABEL,	
	ARGC_INCFILESTRARRAY,
	
	ARGC_INPUTPORTTYPE,
	ARGC_OUTPUTPORTTYPE,
	
	ARGC_INITIALFUNCALLSTR,
	ARGC_ENABLEFUNCALLSTR,
	ARGC_OUTPUTFUNCALLSTR,
	ARGC_DISABLEFUNCALLSTR,
	ARGC_INPUTPORTDIMINFO,
	ARGC_OUTPUTPORTDIMINFO,
    
    __PARAM_COUNT
};

#define INPUTPORTCOUNT(S) (int) mxGetScalar(ssGetSFcnParam(S, ARGC_INPUT_PORTCOUNT))
#define INPUTPORTTYPDID(S) ssGetSFcnParam(S, ARGC_INPUT_PORTID)
#define OUTPUTPORTCOUNT(S) (int) mxGetScalar(ssGetSFcnParam(S, ARGC_OUTPUT_PORTCOUNT))
#define OUTPUTPORTTYPDID(S) ssGetSFcnParam(S, ARGC_OUTPUT_PORTID)
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME))
#define BLOCKID(S) ssGetSFcnParam(S, ARGC_BLOCKID)
#define CUSTOMINPORTLABEL(S) ssGetSFcnParam(S, ARGC_CUSTOMINPORTLABEL)
#define CUSTOMOUTPORTLABEL(S) ssGetSFcnParam(S, ARGC_CUSTOMOUTPORTLABEL)
#define INCFILESTRARRAY(S) ssGetSFcnParam(S, ARGC_INCFILESTRARRAY)
#define INPUTPORTTYPE(S) ssGetSFcnParam(S, ARGC_INPUTPORTTYPE)
#define OUTPUTPORTTYPE(S) ssGetSFcnParam(S, ARGC_OUTPUTPORTTYPE)
#define INITIALFUNCALLSTR(S)  ssGetSFcnParam(S, ARGC_INITIALFUNCALLSTR)
#define ENABLEFUNCALLSTR(S)  ssGetSFcnParam(S, ARGC_ENABLEFUNCALLSTR)
#define OUTPUTFUNCALLSTR(S)  ssGetSFcnParam(S, ARGC_OUTPUTFUNCALLSTR)
#define DISABLEFUNCALLSTR(S)  ssGetSFcnParam(S, ARGC_DISABLEFUNCALLSTR)
#define INPUTPORTDIMINFO(S) ssGetSFcnParam(S, ARGC_INPUTPORTDIMINFO)
#define OUTPUTPORTDIMINFO(S) ssGetSFcnParam(S, ARGC_OUTPUTPORTDIMINFO)

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int input_count;
    int output_count;
    real_T * inputportdiminfo = (real_T *) mxGetPr(INPUTPORTDIMINFO(S));
    real_T * outputportdiminfo = (real_T *) mxGetPr(OUTPUTPORTDIMINFO(S));
    int portwidth, nRows, nCols;
    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	
	input_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUTPORTDIMINFO))/4;
    output_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUTPORTDIMINFO))/4;
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return;
    for (k = 0; k < input_count; k++){
        ssSetInputPortDirectFeedThrough(S, k, 1);
		ssSetInputPortDataType(S, k, (int)inputportdiminfo[(input_count * 3) + k]);
        if(!ssSetInputPortDimensionInfo(S, k, DYNAMIC_DIMENSION)) return;
    }
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return;
    
    for (k = 0; k < output_count; k++){
		ssSetOutputPortDataType(S, k, (int)outputportdiminfo[(output_count * 3) + k]);
		portwidth = (int) outputportdiminfo[(output_count * 2) + k];
        if (portwidth == 1) {
            ssSetOutputPortWidth(S, k, 1);
        }
        else {
            nRows = (int) outputportdiminfo[(output_count * 0) + k];
            nCols = (int) outputportdiminfo[(output_count * 1) + k];
            ssSetOutputPortMatrixDimensions(S, k, nRows, nCols);
        }
    }
    
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

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
/* Function: mdlSetInputPortDimensionInfo ====================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an input port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given port
 *    can also have their dimensions set.
 */
static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo) {
    
    /* Set input port dimension */
    int_T inWidth = ssGetInputPortWidth(S, port);
    
    if (inWidth == DYNAMICALLY_SIZED){
        if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
    }
    
} /* end mdlSetInputPortDimensionInfo */

# define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
/* Function: mdlSetOutputPortDimensionInfo ===================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an output port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given
 *    port can also have their dimensions set.
 */
static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo) {
    int_T outWidth = ssGetOutputPortWidth(S, port);
    
    if (outWidth == DYNAMICALLY_SIZED){
        if(!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
    }
} /* end mdlSetOutputPortDimensionInfo */

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
/* Function: mdlSetDefaultPortDimensionInfo ====================================
 *    This routine is called when Simulink is not able to find dimension
 *    candidates for ports with unknown dimensions. This function must set the
 *    dimensions of all ports with unknown dimensions.
 */
static void mdlSetDefaultPortDimensionInfo(SimStruct *S) {
    int k;
    
    for (k = 0; k < INPUTPORTCOUNT(S); k++){
        if(!ssSetInputPortMatrixDimensions(S, k, 1, 1)) return;
    }
} /* end mdlSetDefaultPortDimensionInfo */
#endif

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
        count ++; p++;
    }
    return count;
}

#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
	int incfilestrarraycnt = 0;
    char * blockid = mxArrayToString(BLOCKID(S));
    char * custominportlabel = mxArrayToString(CUSTOMINPORTLABEL(S));
    char * customoutportlabel = mxArrayToString(CUSTOMOUTPORTLABEL(S));
    char * incfilestrarray = mxArrayToString(INCFILESTRARRAY(S));
	char * initialfuncallstr = mxArrayToString(INITIALFUNCALLSTR(S));
    char * enablefuncallstr = mxArrayToString(ENABLEFUNCALLSTR(S));
    char * outputfuncallstr = mxArrayToString(OUTPUTFUNCALLSTR(S));
    char * disablefuncallstr = mxArrayToString(DISABLEFUNCALLSTR(S));
	
	incfilestrarraycnt = get_list_count(incfilestrarray);			
	
	if (!ssWriteRTWParamSettings(S, 9,
			SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
			SSWRITE_VALUE_QSTR, "blockid", blockid,
			SSWRITE_VALUE_VECT_STR, "custominportlabel", custominportlabel, INPUTPORTCOUNT(S),
			SSWRITE_VALUE_VECT_STR, "customoutportlabel", customoutportlabel, OUTPUTPORTCOUNT(S),
			SSWRITE_VALUE_VECT_STR, "incfilestrarray", incfilestrarray, incfilestrarraycnt,
			SSWRITE_VALUE_QSTR, "initialfuncallstr", initialfuncallstr,
			SSWRITE_VALUE_QSTR, "enablefuncallstr", enablefuncallstr,
			SSWRITE_VALUE_QSTR, "outputfuncallstr", outputfuncallstr,
			SSWRITE_VALUE_QSTR, "disablefuncallstr", disablefuncallstr
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	
    mxFree(blockid);
    mxFree(custominportlabel);
    mxFree(customoutportlabel);
    mxFree(incfilestrarray);
	mxFree(initialfuncallstr);
    mxFree(enablefuncallstr);
    mxFree(outputfuncallstr);
    mxFree(disablefuncallstr);
}

/* Enforce use of inlined S-function   *
 * (e.g. must have TLC file stm32f4_digital_output.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_basic_customcode.c"
#endif

