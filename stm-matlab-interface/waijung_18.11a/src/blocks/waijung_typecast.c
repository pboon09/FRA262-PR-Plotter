#define S_FUNCTION_NAME  waijung_typecast
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 2 /* Total number of block parameters */

#define MODE(S) (char*)mxArrayToString(ssGetSFcnParam(S, 0)) /* I2C Module */
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 1)) /* BlockID */

#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

static void mdlInitializeSizes(SimStruct *S) {
	int k;
	int InPorts, OutPorts;
    
    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    
	ssSetNumSFcnParams(S, NPAR);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	for (k = 0; k < NPAR; k++) {
		ssSetSFcnParamNotTunable(S, k);
	}
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);
    
    /* Determine number of input and output port */
    if(!strcmp("double2bytes", MODE(S))) {
        InPorts = 1;
        OutPorts = 8;        
    }
    else if(!strcmp("single2bytes", MODE(S))) {
        InPorts = 1;
        OutPorts = 4;
    }
    else if(!strcmp("bytes2double", MODE(S))) {
        InPorts = 8;
        OutPorts = 1;
    }
    else if(!strcmp("bytes2single", MODE(S))) {
        InPorts = 4;
        OutPorts = 1;
    }
    else {
        ssSetErrorStatus(S, "Invalid mode.");
        return;
    }
    
    /* Input port */
    if (!ssSetNumInputPorts(S, InPorts)) return; /* Number of input ports */
    for (k = 0; k < InPorts; k++) {
		ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortComplexSignal(S,  k, COMPLEX_NO);
        ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
            
        if(!strcmp("double2bytes", MODE(S)))
            ssSetInputPortDataType(S, k, SS_DOUBLE);
        else if(!strcmp("single2bytes", MODE(S)))
            ssSetInputPortDataType(S, k, SS_SINGLE);
        else
            ssSetInputPortDataType(S, k, SS_UINT8);
		ssSetInputPortWidth(S, k, 1);
	}
    
    /* Output port */
    if (!ssSetNumOutputPorts(S, OutPorts)) return; /* Number of output ports */
    for (k = 0; k < OutPorts; k++) {
        ssSetOutputPortComplexSignal(S,  k, COMPLEX_NO);
        if(!strcmp("bytes2double", MODE(S)))
            ssSetOutputPortDataType(S, k, SS_DOUBLE);
        else if(!strcmp("bytes2single", MODE(S)))
            ssSetOutputPortDataType(S, k, SS_SINGLE);
        else
            ssSetOutputPortDataType(S, k, SS_UINT8);
		ssSetOutputPortWidth(S, k, 1);
	}
    
	ssSetNumSampleTimes(S, 1);
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S, 0, -1);
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
    int i;
    int input_count;
    int output_count;
    
    /* Input */
    static uint8_T bytes[8];
    static real64_T double_val;
    static real32_T single_val;
    /* Output */
    uint8_T  *p_byte_val;
    real64_T *p_double_val;
    real32_T *p_single_val;
    
	/* Clear input */
    memset(bytes, sizeof(bytes), 0);
    
    /* Collect input */
    input_count = ssGetNumInputPorts(S);
    //printf("Input count: %d\n", input_count);
    for(i=0; i<input_count; i++) {
        switch(ssGetInputPortDataType(S, i)) {
            case SS_UINT8:
                if(i < sizeof(bytes))
                    bytes[i] = *(const uint8_T*) ssGetInputPortSignal(S, i);
                else
                    ssSetErrorStatus(S, "Invalid input port count.");
                break;
                
            case SS_DOUBLE:
                double_val = *(const real64_T*) ssGetInputPortSignal(S, i);
                memcpy(bytes, (void*)&double_val, 8);
                break;
                
            case SS_SINGLE:
                single_val = *(const real32_T*) ssGetInputPortSignal(S, i);
                memcpy(bytes, (void*)&single_val, 4);
                break;
                
            default:
                ssSetErrorStatus(S, "Invalid input data type.");
                return;
        }
    }
    
    //printf("-----------------------\n");
    //for(i=0; i<8; i++) {
    //    printf("Data%d=%d\n", i, bytes[i]);
    //}
    
    /* Drive output */
    output_count = ssGetNumOutputPorts(S);
    //printf("Output count: %d\n", output_count);
    for(i=0; i<output_count; i++) {
        switch(ssGetOutputPortDataType(S, i)) {
            case SS_UINT8:
                p_byte_val = (uint8_T*) ssGetOutputPortSignal(S, i);
                *p_byte_val = bytes[i];
                break;
            case SS_DOUBLE:
                p_double_val = (real64_T*) ssGetOutputPortSignal(S, i);
                *p_double_val = *(real64_T*)&bytes[0];
                break;
            case SS_SINGLE:
                p_single_val = (real32_T*) ssGetOutputPortSignal(S, i);
                *p_single_val = *(real32_T*)&bytes[0];
                break;
            default:
                ssSetErrorStatus(S, "Invalid output data type.");
                return;
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
	int NOutputPara = 2; /* Number of parameters to output to model.rtw */
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "mode", MODE(S),
			SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S)
			)) {
		return; /* An error occurred which will be reported by SL */
	}
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file waijung_typecast.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_typecast.c"
#endif

