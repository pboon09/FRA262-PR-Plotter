/* Implement Basic Timer Time Base Asynchronous Block */

#define S_FUNCTION_NAME  stm32f4_bttbisr	/* must have */
#define S_FUNCTION_LEVEL 2	/* must have */

#include "simstruc.h" 		/* must have */

#define N_PAR 8		/* Total number of block parameters */ /* must have */


        /*
         *  Default indexing:
         * Popup order 1,2,3,...
         * Checkbox True = 1, False = 0
         */
                
        /* Facotr to consider scalar or vector
         * If scalar use: #define TIMERISR_SETTING(S)  mxGetScalar(ssGetSFcnParam(S, TIMERISR_SETTING_ARGC))
         * If vector use: #define ADC1_CH(S)  ssGetSFcnParam(S, ADC1_CH_ARGC)
         */
        
#define TIMERISR_TIMER(S)  ssGetSFcnParam(S, 0)
#define TIMERISR_NVICPRIORITYGROUP(S)  ssGetSFcnParam(S, 1)
#define TIMERISR_IRQPREEMPTIONPRIORITY(S)  ssGetSFcnParam(S, 2)
#define TIMERISR_IRQSUBPRIORITY(S)  ssGetSFcnParam(S, 3)
#define SAMPLETIME(S)  mxGetScalar(ssGetSFcnParam(S, 4))
#define TIMERISR_AUTORELOAD(S)  ssGetSFcnParam(S, 5)
#define TIMERISR_PRESCALER(S)  ssGetSFcnParam(S, 6)
#define BLOCKID(S) (char*)mxArrayToString(ssGetSFcnParam(S, 7))

/* A minimum of 4 functions must be implemented */
/* 1. Initialize Size */
/* 2. Initialize Sampletime */
/* 3. Output */
/* 4. Terminate */

/* Determine the size of S-Function I/O port
 * How many ports? = How many output from block ?
 * Width? = Vector signal
 */
static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int priority = 0;
    
    /* char * pintype = mxArrayToString(TIMERISR_PINTYPE(S)); */
    
    ssSetNumSFcnParams(S, N_PAR);	/* Set and Check parameter count  */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < N_PAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    
    /* mexPrintf("\nparam: %f", SAMPLETIME(S)); */
    
    if (!ssSetNumInputPorts(S, 0)) return; /* Number of input ports */
    
    if (!ssSetNumOutputPorts(S, 1)) return; /* Number of output ports */
    ssSetOutputPortWidth(S, 0, 1);
    
    /* sample times */
    ssSetNumSampleTimes(S, 1);
    
    /* options */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
            SS_OPTION_ASYNCHRONOUS_INTERRUPT));
    
    /* Set up asynchronous timer attributes */
    ssSetTimeSource(S,SS_TIMESOURCE_BASERATE);

    /* Set up asynchronous task priority */
    ssSetAsyncTaskPriorities(S, 1, &priority);
    
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    //ssSetSampleTime(S, 0, SAMPLETIME(S));
    //ssSetSampleTime(S, 0, 0.2);
    //ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    
    /* Used to set port to a function call output */
    ssSetCallSystemOutput(S, 0);  /* call on first element */
} /* end mdlInitializeSampleTimes */

static void mdlOutputs(SimStruct *S, int_T tid) {
    if (ssGetNumInputPorts(S) == 0) {
        if (!ssCallSystemWithTid(S,0,tid)) {
            /* Error occurred which will be reported by Simulink */
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
    int NOutputPara = 7; /* Number of parameters to output to model.rtw */

	char * timer = mxArrayToString(TIMERISR_TIMER(S));
    char * nvicprioritygroup = mxArrayToString(TIMERISR_NVICPRIORITYGROUP(S));
    char * irqpreemptionpriority = mxArrayToString(TIMERISR_IRQPREEMPTIONPRIORITY(S));
    char * irqsubpriority = mxArrayToString(TIMERISR_IRQSUBPRIORITY(S));
    char * autoreload = mxArrayToString(TIMERISR_AUTORELOAD(S));
    char * prescaler = mxArrayToString(TIMERISR_PRESCALER(S));

    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "timer", timer,            
            SSWRITE_VALUE_QSTR, "nvicprioritygroup", nvicprioritygroup,
            SSWRITE_VALUE_QSTR, "irqpreemptionpriority", irqpreemptionpriority,
            SSWRITE_VALUE_QSTR, "irqsubpriority", irqsubpriority,
            SSWRITE_VALUE_QSTR, "autoreload", autoreload,
			SSWRITE_VALUE_QSTR, "prescaler", prescaler, 
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S) 
			)) {
        return; /* An error occurred which will be reported by SL */
    }
}

/*=======================================*
 * Enforce use of inlined S-function      *
 * (e.g. must have TLC file systick.tlc)  *
 *=======================================*/

#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_bttbisr.c"
#endif
