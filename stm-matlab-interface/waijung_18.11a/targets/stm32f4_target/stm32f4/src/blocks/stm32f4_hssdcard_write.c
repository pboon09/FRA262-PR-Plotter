#define S_FUNCTION_NAME  stm32f4_hssdcard_write
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    
    /*
     * packetmode
     * recordmode
     * recordcount
     * asciiheader
     * asciiformat
     * inputtype
     * outputtype
     * sampletime
     * sampletimestr
     * blockid
     */
    ARGC_PACKETMODE = 0,
    ARGC_RECORDMODE,
    ARGC_RECORDCOUNT,
    ARGC_RECORDSIZE,
    ARGC_ASCIIHEADER,
    ARGC_ASCIIFORMAT,
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    ARGC_BUFFER_SIZE,
    ARGC_FILE_SIZE,
    
    ARGC_GPIO_PORT,
    ARGC_GPIO_STA_BUSY,
    ARGC_GPIO_STA_SUCCESS,
    ARGC_GPIO_STA_ERROR,
    
    ARGC_SDCARD_OPTION,
    ARGC_CONFSTR,
    
    ARGC_SAMPLETIME,
    ARGC_SAMPLETIMESTR,
    ARGC_BLOCKID, 
    
    __PARAM_COUNT
};

#define SAMPLETIME(S)    mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR)) /* Compiled sample time (sec) in string */
#define BLOCKID(S)      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID)) /* BlockID */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    
    int input_count;
    int output_count;
    
    /* Parameter validatone */    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    input_count = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[0]);
    output_count = (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[0]);
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
   
    /* Port */
    for(k=0; k<output_count; k++) {
        ssSetOutputPortWidth(S, k, 1);
        ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[k+1]));
    }

    for(k=0; k<input_count; k++) {
        ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortWidth(S, k, 1);
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k+1]));
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
    int NOutputPara;

    char* packetmode;
    char* recordmode;
    char* asciiheader;    
    char* asciiformat;
    char* buffersize;
    
    char* gpio_port;
    char* gpio_sta_busy;
    char* gpio_sta_success;
    char* gpio_sta_error;
    
    char* sdcardoption;
    char* confstr;
    
    char* sampletimestr;
    char* blockid;
    
    packetmode = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PACKETMODE));
    recordmode = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_RECORDMODE));
    asciiheader = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIHEADER));  
    asciiformat = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_ASCIIFORMAT));  
    buffersize = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BUFFER_SIZE));
    
    gpio_port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_GPIO_PORT));
    gpio_sta_busy = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_GPIO_STA_BUSY));
    gpio_sta_success = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_GPIO_STA_SUCCESS));
    gpio_sta_error = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_GPIO_STA_ERROR));
    
    sdcardoption = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_SDCARD_OPTION));
    confstr =      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONFSTR));
    
    sampletimestr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    NOutputPara = 17; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "packetmode", packetmode,
            SSWRITE_VALUE_QSTR, "recordmode", recordmode,
            SSWRITE_VALUE_QSTR, "asciiheader", asciiheader,
            SSWRITE_VALUE_QSTR, "asciiformat", asciiformat,
            SSWRITE_VALUE_NUM, "recordcount", mxGetScalar(ssGetSFcnParam(S, ARGC_RECORDCOUNT)),
            SSWRITE_VALUE_NUM, "recordsize", mxGetScalar(ssGetSFcnParam(S, ARGC_RECORDSIZE)),
            SSWRITE_VALUE_NUM, "filesize", mxGetScalar(ssGetSFcnParam(S, ARGC_FILE_SIZE)),
            SSWRITE_VALUE_QSTR, "buffersize", buffersize,
            SSWRITE_VALUE_VECT, "inputtypearray", mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)), mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)),
            SSWRITE_VALUE_QSTR, "gpio_port", gpio_port,
            SSWRITE_VALUE_QSTR, "gpio_sta_busy", gpio_sta_busy,
            SSWRITE_VALUE_QSTR, "gpio_sta_success", gpio_sta_success,
            SSWRITE_VALUE_QSTR, "gpio_sta_error", gpio_sta_error,
            
            SSWRITE_VALUE_QSTR, "sdcardoption", sdcardoption,
            
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid
            )) {
        return; /* An error occurred which will be reported by SL */
    }
    
    if (!ssWriteRTWStrVectParam(S, "typestr", "[\"double\", \"float\", \"int8_t\", \"uint8_t\", \"int16_t\", \"uint16_t\", \"int32_t\", \"uint32_t\"]", 8)) {
        return;
    }
    
    if (!ssWriteRTWStrVectParam(S, "confstr", confstr, 2)) {
        return;
    }    
    
    mxFree(packetmode);
    mxFree(recordmode);
    mxFree(asciiheader);    
    mxFree(asciiformat);
    mxFree(buffersize);
    
    mxFree(gpio_port);
    mxFree(gpio_sta_busy);
    mxFree(gpio_sta_success);
    mxFree(gpio_sta_error);
    
    mxFree(sdcardoption);
    mxFree(confstr);
    
    mxFree(sampletimestr);
    mxFree(blockid);   
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_hssdcard_write.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_hssdcard_write.c"
#endif

