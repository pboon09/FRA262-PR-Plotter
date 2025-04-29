#define S_FUNCTION_NAME  waijung_sqlite
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    //ARGC_PORT = 0,
    ARGC_TRANSFER = 0,
    ARGC_STATEMENT,

    ARGC_FILENAME,
    ARGC_TABLENAME,
    ARGC_COLUMN,
    ARGC_CONDITION,
    
    ARGC_FILENAME_OPTION,
    ARGC_TABLENAME_OPTION,
    
    ARGC_INPUT_ARRAY,
    ARGC_OUTPUT_ARRAY,
    
    ARGC_PREVIEW,
    ARGC_COLCOUNT,
    
    ARGC_COLUMN_NAMES,
    ARGC_COLUMN_TYPES,
    
    ARGC_ENABLESTATUS,
    
    //ARGC_BUFFER_SIZE,
    
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define TYPEID_COUNT(S)  mxGetScalar(ssGetSFcnParam(S, ARGC_TYPEID_COUNT))
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
    
    
    /* Port count */
    input_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_ARRAY));
    output_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY));
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
    
    /* Port */
    for(k=0; k<output_count; k++) {
        ssSetOutputPortWidth(S, k, 1);
        ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_ARRAY)))[k]));
    }
    
    for(k=0; k<input_count; k++) {
        ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortWidth(S, k, 1);
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_ARRAY)))[k]));
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
    char *transfer;
    char *statement;
    char *filename;
    char *tablename;
    char *condition;
    char *filenameoption;
    char *tablenameoption;
    
    char *column;
    char *preview;
    char *columnnames;
    char *columntypes;
    char *blockid;
    
    /* Collect string */
    transfer = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TRANSFER));
    statement = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_STATEMENT));
    filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
    tablename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TABLENAME));
    condition = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_CONDITION));
    filenameoption = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME_OPTION));
    tablenameoption = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TABLENAME_OPTION));
    column = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_COLUMN));
    preview = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PREVIEW));
    columnnames = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_COLUMN_NAMES));
    columntypes = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_COLUMN_TYPES));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    NOutputPara = 13; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            /*SSWRITE_VALUE_NUM, "port", mxGetScalar(ssGetSFcnParam(S, ARGC_PORT)),*/
            SSWRITE_VALUE_QSTR, "transfer", transfer,
            SSWRITE_VALUE_QSTR, "filename", filename,
            SSWRITE_VALUE_QSTR, "tablename", tablename,
            SSWRITE_VALUE_QSTR, "condition", condition,
            SSWRITE_VALUE_QSTR, "filenameoption", filenameoption,
            SSWRITE_VALUE_QSTR, "tablenameoption", tablenameoption,
            SSWRITE_VALUE_QSTR, "statement", statement,
            SSWRITE_VALUE_QSTR, "column", column,
            SSWRITE_VALUE_QSTR, "preview", preview,
            SSWRITE_VALUE_NUM, "columncount", mxGetScalar(ssGetSFcnParam(S, ARGC_COLCOUNT)),
            SSWRITE_VALUE_NUM, "enablestatus", mxGetScalar(ssGetSFcnParam(S, ARGC_ENABLESTATUS)),            
            /*SSWRITE_VALUE_NUM, "buffersize", mxGetScalar(ssGetSFcnParam(S, ARGC_BUFFER_SIZE)),*/
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }              
    
    if (!ssWriteRTWStrVectParam(S, "columnnames", columnnames, (int)((double)mxGetScalar(ssGetSFcnParam(S, ARGC_COLCOUNT))))){
        return;
    }
    if (!ssWriteRTWStrVectParam(S, "columntypes", columntypes, (int)((double)mxGetScalar(ssGetSFcnParam(S, ARGC_COLCOUNT))))){
        return;
    }
    
    mxFree(transfer);
    mxFree(filename);
    mxFree(tablename);
    mxFree(condition);
    mxFree(filenameoption);
    mxFree(tablenameoption);    
    mxFree(statement);
    mxFree(column);
    mxFree(preview);
    mxFree(columnnames);
    mxFree(columntypes);
    mxFree(blockid);    
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_sqlite.c"
#endif

