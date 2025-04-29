#define S_FUNCTION_NAME  waijung_sqliteconnstr
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARGC_STATEMENT = 0,
    ARGC_CONDITION,
    ARGC_FILENAME,
    ARGC_TABLENAME,
    
    ARGC_COLUMN_NAMES,
    ARGC_COLUMN_TYPES,
    ARGC_TYPEID_ARRAY,
    ARGC_TYPEID_COUNT,
    
    ARGC_PORT,
    ARGC_QUERY_FORMAT,
    ARGC_DEFAULT_QUERY_FORMAT,
    ARGC_SCANF_FORMAT,
    
    ARGC_SAMPLETIME,
    ARGC_SAMPLETIMESTR,
    ARGC_BLOCKID,
    
    __PARAM_COUNT
};

#define TYPEID_COUNT(S)  mxGetScalar(ssGetSFcnParam(S, ARGC_TYPEID_COUNT))
#define SAMPLETIME(S)    mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */
#define SAMPLETIMESTR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIMESTR)) /* Compiled sample time (sec) in string */
#define BLOCKID(S)      (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID)) /* BlockID */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    //int NPorts;
    int INPortCount;
    int OUTPortCount;
    
    /* Statement */
    char statement_str[16];
    char *statement_ptr;
    
    /* Parameter validatone */    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* 
     * Copy string 
     */
    statement_ptr = (char*)mxArrayToString(ssGetSFcnParam(S,ARGC_STATEMENT));
    strcpy(statement_str, statement_ptr);
    mxFree(statement_ptr);
    
    if(!strcmp(statement_str, "SELECT")) {
        INPortCount = 0;
        OUTPortCount = (int)(((double*)mxGetPr(ssGetSFcnParam(S,ARGC_TYPEID_ARRAY)))[0]); //(int)mxGetScalar(ssGetSFcnParam(S, ARGC_TYPEID_COUNT));
    }
    else { /* INSERT, UPDATE */
        INPortCount = (int)(((double*)mxGetPr(ssGetSFcnParam(S,ARGC_TYPEID_ARRAY)))[0]); //(int)mxGetScalar(ssGetSFcnParam(S, ARGC_TYPEID_COUNT));
        OUTPortCount = 0;
    }
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, INPortCount)) return; /* Number of input ports */
    for(k=0; k<INPortCount; k++) {
        ssSetInputPortDirectFeedThrough(S, k, 1);
        ssSetInputPortWidth(S, k, 1);
        ssSetInputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S,ARGC_TYPEID_ARRAY)))[k+1]));
    }
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, OUTPortCount)) return; /* Number of output ports */
    for(k=0; k<OUTPortCount; k++) {
        ssSetOutputPortWidth(S, k, 1);
        ssSetOutputPortDataType(S, k, (int)(((double*)mxGetPr(ssGetSFcnParam(S,ARGC_TYPEID_ARRAY)))[k+1]));
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
    char *port;
    char *filename;
    char *tablename;
    char *statement;
    char *columnnames;
    char *columntypes;
    char *queryformat;
    char *defaultqueryformat;
    char *scanfformat;
    
    char *blockid;    
    
    /* Collect string */
    port = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_PORT));    
    filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
    tablename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_TABLENAME));
    statement = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_STATEMENT));
    columnnames = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_COLUMN_NAMES));
    columntypes = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_COLUMN_TYPES));
    queryformat = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_QUERY_FORMAT));
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    defaultqueryformat = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_DEFAULT_QUERY_FORMAT));
    scanfformat = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_SCANF_FORMAT));

    NOutputPara = 11; /* Number of parameters to output to model.rtw */
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "port", port,
            SSWRITE_VALUE_QSTR, "filename", filename,
            SSWRITE_VALUE_QSTR, "tablename", tablename,
            SSWRITE_VALUE_QSTR, "statement", statement,
            SSWRITE_VALUE_QSTR, "queryformat", queryformat,
            SSWRITE_VALUE_QSTR, "defaultqueryformat", defaultqueryformat,
            SSWRITE_VALUE_QSTR, "scanfformat", scanfformat,
            SSWRITE_VALUE_NUM, "typeidcount", TYPEID_COUNT(S),
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_NUM, "sampletimestr", SAMPLETIMESTR(S),
            SSWRITE_VALUE_QSTR, "blockid", BLOCKID(S)
            )) {
        return; /* An error occurred which will be reported by SL */
    }              
    
    if (!ssWriteRTWStrVectParam(S, "columnnames", columnnames, (int)((double)TYPEID_COUNT(S)))){
        return;
    }
    if (!ssWriteRTWStrVectParam(S, "columntypes", columntypes, (int)((double)TYPEID_COUNT(S)))){
        return;
    }
    
    mxFree(port);
    mxFree(filename);
    mxFree(tablename);
    mxFree(statement);
    mxFree(columnnames);
    mxFree(columntypes);
    mxFree(queryformat);
    mxFree(defaultqueryformat);
    mxFree(scanfformat);
    mxFree(blockid);    
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_cansetup.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function waijung_sqliteconnstr.c"
#endif

