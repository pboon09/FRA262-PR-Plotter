/* Implement stm32f4_advanced_pwm.c */

#define S_FUNCTION_NAME  stm32f4_advanced_pwm	/* must have */
#define S_FUNCTION_LEVEL 2	/* must have */

#include "simstruc.h" 		/* must have */

#define N_PAR 22		/* Total number of block parameters */ /* must have */

enum {
    PWM_TIMER_ARGC = 0, /* timer */
    PWM_SETTING_ARGC, /* setting */
    PWM_PERIOD_ARGC, /* period */
    PWM_COUNTERMODE_ARGC, /* countermode */
    PWM_USE_DEADTIME_ARGC, /* use_deadtime */
    PWM_DEADTIME_ARGC, /* deadtime */
    PWM_LOCKLEVEL_ARGC, /* locklevel */
    PWM_BREAKINPUT_ARGC, /* breakinput */
    PWM_AUTO_OUTPUTENABLE_ARGC, /* automaticoutputenable */
    PWM_OP1_ARGC, /* op1 */
    PWM_OP1N_ARGC, /* op1n */
    PWM_OP2_ARGC, /* op2 */
    PWM_OP2N_ARGC, /* op2n */
    PWM_OP3_ARGC, /* op3 */
    PWM_OP3N_ARGC, /* op3n */
    PWM_OP4_ARGC, /* op4 */
    PWM_SAMPLETIME_ARGC,
    PWM_PERIOD_REG_ARGC,
    PWM_PRESC_REG_ARGC,
    PWM_DEADTIME_REG_ARGC,
    PWM_OUTPUT_STATE_ARGC,
    PWM_BLOCK_ID,
};
        
/*
 *  Default indexing:
 * Popup order 1,2,3,...
 * Checkbox True = 1, False = 0
 */


/* Facotr to consider scalar or vector
 * If scalar use: #define PWM_SETTING(S)  mxGetScalar(ssGetSFcnParam(S, PWM_SETTING_ARGC))
 * If vector use: #define ADC1_CH(S)  ssGetSFcnParam(S, ADC1_CH_ARGC)
 */
#define PWM_TIMER(S)                (int)mxGetScalar(ssGetSFcnParam(S, PWM_TIMER_ARGC))
#define PWM_SETTING(S)              (int)mxGetScalar(ssGetSFcnParam(S, PWM_SETTING_ARGC))
#define PWM_PERIOD(S)               (double)mxGetScalar(ssGetSFcnParam(S, PWM_PERIOD_ARGC))
#define PWM_ALIGNMODE(S)            (char*)mxArrayToString(ssGetSFcnParam(S, PWM_COUNTERMODE_ARGC))
#define PWM_USE_DEADTIME(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_USE_DEADTIME_ARGC))
#define PWM_DEADTIME(S)             (double)mxGetScalar(ssGetSFcnParam(S, PWM_DEADTIME_ARGC))
#define PWM_LOCKLEVEL(S)            (char*)mxArrayToString(ssGetSFcnParam(S, PWM_LOCKLEVEL_ARGC))
#define PWM_BREAKINPUT(S)           (int)mxGetScalar(ssGetSFcnParam(S, PWM_BREAKINPUT_ARGC))
#define PWM_AUTO_OUTPUTENABLE(S)    (int)mxGetScalar(ssGetSFcnParam(S, PWM_AUTO_OUTPUTENABLE_ARGC))
#define PWM_OUTPUT1_MODE(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP1_ARGC))
#define PWM_OUTPUT1N_MODE(S)        (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP1N_ARGC))
#define PWM_OUTPUT2_MODE(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP2_ARGC))
#define PWM_OUTPUT2N_MODE(S)        (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP2N_ARGC))
#define PWM_OUTPUT3_MODE(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP3_ARGC))
#define PWM_OUTPUT3N_MODE(S)        (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP3N_ARGC))
#define PWM_OUTPUT4_MODE(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_OP4_ARGC))
#define PWM_SAMPLETIME(S)           mxGetScalar(ssGetSFcnParam(S, PWM_SAMPLETIME_ARGC))
#define PWM_PERIOD_REG(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_PERIOD_REG_ARGC))
#define PWM_PRESC_REG(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_PRESC_REG_ARGC))
#define PWM_DEADTIME_REG(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_DEADTIME_REG_ARGC))
#define PWM_OUTPUT_STATE(S)         (int)mxGetScalar(ssGetSFcnParam(S, PWM_OUTPUT_STATE_ARGC))
#define BLOCKID(S)                  (char*)mxArrayToString(ssGetSFcnParam(S, PWM_BLOCK_ID))


#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))
#define IS_PARAM_SINGLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsSingle(pVal))
#define IS_PARAM_INT8(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsInt8(pVal))
#define IS_PARAM_INT16(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsInt16(pVal))
#define IS_PARAM_INT32(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsInt32(pVal))
#define IS_PARAM_UINT8(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsUint8(pVal))
#define IS_PARAM_UINT16(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsUint16(pVal))
#define IS_PARAM_UINT32(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsUint32(pVal))
#define IS_PARAM_BOOLEAN(pVal) (mxIsLogical(pVal) && !mxIsEmpty(pVal) &&\
!mxIsSparse(pVal))
#define IS_PARAM_CHAR(pVal)     (mxIsChar(pVal))

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
    int nCH, k;
    int param_index;
    
    ssSetNumSFcnParams(S, N_PAR);	/* Set and Check parameter count  */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < N_PAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
 
    /* Verify Parameters */
    for(param_index = 0; param_index < N_PAR; param_index++) {
        switch(param_index) {
            /* Char */
            case PWM_COUNTERMODE_ARGC:
            case PWM_LOCKLEVEL_ARGC:
            case PWM_BLOCK_ID:                
                if(!IS_PARAM_CHAR(ssGetSFcnParam(S, param_index))) {
                    mexPrintf("Parameter %d is invalid!\n", param_index+1);
                    ssSetErrorStatus(S, "Invalid parameter type!");
                    return; 
                }
                break;                
                /* Double */
            case PWM_TIMER_ARGC:                
            case PWM_SETTING_ARGC:
            case PWM_PERIOD_ARGC:
            case PWM_USE_DEADTIME_ARGC:
            case PWM_DEADTIME_ARGC:
            case PWM_BREAKINPUT_ARGC:
            case PWM_AUTO_OUTPUTENABLE_ARGC:
            case PWM_OP1_ARGC:
            case PWM_OP1N_ARGC:
            case PWM_OP2_ARGC:
            case PWM_OP2N_ARGC:
            case PWM_OP3_ARGC:
            case PWM_OP3N_ARGC:
            case PWM_OP4_ARGC:
            case PWM_SAMPLETIME_ARGC:
            case PWM_PERIOD_REG_ARGC:
            case PWM_PRESC_REG_ARGC:
            case PWM_DEADTIME_REG_ARGC:
            case PWM_OUTPUT_STATE_ARGC:
                if(!IS_PARAM_DOUBLE(ssGetSFcnParam(S, param_index))) {
                    mexPrintf("Parameter %d is invalid!\n", param_index+1);
                    ssSetErrorStatus(S, "Invalid parameter type!");
                    return; 
                }
                break;
            default:
                ssSetErrorStatus(S, "Invalid parameter!");
                break;
        }
    }
    
    /* Calculate number of input port */
    nCH = 0;
    if((PWM_OUTPUT1_MODE(S) > 1) || (PWM_OUTPUT1N_MODE(S) > 1)) nCH++;
    if((PWM_OUTPUT2_MODE(S) > 1) || (PWM_OUTPUT2N_MODE(S) > 1)) nCH++;
    if((PWM_OUTPUT3_MODE(S) > 1) || (PWM_OUTPUT3N_MODE(S) > 1)) nCH++;
    if(PWM_OUTPUT4_MODE(S) > 1) nCH++;
    
    /* Number of input ports */
    if (!ssSetNumInputPorts(S, nCH))
        return; 
    
    for (k = 0; k < nCH; k++) {
        ssSetInputPortDataType(S, k, DYNAMICALLY_TYPED);
        /* DYNAMICALLY_TYPED allows the input to be inherited ()
         * determined at run time.
         */        
        ssSetInputPortWidth(S, k, 1);
        /* !!! */
        ssSetInputPortDirectFeedThrough(S, k, 1);
    }
    
    /* Number of output ports */
    if (!ssSetNumOutputPorts(S, 0))
        return; 
    
    /* sample times */
    ssSetNumSampleTimes(S, 1);
    
    /* options */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, PWM_SAMPLETIME(S));
} /* end mdlInitializeSampleTimes */

#define MDL_ENABLE
#if defined(MDL_ENABLE) && defined(MATLAB_MEX_FILE) 
void mdlEnable(SimStruct *S){
}
#endif 

#define MDL_DISABLE
#if defined(MDL_DISABLE) && defined(MATLAB_MEX_FILE) 
static void mdlDisable(SimStruct *S)
{ 
} 
#endif 

static void mdlOutputs(SimStruct *S, int_T tid) {
    
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
    
    uint32_T timer_module;
    uint32_T pwm_period;
    uint32_T pwm_presc;
    uint32_T pwm_deadtime_val;    
    char break_input_buff[128];
    char break_polarity_buff[128];
    char auto_outenable_buffer[128];
    char output_state_buffer[128];
    char output_n_state_buffer[128];
    uint32_T ch1_enabled;
    uint32_T ch2_enabled;
    uint32_T ch3_enabled;
    uint32_T ch4_enabled;
    char output1_polarity_buffer[128];
    char output1_idle_buffer[128];
    char output1_n_polarity_buffer[128];
    char output1_n_idle_buffer[128];
    char output2_polarity_buffer[128];
    char output2_idle_buffer[128];
    char output2_n_polarity_buffer[128];
    char output2_n_idle_buffer[128];    
    char output3_polarity_buffer[128];
    char output3_idle_buffer[128];
    char output3_n_polarity_buffer[128];
    char output3_n_idle_buffer[128];    
    char output4_polarity_buffer[128];
    char output4_idle_buffer[128];
    char output4_n_polarity_buffer[128];
    char output4_n_idle_buffer[128];    
    
    /* Timer module index */
    if(PWM_TIMER(S) == 1)
        timer_module = 1;
    else
        timer_module = 8;    
    /* Timer Period register */
    pwm_period = PWM_PERIOD_REG(S);
    /* Timer Prescale register */
    pwm_presc = PWM_PRESC_REG(S);
    /* Alignment mode */
    if(PWM_USE_DEADTIME(S))
        pwm_deadtime_val = (uint32_T)PWM_DEADTIME_REG(S);
    else
        pwm_deadtime_val = 0;
    /* Break input */
    if(PWM_BREAKINPUT(S) == 1) { // Disable
        sprintf(break_input_buff, "%s", "TIM_Break_Disable");
        sprintf(break_polarity_buff, "%s", "TIM_BreakPolarity_High");
    }
    else if(PWM_BREAKINPUT(S) == 2) { // Enable, polarity High
        sprintf(break_input_buff, "%s", "TIM_Break_Enable");
        sprintf(break_polarity_buff, "%s", "TIM_BreakPolarity_High");        
    }
    else {
        sprintf(break_input_buff, "%s", "TIM_Break_Enable");
        sprintf(break_polarity_buff, "%s", "TIM_BreakPolarity_Low");                
    }
    /* Auto output enable */
    if(PWM_AUTO_OUTPUTENABLE(S) == 1) // OFF
        sprintf(auto_outenable_buffer, "%s", "TIM_AutomaticOutput_Disable");
    else // ON
        sprintf(auto_outenable_buffer, "%s", "TIM_AutomaticOutput_Enable");
    /* Output state */
    switch(PWM_OUTPUT_STATE(S))
    {
        case 1: // Output state enable
            sprintf(output_state_buffer, "%s", "TIM_OutputState_Enable");
            sprintf(output_n_state_buffer, "%s", "TIM_OutputNState_Disable");
            break;
            
        case 2: // Output N state enable
            sprintf(output_state_buffer, "%s", "TIM_OutputState_Disable");
            sprintf(output_n_state_buffer, "%s", "TIM_OutputNState_Enable");
            break;
            
        case 3: // Both enable
        default:
            sprintf(output_state_buffer, "%s", "TIM_OutputState_Enable");
            sprintf(output_n_state_buffer, "%s", "TIM_OutputNState_Enable");            
            break;
    }    
    /* Ch 1 enabled */
    if((PWM_OUTPUT1_MODE(S) > 1) || (PWM_OUTPUT1N_MODE(S) > 1)) ch1_enabled = 1;
        else ch1_enabled = 0;
    /* Ch 2 enabled */
    if((PWM_OUTPUT2_MODE(S) > 1) || (PWM_OUTPUT2N_MODE(S) > 1)) ch2_enabled = 1;
        else ch2_enabled = 0;
    /* Ch 3 enabled */
    if((PWM_OUTPUT3_MODE(S) > 1) || (PWM_OUTPUT3N_MODE(S) > 1)) ch3_enabled = 1;
        else ch3_enabled = 0;
    /* Ch 4 enabled */
    if(PWM_OUTPUT4_MODE(S) > 1) ch4_enabled = 1;
        else ch4_enabled = 0;    
    /* Output1 polarity/idle state */
    switch(PWM_OUTPUT1_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output1_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output1_idle_buffer, "%s", "TIM_OCIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output1_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output1_idle_buffer, "%s", "TIM_OCIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output1_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output1_idle_buffer, "%s", "TIM_OCIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output1_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output1_idle_buffer, "%s", "TIM_OCIdleState_Reset");             
            break;
    }
    /* Output1 n polarity/idle state */
    switch(PWM_OUTPUT1N_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output1_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output1_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output1_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output1_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output1_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output1_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output1_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output1_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");             
            break;
    }
    /* Output2 polarity/idle state */
    switch(PWM_OUTPUT2_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output2_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output2_idle_buffer, "%s", "TIM_OCIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output2_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output2_idle_buffer, "%s", "TIM_OCIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output2_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output2_idle_buffer, "%s", "TIM_OCIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output2_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output2_idle_buffer, "%s", "TIM_OCIdleState_Reset");             
            break;
    }
    /* Output2 n polarity/idle state */
    switch(PWM_OUTPUT2N_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output2_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output2_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output2_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output2_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output2_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output2_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output2_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output2_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");             
            break;
    }
    /* Output3 polarity/idle state */
    switch(PWM_OUTPUT3_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output3_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output3_idle_buffer, "%s", "TIM_OCIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output3_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output3_idle_buffer, "%s", "TIM_OCIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output3_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output3_idle_buffer, "%s", "TIM_OCIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output3_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output3_idle_buffer, "%s", "TIM_OCIdleState_Reset");             
            break;
    }
    /* Output3 n polarity/idle state */
    switch(PWM_OUTPUT3N_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output3_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output3_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output3_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output3_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output3_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output3_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output3_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output3_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");             
            break;
    }
 /* Output4 polarity/idle state */
    switch(PWM_OUTPUT4_MODE(S)) {
        case 1: // Automatic
        case 2: // Polarity HIGH and SET when idle
            sprintf(output4_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output4_idle_buffer, "%s", "TIM_OCIdleState_Set");            
            sprintf(output4_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output4_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");            
            break;
        case 3: // Polarity HIGH and RESET when idle
            sprintf(output4_polarity_buffer, "%s", "TIM_OCPolarity_High");
            sprintf(output4_idle_buffer, "%s", "TIM_OCIdleState_Reset");              
            sprintf(output4_n_polarity_buffer, "%s", "TIM_OCNPolarity_High");
            sprintf(output4_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");              
            break;
        case 4: // Polarity LOW and SET when idle
            sprintf(output4_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output4_idle_buffer, "%s", "TIM_OCIdleState_Set");             
            sprintf(output4_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output4_n_idle_buffer, "%s", "TIM_OCNIdleState_Set");             
            break;
        case 5: //Polarity LOW and RESET when idle
            sprintf(output4_polarity_buffer, "%s", "TIM_OCPolarity_Low");
            sprintf(output4_idle_buffer, "%s", "TIM_OCIdleState_Reset");             
            sprintf(output4_n_polarity_buffer, "%s", "TIM_OCNPolarity_Low");
            sprintf(output4_n_idle_buffer, "%s", "TIM_OCNIdleState_Reset");             
            break;
    }
    
    NOutputPara = 32;
    if (!ssWriteRTWParamSettings(S, NOutputPara, 
            /* timer_module */
            SSWRITE_VALUE_DTYPE_NUM, "timer_module",
            &timer_module, DTINFO(SS_UINT32, COMPLEX_NO),
            /* pwm_period */
            SSWRITE_VALUE_DTYPE_NUM, "pwm_period",
            &pwm_period, DTINFO(SS_UINT32, COMPLEX_NO),
            /* pwm_presc */
            SSWRITE_VALUE_DTYPE_NUM, "pwm_presc",
            &pwm_presc, DTINFO(SS_UINT32, COMPLEX_NO),
            /* Alignment mode */
            SSWRITE_VALUE_QSTR, "pwm_align_mode", PWM_ALIGNMODE(S),
            /* Dead time */
            SSWRITE_VALUE_DTYPE_NUM, "pwm_deadtime_val",
            &pwm_deadtime_val, DTINFO(SS_UINT32, COMPLEX_NO),
            /* Lock level */
            SSWRITE_VALUE_QSTR, "pwm_lock_level", PWM_LOCKLEVEL(S),
            /* Break Input */
            SSWRITE_VALUE_QSTR, "pwm_break_input", (char*)break_input_buff,
            /* Break Input Polarity */
            SSWRITE_VALUE_QSTR, "pwm_break_polarity", (char*)break_polarity_buff,
            /* Auto output enable */
            SSWRITE_VALUE_QSTR, "pwm_auto_outenable", (char*)auto_outenable_buffer,
            /* Output state */            
            SSWRITE_VALUE_QSTR, "output_state_buffer", (char*)output_state_buffer,
            /* Output N state */
            SSWRITE_VALUE_QSTR, "output_n_state_buffer", (char*)output_n_state_buffer,
            /* Ch1 enable */
            SSWRITE_VALUE_DTYPE_NUM, "ch1_enabled",
            &ch1_enabled, DTINFO(SS_UINT32, COMPLEX_NO),            
            /* Ch2 enable */
            SSWRITE_VALUE_DTYPE_NUM, "ch2_enabled",
            &ch2_enabled, DTINFO(SS_UINT32, COMPLEX_NO),            
            /* Ch3 enable */
            SSWRITE_VALUE_DTYPE_NUM, "ch3_enabled",
            &ch3_enabled, DTINFO(SS_UINT32, COMPLEX_NO),            
            /* Ch4 enable */
            SSWRITE_VALUE_DTYPE_NUM, "ch4_enabled",
            &ch4_enabled, DTINFO(SS_UINT32, COMPLEX_NO),
            /* Output1 polarity/idle */
            SSWRITE_VALUE_QSTR, "output1_polarity", (char*)output1_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output1_idle", (char*)output1_idle_buffer,
            /* Output1 n polarity/idle */
            SSWRITE_VALUE_QSTR, "output1_n_polarity", (char*)output1_n_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output1_n_idle", (char*)output1_n_idle_buffer,
            /* Output2 polarity/idle */
            SSWRITE_VALUE_QSTR, "output2_polarity", (char*)output2_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output2_idle", (char*)output2_idle_buffer,
            /* Output2 n polarity/idle */
            SSWRITE_VALUE_QSTR, "output2_n_polarity", (char*)output2_n_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output2_n_idle", (char*)output2_n_idle_buffer,
            /* Output3 polarity/idle */
            SSWRITE_VALUE_QSTR, "output3_polarity", (char*)output3_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output3_idle", (char*)output3_idle_buffer,
            /* Output3 n polarity/idle */
            SSWRITE_VALUE_QSTR, "output3_n_polarity", (char*)output3_n_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output3_n_idle", (char*)output3_n_idle_buffer,
            /* Output4 polarity/idle */
            SSWRITE_VALUE_QSTR, "output4_polarity", (char*)output4_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output4_idle", (char*)output4_idle_buffer,
            /* Output4 n polarity/idle */
            SSWRITE_VALUE_QSTR, "output4_n_polarity", (char*)output4_n_polarity_buffer,
            SSWRITE_VALUE_QSTR, "output4_n_idle", (char*)output4_n_idle_buffer,
            /* Block ID */
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
#error "Attempted use non-inlined S-function stm32f4_advanced_pwm.c"
#endif
