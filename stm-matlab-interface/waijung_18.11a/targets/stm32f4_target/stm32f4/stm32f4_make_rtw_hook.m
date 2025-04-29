function stm32f4_make_rtw_hook(hookMethod,modelName,rtwroot,templateMakefile,buildOpts,buildArgs)
% Based on ert_make_rtw_hook.m
switch hookMethod
    case 'error'
        % Called if an error occurs anywhere during the build.  If no error occurs
        % during the build, then this hook will not be called.  Valid arguments
        % at this stage are hookMethod and modelName. This enables cleaning up
        % any static or global data used by this hook file.
        disp(['### Real-Time Workshop build procedure for model: ''' modelName...
            ''' aborted due to an error.']);
    case 'entry'
        % Called at start of code generation process (before anything happens.)
        % Valid arguments at this stage are hookMethod, modelName, and buildArgs.
        disp(sprintf(['\n### Starting Real-Time Workshop build procedure for ', ...
            'model: %s'],modelName)); %#ok<DSPS>
        stm32f4_entry_hook(modelName);
    case 'before_tlc'
        % Called just prior to invoking TLC Compiler (actual code generation.)
        % Valid arguments at this stage are hookMethod, modelName, and
        % buildArgs
        stm32f4_before_tlc_hook(modelName);
    case 'after_tlc'
        % Called just after to invoking TLC Compiler (actual code generation.)
        % Valid arguments at this stage are hookMethod, modelName, and
        % buildArgs
        stm32f4_after_tlc_hook(modelName);
    case 'before_make'
        % Called after code generation is complete, and just prior to kicking
        % off make process (assuming code generation only is not selected.)  All
        % arguments are valid at this stage.
        stm32f4_before_make_hook(modelName);
    case 'after_make'
        % Called after make process is complete. All arguments are valid at
        % this stage.
        stm32f4_after_make_hook(modelName);
    case 'exit'
        % Called at the end of the RTW build process.  All arguments are valid
        % at this stage.
        stm32f4_exit_hook(modelName);
        disp(['### Successful completion of Real-Time Workshop build ',...
            'procedure for model: ', modelName]);        
end

