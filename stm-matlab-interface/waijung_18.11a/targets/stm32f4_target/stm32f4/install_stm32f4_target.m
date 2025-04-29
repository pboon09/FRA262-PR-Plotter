function install_stm32f4_target
    
    waijung.getTarget(waijung_targetname);
    disp('**********************************************')
    disp(['Installing ''' waijung_targetname ''' Target '])
    disp('**********************************************')
    disp('Adding path...')
    eval(['addpath(' waijung_targetname 'root)']);
    eval(['addpath(fullfile(' waijung_targetname 'root, ''src'', ''blocks''))']);
    eval(['addpath(fullfile(' waijung_targetname 'root, ''src''))']);
    eval(['addpath(fullfile(' waijung_targetname 'root, ''plugin''))']);
    savepath;
    sl_refresh_customizations; % refresh to update newly registered device
    disp('Adding path OK')
    
    disp('Install target specifics')
    
    disp('Automatically search and install ST Link Utility AND ST Link Driver')    
    % Checking and installing ST Link Utility AND ST Link Driver
    [stlinkavailable, stdriveravailable] = stm32f4.stLinkAvailable;
    
    if ~((stlinkavailable+stdriveravailable)==2)
        if ~stlinkavailable
            if ~stdriveravailable
                missing_tool_str = 'ST-Link Utility AND ST-Link Driver';
            else
                missing_tool_str = 'ST-Link Utility';
            end
        else
            if ~stdriveravailable
                missing_tool_str = 'ST-Link Driver';
            end
        end
                
        choice = questdlg(['It seems that ' missing_tool_str ' have NOT yet been installed on this computer. ' char([10 10]) ...
            'You should have ' missing_tool_str ' installed to take full advantage of Waijung and STM32F4 Target Auto Download capability.'  char([10 10]) ...
            'The most up-to-date version of ' missing_tool_str 'can be downloaded from http://www.st.com/web/en/catalog/tools/PF258168.' char([10 10])...
            'We recommend that you cancel this Waijung installation and manually install ' missing_tool_str ', before running Waijung installation again.' char([10 10])...
            ], ['Missing ' missing_tool_str ], ['Cancel this Waijung installation to install ' missing_tool_str ' first'], ['Proceed with Waijung installation without ' missing_tool_str ], ['Cancel this Waijung installation to install ' missing_tool_str ' first']);
        switch choice
            case ['Proceed with Waijung installation without ' missing_tool_str ]
                str = ['' missing_tool_str ' have NOT been installed properly.' char(10)];
                str = [str 'You should manually install ' missing_tool_str ', and run Waijung installation again to take full advantage of Waijung and STM32F4 Target Auto Download capability.'];
                waitfor(warndlg(str,'Warning','OK'));
                drawnow
            case ['Cancel this Waijung installation to install ' missing_tool_str ' first']
                str = ['Waijung Installation Aborted.' char(10)];
                error(str);
            otherwise
                error('Unknown choices.')
        end
    else
        stm32f4.setStLinkBinPath;
    end
    
    disp('Update related paths')
    targetroot = eval([waijung_targetname 'root']);
    eval(['xmldat = ''' targetroot '' filesep '' waijung_targetname '.wjdat'';'])
    if ~isempty(dir(xmldat))
        targetrootinfile = waijung_readxml(xmldat, '/target/path/root');
        if ~strcmp(targetrootinfile, targetroot)
            waijung_writexml(xmldat, '/target/path/root', targetroot, 'set');
        end
    end
    
    wjdatPath = fullfile(targetroot,[waijung_targetname '.wjdat']);
    disp('Setup paths for MDK-ARM ...')
    waijung.setMDKARM(wjdatPath, waijung_targetname);
    disp('Setup paths for EWARM ...')
    waijung.setEWARM(wjdatPath, waijung_targetname);
    disp('Setup paths for GNU-ARM ...')
    waijung.setGNUARM(wjdatPath, waijung_targetname);
    disp('Compilers'' path setup completed.')
    
    waijung.updatePlugin(waijung_targetname)
    disp(['''' waijung_targetname ''' Target installation completed successfully.'])
    