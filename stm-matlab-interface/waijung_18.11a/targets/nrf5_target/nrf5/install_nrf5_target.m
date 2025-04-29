function install_nrf5_target
    
    waijung.getTarget(waijung_targetname);
    waijung.addInstallPath(waijung_targetname);
    
    disp('Install target specifics')
       
    disp('Update related paths')
    % Moved here to optimize for time.
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
    
    %waijung.updatePlugin(waijung_targetname)
    %disp(['''' packagename ''' Target installation completed successfully.'])
    