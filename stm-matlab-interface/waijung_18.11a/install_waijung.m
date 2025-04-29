function install_waijung
    
    clc
    disp('Pre-Installation')
    disp(['Host computer: ' computer])
    osversion = evalc('!ver');
    osversion(osversion==10) = []; % remove newline characters
    disp(['Operating System: ' osversion])
    disp('Checking previous Waijung installation (if any)...')
    uninstall_waijung % if any previous installation exists
    current_dir = pwd;
    try
        cd('src')
    catch
        str = ['Can not find ''src'' folder.' char(10) ...
            'Make sure that:' char(10)...
            '1. You have extracted the downloaded archive (*.7z).' char(10)...
            '2. Matlab ''Current Directory'' is the extracted folder.' char(10)...
            '3. Run install_waijung.m from the extrated folder and NOT from the archive.' char(10)...
            'Abort installation.'];
        error(str)
    end
    
    disp('Checking Matlab...')
    if (str2double(waijung.getMatlab.year) < 2009)
        error('Waijung needs Matlab R2009a or later.')
    else
        disp(['Matlab release: ' waijung.getMatlab.release '. OK.'])
    end
    
    disp('Checking MDK-ARM...')
    mdkarm = waijung.checkMDKARM;
    found_mdk = ~isempty(mdkarm.version);
    if found_mdk
        if (str2double(mdkarm.version) < 4.6)
            warning(['Waijung needs MDK-ARM version 4.6 or later. Your current MDK-ARM installation is version ' mdkarm.version ' at root: ' mdkarm.root '.'])
        else
            disp(['Found MDK-ARM version ' mdkarm.version ' installed in: ' mdkarm.root '. OK.'])
        end
    else
        disp('No KEIL MDK-ARM found on the system.')
    end
    
    disp('Checking EWARM...')
    ewarm = waijung.checkEWARM;
    found_ewarm = ~isempty(ewarm.root);
    if found_ewarm
        disp(['Found EWARM installation in: ''' ewarm.root '''.']);
    else
        disp('No IAR EWARM installation found on the system.');
    end
    
    % To install properly, Matlab must be openned with Administrator Privilege.
    disp('Installing ''Waijung'' ...')
    disp('Adding path...')
    addpath(waijungroot);
    addpath(fullfile(waijungroot, 'src', 'blocks'));
    addpath(fullfile(waijungroot, 'src'));
    savepath;
    disp('Adding path OK')
    sl_refresh_customizations; % refresh to update newly registered device
    waijung.refreshLibraryBrowser;
    disp('''Waijung'' installation completed successfully.')
    
	% House keeping	
	file1 = fullfile(waijungroot,'src','blocks','waijung_profiler.tlc');
	if ~isempty(dir(file1))
		disp(['Detected obsolete file:' file1])
		delete(file1);
		disp(['Remove obsolete file completed:' file1])
	end
	
    % Install all available targets
    targets = waijung.listTarget;
    for idx = 1:length(targets)
        target = char(targets{idx});
        target_install_dir = fullfile(waijungroot,'targets',[target '_target'],target);
        cd(target_install_dir)
        eval(['install_' target '_target']);
    end
    cd(current_dir);
    
    choice = questdlg(['Waijung can periodically check online for updates automatically.' char(10) ...
        'Would you like to enable this feature?' char(10)],...
        'Enable Automatic Update Checking?', 'Yes', 'No', 'Yes');
    switch choice
        case 'Yes'
            waijung.enableAutomaticCheckForUpdate;
        case 'No'
            waijung.disableAutomaticCheckForUpdate;
        otherwise
            % do nothing
    end
    disp('Finish Waijung Installation.')
end
