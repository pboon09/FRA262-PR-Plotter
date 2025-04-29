function uninstall_waijung(option)
    
    % Usage:
    % uninstall_waijung
    %
    % or
    % uninstall_waijung('hideprogress')
    
    hideprogress = 0;
    if (nargin > 0)
        if strcmp(option,'hideprogress')
            hideprogress = 1;
        end
    end
    
    p = regexp(path,';','split');
    
    path2be_remove_cell = {};
    for idx = 1:length(p)
        % Search for directories who name contain string in the form "targets\*_target"
        [str_start,str_end] = regexp(p{idx},'targets\\\w+_target');
        if ~isempty(str_start)
            % Use only waijung root directory
            path2be_remove = p{idx}(1:str_start-2);
            if isempty(strmatch(path2be_remove, path2be_remove_cell, 'exact'))
                path2be_remove_cell{end+1,1} = path2be_remove;
            end
        end
    end
    
    toberemovecells = [];
    for idx = 1:length(path2be_remove_cell)
        if isempty(toberemovecells)
            toberemovecells = ~cellfun(@isempty,strfind(p,path2be_remove_cell{idx}));
        else
            toberemovecells = toberemovecells + ~cellfun(@isempty,strfind(p,path2be_remove_cell{idx}));
        end
    end
    
    for idx = 1:length(p)
        if ~isempty(toberemovecells)
            if toberemovecells(idx)
                if ~hideprogress
                    disp(['Removing path: ' p{idx}]);
                end
                rmpath(p{idx})
            end
        end
    end
    savepath
    
    sl_refresh_customizations;
    if ~isempty(idx)
        if ~hideprogress
            disp('Waijung unintallation complete.')
        end
    else
        if ~hideprogress
            disp('No Waijung installation found in the system.')
        end
    end
end
