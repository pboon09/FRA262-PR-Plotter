function root = nrf5srcpath
	
	targetname = waijung.getTarget;
	targetroot = eval([targetname 'root']);
	
	targetsetup = waijung.getTargetSetup;
	if isempty(targetsetup)
		softdevice = '';
	else
		softdevice = targetsetup.softdevice;
	end		
	switch softdevice
		case 'S332'
			root = [fullfile(targetroot,'utils',[targetname '_lib_s332'],'src') '; '];
		case 'S132'
			root = [fullfile(targetroot,'utils',[targetname '_lib_s132'],'src') '; '];
		otherwise
			root = [fullfile(targetroot,'utils',[targetname '_lib'],'src') '; '];
	end
	