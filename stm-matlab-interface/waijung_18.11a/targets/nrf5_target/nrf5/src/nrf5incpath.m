function root = nrf5incpath
	
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
			root = [fullfile(targetroot,'utils',[targetname '_lib_s332'],'inc') '; '];
		case 's132'
			root = [fullfile(targetroot,'utils',[targetname '_lib_s132'],'inc') '; '];
		otherwise
			root = [fullfile(targetroot,'utils',[targetname '_lib'],'inc') '; '];
	end
	