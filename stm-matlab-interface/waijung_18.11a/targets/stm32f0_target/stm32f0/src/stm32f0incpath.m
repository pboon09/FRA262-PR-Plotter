function root = stm32f0incpath
    
	root = [fullfile(stm32f0root,'utils','STM32F0xx_StdPeriph_Lib','Libraries','STM32F0xx_StdPeriph_Driver','inc') '; '];
    %     root = [fullfile(stm32f0root,'utils','stm32f0xx_DSP_StdPeriph_Lib_V1.0.1','Libraries','stm32f0xx_StdPeriph_Driver','inc') '; '...
    %         fullfile(matlabroot,'toolbox','rtw','targets','common','profile','execution')];