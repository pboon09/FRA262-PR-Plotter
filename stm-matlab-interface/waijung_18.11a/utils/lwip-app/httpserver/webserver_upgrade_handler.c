
#include "waijung_hwdrvlib.h"
#include "webserver_mem_handler.h"
#include "webserver_debug_port.h"
#include "stm32f4xx_flash.h"
#include "amg_utils.h"
#include "ff.h"

/*
	Sector 0 	0x0800 0000 - 0x0800 3FFF 16 	Kbyte
	Sector 1 	0x0800 4000 - 0x0800 7FFF 16 	Kbyte
	Sector 2 	0x0800 8000 - 0x0800 BFFF 16 	Kbyte
	Sector 3 	0x0800 C000 - 0x0800 FFFF 16 	Kbyte
	Sector 4 	0x0801 0000 - 0x0801 FFFF 64 	Kbyte
	Sector 5 	0x0802 0000 - 0x0803 FFFF 128	Kbyte
	Sector 6 	0x0804 0000 - 0x0805 FFFF 128	Kbyte
	Sector 11	0x080E 0000 - 0x080F FFFF 128	Kbyte
*/

void webserver_virtualdisk_unlock(void)
{
	WEBSERVER_DEBUG_PRINT("Unlock v_disk.");
	FLASH_Unlock(); 
}

/* Erase flash memory in Virtual disk sector.
*/
void webserver_virtualdisk_erase(uint16_t key1, uint32_t key2)
{
	WEBSERVER_DEBUG_PRINT("Erasing virtual disk sector...");
	
	/* Use protect key like old programming style */
	if(key1 == 0xAA55) {
		if(key2 == 0x55553333) {			
			
			/* Sector 7-11 (640kBytes) */			
			/*FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); */ /* 128 Kbyte */
			FLASH_EraseSector(FLASH_Sector_8, VoltageRange_3); /* 128 Kbyte */
			FLASH_EraseSector(FLASH_Sector_9, VoltageRange_3); /* 128 Kbyte */
			FLASH_EraseSector(FLASH_Sector_10, VoltageRange_3); /* 128 Kbyte */
			FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3); /* 128 Kbyte */
			
			WEBSERVER_DEBUG_PRINT("Done.");
			return;
		}
	}
	WEBSERVER_DEBUG_PRINT("Fail.");
}

uint8_t webserver_virtualdisk_write(uint32_t Offser, uint32_t* Data ,uint16_t DataLength)
{
	uint32_t i;
	uint32_t FlashAddress;

	FlashAddress = (uint32_t)VIRTUAL_DISK_START+Offser;	
	
	WEBSERVER_DEBUG_PRINT("Write v_disk addr: %X", FlashAddress);
	
	for(i=0; i<DataLength; i++) {
		/* Address is out of range */
		if(FlashAddress >= VIRTUAL_DISK_END) {
			WEBSERVER_DEBUG_PRINT("Out of range.");
			return 3;
		}
		/* Program Word */
		if (FLASH_ProgramWord(FlashAddress, Data[i]) == FLASH_COMPLETE)
		{
			/* Verify */
			if (*(uint32_t*)FlashAddress != Data[i]) {
				WEBSERVER_DEBUG_PRINT("Verify fail.");
				return 2;
			}
			/* Address increment by 4 */
			FlashAddress += 4;
		}
		else {
			WEBSERVER_DEBUG_PRINT("Write fail.");
			return 1;
		}
	}
	return 0;
}

/* Read/ Write IAP configuration file */
extern int SYSTEM_Handle_CreateIAPConf(int target, const char *filepath, int enable, int status);
extern int SYSTEM_Handle_ReadIAPConf(char *filepath, int *target, int *enable, int *status);

const char *ini_upgrade_group = "[Upgrade]\r\n";
const char *ini_upgrade_enable = "Enable=1\r\n";
const char *ini_upgrade_disable = "Enable=0\r\n";
uint8_t webserver_setupgrade_tofile(char *filename, int target, uint8_t enbale)
{
	if(SYSTEM_Handle_CreateIAPConf(target, filename, enbale, -1) == 0)
		return 0; /* Success */
	else
		return 1; /* Fail */
}

void webserver_getupgrade_fromfile(char *filename_buffer, uint8_t *enable)
{
	int tmp_enable = 0;
	int tmp_status = -1;
	int tmp_target = -1;
	
	if(SYSTEM_Handle_ReadIAPConf(filename_buffer, &tmp_target, &tmp_enable, &tmp_status) == 0) { /* Success */
		if(tmp_target == 1) /* W2d */
			*enable = (uint8_t)(tmp_enable==1);
		else
			*enable = 0;
	}
	else { /* Fail */
		*enable = 0;
		filename_buffer[0] = '\0';
	}
}

static uint8_t webserver_upgrade_status = 0;
void webserver_upgrade_activate(uint8_t activate)
{
  webserver_upgrade_status = activate;
}

static SYS_TIMER_STRUCT main_upgrade_program_timer = {0,0};
const char system_upgrade_fail[] = "<html><body><b><font color=\"Red\">System upgrade is failed.</font></b><p><a href=\"/\">home</a></body></html>";
const char system_upgrade_pass[] = "<html><body><b><font color=\"Green\">System upgrade is success.</font></b><p><a href=\"/\">home</a></body></html>";
void webserver_upgrade_check(void)
{
	uint32_t offset;
	uint32_t *write_buffer;
	UINT bytes_reading;
	uint8_t enable;
	uint8_t sta;
	uint8_t need_reboot;
	char *filename;
	FIL *f;

	need_reboot = 0;
	
	/* Poll System main application upgrade */
	if(webserver_upgrade_status == 0xBB) {		
		webserver_upgrade_status = 0x55;
		SysTimer_Start(&main_upgrade_program_timer, 3000); /* 3Seconds */
	}
	if(webserver_upgrade_status == 0x55) {
		if(SysTimer_IsTimeout(&main_upgrade_program_timer)) {
			webserver_upgrade_status = 0x00;			
			
			WEBSERVER_DEBUG_PRINT("\r\n***************************\r\nReboot...\r\n***************************");
			WEBSERVER_DEBUG_PRINT("");
			/* Enable WWDG clock */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
			WWDG->CR = 0x80; /* Generate Software reset */			
		}
	}
	

	/* Poll upgrade status */
	if(webserver_upgrade_status == 0xAA) {
		webserver_upgrade_status = 0x00;

		/* Check upgrade file */
		filename = (char*)_memtiny_alloc(); /* _MEMSMALL_SIZE (64). IMPORTANT: Must be free with _memtiny_free() */
		webserver_getupgrade_fromfile(filename, &enable);
		if(enable && *filename) {
			/* Clear system upgrade status */
			if(webserver_setupgrade_tofile(filename, 1, 0) != 0) {
				/* Failed to reset upgrade status. This might from SD card id write protect. */
				WEBSERVER_DEBUG_PRINT("System upgrade stratus is enabled, but ignored due to SD Card might not allow to write.", filename);
			}
			else {
				WEBSERVER_DEBUG_PRINT("System upgrade processing, file=%s", filename);

				/* Open file */
				if((f=_fopen(filename, "r")) != (FIL*)0) {
					/* File size */
					if(f_size(f) == (VIRTUAL_DISK_END - VIRTUAL_DISK_START + 1)) {
						WEBSERVER_DEBUG_PRINT("Disk erasing...");
						/* Disk erase */
						__disable_irq();
						webserver_virtualdisk_unlock();
						webserver_virtualdisk_erase(0xAA55, 0x55553333);
						__enable_irq();
						WEBSERVER_DEBUG_PRINT("Erase done.");

						/* Copy file */
						write_buffer = (uint32_t*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
						offset = 0;
						while(offset < (VIRTUAL_DISK_END - VIRTUAL_DISK_START + 1)) {
							/* Read from file */
							if(f_read(f, write_buffer, _MEMLARGE_SIZE, &bytes_reading) == FR_OK) {
								if(bytes_reading != _MEMLARGE_SIZE) {
									WEBSERVER_DEBUG_PRINT("Failed to read from file.");
									break;
								}
								/* Write to virtual disk */
								__disable_irq();
								sta = webserver_virtualdisk_write(offset, write_buffer, _MEMLARGE_SIZE>>2);
								__enable_irq();
								if(sta != 0) {
									WEBSERVER_DEBUG_PRINT("Terminate system upgrade.");
									break;
								}							
							}
							else {
								WEBSERVER_DEBUG_PRINT("Failed to read from file.");
								break;
							}

							/* Increment data offset */
							offset += _MEMLARGE_SIZE;
						}

						/* Success */
						WEBSERVER_DEBUG_PRINT("***System upgrade success.");

						/* Free mem */
						_memlarge_free(write_buffer);

						/* Need system reboot */
						need_reboot = 1;
					}
					else { WEBSERVER_DEBUG_PRINT("***error, Expect file size is %u bytes", (VIRTUAL_DISK_END - VIRTUAL_DISK_START + 1)); }

					/* Close */
					_fclose(f);
				}
				else { WEBSERVER_DEBUG_PRINT("***error, Failed to open file: \"%s\" for system upgrade.", filename); }
			}

			/* Write System upgrade status */
			{
				UINT written_count;
				if((f=_fopen("system/upgradestatus.html", "w")) != (FIL*)0) {				          
					if(need_reboot) { /* Success */
						WEBSERVER_DEBUG_PRINT("Update status as Success.");
						f_write(f, system_upgrade_pass, strlen(system_upgrade_pass), &written_count);
					}
					else { /* Fail */
						WEBSERVER_DEBUG_PRINT("Update status as Fail.");
						f_write(f, system_upgrade_fail, strlen(system_upgrade_fail), &written_count);
					}
					/* Close */
					_fclose(f);
				}
				else {
					WEBSERVER_DEBUG_PRINT("Failed to write system upgrade status.");
				}
			}
		}
		else {
			if(enable == 0) { WEBSERVER_DEBUG_PRINT("System upgrade status is not enabled."); }
			if((enable == 1) && (*filename == 0)) { WEBSERVER_DEBUG_PRINT("***error, System upgrade file is invalid.");}
		}
		
		//__disable_irq();
		//__enable_irq();

		_memtiny_free(filename);

		/* Reboot system */
		if(need_reboot) {
			WEBSERVER_DEBUG_PRINT("\r\n***************************\r\nReboot...\r\n***************************");
			WEBSERVER_DEBUG_PRINT("");
			/* Enable WWDG clock */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
			WWDG->CR = 0x80; /* Generate Software reset */
		}
	}
}
