
#include "waijung_hwdrvlib.h"
#include "webserver_upgrade_handler.h"
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

const char *ini_upgrade_group = "[Upgrade]\r\n";
const char *ini_upgrade_enable = "Enable=1\r\n";
const char *ini_upgrade_disable = "Enable=0\r\n";
uint8_t webserver_setupgrade_tofile(char *filename, uint8_t enbale)
{
	UINT bytes_written;
	volatile FRESULT res = FR_OK;
	FIL *f;
	
	/* Create system directory */
	if(_dir_create("system/upgrade") != 0) {
		WEBSERVER_DEBUG_PRINT("Failed to create system directory.");
		return 1;
	}
	
	/* Open ini file */
	if((f = _fopen("system/upgrade/conf.ini", "w")) != (void*)0) {
		/* [Upgrade] */
		if((res = f_write(f, ini_upgrade_group, strlen(ini_upgrade_group), &bytes_written)) != FR_OK) {
			WEBSERVER_DEBUG_PRINT("Failed to write conf.ini");
		}
		/* Enable=1|0 */
		if(enbale) {
			if((res == FR_OK) && ((res = f_write(f, ini_upgrade_enable, strlen(ini_upgrade_enable), &bytes_written)) != FR_OK)) {
				WEBSERVER_DEBUG_PRINT("Failed to write conf.ini");
			}
		}
		else {
			if((res == FR_OK) && ((res = f_write(f, ini_upgrade_disable, strlen(ini_upgrade_disable), &bytes_written)) != FR_OK)) {
				WEBSERVER_DEBUG_PRINT("Failed to write conf.ini");			
			}
		}
    /* File=xxxxx.img */
		if((res == FR_OK) && ((res = f_write(f, "File=", strlen("File="), &bytes_written)) != FR_OK)) {
			WEBSERVER_DEBUG_PRINT("Failed to write conf.ini");
		}
		if((res == FR_OK) && ((res = f_write(f,filename, strlen(filename), &bytes_written)) != FR_OK)) {
			WEBSERVER_DEBUG_PRINT("Failed to write conf.ini");
		}
		if((res == FR_OK) && ((res = f_write(f,"\r\n", strlen("\r\n"), &bytes_written)) != FR_OK)) {
			WEBSERVER_DEBUG_PRINT("Failed to write conf.ini");
		}

		/* Close file */
		_fclose(f);
	}
	else {
		WEBSERVER_DEBUG_PRINT("Failed to open conf.ini for writing.");
		return 1;
	}

	if(res != FR_OK)
		return 1;

	return 0; /* No error detected */
}

void webserver_getupgrade_fromfile(char *filename_buffer, uint8_t *enable)
{
	UINT bytes_reading;
	volatile FRESULT res = FR_OK;
	FIL *f;	
	char *buffer1_small = 0;
	char *tmp;
	char *s;
	uint8_t node_found;
	
	*enable = 0;
	filename_buffer[0] = 0;
		
	/* Allocate buffer */
	buffer1_small = (char*)_memsmall_alloc(); /* _MEMSMALL_SIZE (512). IMPORTANT: Must be free with _memsmall_free() */
	if(buffer1_small == 0) {
		if(buffer1_small)
			_memsmall_free(buffer1_small);
		WEBSERVER_DEBUG_PRINT("Out of memory.");
		return;
	}
	
	/* Open ini file */
	if((f = _fopen("system/upgrade/conf.ini", "r")) != (void*)0) {		
		if((res = f_read(f, buffer1_small, _MEMSMALL_SIZE-3, &bytes_reading)) == FR_OK) {			
			buffer1_small[bytes_reading] = '\r';
			buffer1_small[bytes_reading+1] = '\n';
			buffer1_small[bytes_reading+2] = '\0';			
			
			node_found = 0;
			s = buffer1_small;
			while ((tmp=strstr(s, "\r\n"))!= (void*)0) {
				*tmp = '\0';
				tmp += 2; /* Remove "\r\n" */								
				if(node_found == 0) {
					if(strstr(s, "[Upgrade]") != (void*)0)
						node_found = 1;
				}
				else {
					if((strstr(s, "File=")) != (void*)0) {
						s = strstr(s, "File=");
						s+= 5;
						strcpy(filename_buffer, s);
					}
					else if(strstr(s, "Enable=1") != 0) {
						*enable = 1;
					}
				}
				s = tmp;
			}	
		}
		else {
			WEBSERVER_DEBUG_PRINT("Failed to read conf.ini");
		}
		/* Close file */
		_fclose(f);
	}
	else {
		/* Configuration file did not found */
	}
		
	/* Free mem */
	if(buffer1_small)
		_memsmall_free(buffer1_small);
}

static uint8_t webserver_upgrade_status = 0;
void webserver_upgrade_activate(uint8_t activate)
{
  webserver_upgrade_status = activate;
}

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

	/* Poll upgrade status */
	if(webserver_upgrade_status == 0xAA) {
		webserver_upgrade_status = 0x00;

		/* Check upgrade file */
		filename = (char*)_memtiny_alloc(); /* _MEMSMALL_SIZE (64). IMPORTANT: Must be free with _memtiny_free() */
		webserver_getupgrade_fromfile(filename, &enable);
		if(enable && *filename) {
			/* Clear system upgrade status */
			if(webserver_setupgrade_tofile(filename, 0) != 0) {
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
			WWDG->CR = 0x80; /* Gernerate Software reset */
		}
	}
}
