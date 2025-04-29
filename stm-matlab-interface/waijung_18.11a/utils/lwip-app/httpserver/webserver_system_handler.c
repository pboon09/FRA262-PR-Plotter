#include "waijung_hwdrvlib.h"
#include "webserver_mem_handler.h"
#include "webserver_debug_port.h"
#include "webserver_ioresult_handler.h"
#include "amg_utils.h"
#include "ff.h"
#include "ffconf.h"

static int _hex2byte(const char *hex, char *output)
{
	int i;
	char value = 0;
	for (i=0; i<2; i++) {
		value <<= 4;
	  if(strlen(hex) > i) {
			if(((uint8_t)hex[i]>= (uint8_t)'0') && ((uint8_t)hex[i] <= (uint8_t)'9')) {
				value |= (hex[i] - '0');
			}
			else if(((uint8_t)hex[i]>= (uint8_t)'a') && ((uint8_t)hex[i] <= (uint8_t)'f')) {
				value |= (10 + (hex[i] - 'a'));
			}
			else if(((uint8_t)hex[i]>= (uint8_t)'A') && ((uint8_t)hex[i] <= (uint8_t)'F')) {
				value |= (10 + (hex[i] - 'A'));
			}
			else {
				return -1;
			}
    }
  }
	*output = value;
	return 0;
}

/* IAP Target */
static const char SYSTEM_Handle_IAPTarget_Main[]   = "main";
static const char SYSTEM_Handle_IAPTarget_W2d[]    = "w2d";
static const char SYSTEM_Handle_IAPTarget_Subnet[] = "subnet";
static const char SYSTEM_Handle_IAPTarget_Notset[] = "";

/* IAP Config */
static const char SYSTEM_Handle_IAPConf_Enable[]  = "1";
static const char SYSTEM_Handle_IAPConf_Disable[] = "0";
static const char SYSTEM_Handle_IAPConf_Notset[]  = "";

/* IAP Status */
static const char SYSTEM_Handle_IAPStatus_Success[] = "1";
static const char SYSTEM_Handle_IAPStatus_Error[]   = "0";
static const char SYSTEM_Handle_IAPStatus_Notset[]  = "";

static const char SYSTEM_Handle_IAPConf_Directory[] = "system/upgrade";
static const char SYSTEM_Handle_IAPConf_Filename[] = "system/upgrade/conf.ini";
static const char SYSTEM_Handle_IAPConf_Section_STR[] = "[Upgrade]";
static const char SYSTEM_Handle_IAPConf_Enable_STR[]  = "Enable=";
static const char SYSTEM_Handle_IAPConf_Status_STR[]  = "Status=";
static const char SYSTEM_Handle_IAPConf_File_STR[]    = "File=";
static const char SYSTEM_Handle_IAPConf_Target_STR[]  = "Target=";

/* [Upgrade]
 * Enable=1
 * Target=main|w2d|subnetwork
 * File=/system/upgrade/w2d.bin
 * Status=0
 */
int SYSTEM_Handle_ReadIAPConf(char *filepath, int *target, int *enable, int *status)
{
	int sta, section, line_count;
	FIL *f;
	char *s;
	char tmp[256];
	
	/* Default value */
	filepath[0] = '\0';
  *target = -1;
	*enable = -1;
	*status = -1;

  /* Open file for read */	
	if((f = _fopen(SYSTEM_Handle_IAPConf_Filename, "r")) == (void *)0)
		return -1;
	
	/* Scan file */
	sta = 0;
	section = 0;
	line_count = 0;
	while (!f_eof(f) && (sta == 0) && (++line_count < 20 /* Limit 20 lines */)) {
		if ((sta = _freadln(f, tmp, sizeof(tmp), "\n")) == 0) {
			_sstr_rtrim(tmp);
			if(!section) {
				if (!strncmp(tmp, SYSTEM_Handle_IAPConf_Section_STR, strlen(SYSTEM_Handle_IAPConf_Section_STR)))
					section = 1;
			}
			else {
				/* Enable */
				if(!strncmp(tmp,SYSTEM_Handle_IAPConf_Enable_STR, strlen(SYSTEM_Handle_IAPConf_Enable_STR))) {					
					s = &tmp[strlen(SYSTEM_Handle_IAPConf_Enable_STR)];
					while(*s == ' ') s++; /* Remove white space */
					if(!strncmp(s, SYSTEM_Handle_IAPConf_Enable,1))
						*enable = 1;
					else if(!strncmp(s, SYSTEM_Handle_IAPConf_Disable,1))
						*enable = 0;
				}
				/* Status */
				else if(!strncmp(tmp,SYSTEM_Handle_IAPConf_Status_STR, strlen(SYSTEM_Handle_IAPConf_Status_STR))) {
					s = &tmp[strlen(SYSTEM_Handle_IAPConf_Status_STR)];
					while(*s == ' ') s++; /* Remove white space */
					if(!strncmp(s, SYSTEM_Handle_IAPStatus_Success,1))
						*status = 1;
					else if(!strncmp(s, SYSTEM_Handle_IAPStatus_Error,1))
						*status = 0;
				}
				/* File */
				else if(!strncmp(tmp,SYSTEM_Handle_IAPConf_File_STR, strlen(SYSTEM_Handle_IAPConf_File_STR))) {
					s = &tmp[strlen(SYSTEM_Handle_IAPConf_File_STR)];
					while(*s == ' ') s++; /* Remove white space */
					strcpy(filepath, s);
				}				
				/* Target */	
				else if(!strncmp(tmp,SYSTEM_Handle_IAPConf_Target_STR, strlen(SYSTEM_Handle_IAPConf_Target_STR))) {
					s = &tmp[strlen(SYSTEM_Handle_IAPConf_Target_STR)];
					while(*s == ' ') s++; /* Remove white space */
					if(!strcmp(s, SYSTEM_Handle_IAPTarget_Main))
						*target = 0;
					else if(!strcmp(s, SYSTEM_Handle_IAPTarget_W2d))
						*target = 1;
					else if(!strcmp(s, SYSTEM_Handle_IAPTarget_Subnet))
						*target = 2;
				}
			}
		}
	}
	
	/* Close file */
	_fclose(f);
	
	/* Return status */	
	return sta;
}

int SYSTEM_Handle_CreateIAPConf(int target, const char *filepath, int enable, int status)
{
	UINT bw;
	FRESULT res = FR_OK;
	FIL *f;
	const char *s;
	
	/* Create directory */
	_dir_create(SYSTEM_Handle_IAPConf_Directory);
	
  /* Open file for write */	
	if((f = _fopen(SYSTEM_Handle_IAPConf_Filename, "w")) == (void *)0)
		return -1;
	
	/* [Upgrade] */
	if(res == FR_OK) {
		s = "[Upgrade]\r\n";
		res = f_write(f, s, strlen(s), &bw);
	}
	
	/* Target */
	if(res == FR_OK) {
		s = "Target=";
		res = f_write(f, s, strlen(s), &bw);
		if(res == FR_OK) {
			if(target == 0)
				s = &SYSTEM_Handle_IAPTarget_Main[0];
			else if(target == 1)
				s = &SYSTEM_Handle_IAPTarget_W2d[0];
			else if(target == 2)		
				s = &SYSTEM_Handle_IAPTarget_Subnet[0];
			else
				s = &SYSTEM_Handle_IAPTarget_Notset[0];
			if(*s)
				res = f_write(f, s, strlen(s), &bw);
		}
		if(res == FR_OK)
			res = f_write(f, "\r\n", 2, &bw);
	}
	
	/* File */
	if(res == FR_OK) {
		s = "File=";
		res = f_write(f, s, strlen(s), &bw);
		if(res == FR_OK)
			res = f_write(f, filepath, strlen(filepath), &bw);
		if(res == FR_OK)
			res = f_write(f, "\r\n", 2, &bw);
	}
	
	/* Enable */
	if(res == FR_OK) {
		s = "Enable=";
		res = f_write(f, s, strlen(s), &bw);
		if(res == FR_OK) {
			if(enable == 0)
				s = &SYSTEM_Handle_IAPConf_Disable[0];
			else if(enable == 1)
				s = &SYSTEM_Handle_IAPConf_Enable[0];
			else
				s = &SYSTEM_Handle_IAPConf_Notset[0];
			if(*s)
				res = f_write(f, s, strlen(s), &bw);
		}
		if(res == FR_OK)
			res = f_write(f, "\r\n", 2, &bw);
	}
	
	/* Status */
	if(res == FR_OK) {
		s = "Status=";
		res = f_write(f, s, strlen(s), &bw);
		if(res == FR_OK) {
			if(status == 0)
				s = &SYSTEM_Handle_IAPStatus_Error[0];
			else if(status == 1)
				s = &SYSTEM_Handle_IAPStatus_Success[0];
			else
				s = &SYSTEM_Handle_IAPStatus_Notset[0];
			if(*s)
				res = f_write(f, s, strlen(s), &bw);
		}
		if(res == FR_OK)
			res = f_write(f, "\r\n", 2, &bw);
	}	
	
	/* Close file */
	_fclose(f);
	
	/* Return status */	
	if(res == FR_OK)
		return 0;
	else
		return -1;
}

int SYSTEM_Handle_DecodeHex(const char *input, char *output, int output_len)
{
	const char *s;
	int len;
  char tmp[3] = {0,0,0};
  
  /* Check input length */
  len = strlen(input);
  if(len & 0x1)
		return -1;
	if((len / 2) > output_len)
		return -1;
	
	s = input;
	while(*s) {
		strncpy(tmp, s, 2);
    if(_hex2byte(tmp, output) != 0)
			return -1;
		output ++;		
		s += 2;
	}	
	return (len >> 1);
}

/* Return: Number of bytes. Negative number indicates fail.
 */
int SYSTEM_Handle_DecodeNone(const char *input, char *output, int output_len)
{
  /* Scan for escape char */
	char tmp[3] = {0,0,0};
	const char *s;
	const char *s1;
	int index;
	int len;
	int val;

  /* Decode hex */
	index = 0;
	s1 = input;
	while(*s1) {
		s = s1;
		if((s1 = strstr(s, "%")) != 0) {			
			strncpy(&output[index], s, (len=(s1-s))); /* Copy first part */
			index += len;
			if(!strncmp(s1, "%%", 2)) {
				output[index++] = '%';
			}
			else { /* Processing hex */				
				s1++;
				strncpy(tmp, s1, 2);
				if(sscanf(tmp,"%x", &val) != 1)
					return -1;
				output[index++] = (char)val;
			}
			s1+= 2; /* Move over */
		}
		else {
			strncpy(&output[index], s, (len=strlen(s)));
			index += len;
			output[index] = 0;
			return index;
		}
	}
	
	output[index] = 0;
	return index;	
}


/* Error message handler */
typedef enum {
	SYSTEM_HANDLE_ERR_NOMEM, /* Out of memory */
	SYSTEM_HANDLE_ERR_INVALID_FILE //
} SYSTEM_HANDLE_ERR;

void SYSTEM_Handle_GetErrorMsg(SYSTEM_HANDLE_ERR err, char *output)
{
	switch (err)
	{
		/* Out of memory */
		case SYSTEM_HANDLE_ERR_NOMEM:
			strcpy(output, "Error: out of memory");
			break;
		
		default:
			strcpy(output, "Error: unknown");
			break;
	}
}

/* List format:
 * Type: Directory | File | Extension
*/
typedef struct {
  char *action; /* Create | Delete */
  char *session;
	
	char *fenc; /* Encode: "hex", "none" */
	char *fmode; /* File mode to open: r, w, a */
  char *id; /* "0": SD, "1"-INTERNAL, "2"-SQLite */
  char *dir; /* Specify directory path */
  char *file; /* Specify file for the action */
	char *readcount; /* Specify number of bytes to read */
	char *seekpos; /* Seek to position for read */
	
	char *fdata; /* Data to be write into file */
	
	char *iaptarget; /* Target of IAP file: "main", "w2d", "subnetwork" */
	
  char return_file_name[64];
} SYSTEM_CGI_PARAMS;

typedef enum {
	SYSTEM_ACTION_GETVERSION,
	SYSTEM_ACTION_RESET,
} SYSTEM_CGI_ACTION;

/* Configuraion */
#define _SYSTEM_FILE_TIMER_TIMEOUT 60000UL /* 60 seconds */
#define _SYSTEM_FILE_TIMER_RESET() {SysTimer_Start(&system_file_timer, _SYSTEM_FILE_TIMER_TIMEOUT);}

static SYS_TIMER_STRUCT system_file_timer = {0,0};
static FIL *system_file = (FIL *)0; /* NULL */
static char system_action [16] = { 0 }; /* Current action */
//static char system_session[16] = { 0 }; /* Current session id */
static char system_result [64] = { '?' }; /* Current result of system: small result only. Success, session=0xFFFFAAAA */
static char *system_file_read_output = (char *)0;
static int system_file_read_count = 0;
static uint32_t system_restart_session = 0;
static uint32_t system_restart_activated = 0;

#define _SYSTEM_ACTION_GETINFO "getinfo" /* Get system information */
#define _SYSTEM_ACTION_FOPEN   "fopen"   /* Create file */
#define _SYSTEM_ACTION_FCLOSE  "fclose"  /* Close file */
#define _SYSTEM_ACTION_FWRITE  "fwrite"  /* Write file */
#define _SYSTEM_ACTION_FREAD   "fread"   /* Read file */
#define _SYSTEM_ACTION_RESTART "restart" /* MCU Reset */
#define _SYSTEM_ACTION_HELP    "help"    /* List help */
#define _SYSTEM_ACTION_IAPCONF "iapconf" /* IAP configuration file */

/* Default */
static SYSTEM_CGI_PARAMS system_cgi_params;
const char SYSTEM_DEFAULT_ID[] = "";
const char SYSTEM_DEFAULT_NONE[] = "";
const char SYSTEM_DEFAULT_FMODE[] = "r";
const char SYSTEM_DEFAULT_ENCODE[] = "none";
const char SYSTEM_DEFAULT_ACTION[] = _SYSTEM_ACTION_GETINFO; /* Default action when it is not specified */

void SYSTEM_Handle_Init(SYSTEM_CGI_PARAMS *cgi_params)
{
  /* Reset parameters */
  cgi_params->action = (char *)SYSTEM_DEFAULT_ACTION;
	cgi_params->session = (char *)SYSTEM_DEFAULT_NONE;
	cgi_params->fmode = (char *)SYSTEM_DEFAULT_FMODE;
	cgi_params->fenc = (char *)SYSTEM_DEFAULT_ENCODE;
	
	/* Directory */
  cgi_params->id = (char*)SYSTEM_DEFAULT_ID;
  cgi_params->dir = (char*)SYSTEM_DEFAULT_NONE;
  cgi_params->file = (char*)SYSTEM_DEFAULT_NONE;
	
	cgi_params->fdata = (char*)SYSTEM_DEFAULT_NONE;
	cgi_params->readcount = (char*)SYSTEM_DEFAULT_NONE;
	cgi_params->seekpos = (char*)SYSTEM_DEFAULT_NONE;
	
	cgi_params->iaptarget = (char*)SYSTEM_DEFAULT_NONE;
	
	/* Invalid system file handle */
	strcpy(system_result, "Error: invalid action.");

  /* Return filename */
  strcpy((cgi_params->return_file_name), "/system/system.text");
}

/* Periodic system handle: will called during Step function */
void SYSTEM_PriodicHandle(void)
{
  /* Periodic check an opened file */
	if(system_file != (void *)0) {
		if(SysTimer_IsTimeout(&system_file_timer)) {
			/* Close an openning file */
			_fclose(system_file);
			system_file = (FIL *)0;
		}
	}
	
	/* System restart */
	if(system_restart_activated) {
		if(SysTimer_IsTimeout(&system_file_timer)) {
			NVIC_SystemReset();
		}
	}
}

/* Handle File */
const char * SYSTEM_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int i;
	
	WEBSERVER_DEBUG_PRINT("SYSTEM_CGI_Handler...");
	WEBSERVER_DEBUG_PRINT("Param count: %d", iNumParams);

	/* Init CGI parameter */
	strcpy(system_action, SYSTEM_DEFAULT_ACTION);
	SYSTEM_Handle_Init(&system_cgi_params);

	/* Search Params */
	for(i=0; i<iNumParams; i++) {
		/* action */
		if(!strcmp(pcParam[i], "action")) {
			int size_limit;
			system_cgi_params.action = pcValue[i];			
			size_limit = sizeof(system_action);
			if(strlen(system_cgi_params.action) >= size_limit) {
				strncpy(system_action, system_cgi_params.action, size_limit-1);
				system_action[size_limit-1] = '\0';
			}
			else { strcpy(system_action, system_cgi_params.action); }
		}
		/* session */ 
		else if(!strcmp(pcParam[i], "session")) {
			system_cgi_params.session = pcValue[i];
		}
		/* id */
		else if(!strcmp(pcParam[i], "id")) {
			system_cgi_params.id = pcValue[i];
		}
		/* dir */
		else if(!strcmp(pcParam[i], "dir")) {
			system_cgi_params.dir = pcValue[i];
		}
		/* file */
		else if(!strcmp(pcParam[i], "file")) {
			system_cgi_params.file = pcValue[i];
		}
		/* fmode */
		else if(!strcmp(pcParam[i], "fmode")) {
			system_cgi_params.fmode = pcValue[i];
		}
		/* fenc */
		else if(!strcmp(pcParam[i], "fenc")) {
			system_cgi_params.fenc = pcValue[i];
		}
		/* fdata */
		else if(!strcmp(pcParam[i], "fdata")) {
			system_cgi_params.fdata = pcValue[i];
		}
		/* readcount */
		else if(!strcmp(pcParam[i], "readcount")) {
			system_cgi_params.readcount = pcValue[i];
		}
		/* seekpos */
		else if(!strcmp(pcParam[i], "seek")) {
			system_cgi_params.seekpos = pcValue[i];
		}
		/* iaptarget */
		else if(!strcmp(pcParam[i], "iaptarget")) {
			system_cgi_params.iaptarget = pcValue[i];
		}
		
		/* Invalid parameter  */
		else { /* Invalid */ }
	}

	/* Perform system action request */	
	/* --- Get information --- */
	if (!strcmp(system_action, _SYSTEM_ACTION_GETINFO)) {
		strcpy(system_result, ""); /* Result will be update during operation result request. */
	}
	/* --- Open file --- */
	else if (!strcmp(system_action, _SYSTEM_ACTION_FOPEN)) {
		if(system_file == (FIL *)0) { /* Not openning */
			char *file_path;			
			if(*system_cgi_params.file) {			
				/* Full file path */
				if((file_path = (char *)_memsmall_alloc()) != (void *)0) { /* Must be free with _memsmall_free() */
					/* Create dir */
					if(!strcmp(system_cgi_params.fmode, "w")) {
						_fpath(file_path, _MEMSMALL_SIZE, system_cgi_params.id, system_cgi_params.dir, "");
						_dir_create(file_path);
					}
					
					/* Full file path */
					_fpath(file_path, _MEMSMALL_SIZE, system_cgi_params.id, system_cgi_params.dir, system_cgi_params.file);
					WEBSERVER_DEBUG_PRINT("Openning file: %s ...", file_path);
					
					/* Open file */
					system_file = _fopen(file_path, system_cgi_params.fmode);
					if(system_file) {
						_SYSTEM_FILE_TIMER_RESET(); /* Reset file watchdog timer */
						sprintf(system_result, "Success, session=%X", (uint32_t)system_file);				
						WEBSERVER_DEBUG_PRINT("Success");
					}
					else {
						sprintf(system_result, "Error, failed to open file");
						WEBSERVER_DEBUG_PRINT("Fail");
					}
					/* Free mem */
					_memsmall_free(file_path);
				} else { SYSTEM_Handle_GetErrorMsg(SYSTEM_HANDLE_ERR_NOMEM, system_result); /* Out of memory */ }
			} else { strcpy(system_result, "Error: file name required"); }
		} else { strcpy(system_result, "Error: last session pending"); }
	}
	/* --- Write file --- */
	else if (!strcmp(system_action, _SYSTEM_ACTION_FWRITE)) {
		uint32_t session;		
		if(*system_cgi_params.fenc && (!strcmp(system_cgi_params.fenc, "none") || !strcmp(system_cgi_params.fenc, "hex"))) {
			if(*system_cgi_params.fdata) {
				UINT bw;
			  int count = 0;
			  char *data;
			  if((data = (char *)_memmedium_alloc()) != 0) { /* Must be free with: _memmedium_free() */			
			    if(!strcmp(system_cgi_params.fenc, "hex"))
				    count = SYSTEM_Handle_DecodeHex(system_cgi_params.fdata, data, _MEMMEDIUM_SIZE);
			    else
				    count = SYSTEM_Handle_DecodeNone(system_cgi_params.fdata, data, _MEMMEDIUM_SIZE);
			    if(count > 0) {
			      if((*system_cgi_params.session) && (sscanf(system_cgi_params.session, "%X", &session) == 1) \
		  		    && (session == (uint32_t)system_file)) {
							/* Write to file */
							if (f_write (system_file, data, count, &bw) == FR_OK)
	  			      strcpy(system_result, "Success");
							else
								strcpy(system_result, "Error: write failed");
				      _SYSTEM_FILE_TIMER_RESET(); /* Reset file watchdog timer */
			      } else { strcpy(system_result, "Error: invalid session"); }
				  } else { strcpy(system_result, "Error: invalid data"); }					
					_memmedium_free(data); /* Free mem */
		    } else { strcpy(system_result, "Error: out of memory"); }
		  } else { strcpy(system_result, "Error: invalid data"); }
	  } else { strcpy(system_result, "Error: invalid encode"); }
	}
	/* --- Read file --- */
	else if (!strcmp(system_action, _SYSTEM_ACTION_FREAD)) {
		uint32_t session, reading, readcount, fpos, seek;
		/* Session */
		if((*system_cgi_params.session) \
		  && (sscanf(system_cgi_params.session, "%X", &session) == 1) && (session == (uint32_t)system_file)) {
			/* Read count */
    	if((*system_cgi_params.readcount) \
		    && (sscanf(system_cgi_params.readcount, "%u", &readcount) == 1) \
			  && (readcount > 0) && (readcount <= 1024)) {
				/* Seek pos */
				if((*system_cgi_params.seekpos) && (sscanf(system_cgi_params.seekpos, "%u", &seek) == 1)) {
					/* Seek */
					f_lseek(system_file, seek);
				}
				/* Read buffer */
				if(system_file_read_output == (void *)0)
					system_file_read_output = _memmedium_alloc(); /* Will be free later on return data. */
				if(system_file_read_output) {
					fpos = f_tell(system_file);
					system_file_read_count = 0;
					if(f_read(system_file, system_file_read_output, readcount, &reading) == FR_OK) {
						system_file_read_count = reading;
						sprintf(system_result, "Success,%u,%u/%u", reading, fpos, (uint32_t)f_size(system_file));
						_SYSTEM_FILE_TIMER_RESET(); /* Reset file watchdog timer */				
					} else { strcpy(system_result, "Error: read fail"); }		
				} else { strcpy(system_result, "Error: out of memory"); }		
			} else { strcpy(system_result, "Error: invalid read count"); }
	  }	else { strcpy(system_result, "Error: invalid session"); }
	}
	/* --- Close file --- */
	else if (!strcmp(system_action, _SYSTEM_ACTION_FCLOSE)) {
		uint32_t session;
	  if((*system_cgi_params.session) \
		  && (sscanf(system_cgi_params.session, "%X", &session) == 1) \
		  && (session == (uint32_t)system_file)) {
				_fclose(system_file);
				system_file = (FIL *)0;
				strcpy(system_result, "Success");
		}
		else { strcpy(system_result, "Error: invalid session"); }
	}
	/* --- Restart --- */
	else if (!strcmp(system_action, _SYSTEM_ACTION_RESTART)) {
		uint32_t session;
	  if(*system_cgi_params.session) {
		  if ((sscanf(system_cgi_params.session, "%u", &session) == 1) \
				&& (session == system_restart_session)) {
				/* Activate system restart flag */
				system_restart_activated = 1;
				SysTimer_Start(&system_file_timer, 3000); /* System restart within 3sec */
				strcpy(system_result, "Success, system is restarting");
			}
			else { strcpy(system_result, "Error: invalid session");	}
		}
		else { /* Generate session */
			system_restart_session = SYS_CURRENT_TICK;
			sprintf(system_result, "Pending: %u", system_restart_session);
    }
	}
	/* Iap Config */
	else if (!strcmp(system_action, _SYSTEM_ACTION_IAPCONF)) {
		int target;
		char *filepath;
		
		/* If file is still open ? */
		if(system_file == (FIL *)0) {
			if(!strcmp(system_cgi_params.iaptarget, SYSTEM_Handle_IAPTarget_Main))
				target = 0;
			else if(!strcmp(system_cgi_params.iaptarget, SYSTEM_Handle_IAPTarget_W2d))
				target = 1;
			else if(!strcmp(system_cgi_params.iaptarget, SYSTEM_Handle_IAPTarget_Subnet))
				target = 2;
			else
				target = -1;
			/* Target is valid? */
			if(target >= 0) {
				if((filepath = _memsmall_alloc()) != 0) {
					_fpath(filepath, _MEMSMALL_SIZE, system_cgi_params.id, system_cgi_params.dir, system_cgi_params.file);
					if(SYSTEM_Handle_CreateIAPConf(target, filepath, 1, 0) == 0) {
						strcpy(system_result, "Success");
					}
					else { strcpy(system_result, "Error: failed to write config file"); }
					_memsmall_free(filepath); /* Free mem */
				}
				else { strcpy(system_result, "Error: out of memory"); }
			}
			else { strcpy(system_result, "Error: invalid target"); }
		}
		else { strcpy(system_result, "Error: last session pending"); }
	}

	/* Return file. */
	return (const char*)(system_cgi_params.return_file_name);	
}

int SYSTEM_Action_Get(const char *name, char *buffer, int buffer_size) {
	int count = -1;
	
	buffer[0] = '\0';
	/* Get system output text */
	if(!strcmp(name, "/system/system.text")) {
		/* --- Get system information --- */
		if(!strcmp(system_action, _SYSTEM_ACTION_GETINFO)) {
			strcpy(buffer,
			"Build date: "
			__DATE__
			" "
			__TIME__
			"\r\n"
			"Waijung: "
			__WAIJUNG_VERSION__
			);
		}
		/* --- Read data --- */
		else if(!strcmp(system_action, _SYSTEM_ACTION_FREAD)) {
			count = strlen(system_result);
			strcpy(buffer, system_result);
			if(system_file_read_output) {
				strcpy(&buffer[strlen(buffer)], ",");
				count ++;
				memcpy(&buffer[strlen(buffer)], system_file_read_output, system_file_read_count);
				count += system_file_read_count;
			}
		}
		/* --- Help --- */
		else if(!strcmp(system_action, _SYSTEM_ACTION_HELP)) {
			strcpy(
			buffer,
			"=== System CGI Help ===\r\n"
			"Syntax: \\system.html?action=<action>&session=<session>&id=<id>&dir=<dir>&file=<file>&fmode=<fmode>&fenc=<fenc>&fdata=<fdata>&readcount=<readcount>&seekpos=<seekpos>\r\n"
			"Parameters:\t\n"
			"\tid: disk id, value=<0|1|2>\r\n"
			"\tdir: directory name\r\n"
			"\tfile: file name\r\n"
			"\tfmode: file operation mode, value=<r,w,a>. r - read, w - write, a - append\r\n"
			"\tfenc: encoding data to write, value=<none|hex>.\r\n"
			"\tfdata: data to Write, max 512 with hex encoding and 1024 without encoding\r\n"
			"\treadcount: bytes to Read, Max 1024\r\n"
			"\tseekpos: data position to Read (optional)\r\n\r\n"
			"Action:\r\n"
			"1. Open file for Read or Write\r\n\tParameters: id, dir, file, fmode=r\r\n\tReturn: OnSuccess - \"Success, session=<session>\"\r\n"
			"\tExample1 (Read): .../system.html?action=fopen&id=0&dir=&file=f.txt&fmode=r\r\n"
			"\tExample2 (Write): .../system.html?action=fopen&id=0&dir=&file=f.txt&fmode=w\r\n"
			"\r\n"
			"\tNote: File will close automatically within 60 sec when no action\r\n"
			"\tNote: on success, it return the operation session, keep it for next action fread or fwrite.\r\n\r\n"
			"2. Read data from file\r\n\tParameters: session, readcount, seekpos\r\n\tReturn: OnSuccess - \"Success,<data count>,<current file pos>/<filesize>,<data>\", OnError - \"Error: ...\"\r\n"
			"\tExample: .../system.html?action=fread&session=CAEDFFFF&readcount=512\r\n"
			"3.Write data to file\r\n\tParameters: session,fenc,fdata\r\n\tReturn: Onsuccess - \"Success\", OnError - \"Error: ...\"\r\n"
			"\tExample1 (Write binary file, 6bytes): .../system.html?action=fwrite&session=CAEDFFFF&fenc=hex&fdata=AABBCCDDEEFF\r\n"
			"\tExample2 (Write text file): .../system.html?action=fwrite&session=CAEDFFFF&fenc=none&fdata=Hello\r\n"
			"4. Close file\r\n\tParameter: session\r\n"
			"\tExample: .../system.html?action=fclose&session=CAEDFFFF\r\n"
			"5. System restart\r\n\tParameter: session\r\n\tReturn: Onsuccess - \"Success, ...\", OnError - \"Error: ...\"\r\n"
			"\tStep1: request for system restart.\r\n\t\tExample: .../system.html?action=restart\r\n\t\tNote: Keep the return session for next step\r\n"
			"\tStep2: activation.\r\n\t\tExample: .../system.html?action=restart&session=123456\r\n"
			);
		}		
		
		/* --- Other status of operation --- */
		else { strcpy(buffer, system_result); }
	}
	else { strcpy(buffer, "Error: invalid file request."); }
	
	/* Free mem */
	if(system_file_read_output) {
		_memmedium_free(system_file_read_output); /* Free mem */
		system_file_read_output = (void *)0;
	}
	
	if(count < 0)
		count = strlen(buffer);
	
	/* Return */
	return count;
}

