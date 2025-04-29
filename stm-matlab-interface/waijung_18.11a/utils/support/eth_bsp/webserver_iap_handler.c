#include "waijung_hwdrvlib.h"
#include "webserver_iap_handler.h"
#include "webserver_mem_handler.h"
#include "webserver_debug_port.h"
#include "amg_utils.h"
#include "ff.h"

#define WEBSERVER_IAP_TIMEOUT_MS	5000UL

typedef enum  {
	IAP_GET_ID,
	IAP_GET_FILE,
	IAP_DATA,
	IAP_CALLBACK,
	IAP_ERROR_IO,
	IAP_ERROR_ID,
	IAP_ERROR_INVALID,
	IAP_ERROR_TIMEOUT,
	IAP_ERROR_USERCODE,
	IAP_SUCCESS
} IAP_STATE;

typedef struct {
	unsigned int device;
	FIL *f;
	IAP_STATE State;
	unsigned int UserErrCode;
} IAP_PROCESS_STRUCT;

static IAP_PROCESS_STRUCT iap_process =
{
	0xFFFFFFFF,	/* Device */
	(FIL*)0,	/* File */
	IAP_GET_ID,	/* State */
	0
};

static char webserver_iap_filename[32]; /* File path length for IAP cannot exceed 31 chars */

//SysTimer_Start(SYS_TIMER_STRUCT* timer, uint32_t ms)
static SYS_TIMER_STRUCT iap_watchdog_timer = {0UL, 0UL};

/* Initial IAP process with specify filename */
int webserver_iap_activate(char *filename)
{
	/* Store filename, validate later */
	strncpy(webserver_iap_filename, filename, sizeof(webserver_iap_filename));
	webserver_iap_filename[sizeof(webserver_iap_filename)-1] = '\0';

	/* Check processing state */
	iap_process.State = IAP_DATA;

	return 0;
}

#if WEBSERVER_IAP_ENABLE
/* 
** Callback function
** Return
**  Status: 0-Success, 1-Busy (need status poll until Success or fail), 2-Fail, ...
*/
void webserver_iap_interface_isr(unsigned int state, 
	unsigned int device, 
	unsigned int offset, 
	unsigned int tot_size, 
	unsigned int data, 
	unsigned int *status);

/* IAP progress */
static int iap_callback_state = 0;
static int iap_callback_offset = 0;
static int iap_callback_totsize = 0;
static int iap_callback_data = 0;
static unsigned int iap_callback_status = 0;

/* Periodic checking status of IAP */
/* Called by Webserver block */
void webserver_iap_process(void)
{	
	/* Watchdog timer check */
	if(iap_watchdog_timer.length && SysTimer_IsTimeout(&iap_watchdog_timer)) {
		iap_process.State = IAP_ERROR_TIMEOUT;
		SysTimer_Start(&iap_watchdog_timer, 0); /* Stop timer */
	}	

	/* Do something with file processing. */
	if(iap_process.State == IAP_DATA) {
		/* Close if openning */
		if(iap_process.f) {			
			_fclose(iap_process.f);
			iap_process.f = (void*)0;
		}
		if((iap_process.device & 0xE0000000) == 0) {
			/* Close file if openning */
			WEBSERVER_DEBUG_PRINT("Open file for IAP: %s", webserver_iap_filename);
			if((iap_process.f = _fopen(webserver_iap_filename, "r")) != (void*)0) {
				if((f_size(iap_process.f) & 0x3)==0) { /* Multiple of 4 */
					/* Ready to proceed */
					iap_callback_state = 0;
					iap_callback_offset = 0;
					iap_callback_totsize = 0;
					iap_process.State = IAP_CALLBACK;

					/* Reset timer */
					SysTimer_Start(&iap_watchdog_timer, WEBSERVER_IAP_TIMEOUT_MS);
				} else { iap_process.State = IAP_ERROR_INVALID; }
			} else { iap_process.State = IAP_ERROR_IO; }
		} else { iap_process.State = IAP_ERROR_ID; }
	}

	/* Process callback to Webserver IAP interface block */
	if(iap_process.State == IAP_CALLBACK) {
		iap_callback_status = 0xFFFFFFFF;
		switch(iap_callback_state) {
			/* === Callback state: INIT === */
			case 0:
			/* === Callback state: Get programming mode === */
			case 1:
			/* === Exit programming mode === */
			case 3:
			/* === Get programming status === */
			case 4:
			/* === Restart === */
			case 5:
				/* Callback */
				webserver_iap_interface_isr(
					iap_callback_state, /* Initial programming mode */
					iap_process.device, 
					iap_callback_offset, 
					f_size(iap_process.f), 
					iap_callback_data, 
					&iap_callback_status);
				/* Validate state */
				if(iap_callback_status == 0xFFFFFFFF) {	/* Pending ... */ }
				else if (iap_callback_status == 0x0){ /* Accept */
					iap_callback_state++;
					/* Reset timer */
					SysTimer_Start(&iap_watchdog_timer, WEBSERVER_IAP_TIMEOUT_MS);
				}
				else { /* Fail (Code) */
					iap_process.UserErrCode = iap_callback_status;
					iap_process.State = IAP_ERROR_USERCODE;
				}
				break;

			/* === Write programming data === */
			case 2:
				if(!f_eof(iap_process.f)) {
					UINT reading;
					if((f_read (iap_process.f, &iap_callback_data, 4, &reading) == FR_OK) && (reading == 4)) {
						/* Callback */
						webserver_iap_interface_isr(
							iap_callback_state, /* Initial programming mode */
							iap_process.device, 
							iap_callback_offset, /* Note: byte offset */
							(iap_callback_totsize = (unsigned int)f_size(iap_process.f)), 
							iap_callback_data, 
							&iap_callback_status);
						if(iap_callback_status == 0xFFFFFFFF) {	/* Pending ... */ 
							/* Move back pointer */
					        f_lseek(iap_process.f, f_tell(iap_process.f)-4);
						}
						else if (iap_callback_status == 0x0){ /* Accept */
							iap_callback_offset += 4;
							/* Reset timer */
							SysTimer_Start(&iap_watchdog_timer, WEBSERVER_IAP_TIMEOUT_MS);
						}
						else { /* Fail (Code) */
							iap_process.UserErrCode = iap_callback_status;
							iap_process.State = IAP_ERROR_USERCODE;
						}						
					}
					else { iap_process.State = IAP_ERROR_IO; }
				}
				else { iap_callback_state++; }

				break;

			/* === Success === */
			case 6:
				iap_process.State = IAP_SUCCESS;
				SysTimer_Start(&iap_watchdog_timer, 0); /* Stop timer */
				break;

			default:
				break;
		}
	}
	/* Clean up */
	else {
		if(iap_process.f) {
			_fclose(iap_process.f);
			iap_process.f = (void*)0;
		}
	}
}

typedef struct {
  char *action;
  char *sourcefile; /* Source of firmware file */
  char *id;
  char return_file_name[32];  
} IAP_CGI_PARAMS;

IAP_CGI_PARAMS iap_cgi_params;
const char IAP_DEFAULT_NONE[] = "";
const char IAP_DEFAULT_ACTION[] = "status"; /* Default to update IAP status */

void IAP_Handle_Init(IAP_CGI_PARAMS *cgi_params)
{
  /* Reset parameters */
  cgi_params->action = (char*)IAP_DEFAULT_ACTION;
  cgi_params->sourcefile = (char*)IAP_DEFAULT_NONE;
  cgi_params->id = (char*)IAP_DEFAULT_NONE;

  /* Return filename */
  strcpy((cgi_params->return_file_name), "/system/iap.html");
}

/* Setup content of status file */
/* This allow user to see progress on IAP */
/* Handle SQLite */
const char * IAP_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int i;
	unsigned int tmp;

	WEBSERVER_DEBUG_PRINT("IAP_CGI_Handler...");
	WEBSERVER_DEBUG_PRINT("Param count: %d", iNumParams);

	/* Init CGI parameter */
	IAP_Handle_Init(&iap_cgi_params);

	/* Search params */
	for(i=0; i<iNumParams; i++) {
		/* action */
		if(!strcmp(pcParam[i], "action")) {
			iap_cgi_params.action = pcValue[i];
		}
		/* id */
		if(!strcmp(pcParam[i], "id")) {
			iap_cgi_params.id = pcValue[i];
		}
	}

	/* Setup action */
	if(!strcmp(iap_cgi_params.action, "init")) {
		iap_process.State = IAP_GET_ID;
		/* Process ID */
		if(sscanf(iap_cgi_params.id, "%u", &tmp) == 1) {
			iap_process.device = tmp;
		}
		else {
			iap_process.device = 0xFFFFFFFF;
		}
	}
	else if(!strcmp(iap_cgi_params.action, "getfile")) {
		iap_process.State = IAP_GET_FILE;
		/* Process ID */
		if(sscanf(iap_cgi_params.id, "%u", &tmp) == 1) {
			iap_process.device = tmp;
		}
		iap_callback_state = 0;
		iap_callback_offset = 0;
		iap_callback_totsize = 0;
	}

	/* Return file. */
	return ("/system/iap.html");	
}

/* HTML pattern */
const char *iap_resp_header = "<!DOCTYPE html>\r\n<html>\r\n";
const char *iap_resp_body = "<body>";
const char *iap_resp_footer = "</body></html>";

//static int process_index = 0;

const char *WEBSERVER_IAP_ERROR_IO = "<h3><font color=\"red\">I/O Error.</font></h3>";
const char *WEBSERVER_IAP_ERROR_ID = "<h3><font color=\"red\">Invalid device ID.</font></h3>";
const char *WEBSERVER_IAP_ERROR_INVALID = "<h3><font color=\"red\">Invalid file.</font></h3>";
const char *WEBSERVER_IAP_ERROR_TIMEOUT = "<h3><font color=\"red\">Operation timed-out.</font></h3>";
const char *WEBSERVER_IAP_ERROR_USERCODE = "<h3><font color=\"red\">Error code: %u</font></h3>";

char *Webserver_Iap_GetErrMsg(unsigned int code)
{
	switch(code)
	{
	case IAP_ERROR_IO:
		return (char*)WEBSERVER_IAP_ERROR_IO;
	case IAP_ERROR_ID:
		return (char*)WEBSERVER_IAP_ERROR_ID;
	case IAP_ERROR_INVALID:
		return (char*)WEBSERVER_IAP_ERROR_INVALID;
	case IAP_ERROR_TIMEOUT:
		return (char*)WEBSERVER_IAP_ERROR_TIMEOUT;
	case IAP_ERROR_USERCODE:
		return (char*)WEBSERVER_IAP_ERROR_USERCODE;
	default:
		break;
	}
	return (char*)"";
}

/* Generate file */
int IAP_CGI_GetFile(char *buffer, int buffer_size)
{
	char *pBuff;
	int len;

	/* Initial buffer */
	pBuff = buffer;
	pBuff[0] = '\0';

	/* HTML Header */
	strcpy((pBuff += strlen(pBuff)), iap_resp_header);

	switch (iap_process.State) {
	case IAP_GET_ID:
		sprintf((pBuff += strlen(pBuff)),
			"<head><title>IAP</title></head><body>"
			"<h3>=== Webserver IAP interface ===</h3>"
			"<h3>Step 1: Please specify device ID to program</h3>"
			"<p><form action=\"/iap.html\" method=\"get\">"
			"<input type=\"hidden\" name=\"action\" value=\"getfile\"></h3>"
			"<b>ID:</b> <input type=\"text\" name=\"id\" value=\"%d\">"
			"<input type=\"submit\" value=\"Next >>\">"
			"</form>"
			"<p><a href=\"/\">home</a>", (iap_process.device == 0xFFFFFFFF)?0:iap_process.device);

		break;

	case IAP_GET_FILE:
		strcpy((pBuff += strlen(pBuff)),
			"<head><title>Upload file...</title></head>\r\n"
			"<body>\r\n"
			"<h3>=== Webserver IAP interface ===</h3>"
			"<h3>Step 2: Please select firmware file to upload.</h3>"
			"<form action=\"/iap.html\" enctype=\"multipart/form-data\" method=\"post\">\r\n"
			"<input name=\"directory\" type=\"hidden\" value=\"system/iap\" size=\"40\">\r\n"
			"<input name=\"datafile\" type=\"file\" size=\"40\">\r\n"
			"<p><input type=\"submit\" value=\"Upload\"></p></form>\r\n"
			"<p><a href=\"/\">home</a>\r\n");

		break;

	case IAP_DATA:
	case IAP_CALLBACK:
		strcpy((pBuff += strlen(pBuff)),	
			"<head><title>Please wait...</title>\r\n<meta http-equiv=\"refresh\" content=\"1\">\r\n</head>\r\n"
			"<body>\r\n"
			"<h3>=== Webserver IAP interface ===</h3>"
			"<h3>IAP is in progress, please wait.</h3>"
			);

		sprintf((pBuff += strlen(pBuff)), "Progress: %u out of %u bytes.", iap_callback_offset, iap_callback_totsize);

		break;

	case IAP_ERROR_IO:
	case IAP_ERROR_ID:
	case IAP_ERROR_INVALID:
	case IAP_ERROR_TIMEOUT:
	case IAP_ERROR_USERCODE:
		sprintf((pBuff += strlen(pBuff)),
			"<head><title>Error...</title>\r\n</head>\r\n"
			"<body>\r\n"
			"<h3>=== Webserver IAP interface ===</h3>"
			"IAP State: %u<br>", iap_callback_state
			);
		if(iap_process.State == IAP_ERROR_USERCODE) 
			sprintf((pBuff += strlen(pBuff)), Webserver_Iap_GetErrMsg(iap_process.State), iap_process.UserErrCode);
		else
			strcpy((pBuff += strlen(pBuff)), Webserver_Iap_GetErrMsg(iap_process.State));		

		sprintf((pBuff += strlen(pBuff)),
			"<p><a href=\"/iap.html?action=init&id=%u\">re-try</a></p>", (iap_process.device == 0xFFFFFFFF)?0:iap_process.device
		);
		break;

	case IAP_SUCCESS:
		strcpy((pBuff += strlen(pBuff)),
			"<head><title>Success...</title>\r\n</head>\r\n"
			"<body>\r\n"
			"<h3>=== Webserver IAP interface ===</h3>"
			"<h3><font color=\"green\">Success.</font></h3>");
		sprintf((pBuff += strlen(pBuff)),
			"<p><a href=\"/iap.html?action=init&id=%u\">re-start</a></p>", (iap_process.device == 0xFFFFFFFF)?0:iap_process.device
			);
		break;
	default:
		break;
	}

	/* HTML Footer */
	strcpy((pBuff += strlen(pBuff)), iap_resp_footer);

	/* Return length of return data to buffer */	
	len = (int)((pBuff += strlen(pBuff)) - buffer);
	WEBSERVER_DEBUG_PRINT("IAP content len: %d", len);
	return len;
}

#endif /* WEBSERVER_IAP_ENABLE */
