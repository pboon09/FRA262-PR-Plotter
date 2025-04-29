#include "waijung_hwdrvlib.h"
#include "webserver_mem_handler.h"
#include "webserver_debug_port.h"
#include "webserver_ioresult_handler.h"
#include "amg_utils.h"
#include "ff.h"
#include "ffconf.h"

/**
  * @brief  Copy disk content in the explorer list
  * @param  path: pointer to root path
  * @retval Status
  */
static char lfn[_MAX_LFN];
static uint8_t Explore_PathEx (char* path, char *buffer, int buffer_size, uint8_t showsize, uint8_t showdatetime)
{
  FRESULT res;
  FILINFO fno;
  DIR dir;
  char *fn;
  int buffer_index = 0;
  char *line_buffer_memsmall;

  line_buffer_memsmall = _memsmall_alloc();
  if(!line_buffer_memsmall) {
	  WEBSERVER_DEBUG_PRINT("Failed to allocate mem.");
	  return 0xFF;
  }

#if _USE_LFN  
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif

  res = f_opendir(&dir, path);
  if (res == FR_OK) {
    while (1) {
      res = f_readdir(&dir, &fno);

      if (res != FR_OK || fno.fname[0] == 0) {
        break;
      }
      if (fno.fname[0] == '.') {
        continue;
      }
#if _USE_LFN
      fn = *fno.lfname ? fno.lfname : fno.fname;
#else
      fn = fno.fname;
#endif
	  WEBSERVER_DEBUG_PRINT("  -> %s", fn);

	  /* File name and attribute */
	  sprintf (line_buffer_memsmall, "%s,%s", fn, ((fno.fattrib & AM_DIR) == AM_DIR)?"DIR":"FILE");
	  /* File size */
	  if(showsize)
		sprintf (&line_buffer_memsmall[strlen(line_buffer_memsmall)], ",%u", fno.fsize);
	  /* File Date/Time*/
	  if(showdatetime)
		sprintf (&line_buffer_memsmall[strlen(line_buffer_memsmall)], ",%u", ((uint32_t)fno.fdate<<16) | ((uint32_t)fno.ftime));
	  /* New line */
	  strcpy (&line_buffer_memsmall[strlen(line_buffer_memsmall)], "\n");

	  if((buffer_index+strlen(line_buffer_memsmall)) < buffer_size) {
		strcpy(&buffer[buffer_index], line_buffer_memsmall);
	    buffer_index += strlen(line_buffer_memsmall);
	  }
	  else {
		  if((buffer_size - buffer_index) > 32)
			strcpy (&line_buffer_memsmall[strlen(line_buffer_memsmall)], "More files...\n");
		  else
			strcpy (&line_buffer_memsmall[buffer_size-32], "More files...\n");
		  break;
	  }
    }
  }

  /* Free */
  _memsmall_free(line_buffer_memsmall);

  /* Return status */
  return res;
}

/* List format:
 * Type: Directory | File | Extension
*/
typedef struct {
  char *action; /* download | delete | list */
  char *dir; /* Specify directory path */
  char *file; /* Specify file for the action */
  char *id; /* "0": SD, "1"-INTERNAL, "2"-SQLite */
  char *output; /* "html": Setup file list view */
  char return_file_name[64];  
  uint8_t showsize;
  uint8_t showdatetime;
} FILE_CGI_PARAMS;

typedef enum {
	FILE_ACTION_LIST,
	FILE_ACTION_DELETE,	
} FILE_CGI_ACTION;

typedef struct {	
	uint8_t SizeEnable;
	uint8_t DataTimeEnable;
} FILE_LIST_CONFIGURATION;

/* Configuraion */
static char file_list_path[_MAX_LFN] = {0}; /* Store current path */
static char file_list_status[32] = {0}; /* Error: operation fail or Success. */

/**/
FILE_CGI_PARAMS file_cgi_params;
const char FILE_DEFAULT_ID[] = "0";
const char FILE_DEFAULT_NONE[] = "";
const char FILE_DEFAULT_ACTION[] = "list"; /* Default to list directory, it action did not specified */

void FILE_Handle_Init(FILE_CGI_PARAMS *cgi_params)
{
  /* Reset parameters */
  cgi_params->action = (char*)FILE_DEFAULT_ACTION;
  cgi_params->dir = (char*)FILE_DEFAULT_NONE;
  cgi_params->file = (char*)FILE_DEFAULT_NONE;
  cgi_params->id = (char*)FILE_DEFAULT_ID;
  cgi_params->showsize = 0;
  cgi_params->showdatetime = 0;

  /* Return filename */
  strcpy((cgi_params->return_file_name), "/dir_output.txt");
}

/* Handle File */
const char * FILE_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int i;
	//unsigned int tmp;

	WEBSERVER_DEBUG_PRINT("FILE_CGI_Handler...");
	WEBSERVER_DEBUG_PRINT("Param count: %d", iNumParams);

	/* Init CGI parameter */
	FILE_Handle_Init(&file_cgi_params);

	/* Search params */
	for(i=0; i<iNumParams; i++) {
		/* action */
		if(!strcmp(pcParam[i], "action")) {
			file_cgi_params.action = pcValue[i];
		}
		/* dir */
		else if(!strcmp(pcParam[i], "dir")) {
			file_cgi_params.dir = pcValue[i];
		}
		/* file */
		else if(!strcmp(pcParam[i], "file")) {
			file_cgi_params.file = pcValue[i];
		}
		/* id */
		else if(!strcmp(pcParam[i], "id")) {
			file_cgi_params.id = pcValue[i];
		}
		/* output */
		else if(!strcmp(pcParam[i], "output")) {
			file_cgi_params.output = pcValue[i];
		}
		/* fsize */
		else if(!strcmp(pcParam[i], "fsize")) {
			if(!strcmp(pcValue[i], "on") || !strcmp(pcValue[i], "1"))
				file_cgi_params.showsize = 1;
		}
		/* showdatetime */
		else if(!strcmp(pcParam[i], "fdatetime")) {
			if(!strcmp(pcValue[i], "on") || !strcmp(pcValue[i], "1"))
				file_cgi_params.showdatetime = 1;
		}
	}

	/* List file/ folder at a specified path */
	if(!strcmp(file_cgi_params.action, "list")|| !strcmp(file_cgi_params.action, "")) {
		/* Store path */
		sprintf(file_list_path, "%s:/%s", file_cgi_params.id, file_cgi_params.dir);

		/* Return file*/
		strcpy(file_cgi_params.return_file_name, "/system/dir.text");
	}
	/* Download */
	else if(!strcmp(file_cgi_params.action, "download")) {
		sprintf(file_cgi_params.return_file_name, "%s:/", file_cgi_params.id);
		if(*(file_cgi_params.dir))
			sprintf(&(file_cgi_params.return_file_name)[strlen((file_cgi_params.return_file_name))], "%s/", file_cgi_params.dir);
		sprintf(&(file_cgi_params.return_file_name)[strlen((file_cgi_params.return_file_name))], "%s", file_cgi_params.file);
	}
	/* Delete */
	else if(!strcmp(file_cgi_params.action, "delete")) {
		
		sprintf(file_list_path, "%s:/", file_cgi_params.id);
		if(*(file_cgi_params.dir)) {
			sprintf(&file_list_path[strlen(file_list_path)], "%s/", file_cgi_params.dir);
		}
		sprintf(&file_list_path[strlen(file_list_path)], "%s", file_cgi_params.file);
		WEBSERVER_DEBUG_PRINT("Deleting file: %s", file_list_path);
		if(f_unlink (file_list_path) == FR_OK) {
			strcpy(file_list_status, "Success.");
		}
		else {
			strcpy(file_list_status, "Error: operation failed.");
		}
		strcpy(file_cgi_params.return_file_name, "/system/dir_result.text");
	}	

	/* Return file. */
	return (const char*)(file_cgi_params.return_file_name);	
}

int FILEList_Get(const char *name, char *buffer, int buffer_size)
{
	/* List dir */
	if(!strcmp(name, "/system/dir.text")) {
		WEBSERVER_DEBUG_PRINT("Listing directory path: %s", file_list_path);
		if(Explore_PathEx(file_list_path, buffer, buffer_size, file_cgi_params.showsize, file_cgi_params.showdatetime) != 0) {
			/* Fail */
			strcpy(buffer, "Error: failed to generate file list.");
		}
	}
	else if(!strcmp(name, "/system/dir_result.text")) {
		strcpy(buffer, file_list_status);
	}
	else {
		strcpy(buffer, "Error: invalid file request.");
	}
	return strlen(buffer);
}
