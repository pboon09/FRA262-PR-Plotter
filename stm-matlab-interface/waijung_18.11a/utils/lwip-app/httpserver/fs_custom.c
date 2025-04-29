
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "fs.h"
#include "amg_utils.h"
#include "webserver_mem_handler.h"
#include "webserver_debug_port.h"
#include "webserver_iap_handler.h"
#include "webserver_ioresult_handler.h"

//extern int vars_get_index(const char *name, int len);
extern int vars_getindex(const char *name, uint8_t rw, int len);
extern int vars_get_value(int index, char *buffer, int buffer_len);
extern int vars_get_typeid(int index, uint8_t *typeid);

extern int IAP_CGI_GetFile(char *buffer, int buffer_size);
extern int UPLOAD_CGI_GetFile(const char *name, char *buffer, int buffer_size);
extern int FILEList_Get(const char *name, char *buffer, int buffer_size);
extern int VARList_Get(const char *name, char *buffer, int buffer_size);
extern int SYSTEM_Action_Get(const char *name, char *buffer, int buffer_size);

void pre_processing_file_request(const char *name);
int processing_filename(char* input, char* output, int output_size) ;

int fs_open_custom(struct fs_file *file, const char *name)
{
	FIL *f;
	char *varname;
	char *pszExt;
	char *pszWork;
	char *data;
	int var_index;
	uint8_t vartype_id;

	char *tmp_filename;

	/* File request */
	WEBSERVER_DEBUG_PRINT("Request: %s", name);
		
	/* Get file extension */
	pszExt = NULL;
	pszWork = strchr(name, '.');
	while(pszWork) {
		pszExt = pszWork + 1;
		pszWork = strchr(pszExt, '.');
	}
	
	/* Check if it is vars request */
	//if(pszExt && (strcmp(pszExt, "txt") == 0)) {
	if(pszExt && ((strcmp(pszExt, "txt") == 0) || (strncmp((char*)name, "/html/", 6) == 0) || (strncmp((char*)name, "html/",5) == 0))) {
		/* Remove slash */
		if(*(varname=(char*)name) == '/')
			varname++;
		if (strncmp((char*)varname, "html/",5) == 0) {
			varname+=5;
		}
		
		/* If variable name is valid */
		if((var_index = vars_getindex(varname, 1, pszExt-varname-1)) >= 0) {				
			/* Get var to buffer */
			if((vars_get_typeid(var_index, &vartype_id) == 0) && (vartype_id <= 7)){
				data = (char*)_memtiny_alloc(); /* _MEMTINY_SIZE (64). IMPORTANT: Must be free with _memtiny_free() */
				file->dynamic_buffer_size = _MEMTINY_SIZE;
			}
			else {
				data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
				file->dynamic_buffer_size = _MEMLARGE_SIZE;
			}

			if(data != NULL) {
				file->is_dynamic_buffer = 1;
				vars_get_value(var_index, data, file->dynamic_buffer_size);
			
				/* Setup */	
				file->is_custom_var = 1;
			
				file->custom_bytes_count = 0;
				file->custom_bytes_index = 0;
				file->ff = NULL;
				file->data = data;
			
				/* Contents */
				file->index = file->len = strlen(data);
				file->pextension = NULL;
				file->http_header_included = 0;
			
				/* Return open is valid */
				return 1;			
			}
		}
	}
	/* Variables request as list */
	else if(pszExt && (strcmp(pszExt, "vars") == 0)) {
		char *nam = (void*)0;
		char *pos = (void*)0;
		char sep = 0;
		//char *mem_tiny_filename;

		/* Buffer allocation for vars value */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		tmp_filename = (char*)_memtiny_alloc();
		if((data == (void*)0) || (tmp_filename == (void*)0)) {
			_memlarge_free(data);
			_memtiny_free(tmp_filename);
			return 0; /* No mem */
		}

		file->dynamic_buffer_size = _MEMLARGE_SIZE;

		/* Remove slash */
		if(*(varname=(char*)name) == '/')
			varname++;		

		/* Initial file data buffer */
		data[0] = '\0';

		/* Split string by comma */
		nam = varname;
		sep = 0;
		while (((pos = strstr(nam, ",")) != (void*)0) || ((pos = strstr(nam, ".")) != (void*)0)) {
			/* Insert separater */
			if(sep)
			 { sprintf(&data[strlen(data)], "%c", sep); }

			/* prepare varname */
			strncpy(tmp_filename, nam, pos-nam);			
			//strcpy(&tmp_filename[pos-nam], ".txt");
			tmp_filename[pos-nam] = '\0';

			/* Check if variable name is existing */
			if((var_index = vars_getindex(tmp_filename, 1, strlen(tmp_filename))) >= 0) {
				/* Get variable value */
				vars_get_value(var_index, &data[strlen(data)], file->dynamic_buffer_size-strlen(data));
			}
			else { /* Invalid name */
				strcpy(&data[strlen(data)], "404: Not found.");
			}
			/* Remove comma ',' */
			nam = ++pos;
			sep = '\n';
		}

		_memtiny_free(tmp_filename);

		file->is_dynamic_buffer = 1;
		
		/* Setup */	
		file->is_custom_var = 1;
			
		file->custom_bytes_count = 0;
		file->custom_bytes_index = 0;
		file->ff = NULL;
		file->data = data;
			
		/* Contents */
		file->index = file->len = strlen(data);
		file->pextension = NULL;
		file->http_header_included = 0;
		
		/* Return open is valid */
		return 1;
	}
#if WEBSERVER_IAP_ENABLE
	/* Check if it is IAP status file */
	else if(!strcmp(name, "/system/iap.html")) {
		/* Dynamic memory allocation for file request */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		file->dynamic_buffer_size = _MEMLARGE_SIZE;
		if(data != NULL) {
			file->is_dynamic_buffer = 1;

			/* Setup */	
			file->is_custom_var = 1;			
			file->custom_bytes_count = 0;
			file->custom_bytes_index = 0;
			file->ff = NULL;
			file->data = data;
			
			/* Contents */
			file->index = file->len = IAP_CGI_GetFile(data, _MEMLARGE_SIZE);
			file->pextension = NULL;
			file->http_header_included = 0;
			
			/* Return open is valid */
			return 1;			
		}
	}
#endif
	/* Check if it is system/upload file */
	else if(!strcmp(name, "/system/upload.html") || !strcmp(name, "/system/uploadstatus.html") || !strcmp(name, "/system/upgradestatus.html")) {
		/* Dynamic memory allocation for file request */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		file->dynamic_buffer_size = _MEMLARGE_SIZE;
		if(data != NULL) {
			file->is_dynamic_buffer = 1;

			/* Setup */	
			file->is_custom_var = 1;			
			file->custom_bytes_count = 0;
			file->custom_bytes_index = 0;
			file->ff = NULL;
			file->data = data;
			
			/* Contents */
			file->index = file->len = UPLOAD_CGI_GetFile(name, data, _MEMLARGE_SIZE);
			file->pextension = NULL;
			file->http_header_included = 0;
			
			/* Return open is valid */
			return 1;			
		}
	}
	/* Directory list */
	else if(!strcmp(name, "/system/dir.text") || !strcmp(name, "/system/dir_result.text")) {
		/* Dynamic memory allocation for file request */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		file->dynamic_buffer_size = _MEMLARGE_SIZE;
		if(data != NULL) {
			file->is_dynamic_buffer = 1;

			/* Setup */	
			file->is_custom_var = 1;			
			file->custom_bytes_count = 0;
			file->custom_bytes_index = 0;
			file->ff = NULL;
			file->data = data;
			
			/* Contents */
			file->index = file->len = FILEList_Get(name, data, _MEMLARGE_SIZE);
			file->pextension = NULL;
			file->http_header_included = 0;
			
			/* Return open is valid */
			return 1;			
		}		
	}
	/* System command */
	else if(!strcmp(name, "/system/system.text")) {
		/* Dynamic memory allocation for file request */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		file->dynamic_buffer_size = _MEMLARGE_SIZE;
		if(data != NULL) {
			file->is_dynamic_buffer = 1;

			/* Setup */	
			file->is_custom_var = 1;			
			file->custom_bytes_count = 0;
			file->custom_bytes_index = 0;
			file->ff = NULL;
			file->data = data;
			
			/* Contents */
			file->index = file->len = SYSTEM_Action_Get(name, data, _MEMLARGE_SIZE);
			file->pextension = NULL;
			file->http_header_included = 0;
			
			/* Return open is valid */
			return 1;			
		}		
	}	
	/* Variable list */
	else if(!strcmp(name, "/system/varlist.text") || !strcmp(name, "/system/varlist.html")) {
		/* Dynamic memory allocation for file request */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		file->dynamic_buffer_size = _MEMLARGE_SIZE;
		if(data != NULL) {
			file->is_dynamic_buffer = 1;

			/* Setup */	
			file->is_custom_var = 1;			
			file->custom_bytes_count = 0;
			file->custom_bytes_index = 0;
			file->ff = NULL;
			file->data = data;
			
			/* Contents */
			file->index = file->len = VARList_Get(name, data, _MEMLARGE_SIZE);
			file->pextension = NULL;
			file->http_header_included = 0;
			
			/* Return open is valid */
			return 1;			
		}		
	}
	/* Check if it is IO Result file */
	else if(!strcmp(name, WEBSERVER_IORESULT_FILENAME)) {
		/* Dynamic memory allocation for file request */
		data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		file->dynamic_buffer_size = _MEMLARGE_SIZE;
		if(data != NULL) {
			file->is_dynamic_buffer = 1;

			/* Setup */	
			file->is_custom_var = 1;			
			file->custom_bytes_count = 0;
			file->custom_bytes_index = 0;
			file->ff = NULL;
			file->data = data;
			
			/* Contents */
			file->index = file->len = IOResult_GetFile(data, _MEMLARGE_SIZE);
			file->pextension = NULL;
			file->http_header_included = 0;
			
			/* Return open is valid */
			return 1;			
		}
	}

	/* File may locate on SD Card or Virtual Disk */
	if((tmp_filename = (char*)_memlarge_alloc()) == (void*)0) { /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
		return 0;
	}

	if(processing_filename((char*)name, &tmp_filename[2], _MEMLARGE_SIZE-2) != 0) {
		_memlarge_free(tmp_filename);
		return 0;
	}

	if((strncmp(&tmp_filename[2], "/0:", 3) == 0)||(strncmp(&tmp_filename[2], "/1:", 3) == 0)||(strncmp(&tmp_filename[2], "/2:", 3) == 0)) {
      f = _fopen(&tmp_filename[3], "r");
	  if(f == NULL) {
		_memlarge_free(tmp_filename);
		return 0;
	  }
	}
	else if((strncmp(&tmp_filename[2], "0:", 2) == 0)||(strncmp(&tmp_filename[2], "1:", 2) == 0)||(strncmp(&tmp_filename[2], "2:", 2) == 0)) {
      f = _fopen(&tmp_filename[2], "r");
	  if(f == NULL) {
		_memlarge_free(tmp_filename);
		return 0;
	  }
	}
	else {		
		strncpy(tmp_filename, "0:", 2);
		f = _fopen(tmp_filename, "r");
		print_file_error();
		if(f == NULL) {
			strncpy(tmp_filename, "1:", 2);
			f = _fopen(tmp_filename, "r");
			print_file_error();
			if(f == NULL) {
				_memlarge_free(tmp_filename);
				return 0;
			}
		}
	}

	/* Success */
	_memlarge_free(tmp_filename);
	
	/* Setup */	
	file->data = (char*)_memlarge_alloc(); /* _MEMLARGE_SIZE (2048). IMPORTANT: Must be free with _memlarge_free() */
    file->dynamic_buffer_size = _MEMLARGE_SIZE;

	file->custom_bytes_count = 0;
	file->custom_bytes_index = 0;
	file->ff = f;

	file->is_dynamic_buffer = 1;
	if(file->data == NULL) {
		_fclose(f);
		return 0;
	}
	file->len = f_size(f);
	file->index = f_size(f);
	file->pextension = NULL;
	file->http_header_included = 0;
	file->is_custom_var = 0;
	
	/* File valid */
	return 1;
}

void fs_close_custom(struct fs_file *file)
{
	/* Close and free */
	if(file->is_dynamic_buffer) {
		if(file->dynamic_buffer_size == _MEMTINY_SIZE) {
			_memtiny_free((char*)file->data);
		}
		else {
			_memlarge_free((char*)file->data);
		}
	}
	_fclose(file->ff);
}

int fs_read_custom(struct fs_file *file, char *buffer, int count)
{
	unsigned int actual_count;
	
	/* return 0 when success */
	if(file->is_custom_var) {
		return strlen(buffer); /* TODO: */
	}
	else {
		if(f_read(file->ff, buffer, count, &actual_count) != 0) {
			print_file_error();
			return -1;
		}
		else {
			WEBSERVER_DEBUG_PRINT("Return bytes: %d", actual_count);
		}
	}
	/* return number of bytes reading */
	return (int)actual_count;
}

int processing_filename(char* input, char* output, int output_size) {
	char tmp[3] = {0,0,0};
	char *s;
	char *s1;
	int index;
	int len;
	int val;
	
    /* Replace '+' with space */
    s1 = input;
	while((s = strstr(s1, "+")) != (char*)0) *s=' ';

    /* Decode hex */
	index = 0;
	s1 = input;
	while(*s1) {
		s = s1;
		if((s1 = strstr(s, "%")) != 0) {
			/* Copy first part */
			strncpy(&output[index], s, (len=(s1-s)));
			index += len;

			if(!strncmp(s1, "%%", 2)) {
				output[index++] = '%';
			}
			else {
				/* Processing hex */
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
			return 0;
		}
	}
	output[index] = 0;
	return 0;
}
