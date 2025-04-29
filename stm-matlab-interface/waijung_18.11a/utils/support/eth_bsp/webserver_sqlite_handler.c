
#include "waijung_hwdrvlib.h"
#include "webserver_mem_handler.h"

/* ##SqLite */

/* SQLite interface */
extern char SQLITE_Buffer[];
extern uint16_t SQLite_Buffer_Index;
static SYS_TIMER_uS_STRUCT WebSQLite_uS_Timer;
extern void SQLiteDataQuery_write(char *data, uint16_t count);
extern void SQLiteDataQuery_read(char *buffer, uint16_t buffer_size, uint16_t* reading_count);

/* Output */
#define SQLITE_OUTPUT_BUFFER_SIZE  (SQLITE_BUFFER_SIZE)
char SQliteOutputBuffer[SQLITE_OUTPUT_BUFFER_SIZE];

/* Error code */
typedef enum {
  SQLITE_ERR_PROMPT,                   /* SQLite cannot set prompt */
  SQLITE_ERR_CONNECTION,               /* SQLite connection fail */
  SQLITE_ERR_SDCARD_FAIL,              /* Failed to access SD Card */
} SQLITE_ERROR_CODE;

/* Get error message from error code:
 * Usage: _webserver_sqlite_geterrmsg(<errcode>, output, output_size);
 */
void _webserver_sqlite_geterrmsg(SQLITE_ERROR_CODE err, char *buffer, uint16_t buffer_size)
{
  switch (err) {
    /* SQLite prompt selection fail */
    case SQLITE_ERR_PROMPT:
      strcpy(buffer, "Error: Failed to set prompt.");
      break;
		
    /* SQLite connection fail/ or not inserted */
    case SQLITE_ERR_CONNECTION:
      strcpy(buffer, "Error: SQLite connection fail.");
      break;
		
    /* SD Card fail/ not inserted */
    case SQLITE_ERR_SDCARD_FAIL:
      strcpy(buffer, "Error: Failed to access system storage card.");
      break;

    /* Unknown error code */
    default:
      strcpy(buffer, "Error: Unknown.");
      break;
  }
}

/* SQLite I/O */
void _webserver_sqlite_flushrx(char *buffer, uint16_t buffer_size)
{
  uint16_t reading_count;

  SysTimer_uS_Start(&WebSQLite_uS_Timer, 200000); /* 200ms */

  /* Clean Rx buffer */
  do {
    SQLiteDataQuery_read(buffer, buffer_size, &reading_count);
    if(SysTimer_uS_IsTimeout(&WebSQLite_uS_Timer)) {
      WEBSERVER_DEBUG_PRINT("Heavy traffic pending on SQLite port.");
      return;
    }
  } while (reading_count > 0);
}

void _webserver_sqlite_reset(void)
{
  char reading;
  uint16_t reading_count;

  /* Activate reset */
  SQLITE_RESET();

  SysTimer_uS_Start(&WebSQLite_uS_Timer, 200000); /* 200ms */

  /* Wait for a byte return */
  SysTimer_uS_Start(&WebSQLite_uS_Timer, 200000); /* 200ms */
  do {
    SQLiteDataQuery_read(&reading, 1, &reading_count);
    /* If no byte return within SQLITE_RESET_WAIT*/
    if(SysTimer_uS_IsTimeout(&WebSQLite_uS_Timer)) {
      return;
    }
  } while (reading_count == 0);

  /* Wait for no byte return for 1ms*/
  SysTimer_uS_Start(&WebSQLite_uS_Timer, 1000); /* 1ms */
  do {
    SQLiteDataQuery_read(&reading, 1, &reading_count);
    if(reading_count > 0) {
      SysTimer_uS_Start(&WebSQLite_uS_Timer, 1000); /* 1ms */
    }
  } while (SysTimer_uS_IsTimeout(&WebSQLite_uS_Timer) == 0);
}

/* 0 - Success */
int8_t _webserver_sqlite_getprompt(char *buffer, uint16_t buffer_size)
{
  uint16_t reading_count;
  uint16_t reading_index;

  /* Clean Existing buffer */
  _webserver_sqlite_flushrx(buffer, buffer_size);

  /* Write \n */
  SQLiteDataQuery_write("\n", 1);
  SysTimer_uS_Start(&WebSQLite_uS_Timer, 500000); /* 500ms */
  
  /* Get */
  buffer[reading_index = 0] = '\0';
  do {
    SQLiteDataQuery_read(&buffer[reading_index], (buffer_size-reading_index-1), &reading_count);
    reading_index += reading_count;
    buffer[reading_index] = '\0';
    /* Check */
    if(strncmp(buffer, ">", 1) == 0) { /* Compatible with SQLite_FW0 */
      return 0; /* Success */
    }
    if (strcmp(buffer, SQLITE_PROMPT_STR) == 0) { /* Prompt */
      return 0; /* Success */
    }
  } while ((reading_index < buffer_size) && (SysTimer_uS_IsTimeout(&WebSQLite_uS_Timer)==0)); 

  /* Timeout or overflow */
  return 1;
}

/* 0-Success */
int8_t _webserver_sqlite_setmainprompt(char *working_buffer, uint16_t working_buffer_size)
{
  uint8_t retry_count = 0;

___retry:

  /* Get prompt */
  if(_webserver_sqlite_getprompt(working_buffer, working_buffer_size) != 0) {
    /* Reset */
    _webserver_sqlite_reset();
    /* Try again */
    if(_webserver_sqlite_getprompt(working_buffer, working_buffer_size) != 0) {
      return 1; /* Fail */
    }
  }

  /* Check prompt */
  if(strncmp(working_buffer, ">", 1) == 0) { /* Success */
    return 0;
  }
  else {
    if(retry_count ++ < 1) {
      SQLiteDataQuery_write(".quit", 5);
      goto ___retry;
    }
    return 1;
  }
}

void _webserver_sqlite_exit_sqlite_prompt(void)
{
  SQLiteDataQuery_write(".quit\n", 6);
}

/* 0 - Success */
int8_t _webserver_sqlite_getresult(char *buffer, uint16_t buffer_size, const char *prompt)
{
  char *s;
  uint16_t reading_count;
  uint16_t reading_index;

  SysTimer_uS_Start(&WebSQLite_uS_Timer, SQLITE_TIMEOUT);
  
  /* Get */
  buffer[reading_index = 0] = '\0';
  do {
    /* Ask for continue prompt */
    if(reading_index >= strlen(SQLITE_ASK_PROMPT_STR)) {
      if(strcmp(&buffer[reading_index - strlen(SQLITE_ASK_PROMPT_STR)], SQLITE_ASK_PROMPT_STR) == 0) {
        WEBSERVER_DEBUG_PRINT("SQLite ask to continue.");
        /* Roll-back */
        reading_index -= strlen(SQLITE_ASK_PROMPT_STR);
        buffer[reading_index] = '\0';
        /* Continue to receive data */
        SQLiteDataQuery_write("\n", 1);
      }
    }
    
    /* Continue prompt ?: "   ...> " */
    if((reading_index >= strlen(SQLITE_CONTINUE_PROMPT_STR)) \
      && (strcmp(&buffer[reading_index-strlen(SQLITE_CONTINUE_PROMPT_STR)], SQLITE_CONTINUE_PROMPT_STR) == 0)) {      
      s = "Error: Incomplete input query string.";
      strcpy(buffer, s);
      WEBSERVER_DEBUG_PRINT("Incomplete input query string");
      return 1;      
    }

    /* Read data */
    SQLiteDataQuery_read(&buffer[reading_index], (buffer_size-reading_index-1)>512?512:(buffer_size-reading_index-1), &reading_count);
    reading_index += reading_count;
    buffer[reading_index] = '\0';
    //WEBSERVER_DEBUG_PRINT(&buffer[reading_index-reading_count]);
    /* Check */
    if((reading_index >= strlen(prompt)) && (strcmp(&buffer[reading_index-strlen(prompt)], prompt) == 0)){
      buffer[reading_index-strlen(prompt)] = '\0';
      return 0; /* Success */      
    }

    if(reading_count > 0) {
      SysTimer_uS_Start(&WebSQLite_uS_Timer, SQLITE_TIMEOUT);
    }
  } while ((reading_index < (buffer_size-1)) && (SysTimer_uS_IsTimeout(&WebSQLite_uS_Timer)==0)); 

  if(reading_index >= (buffer_size-1)) {
    s = "\nError: More data cannot be displayed due to memory limitation.";
    strcpy(&buffer[buffer_size-strlen(s)-1], s);
    WEBSERVER_DEBUG_PRINT("Overflow: %d,%d", reading_index,buffer_size);
    /* Reset SQLite */
    SQLiteDataQuery_write(".reset\n", 7); //SQLITE_RESET();

    /* Considered as success */
    return 0;
  }
  if(SysTimer_uS_IsTimeout(&WebSQLite_uS_Timer)) {
    if(strncmp(buffer, "Error", 5) != 0) {
      strcpy(buffer, "Error: Timeout while waiting for data.");
    }
    WEBSERVER_DEBUG_PRINT("Timeout.");
  }

  /* Timeout or overflow */
  return 1;
}

/* 0 - Success */
int8_t _webserver_sqlite_submit(const char *query, const char *prompt, char *output, uint16_t output_size)
{
  /* Clean Existing buffer */
  _webserver_sqlite_flushrx(output, output_size);

  /* Write */
  SQLiteDataQuery_write((char*)query, strlen(query));

  return _webserver_sqlite_getresult(output, output_size, prompt);
}

/* CGI handle for SQLite */
int8_t processing_query(char* input, char* output, int output_size)
{
	char tmp[3];
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
			
			/* Move over */
			s1+= 2;
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

void strlowercase(char*buffer)
{
	char c;
	while(*buffer) {
		c = *buffer;
		if((c >= 'A') && (c <= 'Z'))
          *buffer = 'a' + (c-'A');
		buffer++;
	}
}

typedef struct {
  char *action;
  char *mode;
  char *db;
  char *header;
  char *query;
  char *output;
  char *table; /* Used by action: viewer */
  char *columns; /* Used by action: viewer */
  char return_file_name[32];
} SQLITE_CGI_PARAMS;

SQLITE_CGI_PARAMS sqlite_cgi_params;
const char SQLITE_DEFAULT_NONE[] = "";
const char SQLITE_DEFAULT_ACTION[] = "query";
const char SQLITE_DEFAULT_HEADER[] = "on";

void SQLite_Handle_Init(SQLITE_CGI_PARAMS *cgi_params)
{
  /* Reset parameters */
  cgi_params->action = (char*)SQLITE_DEFAULT_ACTION;
  cgi_params->mode = (char*)SQLITE_DEFAULT_NONE;
  cgi_params->db = (char*)SQLITE_DEFAULT_NONE;
  cgi_params->header = (char*)SQLITE_DEFAULT_HEADER;
  cgi_params->query = (char*)SQLITE_DEFAULT_NONE;
  cgi_params->output = (char*)SQLITE_DEFAULT_NONE;
  cgi_params->table = (char*)SQLITE_DEFAULT_NONE;
  cgi_params->columns = (char*)SQLITE_DEFAULT_NONE;
  strcpy((cgi_params->return_file_name), "/sqliteresult.txt");
}

/* Usage example:
 ** sqlite.cgi?query=select * from Test limit 1&mode=csv
 */
void SQLite_Handle_Query(SQLITE_CGI_PARAMS *cgi_params, char *working_buffer, uint16_t working_buffer_size, char *output, uint16_t output_size)
{
  FRESULT res;
  FIL *f;
  char filename[32];

  /* Check Query string */
  if (*cgi_params->query == '\0') {    /* Empty query not allowed */
    strcpy(output, "Error: Query string cannot empty.");
    return;
  }

  /* Pre-processing Query */
  if (processing_query(cgi_params->db, filename, sizeof(filename)) != 0) {
    strcpy(output, "Error: Invalid DB.");
    return;
  }

  /* Select Main prompt */
  if (_webserver_sqlite_setmainprompt(working_buffer, working_buffer_size) != 0) {
    _webserver_sqlite_geterrmsg(SQLITE_ERR_CONNECTION, output, output_size);
    return;
  }

  /* Select SQLite prompt, with specified db file name */
  sprintf(working_buffer, "sqlite %s\n", filename);
  if (_webserver_sqlite_submit(working_buffer, "sqlite> ", output, output_size) != 0) {
    if (*output == 0)
      strcpy(output, "Error: SQLite prompt fail.");
    return;
  }

  /* Set mode */
  if (*(cgi_params->mode)) {
    sprintf(working_buffer, ".mode %s\n", (cgi_params->mode));
    if ((_webserver_sqlite_submit(working_buffer, "sqlite> ", output,
          output_size) != 0) || (*output != '\0'))
      return;
  }

  /* Header */
  sprintf(working_buffer, ".header %s\n", (cgi_params->header));
  if ((_webserver_sqlite_submit(working_buffer, "sqlite> ", output, output_size) != 0) || (*output != '\0')) {
    return;
  }

  /* Query */
  if (processing_query(cgi_params->query, working_buffer, working_buffer_size) != 0) { /* Normallize Query */
    sprintf(output, "Error: Invalid Query string.");
    return;
  }

  if ((working_buffer[0] != '.') && (working_buffer[strlen(working_buffer)-1] != ';')) { /* Append ";" */
    strcpy(&working_buffer[strlen(working_buffer)], ";");
  }

  strcpy(&working_buffer[strlen(working_buffer)], "\n");
  if (_webserver_sqlite_submit(working_buffer, "sqlite> ", output, output_size) == 0) {
    /* Success */
    /* Check if return empty output, INSERT or UPDATE */
    strcpy(working_buffer, cgi_params->query);
    strlowercase(working_buffer);
    if ((strstr(working_buffer, "select") != working_buffer) && (*output == '\0'))
    {
      strcpy(output, "Query: OK.");
    }
  } else {                             /* Failed */
    if (*output == 0)
      sprintf(output, "Error: SQLite query fail.");
    return;
  }

  /* Quit */
  /* _webserver_sqlite_setmainprompt(working_buffer, working_buffer_size); */
  _webserver_sqlite_exit_sqlite_prompt();
	
  /* Empty filename */  
  filename[0] = '\0';

  /* Check output mode */  
  if(*cgi_params->output) { /* User specify output file */
    strcpy(filename, "/system/");
    strncpy(&filename[strlen(filename)], cgi_params->output, sizeof(filename)-strlen(filename));
  }
  else { /* Write to file due to size */
    /* If size > _MEMLARGE_SIZE => Write to SD Card */
    if (strlen(output) >= (_MEMLARGE_SIZE-1)) {
      strncpy(&filename[strlen(filename)], "/system/sqliteoutput.txt", sizeof(filename)-strlen(filename));
    }
  }

  /* Add NULL terminator */
  if(*filename) {
    filename[sizeof(filename)-1] = '\0';
    /* Make sure directory "system" exists */
    _dir_create("system");
  }

  /* Need return as file? */
  if (*filename) {
    WEBSERVER_DEBUG_PRINT("Redirect to file: %s", filename);
    if ((f = _fopen(filename, "w")) != (void*)0) {
      UINT written;

      /* Write HTML header */
      if(strstr(filename, "sqlite_query.htm")) {
        char *html_buffer;
        const char *html_head = "<!DOCTYPE html><html><title>SQLite Database Query</title><body>";
        const char *form_1 = "<form action=\"/sqlite.cgi\" method=\"get\">"
                             "<h3>SQLite Database Server, advance user</h3>"
                             "<input type=\"hidden\" name=\"action\" value=\"query\" readonly></h3>"
                             "<input type=\"hidden\" name=\"output\" value=\"sqlite_query.html\" readonly></h3>"
                             "<input type=\"hidden\" name=\"mode\" value=\"html\" readonly></h3>"
                             "<b>Database:</b> <input type=\"text\" name=\"db\" maxlength=\"16\" value=\"";
                             /* database.db */                            
        const char *form_2 = "\"><p><b>SQLite input query string (max 512 charactors):</b> <br><input "
                             "type=\"text\" maxlength=\"512\" size=75 name=\"query\" value=\"";
                             /* select * from sensors */
        const char *form_3 = "\"><input type=\"submit\" value=\"Submit\"><br></form>";
        const char *table_head = "<p><b>Output:</b><table border=\"1\">";
        
        /* Init buffer */
        if((html_buffer = _memlarge_alloc()) != (void*)0) {
          memset(html_buffer, ' ', _MEMLARGE_SIZE);
          html_buffer[_MEMLARGE_SIZE-1] = '\n';

          /* Setup Html head */
          strcpy(html_buffer, html_head);
          strcpy(&html_buffer[strlen(html_buffer)], form_1);
          strcpy(&html_buffer[strlen(html_buffer)], cgi_params->db);
          strcpy(&html_buffer[strlen(html_buffer)], form_2);
          processing_query(cgi_params->query, &html_buffer[strlen(html_buffer)], _MEMLARGE_SIZE-strlen(html_buffer));
          strcpy(&html_buffer[strlen(html_buffer)], form_3);
          if(!strcmp(cgi_params->mode, "html")) {
            strcpy(&html_buffer[strlen(html_buffer)], table_head);
          }
          f_write(f, html_buffer, _MEMLARGE_SIZE, &written);
          
          /* Free buffer */
          _memlarge_free(html_buffer);
        }           
      }

      /* Write output to file */
      if (f_write(f, output, strlen(output), &written) == FR_OK) {
        WEBSERVER_DEBUG_PRINT("done.");

        /* Re-direct output to file */
        strcpy((cgi_params->return_file_name), filename);
      } else {
        WEBSERVER_DEBUG_PRINT("Failed write output to file.");
      }

      /* Write HTML footer */
      if(strstr(filename, "sqlite_query.htm")) {
        const char *html_foot = "<p><a href=\"/\">home</a></body></html></body></html>";
        const char *table_foot = "</table>";        
        if(!strcmp(cgi_params->mode, "html")) {
          f_write(f, table_foot, strlen(table_foot), &written);
        }
        f_write(f, html_foot, strlen(html_foot), &written);
      }

      /* Close an open file */
      _fclose(f);
    } else {
      WEBSERVER_DEBUG_PRINT("Failed to open file: system/sqliteoutput.txt");
    }
  }
}

void SQLite_Handle_GetInfo(SQLITE_CGI_PARAMS *cgi_params, char *working_buffer,
  uint16_t working_buffer_size, char *output, uint16_t output_size)
{
  UINT written;
  int8_t sta;
  FIL *f;
  char *buff;
  int size;
  uint32_t fw_version = 0xFFFFFFFF;

  /* Init */
  sta = 0;  /* Initial as No error */
  strcpy(output, "");

  /* Create html file */
  if (_dir_create("system") != 0) { /* Make sure directory 'system' exists */
    _webserver_sqlite_geterrmsg(SQLITE_ERR_SDCARD_FAIL, output, output_size);
    sta = -1;
  }

  if (sta == 0) {
    if ((f = _fopen("system/sqlite_info.html", "w")) != (void*)0) {
      /* Write header */
      strcpy(working_buffer,
             "<!DOCTYPE html><html><title>SQLite Database Info</title><body>"
             "<h3>SQLite Database Server information.</h3>"
             "<table border=\"1\"><tr><td><b>Firmware</b></td><td>"
             );

      /* Select Main prompt */
      buff = &working_buffer[strlen(working_buffer)];
      size = working_buffer_size - (uint16_t)strlen(working_buffer);
      if ((sta == 0) && (_webserver_sqlite_setmainprompt(buff,  size) != 0)) {
        _webserver_sqlite_geterrmsg(SQLITE_ERR_CONNECTION, output, output_size);					
        sta = -1;
      }
			
      /* Get aMG SQLite Database Server FW Rev */
      if ((sta == 0) && (_webserver_sqlite_submit("version\n", "> ", buff, size) != 0)) {
		if ((sta == 0) && (_webserver_sqlite_submit("version\n", ">", buff, size) != 0)) { /* Retry, support FW0 */
          _webserver_sqlite_geterrmsg(SQLITE_ERR_CONNECTION, output, output_size);
          sta = -1;
		}
		else {
			sta = 0;
		}
      }

	  if(sta == 0) {
		  /*Version: %d*/
		  if((buff=strstr(buff, "Version")) != (void*)0) {
			  if(sscanf(buff, "Version: %u", &fw_version) != 1)
				fw_version = 0xFFFFFFFF;
		  }
	  }
      buff = &working_buffer[strlen(working_buffer)];
      size = working_buffer_size - (uint16_t)strlen(working_buffer);			

	  if(fw_version < 1) { /* Ask to upgrade sqlite firmware */
		  strcpy(buff, "<font color=#FF0000> Please upgrade SQLite firmware.</font></td></tr><tr><td><b>Library</b></td><td>");
	  }
	  else {
		strcpy(buff, "</td></tr><tr><td><b>Library</b></td><td>");
	  }

      buff = &working_buffer[strlen(working_buffer)];
      size = working_buffer_size - (uint16_t)strlen(working_buffer);			
	
      /* Select SQLite prompt */
      buff = &working_buffer[strlen(working_buffer)];
      size = working_buffer_size - (uint16_t)strlen(working_buffer);
      if ((sta == 0) && (_webserver_sqlite_submit("sqlite\n", "sqlite> ", output, output_size) != 0)) {
        if (*output == 0)
          _webserver_sqlite_geterrmsg(SQLITE_ERR_PROMPT, output, output_size);
        sta = -1;
      }
	
      /* Get SQLite library version */
      if ((sta == 0) && (_webserver_sqlite_submit(".version\n", "sqlite> ", buff, size) != 0)) {
        _webserver_sqlite_geterrmsg(SQLITE_ERR_CONNECTION, output, output_size);					
        sta = -1;
      }

      /* Footter */			
      buff = &working_buffer[strlen(working_buffer)];
      size = working_buffer_size - (uint16_t)strlen(working_buffer);
	  strcpy(buff, "</td></tr></table><p><a href=\"/\">home</a></body></html>");

      /* Write to file */
      if ((sta == 0) && (f_write(f, working_buffer, strlen(working_buffer), &written) != FR_OK)) {
        _webserver_sqlite_geterrmsg(SQLITE_ERR_SDCARD_FAIL, output, output_size);
        sta = -1;        
      }

      /* Close file */
      _fclose(f);
    } else {
      _webserver_sqlite_geterrmsg(SQLITE_ERR_SDCARD_FAIL, output, output_size);
      sta = -1;                        /* Fail */
    }
  }

  /* Return file name */
  if (sta == 0) {                      /* Success */
    strcpy((cgi_params->return_file_name), "/system/sqlite_info.html");
  } else {                             /* Failed */
    if (*output == 0) {
      strcpy(output, "Error: ");
    }
  }
}

void SQLite_Handle_Viewer(SQLITE_CGI_PARAMS *cgi_params, char *working_buffer, uint16_t working_buffer_size, char *output, uint16_t output_size)
{
  UINT written;
  int8_t sta;
  FIL *f;
  char *buff;
  int size;
  char filename[32];

  /* Init */
  sta = 0;  /* Initial as No error */
  strcpy(output, "");

  /* Init */
  sta = 0;  /* Initial as No error */
  strcpy(output, "");

  /* Create html file */
  if (_dir_create("system") != 0) { /* Make sure directory 'system' exists */
    _webserver_sqlite_geterrmsg(SQLITE_ERR_SDCARD_FAIL, output, output_size);
    sta = -1;
  }

  /* Open file */
  if ((sta == 0) && ((f = _fopen("system/sqlite_viewer.html", "w")) == (void*)0)) {
    _webserver_sqlite_geterrmsg(SQLITE_ERR_SDCARD_FAIL, output, output_size);
    sta = -1;
  }

  /* --- Database file provide ? --- */
  if((sta == 0) && (*(cgi_params->db) == '\0')) {
    const char *html_format = 
      "<!DOCTYPE html><html><title>SQLite Database Viewer</title><body>"
      "<h3>SQLite Database Viewer</h3>"
	  "<h3>Step 1 of 4: please specify data base file name</h3>"
      "<p><form action=\"/sqlite.cgi\" method=\"get\">"
      "<input type=\"hidden\" name=\"action\" value=\"viewer\"></h3>"
      "<b>Database:</b> <input type=\"text\" name=\"db\" value=\"database.db\">"
      "<input type=\"submit\" value=\"Submit\">"
      "</form>"
      "<p><a href=\"/\">home</a>"
      "</body></html>";

    /* Write to file */
    if(sta == 0) {
      UINT written;
      if(f_write(f, html_format, strlen(html_format), &written) != FR_OK) {
        _webserver_sqlite_geterrmsg(SQLITE_ERR_SDCARD_FAIL, output, output_size);
        sta = -1;        
      }
    }
  }

  /* --- Table name provided ? --- */
  else if((sta == 0) && (*(cgi_params->table) == '\0')) {
    const char *html_format = 
      "<!DOCTYPE html><html><title>SQLite Database Viewer</title><body>"
      "<h3>SQLite Database Viewer</h3>"
      "<b>Database: </b>%s <a href=\"/sqlite.cgi?action=viewer\"> change</a><br>"
	  "<p><h3>Step 2 of 4, please specify table name:</h3>"
      "<p><form action=\"/sqlite.cgi\" method=\"get\">"
      "<input type=\"hidden\" name=\"action\" value=\"viewer\"></h3>"
      "<input type=\"hidden\" name=\"db\" value=\"%s\">"
      "<b>Table:</b> %s"
	  "<input type=\"submit\" value=\"Submit\">"
      "</form>"
      "<p><a href=\"/\">home</a>"
      "</body></html>";

    /* List tables of database */
    /* Select Main prompt */
    if ((sta == 0) && (_webserver_sqlite_setmainprompt(working_buffer, working_buffer_size) != 0)) {
      _webserver_sqlite_geterrmsg(SQLITE_ERR_CONNECTION, output, output_size);
      sta = -1;
    }

    if ((sta == 0) && (processing_query(cgi_params->db, filename, sizeof(filename)) != 0)) {
      strcpy(output, "Error: Invalid DB.");
      sta = -1;
    }

    /* Select SQLite prompt, with specified db file name */
    if(sta == 0) {
      sprintf(working_buffer, "sqlite %s\n", filename);
      if (_webserver_sqlite_submit(working_buffer, "sqlite> ", output, output_size) != 0) {
        if (*output == 0)
          strcpy(output, "Error: SQLite prompt fail.");
        sta = -1; 
      }
    }

    /* Get tables */
    if(sta == 0) {
		/* Init table list */
		strcpy(output, "");
		/* Sent command */
		strcpy(working_buffer, ".tables\n");
		if (_webserver_sqlite_submit(working_buffer, "sqlite> ", output, output_size) == 0) {
			int table_count;
			char *table_list[16]; /* Max table count is 16 */
			_sstr_rtrim(output); /* Trim */
			if((table_count = _ssplit(output, table_list, 16, " ")) > 0) {
				int i;
				strcpy(working_buffer, "<select name=\"table\">");
				for(i=0; i<table_count; i++) {
					_sprintf_s(&working_buffer[strlen(working_buffer)], 
						working_buffer_size-strlen(working_buffer), 
						"<option value=\"%s\">%s</option>", 
						table_list[i], 
						table_list[i]);
				}
				strcpy(&working_buffer[strlen(working_buffer)], "</select>");
			}
			else { /* Empty */
				strcpy(working_buffer, "The specified database did not contain any data table!");
			}
		} else { /* Failed to get table list from SQLite board */
			if (*output == 0)
				sprintf(output, "Error: SQLite query fail.");
			sta = -1;
		}
    }

    /* Write to file */
    if ((sta == 0) && (_fprintf_s(f, 2048, html_format, cgi_params->db, cgi_params->db, working_buffer) < 0)) {
      sta = -1;
    }
  }
  
  /* --- Column provided? --- */
  else if((sta == 0) && (*(cgi_params->columns) == '\0')) {
    const char *html_format = 
      "<!DOCTYPE html><html><title>SQLite Database Viewer</title><body>"
      "<h3>SQLite Database Viewer</h3>"
      "<b>Database: </b>%s <a href=\"/sqlite.cgi?action=viewer\"> change</a><br>"
      "<b>Table:</b> %s <a href=\"/sqlite.cgi?action=viewer&db=%s\"> change</a><br>"
	  "<p><h3>Step 3 of 4, please select column(s):</h3>"
      "<p><form action=\"/sqlite.cgi\" method=\"get\">"
      "<input type=\"hidden\" name=\"action\" value=\"viewer\"></h3>"
      "<input type=\"hidden\" name=\"db\" value=\"%s\">"
	  "<input type=\"hidden\" name=\"table\" value=\"%s\">"
	  "<b>Columns: </b> "
	  "<input type=\"checkbox\" name=\"column\" value=\"rowid\" checked> rowid"
	  "%s"
	  "<input type=\"submit\" value=\"Submit\">"
      "</form>"
      "<p><a href=\"/\">home</a>"
      "</body></html>";

	strcpy(working_buffer, "");

	/* PRAGMA table_info(sensors) */

    /* Write to file */
    if ((sta == 0) && (_fprintf_s(f, 2048, html_format, 
		cgi_params->db, 
		cgi_params->table, 
		cgi_params->db,
		cgi_params->db, 
		cgi_params->table, 
		working_buffer) < 0)) {
      sta = -1;
    }
  }
  
  /* Close file */
  if(f) {
   _fclose(f);
  }

  /* Return file name */
  if (sta == 0) { /* Success */
    strcpy((cgi_params->return_file_name), "/system/sqlite_viewer.html");
  }
}

/* Handle SQLite */
const char * SQLITE_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	int i;
    WEBSERVER_DEBUG_PRINT("--- SQLITE_CGI_Handler ---");

    /* Init */
    SQLite_Handle_Init(&sqlite_cgi_params);

	/* Search params */
	for(i=0; i<iNumParams; i++) {
      /* Action */
      if(strcmp("action", pcParam[i]) == 0)
	    sqlite_cgi_params.action = pcValue[i];     
    
      /* Query string */
	  else if(strcmp("query", pcParam[i]) == 0)
	    sqlite_cgi_params.query = pcValue[i];

	  /* Mode */
	  else if(strcmp("mode", pcParam[i]) == 0)
		sqlite_cgi_params.mode = pcValue[i];

	  /* Db */
	  else if(strcmp("db", pcParam[i]) == 0)
		sqlite_cgi_params.db = pcValue[i];

      /* Header */
	  else if(strcmp("header", pcParam[i]) == 0)
		sqlite_cgi_params.header = pcValue[i];

      /* Output */
	  else if(strcmp("output", pcParam[i]) == 0)
		sqlite_cgi_params.output = pcValue[i];

      /* Table */
	  else if(strcmp("table", pcParam[i]) == 0)
		sqlite_cgi_params.table = pcValue[i];
	}

    /* Init SQLite output buffer */
    strcpy(SQliteOutputBuffer, "Error: Un-handled action.");

	/* Check if SQlite is currently locked by DiskIO */
	if(SQLiteDataQuery_Lock_Check() == 0) {
		/* Wait while SQLite busy from other process */
		while(SQLiteDataQuery_Background() != 0) {
		  /* Timeout implemented in sub routine */
		}

		/* Process Action */
		/* SQLite query */
		if(!strcmp(sqlite_cgi_params.action, "query")) {
		  SQLite_Handle_Query(&sqlite_cgi_params, \
							  SQLITE_Buffer, SQLITE_BUFFER_SIZE, \
							  SQliteOutputBuffer, SQLITE_OUTPUT_BUFFER_SIZE);      
		}

		/* Get SQLite info */
		else if(!strcmp(sqlite_cgi_params.action, "getinfo")) {
		  SQLite_Handle_GetInfo(&sqlite_cgi_params, \
							  SQLITE_Buffer, SQLITE_BUFFER_SIZE, \
							  SQliteOutputBuffer, SQLITE_OUTPUT_BUFFER_SIZE);      
		}
    
		/* SQLite viewer */
		else if(!strcmp(sqlite_cgi_params.action, "viewer")) {    
		  /* Collect columns name */      
		  char *columns = _memsmall_alloc(); /* size: _MEMSMALL_SIZE */
		  if(columns) {
			columns[0] = '\0'; /* Initial */
			for(i=0; i<iNumParams; i++) {
			  if(strncmp("column", pcParam[i], 4) == 0) {
				_sprintf_s(&columns[strlen(columns)], _MEMSMALL_SIZE-strlen(columns),"%s%s", *columns?",":"", pcValue[i]);
			  }
			}
			sqlite_cgi_params.columns = columns;
			/* Handle */
			SQLite_Handle_Viewer(&sqlite_cgi_params, \
								SQLITE_Buffer, SQLITE_BUFFER_SIZE, \
								SQliteOutputBuffer, SQLITE_OUTPUT_BUFFER_SIZE);      

			/* Free mem */
			_memsmall_free(columns);
		  }
		}
	}

	/* SQLite is Locked, return this error message. */
	else {
		strcpy(SQliteOutputBuffer, "Error: SQLite is currently busy.");
	}
    
    /* Return file name */
    WEBSERVER_DEBUG_PRINT("---");
    return (sqlite_cgi_params.return_file_name);
}


/* =============================SQLite DiskI/O ========================================== */

int SQLite_Disk_Initialize(void) {
	WEBSERVER_DEBUG_PRINT("xxx SQLite Disk Initialize xxx");
	/* Select Main prompt */
	if(_webserver_sqlite_setmainprompt(SQLITE_Buffer, SQLITE_BUFFER_SIZE) != 0) {
		WEBSERVER_DEBUG_PRINT("--> Failed to set main prompt.");
		return -1;
	}

	if(_webserver_sqlite_submit("disk_initialize\n", "> ", SQliteOutputBuffer, SQLITE_OUTPUT_BUFFER_SIZE) != 0) {
		WEBSERVER_DEBUG_PRINT("--> Failed to submit command.");
		return -1;  
	}
                                    
	if(strstr(SQliteOutputBuffer, "Success") == 0) {
		WEBSERVER_DEBUG_PRINT("--> Unexpected: %s", SQliteOutputBuffer);
		return -1;  
	}

	/* Success */
	return 0;
}

//int disk_read_byte_convert()

int SQLite_Disk_Read(unsigned int Sector, unsigned int Sector_count, unsigned char *buffer)
{
	char *p;
	unsigned int i;
	unsigned int j;

	unsigned int val;
	char tmp_convert[3];

	WEBSERVER_DEBUG_PRINT("xxx SQLite Disk Read xxx");

	/* Select Main prompt */
	if(_webserver_sqlite_setmainprompt(SQLITE_Buffer, SQLITE_BUFFER_SIZE) != 0) {
		WEBSERVER_DEBUG_PRINT("--> Failed to set main prompt.");
		return -1;
	}

	for(i=0; i< Sector_count; i++) {
		/* Read a sector */
		sprintf(SQLITE_Buffer, "disk_read,%u\n", (Sector+i));
		if(_webserver_sqlite_submit(SQLITE_Buffer, "> ", SQliteOutputBuffer, SQLITE_OUTPUT_BUFFER_SIZE) != 0) {
			WEBSERVER_DEBUG_PRINT("--> Failed to submit command.");
			return -1;
		}
		/* Complete */
		/*WEBSERVER_DEBUG_PRINT("%s", SQliteOutputBuffer);*/
		p = (char*)SQliteOutputBuffer;
		/* Remove white space */
		while(*p == ' ') p++;

		/* Convert */
		tmp_convert[2] = '\0';
		j = 0;
		while(j < 1024) {
			if((p[j] != 0) && (p[j+1] != 0)) {
				tmp_convert[0] = p[j];
				tmp_convert[1] = p[j+1];
				if(sscanf(tmp_convert, "%X", &val) != 1) {
					WEBSERVER_DEBUG_PRINT("Convert fail: %s", tmp_convert);
					return -1;
				}
                *buffer ++ = (unsigned char)val;
			}
			else {
				WEBSERVER_DEBUG_PRINT("Unexpected value @%u", i*512+j);
				return -1;
			}
			j += 2;
		}
	}

	WEBSERVER_DEBUG_PRINT("Read success");
	return 0;
}
