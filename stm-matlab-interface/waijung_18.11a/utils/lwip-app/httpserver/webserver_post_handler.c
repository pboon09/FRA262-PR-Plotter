
#include "debug.h"
#include "stats.h"
#include "httpd.h"
#include "httpd_structs.h"
#include "tcp.h"
#include "fs.h"
#include "amg_utils.h"
#include "waijung_hwdrvlib.h"
#include "webserver_debug_port.h"
#include "webserver_mem_handler.h"
#include "webserver_iap_handler.h"

#if LWIP_HTTPD_SUPPORT_POST

typedef struct {
	FIL *f;
	enum {
		POST_SYSTEM_UPGRADE,
		POST_SYSTEM_LOGGIN,
		POST_FILE_UPLOAD,
		POST_NONE
	} PostType;
	enum {
		UPLOAD_INIT,
		UPLOAD_HEADER,
		UPLOAD_DATA
	} UploadState;
	void *connection; /* Use for identify connection */
	uint32_t content_len;
	uint32_t content_index;
	uint16_t boundary_tag_len;
	uint32_t timeout_count; /* Timeout counter if last upload is not active for a while */
	uint8_t need_reboot; /* Reboot system after upload finish */
	uint8_t system_upgrade_file;
	uint8_t system_iap_file; /* Redirect file via Webserver IAP */
	char *system_filename;
	err_t err;
	/* Packet buffer */
	uint8_t *packet_write; /* 2kByte, buffer created with _memlarge_alloc() */
	uint16_t packet_write_index; /* Bytes index */
} CGI_POST_STRUCT;

CGI_POST_STRUCT Cgi_Post_Struct = {
	(FIL*)0,
	POST_NONE,
	UPLOAD_INIT,
	(void*)0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	(char*)0,
	ERR_OK,
	(uint8_t*)0,
	0
};

char uploading_filename[128];
char uploading_pathname[128];
char uploading_fullfilepath[256];
char uploading_errorpage[64];
char uploading_successpage[64];


/* TODO: background service to check timeout
*/
void webserver_cgi_process(unsigned int tickid)
{

}

#endif

#if LWIP_HTTPD_SUPPORT_POST
/** Called when a POST request has been received. The application can decide
 * whether to accept it or not.
 *
 * @param connection Unique connection identifier, valid until httpd_post_end
 *        is called.
 * @param uri The HTTP header URI receiving the POST request.
 * @param http_request The raw HTTP request (the first packet, normally).
 * @param http_request_len Size of 'http_request'.
 * @param content_len Content-Length from HTTP header.
 * @param response_uri Filename of response file, to be filled when denying the
 *        request
 * @param response_uri_len Size of the 'response_uri' buffer.
 * @param post_auto_wnd Set this to 0 to let the callback code handle window
 *        updates by calling 'httpd_post_data_recved' (to throttle rx speed)
 *        default is 1 (httpd handles window updates automatically)
 * @return ERR_OK: Accept the POST request, data may be passed in
 *         another err_t: Deny the POST request, send back 'bad request'.
 */
err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                       u16_t http_request_len, int content_len, char *response_uri,
                       u16_t response_uri_len, u8_t *post_auto_wnd)
{
	WEBSERVER_DEBUG_PRINT("*_post_begin");
	
	WEBSERVER_DEBUG_PRINT("uri->%s", uri);
	WEBSERVER_DEBUG_PRINT("http_request->%s", http_request);
	WEBSERVER_DEBUG_PRINT("http_request_len->%d", http_request_len);

	/* Return page */
	strcpy(response_uri, "/system/uploadstatus.html");
	
	if(!strncmp("/upload.cgi", uri, 11) || !(strncmp("/iap.html", uri, 8))) {

		/* Init upload filename */
		strcpy(uploading_filename, "");
		strcpy(uploading_pathname, "");
		strcpy(uploading_fullfilepath, "");
		strcpy(uploading_errorpage, "");
		strcpy(uploading_successpage, "");

		/* Init post struct */
		Cgi_Post_Struct.PostType = POST_FILE_UPLOAD;
		Cgi_Post_Struct.UploadState = UPLOAD_INIT;
		Cgi_Post_Struct.connection = connection;
		Cgi_Post_Struct.content_len = content_len;
		Cgi_Post_Struct.boundary_tag_len = 0;
		Cgi_Post_Struct.content_index = 0;
		Cgi_Post_Struct.system_upgrade_file = 0;
		Cgi_Post_Struct.system_iap_file = 0;
		Cgi_Post_Struct.err = ERR_OK;
		
		/* Check pending from previous operation */
		if(Cgi_Post_Struct.f) {
			_fclose(Cgi_Post_Struct.f);
			Cgi_Post_Struct.f = (void*)0;
		}

		/* packet data */
		if(!Cgi_Post_Struct.packet_write)
			Cgi_Post_Struct.packet_write = (uint8_t*)_memlarge_alloc();
		Cgi_Post_Struct.packet_write_index = 0;
		
		/* Length */
		WEBSERVER_DEBUG_PRINT(" Content Length: %u", content_len);

		/* Create System directory, if not existing! */
		if(_dir_create("system") != 0) {
			WEBSERVER_DEBUG_PRINT("Failed to create system directory: \"system\"");
			Cgi_Post_Struct.err = ERR_ABRT;
		}

		/* Temporary file */
		if((Cgi_Post_Struct.f = _fopen("system/temp.txt", "w")) == (void*)0) {
			WEBSERVER_DEBUG_PRINT("Failed to create temporary file.");
			Cgi_Post_Struct.err = ERR_ABRT;
		}

		/* Timer */
		SysTimer_uS_Start(&httpwebserver_fileupload_timer, 5000000UL); /* 5 Seconds */

		/* Return error status */
		return Cgi_Post_Struct.err;
	}
	else { /* Invalid */
		Cgi_Post_Struct.PostType = POST_NONE;
		Cgi_Post_Struct.err = ERR_ABRT;
	}
	
	return ERR_ABRT;
}
#endif

#if LWIP_HTTPD_SUPPORT_POST
/** Called for each pbuf of data that has been received for a POST.
 * ATTENTION: The application is responsible for freeing the pbufs passed in!
 *
 * @param connection Unique connection identifier.
 * @param p Received data.
 * @return ERR_OK: Data accepted.
 *         another err_t: Data denied, http_post_get_response_uri will be called.
 */

err_t httpd_post_receive_data(void *connection, struct pbuf *p)
{
	//FIL *src_file = 0;
	//err_t err = ERR_OK;
	struct pbuf *q;
	unsigned int actual_count;//, actual_count2;
	//int sta;
	//char *buffer_medium = 0;
	//char *buffer_small = 0;
	//char *seperator;
	//char *tmp;
	//int i;

	/* ATTENTION: Do not return in the middle without free pbuf* */
	
	WEBSERVER_DEBUG_PRINT("*_receive_data");

	/* Save to temporary file */
	if((Cgi_Post_Struct.err == ERR_OK) && (Cgi_Post_Struct.packet_write) && (Cgi_Post_Struct.f)) {
		q = p;
		while(q && (Cgi_Post_Struct.err == ERR_OK)) {
			/* Collect 2kB packet buffer */
			if((Cgi_Post_Struct.packet_write_index + q->len) >= _MEMLARGE_SIZE) {
				/* Write Disk */
				memcpy(&(Cgi_Post_Struct.packet_write[Cgi_Post_Struct.packet_write_index]), q->payload, 
					(_MEMLARGE_SIZE-Cgi_Post_Struct.packet_write_index));

				if(f_write(Cgi_Post_Struct.f, Cgi_Post_Struct.packet_write, _MEMLARGE_SIZE, &actual_count) != 0) {
					WEBSERVER_DEBUG_PRINT("Failed to write pbuf to temporary file.");
					Cgi_Post_Struct.err = ERR_ABRT;
				}
				/* Get the remaining bytes */
				{
					uint8_t *payload;
					payload = (uint8_t *)q->payload;
					memcpy(&(Cgi_Post_Struct.packet_write[0]), &payload[_MEMLARGE_SIZE - Cgi_Post_Struct.packet_write_index], 
						(Cgi_Post_Struct.packet_write_index + q->len) - _MEMLARGE_SIZE);
					Cgi_Post_Struct.packet_write_index = (Cgi_Post_Struct.packet_write_index + q->len) - _MEMLARGE_SIZE;
				}

				/* Timer */
				SysTimer_uS_Start(&httpwebserver_fileupload_timer, 5000000UL); /* 5 Seconds */
			}
			else {
				/* Copy data to buffer */
				memcpy(&(Cgi_Post_Struct.packet_write[Cgi_Post_Struct.packet_write_index]), q->payload, q->len);
				Cgi_Post_Struct.packet_write_index += q->len;
			}

			Cgi_Post_Struct.content_index += q->len;
			q = q->next;
		}

		WEBSERVER_DEBUG_PRINT("%u/%u", Cgi_Post_Struct.content_index, Cgi_Post_Struct.content_len);
	}

	/* Free buffer */
	pbuf_free(p);

	return Cgi_Post_Struct.err;
}
#endif

#if LWIP_HTTPD_SUPPORT_POST
/*  */
const char uploaddone_str[] = 
	"<HTML>"
	"<BODY>"
	"<FONT color=\"#000000\">Success</FONT><BR><A href=\"/system/upload.html\">Upload more file...</A>"
	"or <a href=\"%s\">View file...</a>"
	"<p><a href=\"/\">home</a>"
	"</BODY>"
	"</HTML>";
const char uploadfail_str[] =
	"<HTML>"
	"<BODY><FONT color=\"#ff0000\">Fail</FONT><BR><A href=\"/system/upload.html\">Try again...</A>"
	"<p><a href=\"/\">home</a>"
	"</BODY></HTML>";

int UPLOAD_CGI_GetFile(const char *name, char *buffer, int buffer_size)
{
	/* Upload Form */
	if(!strcmp(name, "/system/upload.html")) {
		strcpy(buffer, \
			"<html><head><title>Upload file...</title></head>"
			"<body>"
			"<B>Size of file being upload should not exceed 1MB.</B><BR>"
			"<form action=\"/upload.cgi\" enctype=\"multipart/form-data\" method=\"post\">"
			"<input name=\"errorpage\" type=\"hidden\" value=\"/system/uploadstatus.html\">"
			"<input name=\"successpage\" type=\"hidden\" value=\"/system/uploadstatus.html\">"
			"Please specify a remote directory (example: <B>files/image</B>):"
			"<BR><input name=\"directory\" type=\"text\" size=\"40\">"
			"<BR>Please specify a file to upload: <BR><input name=\"datafile\" type=\"file\" size=\"40\">"
			"<div><input type=\"submit\" value=\"Upload\"> </div>"
			"</form>"
			"<p><a href=\"/\">home</a>"
			"</body>"
			"</html>");
	}
	/* Upload Status */
	else if(!strcmp(name, "/system/uploadstatus.html")) {
		switch(Cgi_Post_Struct.err) {
		case ERR_OK:
			sprintf(buffer, uploaddone_str, uploading_fullfilepath);
			break;
		default:
			strcpy(buffer, uploadfail_str);
			break;
		}
	}
	/* Upgrade Status */
	else if(!strcmp(name, "/system/upgradestatus.html")) {
		FRESULT fres = FR_NO_FILE;
		FIL *f;
		UINT reading_count = 0;		

		/* Open file for read */
		if((f = _fopen("system/upgradestatus.html", "r")) != 0) {
			/* Load file and return as string */
			fres = f_read(f, buffer, buffer_size-1, &reading_count);

			/* Close */
			_fclose(f);
		}

		if(fres == FR_OK) {
			buffer[reading_count] = '\0';
			return reading_count;
		}
		else {
			strcpy(buffer, "Error: failed to get System upgrade status!");
		}
	}
	/* Invalid */
	else {
		strcpy(buffer, "");
	}
	return strlen(buffer);
}

/** Called when all data is received or when the connection is closed.
 * The application must return the filename/URI of a file to send in response
 * to this POST request. If the response_uri buffer is untouched, a 404
 * response is returned.
 *
 * @param connection Unique connection identifier.
 * @param response_uri Filename of response file, to be filled when denying the request
 * @param response_uri_len Size of the 'response_uri' buffer.
 */
void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{	
	//uint8_t upload_sta;
	uint32_t actual_count;

	WEBSERVER_DEBUG_PRINT("*post_finished");

	/* Write the rest of data into Disk:
	** This data is needed when file size is not multiple of 2kB.
	*/
	if(Cgi_Post_Struct.err == ERR_OK) {
		if(Cgi_Post_Struct.packet_write_index > 0) {
			WEBSERVER_DEBUG_PRINT("Write Rest bytes: %u", Cgi_Post_Struct.packet_write_index);
			if(f_write(Cgi_Post_Struct.f, Cgi_Post_Struct.packet_write, Cgi_Post_Struct.packet_write_index, &actual_count) != 0) {
				WEBSERVER_DEBUG_PRINT("Failed to write pbuf to temporary file.");
				Cgi_Post_Struct.err = ERR_ABRT;
			}
		}
	}

	/* Close file */
	if(Cgi_Post_Struct.f) {
		_fclose(Cgi_Post_Struct.f);
		Cgi_Post_Struct.f = (void*)0;
	}

	/* Free buffer */
	if(Cgi_Post_Struct.packet_write) {
		_memlarge_free(Cgi_Post_Struct.packet_write);
	}

	/* Load file information */
	if(Cgi_Post_Struct.content_index == Cgi_Post_Struct.content_len)
	{
		#define POST_ITEM_ERRORPAGE		0
		#define POST_ITEM_SUCCESSPAGE	1
		#define POST_ITEM_DIRECTORY		2
		#define POST_ITEM_DATAFILE		3

		//uint8_t eoh;
		FIL *f;
		uint32_t reading_count;
		char *pBuff, *s, *eol;
		int len, line_index, post_item;
		uint32_t data_offset;
		uint8_t data_ready;
		char magic_byte = 0;

		/* Working buffer */
		char *post_header = _memlarge_alloc(); /* Must be free with _memlarge_free() */
		char *header_line = _memsmall_alloc(); /* Must be free with _memsmall_free() */
		char *header_boundary = _memsmall_alloc(); /* Must be free with _memsmall_free() */

		/* Header processing */
		if((f = _fopen("system/temp.txt", "r")) == (void*)0) {
			WEBSERVER_DEBUG_PRINT("Failed to open: \"system/temp.txt\"");	
			Cgi_Post_Struct.err = ERR_ABRT;
		}
		else {
			post_header[0] = '\0';
			/* Load first 2kB header into buffer */
			if(f_read(f, post_header, _MEMLARGE_SIZE, &reading_count) == FR_OK) {
				magic_byte = post_header[_MEMLARGE_SIZE-1];

				if(reading_count<_MEMLARGE_SIZE)
					post_header[reading_count] = '\0'; /* Append string terminater */
				else
					post_header[_MEMLARGE_SIZE-1] = '\0'; /* Append string terminater */				
			}
			else {
				WEBSERVER_DEBUG_PRINT("Failed to read header form file.");							
				Cgi_Post_Struct.err = ERR_ABRT;
			}

			/* Initial pBuff (Dynamic pointer) */
			pBuff = (char *)post_header;

			/* Boundary tag */
			if(Cgi_Post_Struct.err == ERR_OK) {				
				eol = strstr(pBuff, "\r\n");
				if(eol) {					
					memcpy(header_boundary, pBuff, (eol-pBuff));
					header_boundary[(eol-pBuff)] = '\0'; /* Append string terminater */					
				}
				else 
				{ Cgi_Post_Struct.err = ERR_ABRT; }
			}

			/*
			------WebKitFormBoundary0xHIfBCuFvRZcsZV
			Content-Disposition: form-data; name="errorpage"

			uploadfail.html
			------WebKitFormBoundary0xHIfBCuFvRZcsZV
			Content-Disposition: form-data; name="successpage"

			uploadsuccess.html
			------WebKitFormBoundary0xHIfBCuFvRZcsZV
			Content-Disposition: form-data; name="directory"

			system/upgrade
			------WebKitFormBoundary0xHIfBCuFvRZcsZV
			Content-Disposition: form-data; name="datafile"; filename="ASDF.txt"
			Content-Type: text/plain

			Test
			------WebKitFormBoundary0xHIfBCuFvRZcsZV--
			*/
			/* Search header */
			line_index = -1;
			post_item = -1;
			data_ready = 0;
			while(!data_ready && ((eol = strstr(pBuff, "\r\n")) != (char *)0)) {
				/* Line */
				len = eol - pBuff;				
				memcpy(header_line, pBuff, len);
				header_line[len] = '\0';

				/* Move pBuff */
				pBuff = eol + strlen("\r\n");

				/* Wait for Boundary */
				if(!strcmp(header_line, header_boundary))
					line_index = 0;

				/* Line process */
				switch(line_index) {
					/* Boundary */
					case 0:
						line_index ++;
						break;

					/* Line 1: Content-Disposition */
					case 1:
						line_index ++;

						/* Accept value: "errorpage", "successpage", "directory", "datafile" */
						if(strstr(header_line, "\"errorpage\"")) {
							post_item = POST_ITEM_ERRORPAGE;
						}
						else if(strstr(header_line, "\"successpage\"")) {
							post_item = POST_ITEM_SUCCESSPAGE;
						}
						else if(strstr(header_line, "\"directory\"")) {
							post_item = POST_ITEM_DIRECTORY;
						}
						else if(strstr(header_line, "\"datafile\"")) {
							post_item = POST_ITEM_DATAFILE;
							/* Get file name */
							if((s=strstr(header_line, "filename")) != 0) {
								if((s=strstr(s, "\"")) != 0) {
									char *e;
									s++; /* Move 1 pos */
									if((e=strstr(s, "\"")) != 0) {
										strncpy(uploading_filename, s, (e-s)); /* Store file name */
										uploading_filename[(e-s)] = '\0';
									}
								}
							}
						}
						else {
							/* Unknown, ignore */
							line_index = -1;
						}
						break;

					/* Line 2: EOL ("\r\n")/ Content-Type*/
					case 2:
						line_index ++;
						break;

					/* Line 3: <Content> */
					case 3:
						switch(post_item)
						{
							case POST_ITEM_ERRORPAGE:
								if(header_line[0])
									strncpy(uploading_errorpage, header_line, sizeof(uploading_errorpage));
								break;
							case POST_ITEM_SUCCESSPAGE:
								if(header_line[0])
									strncpy(uploading_successpage, header_line, sizeof(uploading_successpage));
								break;
							case POST_ITEM_DIRECTORY:
								if(header_line[0])
									strncpy(uploading_pathname, header_line, sizeof(uploading_pathname));
								break;
							case POST_ITEM_DATAFILE:
								/* Now, pBuff is start of data */
								data_offset = (pBuff - post_header);
								data_ready = 1;
								break;
						}
						line_index ++;
						break;

					/* Line 4: */
					case 4:
						break;
				}				
			}

			/* Copy file */
			if(data_ready) {
				char sep;
				FIL *dest_f;

				/* Full file path */
				if(uploading_pathname[0] != 0) {
					/* Find seperator */
					sep = '/';
					if(strstr(uploading_pathname, "\\"))
						sep = '\\';
					/* Combine full path */
					memset(uploading_fullfilepath, 0, sizeof(uploading_fullfilepath));
					strcpy(uploading_fullfilepath, uploading_pathname);
					uploading_fullfilepath[strlen(uploading_fullfilepath)] = sep;
					strcpy(&uploading_fullfilepath[strlen(uploading_fullfilepath)], uploading_filename);

					/* Create destination directory */
					WEBSERVER_DEBUG_PRINT("Create directory (if not existed): %s", uploading_pathname);
					_dir_create(uploading_pathname);
				}
				else {
					/* File name without directory */
					strcpy(uploading_fullfilepath, uploading_filename);
				}

				/* Open destinatino file for Write */
				WEBSERVER_DEBUG_PRINT("Create destination file: %s", uploading_fullfilepath);
				if((dest_f = _fopen(uploading_fullfilepath, "w")) == (void*)0) {
					WEBSERVER_DEBUG_PRINT("Failed to create file.");
					Cgi_Post_Struct.err = ERR_ABRT;
				}
				else {
					FRESULT fres;
					UINT /*reading_count,*/ writing_count, writing_index;
					int packet_counter = 0;
					char *write_packet_buffer;

					/* Allocate Write packet buffer */
					write_packet_buffer = _memlarge_alloc(); /* Must be free with _memlarge_free() */
					if(write_packet_buffer) {
						WEBSERVER_DEBUG_PRINT("Copying file...");

						writing_index = (reading_count - data_offset);
						/* The rest bytes from header processing */
						post_header[_MEMLARGE_SIZE-1] = magic_byte;
						memcpy(&write_packet_buffer[0], &post_header[data_offset], writing_index);
						
						/* Copy file source file to destination file */
						while(!f_eof(f) && (++packet_counter < 10240) && (Cgi_Post_Struct.err == ERR_OK)) { /* packet_counter: limit loop count */
							/* Read data from source */
							// WEBSERVER_DEBUG_PRINT("Read source file...");
							if((fres = f_read(f, post_header, _MEMLARGE_SIZE, &reading_count)) == FR_OK) {
								/* Buffer need more bytes */
								if((writing_index + reading_count) < _MEMLARGE_SIZE) {
									memcpy(&write_packet_buffer[writing_index], post_header, reading_count);
									writing_index+= reading_count;
								}
								/* Over buffer size */
								else {
									memcpy(&write_packet_buffer[writing_index], post_header, _MEMLARGE_SIZE-writing_index);

									/* Write data to destination */
									// WEBSERVER_DEBUG_PRINT("Write destination file...");
									if((fres = f_write (dest_f, write_packet_buffer, _MEMLARGE_SIZE, &writing_count)) == FR_OK) {
										// WEBSERVER_DEBUG_PRINT("Packet %d, %d bytes", packet_counter, writing_count);

										/* Rest bytes */
										memcpy(&write_packet_buffer[0], &post_header[(_MEMLARGE_SIZE-writing_index)], reading_count-(_MEMLARGE_SIZE-writing_index));
										writing_index = reading_count-(_MEMLARGE_SIZE-writing_index);

										/* Small delay may required */
										SysTimer_delay_us(50);
									}
									else {
										WEBSERVER_DEBUG_PRINT("Failed to write data to file, code: %d", (int)fres);
										Cgi_Post_Struct.err = ERR_ABRT;
									}									
								}					
							}
							else {
								WEBSERVER_DEBUG_PRINT("Failed to read data from file, code: %d", (int)fres);
								Cgi_Post_Struct.err = ERR_ABRT;
							}
						}

						/* Write Rest of data */
						if((Cgi_Post_Struct.err == ERR_OK) && (writing_index > 0)) {
							if((fres = f_write (dest_f, write_packet_buffer, writing_index, &writing_count)) != FR_OK) {
								WEBSERVER_DEBUG_PRINT("Failed to write data to file, code: %d", (int)fres);
								Cgi_Post_Struct.err = ERR_ABRT;
							}
						}

						/* Remove End boundary tag */
						if(Cgi_Post_Struct.err == ERR_OK) {
							WEBSERVER_DEBUG_PRINT("Seek file.");
							fres = f_lseek(dest_f, (f_size(dest_f)-(strlen("\r\n") + strlen(header_boundary) + strlen("--\r\n"))));
							if(fres != FR_OK) {
								WEBSERVER_DEBUG_PRINT("Failed to Seek file pos, code: %d", (int)fres);
								Cgi_Post_Struct.err = ERR_ABRT;
							}
						}

						/* Truncate file */
						if(Cgi_Post_Struct.err == ERR_OK) {
							WEBSERVER_DEBUG_PRINT("Truncate file.");
							fres = f_truncate(dest_f);
							if(fres != FR_OK) {
								WEBSERVER_DEBUG_PRINT("Failed to Truncate file, code: %d", (int)fres);
								Cgi_Post_Struct.err = ERR_ABRT;
							}
						}

						if(Cgi_Post_Struct.err == ERR_OK) {
							WEBSERVER_DEBUG_PRINT("Success.");
						}

						/* Free mem */
						_memlarge_free(write_packet_buffer);
					}
					else {
						WEBSERVER_DEBUG_PRINT("Fail to allocate memory for write buffer.");
						Cgi_Post_Struct.err = ERR_ABRT;
					}

					/* Close destination file */
					_fclose(dest_f);
				}
			}
			else {
				WEBSERVER_DEBUG_PRINT("Could not find file content.");	
				Cgi_Post_Struct.err = ERR_ABRT;
			}

			/* Close file */
			_fclose(f);
		}

		/* Free working buffer */
		_memlarge_free(post_header);
		_memsmall_free(header_line);
		_memsmall_free(header_boundary);
	}
	else {
		WEBSERVER_DEBUG_PRINT("File is courrupted!");	
		Cgi_Post_Struct.err = ERR_ABRT;
	}

	if(Cgi_Post_Struct.err == ERR_OK) {
		/* System upgrade file */
		if(!strcmp(uploading_pathname, "system/upgrade")) {
			WEBSERVER_DEBUG_PRINT(">>> System upgrade file: %s", uploading_fullfilepath);

			/* ============= Main program ========================================= */
			if(Cgi_Post_Struct.content_len < (512*1024)) {
				if(webserver_setupgrade_tofile(uploading_fullfilepath, 0, 1) == 0) {
					/* Write upgrade status file */
					{
						UINT written_count;
						FIL *f;
						const char system_upgrade_wait[] = 					
							"<!DOCTYPE html>\r\n"
							"<head><title>Please wait...</title>\r\n<meta http-equiv=\"refresh\" content=\"20; url=/system/upgrade/conf.ini\">\r\n</head>\r\n"
							"<body>\r\n"						
							"<h4>System main program upgrade is in progress, please wait (less than 20 seconds) ...</h4>\r\n"
							"</body>\r\n"
							"</html>"
							;
					
						if((f = _fopen("system/upgradestatus.html", "w")) != 0) {
							/* Write content */
							if(f_write(f, system_upgrade_wait, strlen(system_upgrade_wait), &written_count) != FR_OK)
								Cgi_Post_Struct.err = ERR_ABRT;

							/* Close */
							_fclose(f);
						}
						else {
							Cgi_Post_Struct.err = ERR_ABRT;
						}

						if(Cgi_Post_Struct.err == ERR_OK) {
							/* Activate */
							webserver_upgrade_activate(0xBB);

							strcpy(response_uri, "/system/upgradestatus.html");
							return;
						}
					}
				}
				else {
					WEBSERVER_DEBUG_PRINT("Failed to write configuration file.");	
					Cgi_Post_Struct.err = ERR_ABRT;
				}
			}
			/* ============= W2D ========================================= */
			else {
				/* Set upgrade status to conf.ini */
				if(webserver_setupgrade_tofile(uploading_fullfilepath, 1, 1) == 0) {
					/* Write upgrade status file */
					{
						UINT written_count;
						FIL *f;
						const char system_upgrade_wait[] = 					
							"<!DOCTYPE html>\r\n"
							"<head><title>Please wait...</title>\r\n<meta http-equiv=\"refresh\" content=\"10; url=/system/upgradestatus.html\">\r\n</head>\r\n"
							"<body>\r\n"						
							"<h4>System W2D upgrade is in progress, please wait (less than 30 seconds) ...</h4>\r\n"
							"</body>\r\n"
							"</html>"
							;
					
						if((f = _fopen("system/upgradestatus.html", "w")) != 0) {
							/* Write content */
							if(f_write(f, system_upgrade_wait, strlen(system_upgrade_wait), &written_count) != FR_OK)
								Cgi_Post_Struct.err = ERR_ABRT;

							/* Close */
							_fclose(f);
						}
						else {
							Cgi_Post_Struct.err = ERR_ABRT;
						}

						if(Cgi_Post_Struct.err == ERR_OK) {
							/* Activate */
							webserver_upgrade_activate(0xAA);

							strcpy(response_uri, "/system/upgradestatus.html");
							return;
						}
					}
				}
				else {
					WEBSERVER_DEBUG_PRINT("Failed to write configuration file.");	
					Cgi_Post_Struct.err = ERR_ABRT;
				}
		  }
		}
#if WEBSERVER_IAP_ENABLE
		/* System iap file */
		else if(strncmp(uploading_pathname, "system/iap", 10) == 0) {
			WEBSERVER_DEBUG_PRINT(">>> System IAP file: %s", uploading_fullfilepath);

			/* Activate IAP status, with specify filename */
			webserver_iap_activate(uploading_fullfilepath);

			/* Return IAP process */			
			strcpy(response_uri, "/iap.html");
			return;
		}
#endif
	}

	/* Check re-direct status page */
	{
		if((uploading_errorpage[0] != 0) && (Cgi_Post_Struct.err != ERR_OK)) {
			WEBSERVER_DEBUG_PRINT(">>> Re-direct error page: %s", uploading_errorpage);
			strcpy(response_uri, uploading_errorpage);
			return;
		}
		else if ((uploading_successpage[0] != 0) && (Cgi_Post_Struct.err == ERR_OK)) {
			WEBSERVER_DEBUG_PRINT(">>> Re-direct success page: %s", uploading_successpage);
			strcpy(response_uri, uploading_successpage);
			return;
		}
	}

	/* Return upload status */
	strcpy(response_uri, "/system/uploadstatus.html");
	return;
}

#endif
