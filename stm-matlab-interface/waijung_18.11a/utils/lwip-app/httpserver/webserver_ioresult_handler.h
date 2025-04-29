#include "waijung_hwdrvlib.h"

#ifndef WEBSERVER_IORESULT_HANDLER
#define WEBSERVER_IORESULT_HANDLER 1

#define IO_STATUS_SUCCESS	0 /* Last IO status success */
#define IO_STATUS_ERROR		1 /* Last IO status error */

#define WEBSERVER_IORESULT_FILENAME	"/ioresult.html"

/* Update status */
void IOResult_SetStatus(int status);

/* Generate I/O status file */
int IOResult_GetFile(char *buffer, int buffer_size);

#endif /* WEBSERVER_IORESULT_HANDLER */
