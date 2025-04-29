#include "webserver_ioresult_handler.h"

/* I/O Status, default to success */
static int ioresult_status = 0;

/* Update status */
void IOResult_SetStatus(int status)
{
	ioresult_status = status;
}

/* Generate I/O status file */
int IOResult_GetFile(char *buffer, int buffer_size)
{
	char *pBuff;
	int len;

	/* Initial buffer */
	pBuff = buffer;
	pBuff[0] = '\0';

	/* HTML Header */
	strcpy((pBuff += strlen(pBuff)), 
		"<html>\r\n"
		"<head><title>I/O Status</title></head><body>\r\n"
		);

	switch (ioresult_status) {
		/* Sucess */
	case IO_STATUS_SUCCESS:
		sprintf((pBuff += strlen(pBuff)),
			"Success.");
		break;
		/* Status is error, need return error code. */
	default:
		sprintf((pBuff += strlen(pBuff)),
			"I/O Error status: %d.", ioresult_status);
		break;
	}

	/* HTML Footer */
	strcpy((pBuff += strlen(pBuff)), "</body></html>");

	/* Return length of return data to buffer */	
	len = (int)((pBuff += strlen(pBuff)) - buffer);
	return len;
}

