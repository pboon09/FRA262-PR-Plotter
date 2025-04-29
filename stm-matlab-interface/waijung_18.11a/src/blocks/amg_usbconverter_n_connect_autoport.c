
#include "mex.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "..\..\utils\devices\aMG_USBConnect\ftd2xx.h"
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\i386\\ftd2xx.lib")
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\amd64\\ftd2xx.lib")

BOOL ft231_getportname(FT_HANDLE ftHandle, char *buffer)
{
	LONG lComPortNumber;
	FT_STATUS ftStatus;
	
	ftStatus = FT_GetComPortNumber(ftHandle, &lComPortNumber);
	if ((ftStatus == FT_OK) && (lComPortNumber > 0)) {
		sprintf(buffer, "COM%d", lComPortNumber);
/*
		if (lComPortNumber > 9) {
			sprintf_s(buffer, 32, "\\\\.\\COM%d", (int)lComPortNumber);
			//sprintf_s(buffer, 32, "\\\\\\\\.\\\\COM%d", (int)lComPortNumber);
		}
		else {
			sprintf(buffer, "COM%d", lComPortNumber);
		}
*/
		return TRUE;
	}
	
	return FALSE;
}

FT_STATUS ft231_getvalid_deviceindex (int *deviceIndex)
{
	int i;
	FT_STATUS ftStatus;
	FT_DEVICE_LIST_INFO_NODE *devInfo;
	DWORD numDevs;
	
	// Default index as invalid
	*deviceIndex = -1;
	
	// Create the device information list 
	ftStatus = FT_CreateDeviceInfoList(&numDevs);
	if (ftStatus != FT_OK)
		return ftStatus;
	
	// Get device information list
	if (numDevs == 0) {
		*deviceIndex = -1; // Invalid
		return FT_OK;
	}
	
	// List the device list information
	devInfo = (FT_DEVICE_LIST_INFO_NODE *) mxMalloc(sizeof(FT_DEVICE_LIST_INFO_NODE) * numDevs);
	ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs);
	if (ftStatus == FT_OK) {
		// Search for "aMG USB Connect"
		for (i=0; i<numDevs; i++) {
			if (!strcmp(devInfo[i].Description, "aMG USB Connect")) {
				*deviceIndex = i;				
				break;
			}
		}
		
		// Search for "aMG_USBConverter-N A"
		if (*deviceIndex < 0) {
			for (i=0; i<numDevs; i++) {
				//mexPrintf("=== DeviceNo: %d ===\n", i);
				//mexPrintf("Flags: %X\n", devInfo[i].Flags);
				//mexPrintf("Type: %X\n", devInfo[i].Type);
				//mexPrintf("ID: %X\n", devInfo[i].ID);
				//mexPrintf("LocId: %X\n", devInfo[i].LocId);
				//mexPrintf("Description: %s\n", devInfo[i].Description);
				
				if (devInfo[i].Type == FT_DEVICE_2232H) {
					*deviceIndex = i;
					break;
				}
				/*
				 * FT_DEVICE_BM,
				 * FT_DEVICE_AM,
				 * FT_DEVICE_100AX,
				 * FT_DEVICE_UNKNOWN,
				 * FT_DEVICE_2232C,
				 * FT_DEVICE_232R,
				 * FT_DEVICE_2232H,
				 * FT_DEVICE_4232H,
				 * FT_DEVICE_232H,
				 * FT_DEVICE_X_SERIES
				 */
			}
		}
	}
	mxFree(devInfo);
	
	// Return status
	return ftStatus;
}

char output_port[32];
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	char *output_buf;
	int deviceIndex;
	FT_HANDLE ftHandle;
	FT_STATUS ftStatus;
	
	output_port[0] = '\0'; // Empty string
	if ((ft231_getvalid_deviceindex (&deviceIndex) == FT_OK) && (deviceIndex >= 0)) {
		ftHandle = 0;
		ftStatus = FT_Open(deviceIndex, &ftHandle);
		if (ftStatus == FT_OK) {
			ft231_getportname(ftHandle, output_port);
		}
		if (ftHandle)
			FT_Close(ftHandle);
	}
	
	/* Return */
	output_buf = mxCalloc(sizeof(output_port), sizeof(char));
	strcpy(output_buf, output_port);
	plhs[0] = mxCreateString(output_buf);
	return;
}
