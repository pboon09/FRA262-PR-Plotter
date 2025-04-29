
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <winsock2.h>
#include <iphlpapi.h>
#include <stdio.h>
#include <stdlib.h>
#include "mex.h"

#pragma comment(lib, "IPHLPAPI.lib")

void GetNetwork_AdapterList(char *desc_str, int desc_len, char *ip_str, int ip_len)
{
	IP_ADAPTER_INFO  *pAdapterInfo;
	ULONG            ulOutBufLen;
	DWORD            dwRetVal;
	
	UINT i;
	BOOL firstloop;
	PIP_ADAPTER_INFO pAdapter;
	
	desc_str[0] = '\0';
	ip_str[0] = '\0';

	pAdapterInfo = (IP_ADAPTER_INFO *) malloc( sizeof(IP_ADAPTER_INFO) );
	ulOutBufLen = sizeof(IP_ADAPTER_INFO);

	if (GetAdaptersInfo( pAdapterInfo, &ulOutBufLen) != ERROR_SUCCESS) {
		free (pAdapterInfo);
		pAdapterInfo = (IP_ADAPTER_INFO *) malloc ( ulOutBufLen );
	}

	if ((dwRetVal = GetAdaptersInfo( pAdapterInfo, &ulOutBufLen)) != ERROR_SUCCESS) {
		mexPrintf("Error: \"GetAdaptersInfo\", return: %d\n", dwRetVal);
	}

	firstloop = TRUE;
	pAdapter = pAdapterInfo;
	while (pAdapter) {
		//printf("Adapter Name: %s\n", pAdapter->AdapterName);
		//printf("Adapter Desc: %s\n", pAdapter->Description);
		//printf("\tAdapter Addr: \t");
		
		if (!firstloop) {
			sprintf(&desc_str[strlen(desc_str)], "%s", ",");
			sprintf(&ip_str[strlen(ip_str)], "%s", ",");			
		}
		else {
			firstloop = FALSE;
		}
		sprintf(&desc_str[strlen(desc_str)], "'%s'", pAdapter->Description);
		sprintf(&ip_str[strlen(ip_str)], "'%s'", pAdapter->IpAddressList.IpAddress.String);
		//for (i = 0; i < pAdapter->AddressLength; i++) {
			//if (i == (pAdapter->AddressLength - 1))
			//	printf("%.2X\n",(int)pAdapter->Address[i]);
			//else
			//	printf("%.2X-",(int)pAdapter->Address[i]);
		//}
		//printf("IP Address: %s\n", pAdapter->IpAddressList.IpAddress.String);
		//printf("IP Mask: %s\n", pAdapter->IpAddressList.IpMask.String);
		//printf("\tGateway: \t%s\n", pAdapter->GatewayList.IpAddress.String);
		//printf("\t***\n");
		//if (pAdapter->DhcpEnabled) {
		//	printf("\tDHCP Enabled: Yes\n");
		//	printf("\t\tDHCP Server: \t%s\n", pAdapter->DhcpServer.IpAddress.String);
		//}
		//else
		//  printf("\tDHCP Enabled: No\n");

		pAdapter = pAdapter->Next;
	}
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[])
{
	char ipaddr_buffer[1024];
	char desc_buffer[1024];
	
	// 1. Validate input number, type and convert data
	
	// 2. Validate number of output variable
	
	// 3. Process
	GetNetwork_AdapterList(desc_buffer, sizeof(desc_buffer), ipaddr_buffer, sizeof(ipaddr_buffer));
	
	// 4. Return value to output variable, convert from C data to Matlab data
	
	/* Return */
	//if(nlhs > 0)
		plhs[0]=mxCreateString(desc_buffer);
	if(nlhs > 1)
		plhs[1]=mxCreateString(ipaddr_buffer);
}
