#include "mex.h"
#include "stdio.h"
#include "windows.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    DWORD len = 0;
    DWORD numDevs;
    DWORD Flags;
    DWORD ID;
    DWORD Type;
    DWORD LocId;
    UINT i;
    char SerialNumber[16];
    char Description[64];
    char TypeIndicator[32];
    
    char result[1024];
    char* buffer;
    int device_count;
    int size;
    
    /* For input parameters */
    int interface_id;
    char filter_ch;
    
    /* COM */
    CHAR ValueName[128];
    BYTE Value[128];
    BOOL bSuccess = FALSE;
    HKEY hSERIALCOMM;
    DWORD dwMaxValueNameLen;
    DWORD dwMaxValueLen;
    DWORD dwQueryInfo;
    DWORD dwMaxValueNameSizeInChars;
    DWORD dwMaxValueNameSizeInBytes;
    DWORD dwMaxValueDataSizeInChars;
    DWORD dwMaxValueDataSizeInBytes;
    CHAR* szValueName;
    BYTE* byValue;
    DWORD dwIndex;
    DWORD dwType;
    DWORD dwValueNameSize;
    DWORD dwDataSize;
    CHAR* szPort;
    LONG nEnum;
    CHAR* filter;
    
    (void) plhs;      /* unused parameters */
    (void) prhs;
    
    buffer = result;
    
    /* Check for proper number of input and output arguments */
    if (nrhs != 1) {
        mexErrMsgTxt("Invalid number of input arguments.");
    }
    if(nlhs > 1){
        mexErrMsgTxt("Too many output arguments.");
    }
    
    if(!mxIsChar(prhs[0]))
        mexErrMsgTxt("Input is invalid");
    
    device_count = 0;
    buffer[0] = 0;
    size = 1024;
    
    if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"), 0, KEY_QUERY_VALUE, &hSERIALCOMM) == ERROR_SUCCESS) {
        //Get the max value name and max value lengths
        dwQueryInfo = RegQueryInfoKey(hSERIALCOMM, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &dwMaxValueNameLen, &dwMaxValueLen, NULL, NULL);
        if (dwQueryInfo == ERROR_SUCCESS) {
            dwMaxValueNameSizeInChars = dwMaxValueNameLen + 1; //Include space for the NULL terminator
            dwMaxValueNameSizeInBytes = dwMaxValueNameSizeInChars * sizeof(CHAR);
            dwMaxValueDataSizeInChars = dwMaxValueLen/sizeof(CHAR) + 1; //Include space for the NULL terminator
            dwMaxValueDataSizeInBytes = dwMaxValueDataSizeInChars * sizeof(CHAR);
            
            //Allocate some space for the value name and value data
            szValueName = ValueName;//malloc(dwMaxValueNameLen);
            byValue = Value;//malloc(dwMaxValueLen);
            if(szValueName && byValue) {
                bSuccess = TRUE;
                
                //Enumerate all the values underneath HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM
                dwIndex = 0;
                dwValueNameSize = dwMaxValueNameSizeInChars;
                dwDataSize = dwMaxValueDataSizeInBytes;
                memset(szValueName, 0, dwMaxValueNameSizeInBytes);
                memset(byValue, 0, dwMaxValueDataSizeInBytes);
                nEnum = RegEnumValue(hSERIALCOMM, dwIndex, (LPSTR)szValueName, &dwValueNameSize, NULL, &dwType, byValue, &dwDataSize);
                while (nEnum == ERROR_SUCCESS) {
                    //If the value is of the correct type, then add it to the array
                    if (dwType == REG_SZ) {
                        szPort = (CHAR*)(byValue);
                        if(len = sprintf_s(buffer, size, ((device_count)==0)?"'%s'":",'%s'", (char*)szPort)) {
                            buffer+= len;
                            size -= len;
                            (device_count)++;
                        }
                    }
                    
                    //Prepare for the next time around
                    dwValueNameSize = dwMaxValueNameSizeInChars;
                    dwDataSize = dwMaxValueDataSizeInBytes;
                    memset(szValueName, 0, dwMaxValueNameSizeInBytes);
                    memset(byValue, 0, dwMaxValueDataSizeInBytes);
                    ++dwIndex;
                    nEnum = RegEnumValue(hSERIALCOMM, dwIndex, (LPSTR)szValueName, &dwValueNameSize, NULL, &dwType, byValue, &dwDataSize);
                }
            }
            else {
                SetLastError(ERROR_OUTOFMEMORY);
            }
        }
        
        //Close the registry key now that we are finished with it
        RegCloseKey(hSERIALCOMM);
        
        if (dwQueryInfo != ERROR_SUCCESS)
            SetLastError(dwQueryInfo);
    }
    
    /* Return */
    if(nlhs > 0)
        plhs[0]=mxCreateString(result);
}
