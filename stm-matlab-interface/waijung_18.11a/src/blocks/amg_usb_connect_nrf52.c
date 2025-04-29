#include "mex.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

/* Usage:
 * 1. Check connection:
 *    => [sta, msg] = amg_usb_connect_nrf52('connect')
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 2. Write program:
 *    => [sta, msg] = amg_usb_connect_nrf52('writeflash', <membase>, <memsize>, <filename>);
 *    =>Example, [sta, msg] = amg_usb_connect_nrf52('writeflash', hex2dec('26000'), hex2dec('40000'), 'digital_io_demo.bin');
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 3. Write public key:
 *    => [sta, msg] = amg_usb_connect_nrf52('publickey', <membase>, <memsize>, <public_key>);
 *    =>Example, [sta, msg] = amg_usb_connect_nrf52('publickey', hex2dec('7D000'), hex2dec('1000'), '123bd49bffb387f1de3dc8dc1fc8fcce952739d331fafeb7dd16837154ace7994a0a05af319f68e2a07478477b8db746643d6dd7f5f191193738984ffcd2b963');
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 4. Full Erase:
 *    => [sta, msg] = amg_usb_connect_nrf52('fullerase');
 *    Where, sta = 1 Success, 0 - Error.
 *
 * 5. Run:
 *    => [sta, msg] = amg_usb_connect_nrf52('run');
 *    Where, sta = 1 Success, 0 - Error.
 */

#include "..\..\utils\devices\aMG_USBConnect\ftd2xx.h"
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\i386\\ftd2xx.lib")
#pragma comment(lib,"..\\..\\utils\\devices\\aMG_USBConnect\\amd64\\ftd2xx.lib")

//#include "..\..\utils\devices\amgprog\ftd2xx.h"
//#pragma comment(lib,"..\\..\\utils\\devices\\amgprog\\x86\\ftd2xx.lib")
//#pragma comment(lib,"..\\..\\utils\\devices\\amgprog\\x64\\ftd2xx.lib")

FT_STATUS IAP_Comm_Write (FT_HANDLE ftHandle, const char *TxBuffer, ULONG TxBytes);
FT_STATUS IAP_Comm_Read (FT_HANDLE ftHandle, ULONG RxBytes, char *RxBuffer, ULONG *BytesReceived, ULONG timeout_ms);

#define IAP_LOGGING_HDR "aMG USB Connect: "


/* mex waijung_amg_prog.c "..\utils\devices\amgprog\x64\ftd2xx.lib" */
/* mex waijung_amg_prog.c "..\utils\devices\amgprog\x86\ftd2xx.lib" */
int _validate_ft231x(FT_HANDLE ftHandle, const char *key_str)
{
	FT_STATUS res;
	char Manufacturer[64];
	char ManufacturerId[64];
	char Description[64];
	char SerialNumber[64];
	FT_EEPROM_HEADER ft_eeprom_header; 
	FT_EEPROM_X_SERIES ft_eeprom_x;

	/* Configuration */
	ft_eeprom_header.deviceType = FT_DEVICE_X_SERIES;
	ft_eeprom_x.common = ft_eeprom_header;
	ft_eeprom_x.common.deviceType = FT_DEVICE_X_SERIES;

	/* Read */
	res = FT_EEPROM_Read(
		ftHandle,
		&ft_eeprom_x,
		sizeof(ft_eeprom_x),
		&Manufacturer[0],
		&ManufacturerId[0],
		&Description[0],
		&SerialNumber[0]
		);

	//mexPrintf("Device EEPROM:\n");
	//mexPrintf("\t->Manufacturer: %s\n", Manufacturer);
	//mexPrintf("\t->ManufacturerId: %s\n", ManufacturerId);
	//mexPrintf("\t->Description: %s\n", Description);
	//mexPrintf("\t->SerialNumber: %s\n", SerialNumber);
	
	if ((res == 0) && !strcmp(key_str, Description/*Manufacturer*/)) {
		return 1;
	}
	else {
		return 0;
	}
}

// ========================================================================
// BOOT Mode Control
// ========================================================================
/* CBUS0 -> RESET
 * CBUS1 -> BOOT0
 */
#define RESET_H() _reset_cntrl(ftHandle, TRUE) //FT_ClrRts(ftHandle)
#define RESET_L() _reset_cntrl(ftHandle, FALSE ) //FT_SetRts(ftHandle)
#define BOOT0_H() _boot_cntrl (ftHandle, FALSE) //FT_SetDtr(ftHandle)
#define BOOT0_L() _boot_cntrl (ftHandle, TRUE ) //FT_ClrDtr(ftHandle)

static BYTE _bitband_mask = 0x33;
FT_STATUS _reset_cntrl(FT_HANDLE ftHandle, BOOL state)
{
	if(state)
		_bitband_mask |= 0x01;
	else
		_bitband_mask &= 0xFE;
	return FT_SetBitMode(ftHandle, _bitband_mask, 0x20);
}

FT_STATUS _boot_cntrl(FT_HANDLE ftHandle, BOOL state)
{
	if(state)
		_bitband_mask |= 0x02;
	else
		_bitband_mask &= 0xFD;
	return FT_SetBitMode(ftHandle, _bitband_mask, 0x20);	
}

void _enter_iap_mode(FT_HANDLE ftHandle)
{
	FT_STATUS res = 0;

	/* Reset */
	_bitband_mask = 0x33;
	
	res |= RESET_L();
	res |= BOOT0_H();
	Sleep(100);
	//res |= BOOT0_H();
	//Sleep(20);
	res |= RESET_H();
	Sleep(200);
	if (res != 0)
		mexPrintf("Error: failed to control Reset/boot pin.\n");
}

void _exit_iap_mode(FT_HANDLE ftHandle)
{
	FT_STATUS res = 0;

	res |= RESET_L();
	res |= BOOT0_L();
	Sleep(100);
	res |= RESET_H();
	
	if (res != 0)
		mexPrintf("Error: failed to control Reset/boot pin.\n");
}

// ========================================================================
// IAP Protocol
// ========================================================================
// Refer to AN3155 Application note: 
// USART protocol used in the STM32? bootloader

// Response
#define IAP_ACK      0x79
#define IAP_NACK     0x1F

// Command
#define IAP_CMD_GET				0x00 /* Gets the version and the allowed commamds 
										supported by the current version of the 
										bootloader */
#define IAP_CMD_GET_VER_RDP		0x01 /* Gets the bootloader version and the Read 
										Protection status of the Flash memory */
#define IAP_CMD_GET_ID			0x02 /* Gets the chip ID */
#define IAP_CMD_READ_MEM		0x11 /* Reads up to 256 bytes of memory starting
										from an address specified by the application */
#define IAP_CMD_GO				0x21 /* Jumps to user applicatin code located in
										the internal Flash or in SRAM */
#define IAP_CMD_WRITE			0x31 /* Writes up to 256 bytes to the RAM or 
										Flash memory starting from an address 
										specified by the application */
#define IAP_CMD_ERASE			0x43 /* Erase from one to all the Flash memory pages */
#define IAP_CMD_EXTND_ERASE		0x44 /* Erase from one to all the Flash memory pages
										using two byte address mode available only 
										for v3.0 usart bootloader version and above). */
#define IAP_CMD_WRITE_PROTECT   0x63 /* Enabled the write protect for some sectors */
#define IAP_CMD_WRITE_UNPROTECT 0x73 /* Disable the write protection for all Flash 
										memory sectors */
#define IAP_CMD_READ_PROTECT	0x82 /* Enables the read protection */
#define IAP_CMD_READ_UNPROTECT  0x92 /* Disables the read protection */

#define IAP_CMD_UNLOCK_BOOT_UPGRADE     0xAA /* Use Primary booloader */

typedef struct {
	unsigned char Cmd_Get;
	unsigned char Cmd_Get_Ver_Rdp;
	unsigned char Cmd_Get_Id;
	unsigned char Cmd_Read_Mem;
	unsigned char Cmd_Go;
	unsigned char Cmd_Write;
	unsigned char Cmd_Erase;
	unsigned char Cmd_Erase_Ext;
	unsigned char Cmd_Write_Protect;
	unsigned char Cmd_Write_UnProtect;
	unsigned char Cmd_Read_Protect;
	unsigned char Cmd_Read_UnProtect;
} IAP_CMD_SUPPORT;

typedef struct {
	unsigned char Byte1;
	unsigned char Byte2;
} OPTION_BYTES;

BOOL IAP_Exit(FT_HANDLE ftHandle)
{
	_exit_iap_mode(ftHandle);
	return TRUE;
}

BOOL IAP_Init(FT_HANDLE ftHandle, IAP_CMD_SUPPORT *IapCmdSupport, OPTION_BYTES *OptionsByte, DWORD *Pid, unsigned char *Bootloader_version)
{
	int i, CmdCount;
	char TxData[2];
	char RxBuffer[32];
	char tmp[1024];
	ULONG BytesReceived;
	
	/* Reset */
	memset(IapCmdSupport, sizeof(IAP_CMD_SUPPORT), 0);
	memset(OptionsByte, sizeof(OPTION_BYTES), 0);
	
	/* Initial IAP communication */
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Initializing...\n");
	
	/* Enter BOOT mode */
	_enter_iap_mode(ftHandle);
	Sleep(200); /* Some delay after MCU reset */
	
	// Flush buffer here
	//FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX); // Purge both Rx and Tx buffers
	i = 0;
	while(! IAP_Comm_Read (ftHandle, 1024, &tmp[0], &BytesReceived, 10) && (BytesReceived > 0) && (++i < 100));
	
	Sleep(100); /* Some delay after MCU reset */	
	IAP_Comm_Write(ftHandle, "\x7F", 1);
	if(! IAP_Comm_Read (ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
		return FALSE;
	if (RxBuffer[0] != IAP_ACK) {
		return FALSE;
	}

	/* MCU Operation in Boot */
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("IAP mode enabled\n");
	
	/* === Get supported command === */
	TxData[0] = IAP_CMD_GET;
	TxData[1] = ~IAP_CMD_GET;
	IAP_Comm_Write(ftHandle, TxData, 2);	
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Get supported command.\n");
	Sleep(100);
	/* Get list of supported command */
	if (! IAP_Comm_Read (ftHandle, 32, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived < 4)) {
		mexPrintf("Read failed, %d bytes: %X...\n", BytesReceived, RxBuffer[0]);
		return FALSE;
	}
	if (!((RxBuffer[0] == IAP_ACK) && (RxBuffer[BytesReceived-1] == IAP_ACK))) {
		mexPrintf("NAK.\n");
		return FALSE;
	}

	/* 1: Command support count */
	CmdCount = (int)(RxBuffer[1]);
	if (CmdCount != ((int)BytesReceived-4)) {
		mexPrintf(IAP_LOGGING_HDR);
		mexPrintf("Internal error!\n");
		return FALSE;
	}
	
	/* 2: Boot loader version */
	*Bootloader_version = RxBuffer[2];
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Bootloader v%d.%d\n", (int)(RxBuffer[2] >> 4), (int)(RxBuffer[2] & 0x0F));
	
	/* 3: .... (3+CmdCount) */
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Support => ");
	for (i=0; i<CmdCount; i++) {
		switch (RxBuffer[3+i]) {
			case IAP_CMD_GET: 
				IapCmdSupport->Cmd_Get = 1;
				//mexPrintf("GET");
				break;
			case IAP_CMD_GET_VER_RDP: 
				IapCmdSupport->Cmd_Get_Ver_Rdp = 1;
				//mexPrintf(", GET_VER_RDP");
				break;
			case IAP_CMD_GET_ID: 
				IapCmdSupport->Cmd_Get_Id = 1;
				//mexPrintf(", GET_ID");
				break;
			case IAP_CMD_READ_MEM: 
				IapCmdSupport->Cmd_Read_Mem = 1;
				//mexPrintf(", READ_MEM");
				break;
			case IAP_CMD_GO: 
				IapCmdSupport->Cmd_Go = 1;
				//mexPrintf(", GO");
				break;
			case IAP_CMD_WRITE: 
				IapCmdSupport->Cmd_Write = 1;
				//mexPrintf(", WRITE");
				break;
			case IAP_CMD_ERASE: 
				IapCmdSupport->Cmd_Erase = 1;
				//mexPrintf(", ERASE");
				break;
			case IAP_CMD_EXTND_ERASE: 
				IapCmdSupport->Cmd_Erase_Ext = 1;
				//mexPrintf(", EXT_ERASE");
				break;
			case IAP_CMD_WRITE_PROTECT: 
				IapCmdSupport->Cmd_Write_Protect = 1;
				//mexPrintf(", WRITE_PROTECT");
				break;
			case IAP_CMD_WRITE_UNPROTECT: 
				IapCmdSupport->Cmd_Write_UnProtect = 1;
				//mexPrintf(", WRITE_UNPROTECT");
				break;
			case IAP_CMD_READ_PROTECT: 
				IapCmdSupport->Cmd_Read_Protect = 1;
				//mexPrintf(", READ_PROTECT");
				break;
			case IAP_CMD_READ_UNPROTECT: 
				IapCmdSupport->Cmd_Read_UnProtect = 1;
				//mexPrintf(", READ_UNPROTECT");
				break;
			default:
				break;
		}
	}
	mexPrintf("\n");
	
	/* === Get option bytes === */
	TxData[0] = IAP_CMD_GET_VER_RDP;
	TxData[1] = ~IAP_CMD_GET_VER_RDP;
	IAP_Comm_Write(ftHandle, TxData, 2);
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Get options byte: ");
	if (! IAP_Comm_Read (ftHandle, 5, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 5))
		return FALSE;
	if (!((RxBuffer[0] == IAP_ACK) && (RxBuffer[BytesReceived-1] == IAP_ACK)))
		return FALSE;
	OptionsByte->Byte1 = RxBuffer[2];
	OptionsByte->Byte2 = RxBuffer[3];
	//mexPrintf("Byte1=%d, Byte2=%d\n", (int)OptionsByte->Byte1, (int)OptionsByte->Byte2);
	
	/* === Get PID === */
	TxData[0] = IAP_CMD_GET_ID;
	TxData[1] = ~IAP_CMD_GET_ID;
	IAP_Comm_Write(ftHandle, TxData, 2);
	//mexPrintf(IAP_LOGGING_HDR);
	//mexPrintf("Get PID byte: ");
	if (! IAP_Comm_Read (ftHandle, 5, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 5))
		return FALSE;
	if (!((RxBuffer[0] == IAP_ACK) && (RxBuffer[BytesReceived-1] == IAP_ACK)))
		return FALSE;
	for (i=1; i<4; i++) {
		//mexPrintf(", %x", (DWORD)RxBuffer[i]);
	}
	*Pid = ((DWORD)RxBuffer[2] << 8) | ((DWORD)RxBuffer[3]);
	//mexPrintf(" (0x%X)", *Pid);
	//mexPrintf("\n");
	
	/*  */
	return TRUE;
}

#if 0
BOOL IAP_Erase_Ext_Flash(FT_HANDLE ftHandle, DWORD dwPageIndex)
{
	ULONG BytesReceived;
	char TxData[8];
	char RxBuffer[32];
	
	/* --- Erase command --- */
	TxData[0] = IAP_CMD_EXTND_ERASE;
	TxData[1] = ~IAP_CMD_EXTND_ERASE;	
	IAP_Comm_Write(ftHandle, TxData, 2);
	/* Ack? */
	if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	/* N */
	TxData[0] = 0;
	TxData[1] = 0;
	TxData[2] = (char)(dwPageIndex>>8);
	TxData[3] = (char)dwPageIndex;
	TxData[4] = TxData[0] ^ TxData[1] ^ TxData[2] ^ TxData[3];
	IAP_Comm_Write(ftHandle, TxData, 5);
	Sleep(50);
	/* Ack? */
	if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	
	return TRUE;
}

BOOL IAP_FullErase_Ext_Flash(FT_HANDLE ftHandle)
{
	ULONG BytesReceived;
	char TxData[8];
	char RxBuffer[32];
	
	/* --- Erase command --- */
	TxData[0] = IAP_CMD_EXTND_ERASE;
	TxData[1] = ~IAP_CMD_EXTND_ERASE;	
	IAP_Comm_Write(ftHandle, TxData, 2);
	/* Ack? */
	if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	/* N */
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0x00;
	IAP_Comm_Write(ftHandle, TxData, 3);
	/* Ack? */
	if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 5000) || (BytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	
	return TRUE;
}
#endif //0

BOOL IAP_EraseFlash_Page(FT_HANDLE ftHandle, unsigned char page_start, unsigned char count) {
	unsigned char i;
	ULONG BytesReceived;
	char TxData[8];
	char RxBuffer[32];
	
	for (i=0; i<count; i++) {
		mexPrintf("Erase page: %u\n", (page_start + i));
		/* --- IAP_CMD_ERASE --- */
		TxData[0] = IAP_CMD_ERASE;
		TxData[1] = ~IAP_CMD_ERASE;
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* Page index */
		TxData[0] = (page_start + i);
		TxData[1] = ~(page_start + i);
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
	}
	
	return TRUE;
}

BOOL IAP_Activate_PrimaryBootloader(FT_HANDLE ftHandle) {
	unsigned char i;
	ULONG BytesReceived;
	char TxData[8];
	char RxBuffer[32];
	
	/* --- IAP_CMD_ERASE --- */
	TxData[0] = IAP_CMD_UNLOCK_BOOT_UPGRADE;
	TxData[1] = ~IAP_CMD_UNLOCK_BOOT_UPGRADE;
	IAP_Comm_Write(ftHandle, TxData, 2);
	/* Ack? */
	if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
		return FALSE;
	if (!(RxBuffer[0] == IAP_ACK))
		return FALSE;
	
	return TRUE;
}

BOOL IAP_Read_Flash(FT_HANDLE ftHandle, DWORD address, DWORD count, char *Buffer)
{
	DWORD start_tick;
	int i, CmdCount;
	char TxData[8];
	char RxBuffer[32];
	ULONG BytesReceived, PacketDataCount;
	
	DWORD dwReadIndex;
	DWORD dwReadCountRem, dwReadCountPkt;
    
	start_tick = GetTickCount();
	dwReadIndex = 0;
	dwReadCountRem = count;
	do {
		mexPrintf(IAP_LOGGING_HDR);
		mexPrintf(">> [%u] Read addr: %x", GetTickCount()-start_tick, address);

		/* --- Cmd --- */
		TxData[0] = IAP_CMD_READ_MEM;
		TxData[1] = ~IAP_CMD_READ_MEM;
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* --- Address --- */
		TxData[0] = (char)(address >> 24);
		TxData[1] = (char)(address >> 16);
		TxData[2] = (char)(address >> 8);
		TxData[3] = (char)(address >> 0);
		TxData[4] = (char)(TxData[0] ^ TxData[1] ^ TxData[2] ^ TxData[3]);
		IAP_Comm_Write(ftHandle, TxData, 5);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* --- Num of bytes to read --- */
		if(dwReadCountRem > 256) {
			/* Max read packet 256 bytes */
			PacketDataCount = 256;			
			dwReadCountRem -= 256;
		}
		else {
			PacketDataCount = dwReadCountRem;
			dwReadCountRem = 0;
		}
		
		mexPrintf(", %d bytes\n", PacketDataCount);
		
		TxData[0] = (char)(PacketDataCount-1);
		TxData[1] = (char)(~(PacketDataCount-1));
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 500) || (BytesReceived != 1))
			return FALSE;
		if (!(RxBuffer[0] == IAP_ACK))
			return FALSE;
		/* --- Data --- */
		if (! IAP_Comm_Read(ftHandle, PacketDataCount, &Buffer[dwReadIndex], &BytesReceived, 1000) || (BytesReceived != PacketDataCount)) {
			mexPrintf("\nRead failed, received: %u bytes\n", BytesReceived);
			return FALSE;
		}
		
		/* Next read address */
		address += PacketDataCount;
		dwReadIndex += PacketDataCount;
		
	} while (dwReadIndex < count);
	
	return TRUE;
}

BOOL IAP_Write_Flash(FT_HANDLE ftHandle, const char *Buffer, DWORD address, DWORD count)
{	
	char xor_byte;
	DWORD start_tick;
	int i, CmdCount;
	char TxData[8];
	char RxBuffer[32];
	ULONG BytesReceived, PacketDataCount;
	
	DWORD dwWriteIndex;
	DWORD dwReadCountRem, dwReadCountPkt;
    
	start_tick = GetTickCount();
	dwWriteIndex = 0;
	dwReadCountRem = count;
	do {		
		//mexPrintf(IAP_LOGGING_HDR);
		mexPrintf(">> [%u ms] Write addr: %x\n", GetTickCount()-start_tick, address);

		/* --- Cmd --- */
		TxData[0] = IAP_CMD_WRITE;
		TxData[1] = ~IAP_CMD_WRITE;
		IAP_Comm_Write(ftHandle, TxData, 2);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 1000) || (BytesReceived != 1)) {
			mexPrintf("Read failed-1\n");
			return FALSE;
		}
		if (!(RxBuffer[0] == IAP_ACK)) {
			mexPrintf("NACK-1\n");
			return FALSE;
		}
		/* --- Address --- */
		TxData[0] = (char)(address >> 24);
		TxData[1] = (char)(address >> 16);
		TxData[2] = (char)(address >> 8);
		TxData[3] = (char)(address >> 0);
		TxData[4] = (char)(TxData[0] ^ TxData[1] ^ TxData[2] ^ TxData[3]);
		IAP_Comm_Write(ftHandle, TxData, 5);
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 1000) || (BytesReceived != 1)) {
			mexPrintf("Read failed-2: bytes=%u\n", BytesReceived);
			return FALSE;
		}
		if (!(RxBuffer[0] == IAP_ACK)) {
			mexPrintf("NACK-2\n");
			return FALSE;
		}
		/* --- Num of bytes to read --- */
		if(dwReadCountRem > 128) {
			/* Max read packet 256 bytes */
			PacketDataCount = 128;			
			dwReadCountRem -= 128;
		}
		else {
			PacketDataCount = dwReadCountRem;
			dwReadCountRem = 0;
		}
		
		//mexPrintf(", %d bytes\n", PacketDataCount);
		
		TxData[0] = (char)(PacketDataCount-1);
		IAP_Comm_Write(ftHandle, TxData, 1);
		
		xor_byte = (char)(PacketDataCount-1);
		for (i=0; i<PacketDataCount; i++) {
			xor_byte = xor_byte ^ Buffer[dwWriteIndex+i];
		}
		IAP_Comm_Write(ftHandle, &Buffer[dwWriteIndex], PacketDataCount);
		IAP_Comm_Write(ftHandle, &xor_byte, 1);
		
		/* Ack? */
		if (! IAP_Comm_Read(ftHandle, 1, &RxBuffer[0], &BytesReceived, 1000) || (BytesReceived != 1)) {
			mexPrintf("Read failed-3: bytes=%u\n", BytesReceived);
			return FALSE;
		}
		if (!(RxBuffer[0] == IAP_ACK)) {
			mexPrintf("NACK-3: %X\n", RxBuffer[0]);
			return FALSE;
		}
		
		/* Next read address */
		address += PacketDataCount;
		dwWriteIndex += PacketDataCount;
		
	} while (dwWriteIndex < count);

	return TRUE;	
}

// ========================================================================
// IAP Communication
// ========================================================================
FT_HANDLE SERIAL_GetValid_DeviceIndex (int *deviceIndex)
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
		for (i=0; i<numDevs; i++) {
			// mexPrintf("=== Dev %d: ===\n",i);
			// mexPrintf(" Flags=0x%x\n",devInfo[i].Flags); 
			// mexPrintf(" Type=0x%x\n",devInfo[i].Type); 
			// mexPrintf(" ID=0x%x\n",devInfo[i].ID); 
			// mexPrintf(" LocId=0x%x\n",devInfo[i].LocId); 
			// mexPrintf(" SerialNumber=%s\n",devInfo[i].SerialNumber); 
			// mexPrintf(" Description=%s\n",devInfo[i].Description); 
			// mexPrintf(" ftHandle=0x%x\n",devInfo[i].ftHandle);
			if (!strcmp(devInfo[i].Description, "aMG USB Connect")) {
				*deviceIndex = i;
				break;
			}
		}
	}
	mxFree(devInfo);
	
	// Return status
	return ftStatus;
}

FT_HANDLE IAP_Comm_Open (void)
{
	int deviceNumber;
	FT_HANDLE ftHandle = 0;
	FT_STATUS ftStatus; 
	
	// Get valid number
	ftStatus = SERIAL_GetValid_DeviceIndex (&deviceNumber);
	if ((ftStatus != FT_OK) && (deviceNumber >= 0))
		return 0; // NULL	

	/* Open port */
	ftStatus = FT_Open(deviceNumber, &ftHandle);
	
	/* Latency */
	//if (ftStatus == FT_OK)
	//	ftStatus = FT_SetLatencyTimer(ftHandle, 2);

	/* Data chatacteristics: Set 8 data bits, 1 stop bit and no parity  */
	if (ftStatus == FT_OK)
		ftStatus = FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);

	/* Baudrate */
	if (ftStatus == FT_OK)
		ftStatus = FT_SetBaudRate(ftHandle, 115200);
	
	/* Default Timeout */
	if (ftStatus == FT_OK)
		ftStatus = FT_SetTimeouts(ftHandle, 1000, 1000);
	
	/* Return Handle */
	return ftHandle;
}

void IAP_Comm_Close (FT_HANDLE ftHandle)
{
	if (ftHandle)
		FT_Close(ftHandle);
}

BOOL IAP_Comm_Write (FT_HANDLE ftHandle, const char *TxBuffer, ULONG TxBytes)
{
	ULONG ret;
	return (FT_Write(ftHandle, (char *)TxBuffer, TxBytes, &ret) == FT_OK);
}

extern DWORD WINAPI GetTickCount(void);
BOOL IAP_Comm_Read (FT_HANDLE ftHandle, ULONG RxBytes, char *RxBuffer, ULONG *BytesReceived, ULONG timeout_ms)
{
	DWORD ulRead, ulReading, ulReadIndex;
	FT_STATUS ftStatus;
	DWORD start_tick, current_tick;
	
	/* Get start tick */
	start_tick = GetTickCount();
	ulReadIndex = 0;
	do {
		ftStatus = FT_GetQueueStatus(ftHandle, &ulRead);
		if ((ftStatus == FT_OK) && (ulRead > 0)) {
			if(FT_Read(ftHandle, &RxBuffer[ulReadIndex], ulRead > (RxBytes-ulReadIndex)?(RxBytes-ulReadIndex):ulRead, &ulReading) == FT_OK) {
				ulReadIndex += ulReading;
				/* Update new start tick */
				start_tick = GetTickCount();
			}
		}
		else {
			/* Wait some time */
			Sleep(10);
		}
		
		/* Current Tick */
		current_tick = GetTickCount();
	} while (((current_tick - start_tick) < timeout_ms) && (ulReadIndex < RxBytes));
	
	/* Number of reading */
	*BytesReceived = ulReadIndex;
	
	return (BOOL)(ulReadIndex > 0);
}

/* Working buffer */



// ========================================================================
// Mex function
// ========================================================================
#define CRC_CAL_LEN              0x58

#define INIT_COMMAND_MAX_SIZE    256     /**< Maximum size of the init command stored in dfu_settings. */

#define NRF_DFU_BANK_INVALID     0x00 /**< Invalid image. */
#define NRF_DFU_BANK_VALID_APP   0x01 /**< Valid application. */
#define NRF_DFU_BANK_VALID_SD    0xA5 /**< Valid SoftDevice. */
#define NRF_DFU_BANK_VALID_BL    0xAA /**< Valid bootloader. */
#define NRF_DFU_BANK_VALID_SD_BL 0xAC /**< Valid SoftDevice and bootloader. */


/** @brief Description of a single bank. */

typedef struct {
    uint32_T                image_size;         /**< Size of the image in the bank. */
    uint32_T                image_crc;          /**< CRC of the image. If set to 0, the CRC is ignored. */
    uint32_T                bank_code;          /**< Identifier code for the bank. */
} nrf_dfu_bank_t;

typedef struct {
    uint32_T command_size;              /**< The size of the current init command stored in the DFU settings. */
    uint32_T command_offset;            /**< The offset of the currently received init command data. The offset will increase as the init command is received. */
    uint32_T command_crc;               /**< The calculated CRC of the init command (calculated after the transfer is completed). */
    
    uint32_T data_object_size;          /**< The size of the last object created. Note that this size is not the size of the whole firmware image.*/
    
    union {
        struct {
            uint32_T firmware_image_crc;        /**< CRC value of the current firmware (continuously calculated as data is received). */
            uint32_T firmware_image_crc_last;   /**< The CRC of the last executed object. */
            uint32_T firmware_image_offset;     /**< The offset of the current firmware image being transferred. Note that this offset is the offset in the entire firmware image and not only the current object. */
            uint32_T firmware_image_offset_last;/**< The offset of the last executed object from the start of the firmware image. */
        };
        struct {
            uint32_T sd_start_address;          /**< Value indicating the start address of the SoftDevice source. Used for an SD/SD+BL update where the SD changes size or if the DFU process had a power loss when updating a SD with changed size. */
        };
    };
} dfu_progress_t;

typedef struct {
    uint32_T            crc;                /**< CRC for the stored DFU settings, not including the CRC itself. If 0xFFFFFFF, the CRC has never been calculated. */
    uint32_T            settings_version;   /**< Version of the currect DFU settings struct layout. */
    uint32_T            app_version;        /**< Version of the last stored application. */
    uint32_T            bootloader_version; /**< Version of the last stored bootloader. */
    
    uint32_T            bank_layout;        /**< Bank layout: single bank or dual bank. This value can change. */
    uint32_T            bank_current;       /**< The bank that is currently used. */
    
    nrf_dfu_bank_t      bank_0;             /**< Bank 0. */
    nrf_dfu_bank_t      bank_1;             /**< Bank 1. */
    
    uint32_T            write_offset;       /**< Write offset for the current operation. */
    uint32_T            sd_size;            /**< SoftDevice size (if combined BL and SD). */
    
    dfu_progress_t      progress;           /**< Current DFU progress. */
    
    uint32_T            enter_buttonless_dfu;
    uint8_T             init_command[INIT_COMMAND_MAX_SIZE];  /**< Buffer for storing the init command. */
    
    //nrf_dfu_peer_data_t peer_data;          /**< Not included in calculated CRC. */
    //nrf_dfu_adv_name_t  adv_name;           /**< Not included in calculated CRC. */
} nrf_dfu_settings_t;

//#define NRF_MBR_PARAMS_PAGE_ADDRESS    (0x0007E000UL)
#define BOOTLOADER_SETTINGS_ADDRESS     (0x0007F000UL)
#define APP_SETTING_CODEPAGE_ADDRESS   0x00007D00UL
#define BOOTLOADER_START_ADDR          0x00074000UL
#define MAIN_APPLICATION_START_ADDR    0x00026000UL
#define CODEPAGESIZE                   0x00001000UL //4kBytes

#define NRF_DFU_CURRENT_BANK_0 0x00
#define NRF_DFU_CURRENT_BANK_1 0x01

#define NRF_DFU_BANK_LAYOUT_DUAL   0x00
#define NRF_DFU_BANK_LAYOUT_SINGLE 0x01

#define NRF_DFU_BANK_INVALID     0x00 /**< Invalid image. */
#define NRF_DFU_BANK_VALID_APP   0x01 /**< Valid application. */
#define NRF_DFU_BANK_VALID_SD    0xA5 /**< Valid SoftDevice. */
#define NRF_DFU_BANK_VALID_BL    0xAA /**< Valid bootloader. */
#define NRF_DFU_BANK_VALID_SD_BL 0xAC /**< Valid SoftDevice and bootloader. */

uint32_T crc32_compute(const uint8_T * p_data, uint32_T size, uint32_T const * p_crc) {
    uint32_T crc;
    uint32_T i, j;
    
    
    crc = (p_crc == 0) ? 0xFFFFFFFF : ~(*p_crc);
    for (i = 0; i < size; i++) {
		uint8_T d = p_data[i];
        crc = crc ^ d;//p_data[i];
        for (j = 8; j > 0; j--) {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

void get_bootloader_conf_buffer(const uint8_T *fw, uint32_T size, uint8_T *out, int32_T out_len)
{
    nrf_dfu_settings_t dfu_setting;
    
    memset(&dfu_setting, 0, sizeof(nrf_dfu_settings_t));
    
    dfu_setting.settings_version = 1;
    dfu_setting.app_version = 0;
    dfu_setting.bootloader_version = 0;
    
    dfu_setting.bank_layout = NRF_DFU_BANK_LAYOUT_SINGLE;
    dfu_setting.bank_current = NRF_DFU_CURRENT_BANK_0;
    
    dfu_setting.bank_0.bank_code = NRF_DFU_BANK_VALID_APP;
    dfu_setting.bank_0.image_size = size;
    dfu_setting.bank_0.image_crc = crc32_compute((const uint8_T *)fw, size, 0);
    
    dfu_setting.crc = crc32_compute((const uint8_T *)&dfu_setting + 4, CRC_CAL_LEN, 0);
    
    memcpy(out, &dfu_setting, sizeof(nrf_dfu_settings_t));
}
        
static char Flash_Buffer[1024*1024];
static char Output_Message[1024*1024];

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[])
{
	int output_sta = 0;
	char input_cmd[64] = {0};
	char *output_buf, *input_buf;
	unsigned char Bootloader_version;
	
	OPTION_BYTES OptionsByte;
	DWORD Pid;
	IAP_CMD_SUPPORT IapCmdSupport;
	double *p, result = 0;
	const mwSize dims[] = {1};
	
	/* Init buffer */
	Output_Message[0] = '\0';
	
	/* Validate */
	if (nrhs < 1)
		mexErrMsgTxt("Invalid number of input parameter."); 
	if (nlhs > 2)
		mexErrMsgTxt("Invalid number of output."); 
	if (!mxIsChar(prhs[0]))
		mexErrMsgTxt("Input parameter 1 must be string"); 
	
	/* Create input buffer, !!! Memory buse be free at end function */
	input_buf = mxArrayToString(prhs[0]);
	if(input_buf) {
		if(strlen(input_buf) >= 64) {
			memcpy(input_cmd, input_buf, 63);
			input_cmd[63] = '\0';			
		}
		else { strcpy(input_cmd, input_buf); }
		mxFree(input_buf);
	}
	
	/* ========================
	 * === Command: connect ===
	 */
	if(!strcmp(input_cmd, "connect")) {
		FT_HANDLE ftHandle = 0;
		/* Open */
		ftHandle = IAP_Comm_Open();
		if (ftHandle) {
			/* Validate */
			//if (_validate_ft231x(ftHandle, "Aimagin")) {
			if (_validate_ft231x(ftHandle, "aMG USB Connect")) {			
				/* Connect */
				output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
				// Retry Connect
				if(output_sta == 0) {
					mexPrintf("Try connect...\n");
					output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
				}
				
				if(output_sta != 0) {
					//if (Pid == 0x0448) {
					mexPrintf("FiO Glide v%u.%u\n", Bootloader_version>>4, Bootloader_version & 0xF);
					sprintf(Output_Message, "Connected, FiO Glide v%u.%u", Bootloader_version>>4, Bootloader_version & 0xF);
					//}
					//else {
					//	output_sta = 0;
					//	sprintf(Output_Message, "Invalid MCU device ID, required 0x448, reading is: 0x%x", Pid);
					//}
				}
				else { strcpy(Output_Message, "Failed to connect to Target board."); }
			}
			else {
				output_sta = 0;
				sprintf(Output_Message, "USB to Serial detected but not a valid \"aMG USB Connect\" board.\n");
			}
			
			/* Close */
			if (ftHandle)
				IAP_Comm_Close(ftHandle);
		}
		else {
			output_sta = 0;
			sprintf(Output_Message, "Failed to connect to \"aMG USB Connect\" programmer board.\n");
			mexPrintf(Output_Message);
		}
	}
	
	/* =====================
	 * === Command: readtofile === */
	else if(!strcmp(input_cmd, "readtofile")) {
		FT_HANDLE ftHandle = 0;
		char *output_filename;
		DWORD mem_address, mem_size;
		FILE *fp = NULL;
		double* data_in;
		int count, idx;
		
		/* Assume status is 0 */
		output_sta = 0;
		
		/* Number of input parameter */
		if ((nrhs == 4)
				&& (mxIsDouble(prhs[1]) || mxIsUint32(prhs[1])) /* Address */
				&& (mxIsDouble(prhs[2]) || mxIsUint32(prhs[2])) /* Size */ 
				&& (mxIsChar(prhs[3])) /*File name*/
				) {
			/* Input parameters */
			data_in = mxGetPr(prhs[1]);
			mem_address = (DWORD)data_in[0];
			data_in = mxGetPr(prhs[2]);
			mem_size = (DWORD)data_in[0];
			output_filename = mxArrayToString(prhs[3]);
			
			/* List parameters */
			mexPrintf("Read flash program: address 0x%x, size %u bytes.\n", mem_address, mem_size);
			
			/* Create file */
			if (output_filename) {
				fp = fopen(output_filename, "wb");
				
				/* Open file */
				if (fp == NULL)
					mexPrintf("Failed to create file: %s\n", output_filename);
				else
					mexPrintf("File created: %s\n", output_filename);
			} else { mexPrintf("Invalid file name: %s\n", output_filename); }
			
			/* File valid */
			if (fp) {
				/* Open */
				ftHandle = IAP_Comm_Open();
				if (ftHandle != 0) {
					if ((mem_size > 0) && (mem_size <= 0x100000)) {
						/* Connect */
						output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
						if (output_sta != 0) {
							/* Read mem */
							if (IAP_Read_Flash(ftHandle, mem_address, mem_size, &Flash_Buffer[0])) {
								idx = 0;
								while (output_sta && (mem_size > 0)) {
									if (mem_size > 128)
										count = 128;
									else
										count = mem_size;
									mem_size -= count;
									/* Write to file */
									if (fwrite(&Flash_Buffer[idx], 1, count, fp) != count) {
										output_sta = 0;
									}
									idx += count;
								}
								
							} else { output_sta = 0; }
						}
						
						/* Message */
						if(output_sta != 0)
							strcpy(Output_Message, "Success.");
						else
							strcpy(Output_Message, "Failed.");
					} else { strcpy(Output_Message, "Invalid flash memory size."); }
				} else { strcpy(Output_Message, "Failed to open port."); }
			} else { strcpy(Output_Message, "Failed to create file."); }
		} else { strcpy(Output_Message, "Invalid input parameter."); }
		
		/* --- Cleanup --- */
		if (ftHandle)
			IAP_Comm_Close(ftHandle);
		if (fp)
			fclose(fp);
	}
	/* === Command: writeflash === */
	else if(!strcmp(input_cmd, "writeflash")) {
		size_t f_idx;
		FT_HANDLE ftHandle = 0;
		char *input_filename;
		DWORD mem_address, mem_size;
		FILE *fp = NULL;
		double* data_in;
		
		/* Default memory address */
		memset(Flash_Buffer, 0xFF, 0x80000);		
		
		/* Assume status is 0 */
		output_sta = 0;
		
		/* Number of input parameter */
		if ((nrhs == 4)
				&& (mxIsDouble(prhs[1]) || mxIsUint32(prhs[1])) /* Address */
				&& (mxIsDouble(prhs[2]) || mxIsUint32(prhs[2])) /* Size */ 
				&& (mxIsChar(prhs[3])) /*File name*/
				) {
			/* Input parameters */
			data_in = mxGetPr(prhs[1]);
			mem_address = (DWORD)data_in[0];
			data_in = mxGetPr(prhs[2]);
			mem_size = (DWORD)data_in[0];
			input_filename = mxArrayToString(prhs[3]);
			
			/* List parameters */

			
			/* Create file */
			if (input_filename) {
				fp = fopen(input_filename, "rb");
				
				/* Open file */
				if (fp == NULL)
					mexPrintf("Failed to open file: %s\n", input_filename);
				else
					mexPrintf("File openned: %s\n", input_filename);
			} else { mexPrintf("Invalid file name: %s\n", input_filename); }
			
			/* File valid  */
			if (fp) {
				DWORD file_size;
				
				/* Get file size */
				fseek(fp , 0 , SEEK_END);
				file_size = ftell(fp);
				rewind(fp);
				
				// Display file size
				mexPrintf("Write flash: address 0x%x, size %u bytes.\n\n", mem_address, file_size);
                
                // Adjust file size
                if ((file_size & 0x03) != 0) {
                    file_size |= 0x03;
                    file_size += 0x01;
                    mexPrintf("File size adjusted to %\r\nd", file_size);
                }
  
				/* File size */
				if((file_size <= mem_size) && (file_size > 0)) {					
					/* Open */
					ftHandle = IAP_Comm_Open();
					if (ftHandle != 0) {
						if ((mem_size > 0) && (file_size <= mem_size)) {
							/* Connect */
							output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
							// Retry Connect
							if(output_sta == 0)
								output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
							if(output_sta == 0) {
								mexPrintf("Failed to connect to Target\n");
							}
							
							if (output_sta != 0) {
								DWORD read_count;
								/* Read file */
								f_idx = 0;
								read_count = 0xFFFFFFFF;
								while (!feof(fp) && (f_idx < file_size) && (read_count != 0)) {
									read_count = fread(&Flash_Buffer[f_idx], 1, 512, fp);
									f_idx += read_count;
									if (read_count == 0) {
										output_sta = 0; /* Activate output status Error */
										mexPrintf("Error: failed to read file.\n");
									}
								}
								
								/* Erase flash */
								if ((output_sta != 0) && (Bootloader_version != 0x10)) {
									unsigned int page_count;
																		
									if ((file_size & /*0x3FF*/ (CODEPAGESIZE-1)) != 0)
										page_count = ((file_size | /*0x3FF*/ (CODEPAGESIZE-1)) + 1) / CODEPAGESIZE /*1024*/;
									else {
										page_count = file_size / (CODEPAGESIZE-1) /*1024*/;
									}
									mexPrintf("Erasing...\n");
									if (!IAP_EraseFlash_Page(ftHandle, (unsigned char)(mem_address/CODEPAGESIZE /*1024*/), (unsigned char)page_count)) {
										output_sta = 0;
									}
									if (output_sta)
										mexPrintf("\tdone\n");
									
								}
								
								/* Write mem */
								if (output_sta != 0) {
									if (IAP_Write_Flash(ftHandle, Flash_Buffer, mem_address, file_size))
										mexPrintf("Write flash success.\n");
									else {
										mexPrintf("Error: failed to write to flash.\n");
										output_sta = 0;
									}
								}
								
								// Verify mem
								if (output_sta != 0) {
									
								}
                                
                                // Write bootloader code
                                if (output_sta != 0) {
                                    int idx;
                                    uint8_T boot_conf[512];
                                    
                                    memset(boot_conf, 0, sizeof(boot_conf));
                                    get_bootloader_conf_buffer((const uint8_T *)Flash_Buffer, file_size, boot_conf, sizeof(boot_conf));
                                    /*
                                    mexPrintf("\r\n");
                                    for (idx=0; idx<sizeof(boot_conf); idx++) {
                                        if ((idx& 0x1F) == 0) {mexPrintf("\r\n");}
                                        mexPrintf("0x%x ", boot_conf[idx]);
                                    }
                                    mexPrintf("\r\n");
                                     */
                                    if (IAP_Write_Flash(ftHandle, boot_conf, BOOTLOADER_SETTINGS_ADDRESS, 128))
                                        mexPrintf("Write flash success.\n");
                                    else {
                                        mexPrintf("Error: failed to write to flash.\n");
                                        output_sta = 0;
                                    }
                                }
                                
                                // Write public key
                                if (output_sta != 0) {
                                    
                                }
							}
							
							/* Message */
							if(output_sta != 0)
								strcpy(Output_Message, "Success.");
							else
								strcpy(Output_Message, "Failed.");
						} else { strcpy(Output_Message, "Error: Invalid flash memory size."); }
					} else { strcpy(Output_Message, "Error: Failed to open port."); }
				} else { strcpy(Output_Message, "Error: Invalid file size."); }
			} else { strcpy(Output_Message, "Error: Failed to open file."); }
		} else { strcpy(Output_Message, "Error: Invalid input parameter."); }
		
		/* --- Cleanup --- */
		if (ftHandle)
			IAP_Comm_Close(ftHandle);
		if (fp)
			fclose(fp);
	}
    
    /* === Command: publickey === */
    else if(!strcmp(input_cmd, "publickey")) {
        size_t f_idx;
        FT_HANDLE ftHandle = 0;
        char *input_publickey;
        DWORD mem_address, mem_size;
        FILE *fp = NULL;
        double* data_in;
        
        /* Default memory address */
        memset(Flash_Buffer, 0xFF, 0x01000);
        
        /* Assume status is 0 */
        output_sta = 0;
        
        /* Number of input parameter */
        if ((nrhs == 4)
        && (mxIsDouble(prhs[1]) || mxIsUint32(prhs[1])) /* Address */
        && (mxIsDouble(prhs[2]) || mxIsUint32(prhs[2])) /* Size */
        && (mxIsChar(prhs[3])) /*public key*/
        ) {
            /* Input parameters */
            data_in = mxGetPr(prhs[1]);
            mem_address = (DWORD)data_in[0];
            data_in = mxGetPr(prhs[2]);
            mem_size = (DWORD)data_in[0];
            input_publickey = mxArrayToString(prhs[3]);
            
            if ((strlen(input_publickey) > 0) && (mem_size > 0) && (mem_size <= 0x1000)){
                /* Open */
                ftHandle = IAP_Comm_Open();
                if (ftHandle != 0) {
                    if ((mem_size > 0) && (mem_size <= mem_size)) {
                        int index = 0;
                        const char *s = input_publickey;
                        
                        /* Connect */
                        output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
                        // Retry Connect
                        if(output_sta == 0)
                            output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
                        if(output_sta == 0) {
                            mexPrintf("Failed to connect to Target\n");
                        }
                        
                        // Fill buffer with 0xFF
                        memset(Flash_Buffer, 0xFF, mem_size);
                        
                        // Load public key into buffer
                        while (s && *s) {
                            uint32_T x_val;
                            char tmp[3] = {0, 0, 0};
                            tmp[0] = s[0];
                            tmp[1] = s[1];
                            
                            s++;
                            if (*s) s++;
                            if (sscanf(tmp, "%X", &x_val)) {
                                Flash_Buffer[index++] = (char)(x_val & 0xFF);
                            }
                        }
                        
                        if (index > 0) {
                            // No erase needed
                            
                            // Write
                            if (IAP_Write_Flash(ftHandle, Flash_Buffer, mem_address, mem_size))
                                mexPrintf("Write flash success.\n");
                            else {
                                mexPrintf("Error: failed to write to flash.\n");
                                output_sta = 0;
                            }
                        }
                        
                        /* Message */
                        if(output_sta != 0)
                            strcpy(Output_Message, "Success.");
                        else
                            strcpy(Output_Message, "Failed.");
                    } else { strcpy(Output_Message, "Error: Invalid flash memory size."); }
                } else { strcpy(Output_Message, "Error: Failed to open port."); }
            }
            else {
                // Empty string of public_key, ignore
            }
            /* --- Cleanup --- */
            if (ftHandle)
                IAP_Comm_Close(ftHandle);
            if (fp)
                fclose(fp);
        }
    }

	/* === Command: fullerase === */
	else if(!strcmp(input_cmd, "fullerase")) {
		FT_HANDLE ftHandle = 0;
		
		ftHandle = IAP_Comm_Open();
		if (ftHandle != 0) {
			output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
			if (output_sta) {
				if (Bootloader_version == 0x10) { // Not support by versin 1.0
					mexPrintf("Erase success.\n");
				}
				else 
				// CODE_REGION_1_START				
				if (IAP_EraseFlash_Page(ftHandle, (unsigned char)(MAIN_APPLICATION_START_ADDR/CODEPAGESIZE), (BOOTLOADER_START_ADDR-MAIN_APPLICATION_START_ADDR)/CODEPAGESIZE)) {
					mexPrintf("Erase success.\n");
				}
				else {
					output_sta = 0;
					mexPrintf("Erase failed.\n");
				}
			} else { mexPrintf("Failed.\n"); }
		} else { mexPrintf("Failed to open port.\n"); }
		/* Message */
		if(output_sta != 0)
			strcpy(Output_Message, "Success.");
		else
			strcpy(Output_Message, "Failed.");
		/* --- Cleanup --- */
		if (ftHandle)
			IAP_Comm_Close(ftHandle);
	}
	/* === Command: run === */
	else if(!strcmp(input_cmd, "run")) {
		FT_HANDLE ftHandle = 0;
		
		ftHandle = IAP_Comm_Open();
		if (ftHandle != 0) {
			output_sta = 1;
			IAP_Exit(ftHandle);
		}
		if (output_sta != 0)
			strcpy(Output_Message, "Success.");
		else
			strcpy(Output_Message, "Failed.");
		
		/* --- Cleanup --- */
		if (ftHandle)
			IAP_Comm_Close(ftHandle);		
	}
	
	/* ========================
	 * === Command: primaryboot ===
	 */
#if 0    
	else if(!strcmp(input_cmd, "primaryboot")) {
		int retry_count = 0;
		FT_HANDLE ftHandle = 0;
		/* Open */
		ftHandle = IAP_Comm_Open();
		if (ftHandle) {
			___retry:
			/* Connect */
			output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
			// Retry Connect
			if(output_sta == 0) {
				mexPrintf("Try connect...\n");
				output_sta = (int)IAP_Init(ftHandle, &IapCmdSupport, &OptionsByte, &Pid, &Bootloader_version);
			}
			
			// Display bootloader information
			if(output_sta != 0) {				
				mexPrintf("FiO Glide v%u.%u\n", Bootloader_version>>4, Bootloader_version & 0xF);
				strcpy(Output_Message, "Connected.");
			}
			else { strcpy(Output_Message, "Failed to connect to Target board."); }
			
			// Send command to activate primary bootloader, if major version is higher than 1
			if(output_sta != 0) {
				if ((Bootloader_version>>4) > 1) {
					output_sta = IAP_Activate_PrimaryBootloader(ftHandle);
					if (output_sta && (++retry_count < 3)) {
						goto ___retry;
					}
				}
			}
			
			/* Close */
			if (ftHandle)
				IAP_Comm_Close(ftHandle);
		}
		else {
			output_sta = 0;
			sprintf(Output_Message, "Failed to connect to \"aMG USB Connect\" programmer board.\n");
			mexPrintf(Output_Message);
		}
	}
#endif //0    
	
	/* === Unknown === */
	else {
		strcpy(Output_Message, "Unknown action.");
	}
	
	/* Return */
	plhs[0] = mxCreateDoubleScalar (output_sta); //mxCreateNumericArray(1,dims,mxINT32_CLASS,mxREAL);	
	if (nlhs > 1) {
		output_buf = mxCalloc(1024, sizeof(char));
		strcpy(output_buf, Output_Message);
		plhs[1] = mxCreateString(output_buf);
	}
	return;
}
