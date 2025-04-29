/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

/* This file was modified by Aimagin
*/

#include "diskio.h"		/* FatFs lower layer API */
#include "waijung_hwdrvlib.h" /* Get system configuration */

/* Definitions of physical drive number for each media */
#define SDCARD_DRV		0 /* SD Card via SDIO or SPI interface */
#define IFLASH_DRV		1 /* MCU internal flash drive emulation */
#define SQLITE_DRV		2 /* UART-SQLite emulation interface */

/* Prototypes: SD Card */
DSTATUS SDCARD_disk_initialize(void);
DSTATUS SDCARD_disk_status(void);
DRESULT SDCARD_disk_read(BYTE *buff, DWORD sector, UINT count);
DRESULT SDCARD_disk_write(const BYTE *buff, DWORD sector, UINT count);
DRESULT SDCARD_disk_ioctl(BYTE cmd, void *buff);

/* Prototypes: IFALSH */
DSTATUS IFLASH_disk_initialize(void);
DSTATUS IFLASH_disk_status(void);
DRESULT IFLASH_disk_read(BYTE *buff, DWORD sector, UINT count);
DRESULT IFLASH_disk_write(const BYTE *buff, DWORD sector, UINT count);
DRESULT IFLASH_disk_ioctl(BYTE cmd, void *buff);

/* Prototypes: SQLITE */
DSTATUS SQLITE_disk_initialize(void);
DSTATUS SQLITE_disk_status(void);
DRESULT SQLITE_disk_read(BYTE *buff, DWORD sector, UINT count);
DRESULT SQLITE_disk_write(const BYTE *buff, DWORD sector, UINT count);
DRESULT SQLITE_disk_ioctl(BYTE cmd, void *buff);

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (BYTE pdrv) /* Physical drive nmuber (0..) */
{
	switch (pdrv) {
	case SDCARD_DRV :
		return SDCARD_disk_initialize();

	case IFLASH_DRV :
		return IFLASH_disk_initialize();

	case SQLITE_DRV :
		return SQLITE_disk_initialize();
	}
	return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (BYTE pdrv) /* Physical drive nmuber (0..) */
{
	switch (pdrv) {
	case SDCARD_DRV :
		return SDCARD_disk_status();

	case IFLASH_DRV :
		return IFLASH_disk_status();

	case SQLITE_DRV :
		return SQLITE_disk_status();
	}
	return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
//	BYTE pdrv   : Physical drive nmuber (0..)
//	BYTE *buff  : Data buffer to store read data
//	DWORD sector: Sector address (LBA)
//	UINT count  : Number of sectors to read (1..128)
DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	switch (pdrv) {
	case SDCARD_DRV :
		return SDCARD_disk_read(buff, sector, count);

	case IFLASH_DRV :
		return IFLASH_disk_read(buff, sector, count);

	case SQLITE_DRV :
		return SQLITE_disk_read(buff, sector, count);
	}
	return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
// BYTE pdrv       : Physical drive nmuber (0..)
// const BYTE *buff: Data to be written
// DWORD sector    : Sector address (LBA)
// UINT count      : Number of sectors to write (1..128)
#if _USE_WRITE
DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	switch (pdrv) {
	case SDCARD_DRV :
		return SDCARD_disk_write(buff, sector, count);

	case IFLASH_DRV :
		return IFLASH_disk_write(buff, sector, count);

	case SQLITE_DRV :
		return SQLITE_disk_write(buff, sector, count);
	}

	return RES_PARERR;
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
// BYTE pdrv : Physical drive nmuber (0..)
// BYTE cmd  : Control code
// void *buff: Buffer to send/receive control data
#if _USE_IOCTL
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
	switch (pdrv) {
	case SDCARD_DRV :
		return SDCARD_disk_ioctl(cmd, buff);

	case IFLASH_DRV :
		return IFLASH_disk_ioctl(cmd, buff);

	case SQLITE_DRV :
		return SQLITE_disk_ioctl(cmd, buff);
	}
	return RES_PARERR;
}
#endif

/* ==============================================================================
** SDCARD I/O
** ==============================================================================
*/
#define SECTOR_SIZE 512
volatile SD_Error Status = SD_OK;
SD_CardStatus SDCardStatus;
static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */
extern SD_CardInfo SDCardInfo; /* Extern SD card information */

DSTATUS SDCARD_disk_initialize(void)
{
	/* Initialize SD Card */
	Status = SD_Init(); 
	if (Status != SD_OK)
		return STA_NOINIT;

	/* No Error detected */
	return (DSTATUS)0;			
}

DSTATUS SDCARD_disk_status(void)
{
	/* Check if last operation was not OK, Re init */
	if(Status != SD_OK) {				
		Status = SD_Init();
		if(Status == SD_OK)
			Status = SD_GetCardInfo(&SDCardInfo);
	}
	/* Check card */
	else {
		Status = SD_GetCardInfo(&SDCardInfo);
	}
	
	/* If card not OK */
	if (Status != SD_OK) {
		return STA_NOINIT;
	}

	/* No Error detected */
	return (DSTATUS)0;
}

DRESULT SDCARD_disk_read(BYTE *buff, DWORD sector, UINT count)
{
	int try_count = 0;
	SDTransferState trans_state = SD_TRANSFER_OK;
	SYS_TIMER_uS_STRUCT timer;

	/* Start timer */
	SysTimer_uS_Start(&timer, 5000000UL);

__try_read:
	Status = SD_OK;
#ifdef WEBSERVER_DEBUG_PRINT
	if((unsigned int)buff & 3) {
		WEBSERVER_DEBUG_PRINT("!!! READ Alignment fail, buffer: %u, sector: %u, count: %u", (unsigned int)buff, (unsigned int)sector, (unsigned int)count);
	}
#endif
	/* Read Multiple Blocks */
	Status = SD_ReadMultiBlocks((uint8_t*)(buff), (sector)*SECTOR_SIZE, SECTOR_SIZE, count);
#ifdef WEBSERVER_DEBUG_PRINT
	if(Status != SD_OK) {
		WEBSERVER_DEBUG_PRINT("SD_ReadMultiBlocks(), Error code: %d", Status);
	}
#endif

	/* Check if the Transfer is finished */
	Status = SD_WaitReadOperation();
#ifdef WEBSERVER_DEBUG_PRINT
	if(Status != SD_OK) {
		WEBSERVER_DEBUG_PRINT("SD_WaitReadOperation(), Error code: %d", Status);
	}
#endif

	/* Wait until end of DMA transfer */			
	do { trans_state = SD_GetStatus(); }
	while(trans_state == SD_TRANSFER_BUSY);

#ifdef WEBSERVER_DEBUG_PRINT
	if(trans_state != SD_TRANSFER_OK) {
		WEBSERVER_DEBUG_PRINT("SD_GetStatus(), Error code: %d", trans_state);
	}
#endif

	/* Check status */
	if ((Status == SD_OK) && (trans_state == SD_TRANSFER_OK))
		return RES_OK;

	if((try_count++ < 10) && (!SysTimer_uS_IsTimeout(&timer))) { /* Retry 10 time for read */
		#ifdef WEBSERVER_DEBUG_PRINT
			WEBSERVER_DEBUG_PRINT("Retry read operation...");
		#endif
		goto __try_read;
	}

	/* Error */
	return RES_ERROR;
}

DRESULT SDCARD_disk_write(const BYTE *buff, DWORD sector, UINT count)
{
	SDTransferState trans_state = SD_TRANSFER_OK;

	Status = SD_OK;

#ifdef WEBSERVER_DEBUG_PRINT
	if((unsigned int)buff & 3) {
		WEBSERVER_DEBUG_PRINT("!!! WRITE Alignment fail, buffer: %u, sector: %u, count: %u", (unsigned int)buff, (unsigned int)sector, (unsigned int)count);
	}
#endif
			
	/* Write Multiple Blocks */
	Status = SD_WriteMultiBlocks((uint8_t*)(buff), (sector)*SECTOR_SIZE, SECTOR_SIZE, count);
#ifdef WEBSERVER_DEBUG_PRINT
	if(Status != SD_OK) {
		WEBSERVER_DEBUG_PRINT("SD_WriteMultiBlocks(), Error code: %d", Status);
	}
#endif

	/* Check if the Transfer is finished */
	Status = SD_WaitWriteOperation();
#ifdef WEBSERVER_DEBUG_PRINT
	if(Status != SD_OK) {
		WEBSERVER_DEBUG_PRINT("SD_WaitWriteOperation(), Error code: %d", Status);
	}
#endif

	/* Wait until end of DMA transfer */		
	do { trans_state = SD_GetStatus(); }
	while(trans_state == SD_TRANSFER_BUSY);

#ifdef WEBSERVER_DEBUG_PRINT
	if(trans_state != SD_TRANSFER_OK) {
		WEBSERVER_DEBUG_PRINT("SD_GetStatus(), Error code: %d", trans_state);
	}
#endif
			
	if ((Status == SD_OK) && (trans_state == SD_TRANSFER_OK))
		return RES_OK;

	return RES_ERROR;			
}

DRESULT SDCARD_disk_ioctl(BYTE cmd, void *buff)
{
	switch (cmd)
	{
	case CTRL_SYNC:
		/* no synchronization to do since not buffering in this module */
		return RES_OK;
	case GET_SECTOR_SIZE:
		*(uint16_t*)buff = SECTOR_SIZE;
		return RES_OK;
	case GET_SECTOR_COUNT:
		*(uint32_t*)buff = SDCardInfo.CardCapacity / SECTOR_SIZE;
		return RES_OK;
	case GET_BLOCK_SIZE:
		*(uint32_t*)buff = SDCardInfo.CardBlockSize;
		return RES_OK;				
	default :
		//return RES_OK;
		break;
	}
	return RES_ERROR;
}

/* ==============================================================================
** IFLASH I/O
** ==============================================================================
*/
#ifdef ENABLE_VIRTUAL_DISKIO
extern const char disk_image[];
#define VIRTUAL_DISK_IMAGE_SIZE (VIRTUAL_DISK_END - VIRTUAL_DISK_START + 1)
const BYTE *virtual_disk_img = (const BYTE*)VIRTUAL_DISK_START;
#endif
DSTATUS IFLASH_disk_initialize(void)
{
	#ifdef ENABLE_VIRTUAL_DISKIO
	return (DSTATUS)0;
	#else
	return STA_NOINIT;
	#endif
}

DSTATUS IFLASH_disk_status(void)
{
	#ifdef ENABLE_VIRTUAL_DISKIO
	return (DSTATUS)0;
	#else
	return STA_NODISK;
	#endif	
}

DRESULT IFLASH_disk_read(BYTE *buff, DWORD sector, UINT count)
{
	#ifdef ENABLE_VIRTUAL_DISKIO
	memcpy(buff, &virtual_disk_img[sector*SECTOR_SIZE], count*SECTOR_SIZE);			
	/* Status OK */			
	return RES_OK;
	#else
	return RES_ERROR;
	#endif
}

DRESULT IFLASH_disk_write(const BYTE *buff, DWORD sector, UINT count)
{
	return RES_ERROR;
}

DRESULT IFLASH_disk_ioctl(BYTE cmd, void *buff)
{
	#ifdef ENABLE_VIRTUAL_DISKIO
	switch (cmd)
	{
		case CTRL_SYNC:
			/* no synchronization to do since not buffering in this module */
			return RES_OK;
		case GET_SECTOR_SIZE:
			*(uint16_t*)buff = SECTOR_SIZE;
			return RES_OK;
		case GET_SECTOR_COUNT:
			*(uint32_t*)buff = VIRTUAL_DISK_IMAGE_SIZE / SECTOR_SIZE;
			return RES_OK;
		case GET_BLOCK_SIZE:
			*(uint32_t*)buff = SECTOR_SIZE;
			return RES_OK;
	}
	#endif
	return RES_ERROR;
}

/* ==============================================================================
** SQLITE I/O
** ==============================================================================
*/
#ifdef ENABLE_SQLITE_DISKIO
static int SQLite_Initialize_Done = 0;
extern int SQLite_Disk_Initialize(void);
extern int SQLite_Disk_Read(unsigned int Sector, unsigned int Sector_count, unsigned char *buffer);
#endif
DSTATUS SQLITE_disk_initialize(void)
{
	#ifdef ENABLE_SQLITE_DISKIO
	if(SQLite_Initialize_Done == 0) {
		if(systick_count)
			SQLite_Initialize_Done = 1;
	}
	/* Acquire SQLite connection slot, then lock it */
	if(SQLite_Initialize_Done) {
		SQLiteDataQuery_Lock();
		if(SQLite_Disk_Initialize() == 0) {
			return (DSTATUS)0;
		}
	}
	#endif
	return STA_NOINIT;
}

DSTATUS SQLITE_disk_status(void)
{
	#ifdef ENABLE_SQLITE_DISKIO
	return (DSTATUS)0;
	#else
	return STA_NODISK;
	#endif
}

DRESULT SQLITE_disk_read(BYTE *buff, DWORD sector, UINT count)
{
	#ifdef ENABLE_SQLITE_DISKIO
	SQLiteDataQuery_Lock();
	if(SQLite_Disk_Read((unsigned int)sector, (unsigned int)count, (unsigned char *)buff) == 0) {
		/* Status OK */			
		return RES_OK;
	}
	#endif

	/* Error */
	return RES_ERROR;
}

DRESULT SQLITE_disk_write(const BYTE *buff, DWORD sector, UINT count)
{
	/* TODO:
	** Implement SQLite write...
	*/

	return RES_ERROR;
}

DRESULT SQLITE_disk_ioctl(BYTE cmd, void *buff)
{
#ifdef ENABLE_SQLITE_DISKIO
	switch (cmd)
	{
	case CTRL_SYNC:
		/* no synchronization to do since not buffering in this module */
		return RES_OK;
	case GET_SECTOR_SIZE:
		*(uint16_t*)buff = SECTOR_SIZE;
		return RES_OK;
	case GET_SECTOR_COUNT:
		*(uint32_t*)buff = 8388608; /* 4GB */
		return RES_OK;
	case GET_BLOCK_SIZE:
		*(uint32_t*)buff = SECTOR_SIZE;
		return RES_OK;
	}
#endif
	return RES_ERROR;
}
