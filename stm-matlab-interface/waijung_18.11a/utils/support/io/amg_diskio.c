/* Original file is from Chan FF */

#include "diskio.h"
#include "amg_sdio.h"
#include "waijung_hwdrvlib.h"

extern int SQLite_Disk_Initialize(void);
extern int SQLite_Disk_Read(unsigned int Sector, unsigned int Sector_count, unsigned char *buffer);

#ifdef ENABLE_VIRTUAL_DISKIO
  extern const char disk_image[];
  #define VIRTUAL_DISK_IMAGE_SIZE (VIRTUAL_DISK_END - VIRTUAL_DISK_START + 1)
  const BYTE *virtual_disk_img = (const BYTE*)VIRTUAL_DISK_START;
#endif

#define SECTOR_SIZE 512

__IO SD_Error Status = SD_OK;
SD_CardStatus SDCardStatus;

static volatile DSTATUS Stat = STA_NOINIT;	/* Disk status */

#define SDIO_DRIVE		0 /* SD Card drive, SDIO */
#define FMCU_DRIVE		1 /* Internal MCU Flash (Disk emulation) */
#define SQLITE_DRIVE	2 /* Redirect to SQLite board. */

extern SD_CardInfo SDCardInfo;

/* Initialize a Drive
*/
DSTATUS disk_initialize (BYTE pdrv /* Physical drive nmuber (0..) */ )
{ 
  switch (pdrv) 
  {
	/* SD Card */
    case SDIO_DRIVE:
    {     
      /* Initialize SD Card */
      Status = SD_Init(); 
      
      if (Status != SD_OK)
        return STA_NOINIT;
			
      return 0x00;			
    }

#ifdef ENABLE_VIRTUAL_DISKIO
	/* Internal flash */
	case FMCU_DRIVE:
	{
		return 0;
    }		
#endif

#ifdef ENABLE_SQLITE_DISKIO
    /* SQLite board */
    case SQLITE_DRIVE:
    {
      /* Acquire SQLite connection slot, then lock it */
      SQLiteDataQuery_Lock();
      if(SQLite_Disk_Initialize() == 0) {    
        return 0;
      }
    }
#endif
  }
  
  return STA_NOINIT;
  
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
  switch (pdrv) 
  {
		/* SD Card */
    case SDIO_DRIVE:
    {
		if(Status != SD_OK) {
				
			Status = SD_Init();
			if(Status == SD_OK)
				Status = SD_GetCardInfo(&SDCardInfo);
		}
		else {
			Status = SD_GetCardInfo(&SDCardInfo);
		}

      if (Status != SD_OK) {
        return STA_NOINIT;
		}

			return 0x00;
    }
		
#ifdef ENABLE_VIRTUAL_DISKIO
		/* Internal flash */
		case FMCU_DRIVE:
			/* Status OK */			
			return 0;	
#endif

#ifdef ENABLE_SQLITE_DISKIO
		case SQLITE_DRIVE:  
			/* Status OK */			
			return 0;	
#endif		
  }
  
  return STA_NOINIT;
}

/* Read Sector(s)
*/
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	SDTransferState trans_state = SD_TRANSFER_OK;
	#if !defined(SD_DMA_MODE)
	int secNum;
	#endif
	
  switch (pdrv) 
  {
    case SDIO_DRIVE:
    {     
      Status = SD_OK;
			/* Read Multiple Blocks */
			Status = SD_ReadMultiBlocks((uint8_t*)(buff),(sector)*SECTOR_SIZE,SECTOR_SIZE,count);

			/* Check if the Transfer is finished */
			// TODO:
			// To add check statuc of wait operation
			Status = SD_WaitReadOperation();

			/* Wait until end of DMA transfer */			
			do {
				trans_state = SD_GetStatus();
			}
			while(trans_state == SD_TRANSFER_BUSY);
			
			if ((Status == SD_OK) && (trans_state == SD_TRANSFER_OK))
				return RES_OK;
			else
				return RES_ERROR;
    }
	
#ifdef ENABLE_VIRTUAL_DISKIO
		/* Internal flash */
		case FMCU_DRIVE:
		{
			memcpy(buff, &virtual_disk_img[sector*SECTOR_SIZE], count*SECTOR_SIZE);			
			
			/* Status OK */			
			return RES_OK;
      }	
#endif
				
#ifdef ENABLE_SQLITE_DISKIO
        case SQLITE_DRIVE:
           SQLiteDataQuery_Lock();
           if(SQLite_Disk_Read((unsigned int)sector, (unsigned int)count, (unsigned char *)buff) == 0) {
			 /* Status OK */			
			 return RES_OK;
           }
           else {
             return RES_ERROR;
           }
#endif
  }
  return RES_PARERR;
}

/* Write Sector(s)
*/
#if _READONLY == 0
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	SDTransferState trans_state = SD_TRANSFER_OK;
	#if !defined(SD_DMA_MODE)
	int secNum;
	#endif
	
  switch (pdrv) 
  {
    case SDIO_DRIVE:
    {     
		Status = SD_OK;

			
		/* Write Multiple Blocks */
		Status = SD_WriteMultiBlocks((uint8_t*)(buff),(sector)*SECTOR_SIZE,SECTOR_SIZE,count);

		/* Check if the Transfer is finished */
		// TODO:
		// To add check statuc of wait operation
		Status = SD_WaitWriteOperation();

		/* Wait until end of DMA transfer */		
			do {
				trans_state = SD_GetStatus();
			}
			while(trans_state == SD_TRANSFER_BUSY);
			
			if ((Status == SD_OK) && (trans_state == SD_TRANSFER_OK))
				return RES_OK;
			else
				return RES_ERROR;			
    }
#ifdef ENABLE_SQLITE_DISKIO
        case SQLITE_DRIVE:
			/* Status OK */			
			return RES_OK;
#endif
  }
  return RES_PARERR;
}
#endif /* _READONLY */



/* Miscellaneous Functions
*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  switch (pdrv) 
  {
    case SDIO_DRIVE:
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
			  return RES_OK;

      }

#ifdef ENABLE_VIRTUAL_DISKIO			
	/* Internal flash */
	case FMCU_DRIVE:
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
			return RES_OK;			
#endif

#ifdef ENABLE_SQLITE_DISKIO
     case SQLITE_DRIVE:
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
	   /* Status OK */			
	   return RES_OK;
#endif
    }
  }
  return RES_PARERR;
}
