

#include "amg_utils.h"

/* *************************************************************************
 ** Random generator
 ** *************************************************************************
 */
#if 0
/* Init random generator module.
 */
void stm32_ramdom_init(void) {
    /* Enable RNG clock source */
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    
    /* RNG Peripheral enable */
    RNG_Cmd(ENABLE);
}

/* Get a random value.
 */
uint32_t stm32_random(uint32_t value) {
    uint32_t i;
    uint32_t rnd_number;
    
    /* Wait until one RNG number is ready */
    while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET);
    
    /* Get a 32bit Random number */
    rnd_number = RNG_GetRandomNumber();
    
    /* Truncate result */
    for(i=0; i<32; i++) {
        if((rnd_number = (0xFFFFFFFF >> i) & rnd_number) < value) {
            return rnd_number;
        }
    }
    
    /* Un-expected */
    return 0;
}
#endif

/* *************************************************************************
 ** String buffer processing
 ** *************************************************************************
 */

/* String split:
**  Warning: buffer will be modified.
** Return:
**  Number of item
*/
int _ssplit(char *buffer, char **list, int list_count, char *splitter)
{
	char *s, *p;
	int count;

	count = 0;
	s = buffer;
	while (*s && (count < list_count)) {		
		if((p=strstr(s, splitter)) != (void*)0) { /* splitter found */
			if(p != s) {
				list[count] = s;
				count ++;
				*p = '\0';
				s = p + strlen(splitter);
			}
			else {
				s += strlen(splitter);
			}
		}
		else { /* No splitter found */
			if(*s) {
				list[count] = s;
				s += strlen(s);
				count ++;
			}
		}
	}

	/* Return cout */
	return count;
}

/* Warning: buffer will be modified
*/
void _sstr_rtrim(char *buffer) {
	char *p;

	if((buffer == (void*)0) || (*buffer == '\0'))
		return;

	p = &buffer[strlen(buffer)-1];
	while((p>buffer) && ((*p <= ' ') || (*p >= 127))) {
		*p-- = '\0';
	}
}

/*
 ** String buffer operation
 ** Parameters:
 **  - buffer, Storage location for output
 **  - buffer_size, Maximum number of characters to store.
 **  - format, Format-control string
 ** Return Value
 **  - The number of characters written, or –1 if an error occurred. 
 **    If buffer or format is a null pointer, sprintf_s return -1.
 **    _sprintf_s returns the number of bytes stored in buffer, not counting the terminating null character.
*/
int _sprintf_s(char *buffer, int buffer_size, const char *format,  ... )
{
  char c;
  int rem, count;
  char tmp[8];
  char tmp_output[32];
  char *b, *f, *p, *pos;
  va_list ap;

  /* buffer cannot be NULL */
  if((buffer==(void*)0) || (buffer_size<1))
    return -1;
	
  /* Setup running pointer, counter */
  *(b = buffer) = '\0'; /* NULL */
  f = (char*)format;
  rem = buffer_size-1;
	
  /* Processing */
  va_start(ap, format);
  while (*f) {
    if(rem < 1) { /* Note: not expect a negative value. */
      return (buffer_size-1); /* Not include NULL */
    }
    else if(*f == '%') {
      if(!strncmp(f, "%%", 2)) {
        rem -= 1;
        strncpy(b, "%", 1);
        f += 2;
        *(char*)(b += 1) = '\0'; /* append NULL */		
      }
      else {
        p = tmp;
        do {*p ++ = *f++;	}
        while (*f && ((*f>='0') & (*f<='9') || (*f == '.')));

        c = *f;
        *p ++ = *f++;
        *p ++ = '\0';

        p = tmp_output;
        switch (c) {
          /* char* */
          case 's': case 'S':
            p = (char*)va_arg(ap, char*); /* Ignore precision */
			if(p) {
				count = strlen(p);
				if(count > 255) count = 255; /* Limit string */					
			}
			else { count = 0; }
            break;
          /* char */
          case 'c': case 'C':
            if ((count = sprintf(tmp_output, tmp, (char)va_arg(ap, int))) < 0)
              return -1;
            break;
          /* unsigned int */
          case 'u': case 'U': case 'i': case 'I': case 'o': case 'O': case 'x': case 'X':
            if ((count = sprintf(tmp_output, tmp, (unsigned int)va_arg(ap, unsigned int))) < 0)
              return -1;
            break;
          /* int */
          case 'd': case 'D':
            if ((count = sprintf(tmp_output, tmp, (int)va_arg(ap, int))) < 0)
              return -1;
            break;
          /* float */
          case 'f': case 'F': case 'e': case 'E': case 'g': case 'G':
            if ((count = sprintf(tmp_output, tmp,(float)va_arg(ap, double))) < 0)
              return -1;
            break;
          /* Un-supported */
          default:
            return -1;
        }
        if(count > rem) { count = rem; }
        strncpy(b, p, count);
        rem -= count;				
        *(char*)(b += count) = '\0'; /* append NULL */
      }
    }
    else if((pos=strstr(f, "%")) != (void*)0) { /* found '%' in format string*/			
      count = ((pos-f) < rem)?(pos-f):rem;
      rem -= count;
      strncpy(b, f, count);
      f = pos;
      *(char*)(b += count) = '\0'; /* append NULL */			
    }
    else {
      count = (strlen(f) < rem)?(strlen(f)):rem;
      rem -= count;
      strncpy(b, f, count);
      f += count;
      *(char*)(b += count) = '\0'; /* append NULL */
      return ((buffer_size-rem-1));
    }
  }
  return -1;
}

int _fprintf_s(FIL *file, int blocksize, const char *format,  ... )
{
  UINT written;
  char c;
  int size, count;
  char tmp[8];
  char tmp_output[32];
  char *f, *p, *pos;
  va_list ap;

  /* file cannot be NULL */
  if(file==(void*)0)
    return -1;
	
  /* Setup running pointer, counter */
  f = (char*)format;
  size = 0;

  /* Processing */
  va_start(ap, format);
  while (*f) {
	if(*f == '%') {
      if(!strncmp(f, "%%", 2)) {
	    if(f_write(file, "%", 1, &written) != FR_OK)
          return -1;
        f += 2;
		size += 1;
      }
      else {
        p = tmp;
        do {*p ++ = *f++;	}
        while (*f && ((*f>='0') & (*f<='9') || (*f == '.')));

        c = *f;
        *p ++ = *f++;
        *p ++ = '\0';

        p = tmp_output;
        switch (c) {
          /* char* */
          case 's': case 'S':
            p = (char*)va_arg(ap, char*); /* Ignore precision */
            count = strlen(p);
            break;
          /* char */
          case 'c': case 'C':
            if ((count = sprintf(tmp_output, tmp, (char)va_arg(ap, int))) < 0)
              return -1;
            break;
          /* unsigned int */
          case 'u': case 'U': case 'x': case 'X':
            if ((count = sprintf(tmp_output, tmp, (unsigned int)va_arg(ap, unsigned int))) < 0)
              return -1;
            break;
          /* int */
          case 'd': case 'D':
            if ((count = sprintf(tmp_output, tmp, (int)va_arg(ap, int))) < 0)
              return -1;
            break;
          /* float */
          case 'f': case 'F':
            if ((count = sprintf(tmp_output, tmp,(float)va_arg(ap, double))) < 0)
              return -1;
            break;
          /* Un-supported */
          default:
            return -1;
        }
		size += count;
		while(count > 0) { /* Limit block size */
		  if(count > blocksize) {
	        if(f_write(file, p, blocksize, &written) != FR_OK)
              return -1;
			p += blocksize;
			count -= blocksize;
		  }
		  else {
	        if(f_write(file, p, count, &written) != FR_OK)
              return -1;
			count = 0;
		  }
		}
      }
    }
    else if((pos=strstr(f, "%")) != (void*)0) { /* found '%' in format string*/			
      if(f_write(file, f, (pos-f), &written) != FR_OK)
        return -1;
	  size += (pos-f);
      f = pos;
    }
    else {
      if(f_write(file, f, strlen(f), &written) != FR_OK)
        return -1;
	  size += strlen(f);
      f += strlen(f);
      return size;
    }
  }
  return -1;
}

/* *************************************************************************
 ** FF
 ** *************************************************************************
 */
const char *drives[_VOLUMES] = {"0:", "1:", "2:"};
static FATFS vol[_VOLUMES];
static FRESULT last_io_error = FR_OK;

FRESULT get_last_io_error(void)
{
	return last_io_error;
}

/* Mount media.
 */
int mount_vol(int id) {
    if(f_mount (&vol[id], drives[id], 1) == FR_OK)
        return 0;
    return -1;
}

/* Display error message from chan ff.
 */
void print_file_error(void) {
	/*
    switch(last_io_error) {
        case FR_OK: printf("(0) Succeeded\n"); break;
        case FR_DISK_ERR: printf("(1) A hard error occured in the low level disk I/O layer\n"); break;
        case FR_INT_ERR: printf("(2) Assertion failed\n"); break;
        case FR_NOT_READY: printf("(3) The physical drive cannot work\n"); break;
        case FR_NO_FILE: printf("(4) Could not find the file\n"); break;
        case FR_NO_PATH: printf("(5) Could not find the path\n"); break;
        case FR_INVALID_NAME: printf("(6) The path name format is invalid\n"); break;
        case FR_DENIED: printf("(7) Acces denied due to prohibited access or directory full\n"); break;
        case FR_EXIST: printf("(8) Acces denied due to prohibited access\n"); break;
        case FR_INVALID_OBJECT: printf("(9) The file/directory object is invalid\n"); break;
        case FR_WRITE_PROTECTED: printf("(10) The physical drive is write protected\n"); break;
        case FR_INVALID_DRIVE: printf("(11) The logical drive number is invalid\n"); break;
        case FR_NOT_ENABLED: printf("(12) The volume has no work area\n"); break;
        case FR_NO_FILESYSTEM: printf("(13) There is no valid FAT volume\n"); break;
        case FR_MKFS_ABORTED: printf("(14) The f_mkfs() aborted due to any parameter error\n"); break;
        case FR_TIMEOUT: printf("(15) Could not get a grant to access the volume within defined period\n"); break;
        case FR_LOCKED: printf("(16) The operation is rejected according to the file shareing policy\n"); break;
        case FR_NOT_ENOUGH_CORE: printf("(17) LFN working buffer could not be allocated\n"); break;
        case FR_TOO_MANY_OPEN_FILES: printf("(18) Number of open files > _FS_SHARE\n"); break;
        case FR_INVALID_PARAMETER: printf("(19) Given parameter is invalid\n"); break;
        default:
            printf("Unknown file error!"); break;
    }*/
}

/*
 ** mode:
 ** 00 - Existence only
 ** 02 - Write-only
 ** 04 - Read-only
 ** 06 - Read and write
 */
int _file_access(const char *zFilename, int mode){
    FILINFO finfo;
    FRESULT res;
    
    /* Get file info */
    res = f_stat(zFilename, &finfo);
    
    /* Check mode */
    switch( mode ){
        case 00: // Existing
            if(res==FR_OK)
                return 0;
            break;
        case 02: // Write
            if(finfo.fattrib & FA_WRITE)
                return 0;
            break;
        case 04: // Read
            if(finfo.fattrib & FA_READ)
                return 0;
            break;
        case 06: // Read/Write
            if((finfo.fattrib & FA_WRITE) && (finfo.fattrib & FA_READ))
                return 0;
            break;
        default:
            /* Invalid flags argument */
            break;
    }
    return -1;
}


/* Read line */
int _freadln(FIL* f, char *buffer, int buffer_len, const char* eol) {
    unsigned int actual_count;
    char * tmp;
    
    /* Read */
    if((last_io_error = f_read(f, (void*)buffer, buffer_len, &actual_count)) != FR_OK)
        return -1;
    
    /* NULL */
    if(actual_count < buffer_len)
        buffer[actual_count] = '\0';
    else
        buffer[actual_count-1] = '\0';
    
    /* detect eol*/
    if((tmp = strstr(buffer, eol))!= 0) {
        *tmp = '\0';
        tmp += strlen(eol);
        
        /* Move pointer back */
        if(f_lseek(f, f_tell(f)-(actual_count-(tmp-buffer))) != FR_OK)
            return -1;
    }
    /* did not found eol */
    else {		}
    
    return 0;
}

/* Write string to file.
 */
int _fwritestr(FIL *f, const char* str) {
    unsigned int actual_count = 0;
    last_io_error = (f_write(f, (const void*)str, strlen(str), &actual_count));
    if(actual_count != strlen(str)) {
        /* Write buffer full or less than data to write */
        /* Handle this error when needed. */
    }
    return (int)last_io_error;
}

/* Create directory, with include sub-dir
 */
int _dir_create(const char* dir) {
    #define LIMIT_DIR_LEN		512
            DIR d;
    FRESULT f_res;
    char* seperator; /* '\', '/' */
    char* tmp;
    const char* pos;
    const char* start;
    int res = 0;
    
    /* Check if root directory */
    if(*dir == 0)
        return 0;
    
    /* Length exceed maximum limit (LIMIT_DIR_LEN-1) */
    if(strlen(dir) >= LIMIT_DIR_LEN)
        return -1;
    
    /* Allocate working buffer */
    if((tmp = malloc(LIMIT_DIR_LEN)) == (void*)0) {
        return -1; /* Out of memory */
    }
    
    /* Find seperator char */
    seperator = "/";
    if(strstr(dir, "\\"))
        seperator = "\\";
    
    /* Remove first seperator, if existed */
    pos = dir;
    if(dir[0] == seperator[0])
        pos++;
    
    start = pos;
    while(pos && *pos) {
        /* Contain separator */
        if((pos = strstr(pos, seperator)) != 0) {
            strncpy(tmp, start, (pos-start));
            tmp[(pos-start)] = '\0';
            pos++;
        }
        /* No separator */
        else {
            strcpy(tmp, start);
        }
        /* Check existing */
        f_res = f_opendir(&d, tmp);
        switch(f_res) {
            case FR_NO_PATH:
                if(f_mkdir(tmp) != FR_OK) {
                    res = -1;
                    goto ___exit_point;
                }
                break;
                
            case FR_OK:
                break;
                
            default:
                res = -1;
                goto ___exit_point;
                //break;
        }
    }
    
    /* Free mem */
    ___exit_point:
        
        free(tmp);
        return res;
}

/*
 * function
 * fopen
 * <cstdio>
 * FILE * fopen ( const char * filename, const char * mode );
 *
 * Open file
 * Opens the file whose name is specified in the parameter filename and associates it with a stream that can be identified
 * in future operations by the FILE object whose pointer is returned. The operations that are allowed on the stream and how
 * these are performed are defined by the mode parameter.
 * The running environment supports at least FOPEN_MAX files open simultaneously;
 * FOPEN_MAX is a macro constant defined in <cstdio>.
 *
 * Parameters
 * filename C string containing the name of the file to be opened. This paramenter must follow the file name specifications
 * of the running environment and can include a path if the system supports it. mode C string containing a file access mode.
 * It can be:
 *
 * "r"
 * Open a file for reading. The file must exist.
 *
 * "w"
 * Create an empty file for writing. If a file with the same name already exists its content is erased and the file is treated
 * as a new empty file.
 *
 * "a"
 * Append to a file. Writing operations append data at the end of the file. The file is created if it does not exist.
 *
 * "r+"
 * Open a file for update both reading and writing. The file must exist.
 *
 * "w+"
 * Create an empty file for both reading and writing. If a file with the same name already exists its content is erased and the
 * file is treated as a new empty file.
 *
 * "a+"
 * Open a file for reading and appending. All writing operations are performed at the end of the file, protecting the previous
 * content to be overwritten. You can reposition (fseek, rewind) the internal pointer to anywhere in the file for reading,
 * but writing operations will move it back to the end of file. The file is created if it does not exist.
 *
 * With the mode specifiers above the file is open as a text file. In order to open a file as a binary file, a "b" character
 * has to be included in the mode string. This additional "b" character can either be appended at the end of the string
 * (thus making the following compound modes: "rb", "wb", "ab", "r+b", "w+b", "a+b") or be inserted between the letter and
 * the "+" sign for the mixed modes ("rb+", "wb+", "ab+").
 * Additional characters may follow the sequence, although they should have no effect. For example, "t" is sometimes appended
 * to make explicit the file is a text file.
 * In the case of text files, depending on the environment where the application runs, some special character conversion may
 * occur in input/output operations to adapt them to a system-specific text file format. In many environments, such as most
 * UNIX-based systems, it makes no difference to open a file as a text file or a binary file; Both are treated exactly the same way,
 * but differentiation is recommended for a better portability.
 * For the modes where both read and writing (or appending) are allowed (those which include a "+" sign), the stream should be
 * flushed (fflush) or repositioned (fseek, fsetpos, rewind) between either a reading operation followed by a writing operation
 * or a writing operation followed by a reading operation.
 *
 * Return Value
 * If the file has been successfully opened the function will return a pointer to a FILE object that is used to identify the
 * stream on all further operations involving it. Otherwise, a null pointer is returned.
 */
static FIL f_;
FIL * _fopen(const char *filename, const char *mode) {
    FIL* file;
    //int i;
    
    FRESULT r;
    
    unsigned char fmode;
    unsigned char isExisting;
    unsigned char isAppend;
    unsigned char isTruncate;
    
    isAppend = 0;
    isTruncate = 0;
    fmode = 0;

    /* Check file existing, set flag create new if not existing */
    isExisting = 0;
    r = f_open(&f_, filename, FA_READ);
    if(r == FR_OK) {
        isExisting = 1;
    }
    else if(r==FR_NO_FILE) {
        isExisting = 0;
    }
    else {
        /* Failed to open file due to error, return without close */
        return NULL;
    }
    f_close(&f_);
    
    /* "r", "rb"
     ** Open a file for reading. The file must exist.
     */
    if(!strcmp("r", mode) || !strcmp("rb", mode)) {
        fmode = (FA_READ| FA_OPEN_EXISTING);
    }
    /* 	"w", "wb"
     ** Create an empty file for writing. If a file with the same name already
     ** exists its content is erased and the file is treated
     ** as a new empty file. */
    else if(!strcmp("w", mode) || !strcmp("wb", mode)) {
        fmode = FA_WRITE;
        if(!isExisting) /* Create new if not existing */
            fmode |= FA_CREATE_NEW;
        isTruncate = 1;
    }
    /* a */
    else if(!strcmp("a", mode) || !strcmp("ab", mode) || !strcmp("a+", mode)) {
        fmode = FA_WRITE;
        if(!isExisting) /* Create new if not existing */
            fmode |= FA_CREATE_NEW;
        isAppend = 1;
    }
    /* r+ */
    else if(!strcmp("r+", mode)) {
        fmode = (FA_READ| FA_WRITE);
        //if(!isExisting) /* Create new if not existing */
        //	fmode |= FA_CREATE_NEW;
    }
    /* w+ */
    else if(!strcmp("w+", mode)) {
        fmode = (FA_WRITE);
        if(!isExisting) /* Create new if not existing */
            fmode |= FA_CREATE_NEW;
        isTruncate = 1;
    }
    /* Not supported yet */
    else {
        return NULL;
    }
    
    /* Create file */
    file = (FIL*)malloc(sizeof(FIL));    
    if(file == NULL) {
        return NULL;
    }
    memset(file, 0, sizeof(FIL));    
	    
    last_io_error = f_open(file, filename, fmode);

    /* Goto EOF */
	if(last_io_error == FR_OK) {
		if(isAppend) {
			last_io_error = f_lseek(file, f_size(file));
		}
		else if(isTruncate) {
			last_io_error = f_truncate(file);
	    }
	}
	    
    /* Failed to open file */
    if(last_io_error != FR_OK) {
       // _fclose(file);
        free(file);
        return NULL;
    }
    
    /* No error */
    return file;
}

/* Close file.
 */
int _fclose(FIL *file) {

    /* Do nothing NULL */
    if(file) {
        /* Get FIL */
        f_close(file);
        
        /* Free mem */
        free(file);
    }
    return 0;
}

/* Get file extension
*/
int _fextension_pos(const char *filename)
{
	const char *p;

	p = &filename[strlen(filename)];
	while((p != filename) && (*p != '.'))
		--p;
	return p-filename;
}

int _fpath(char *output, int output_len, const char *id, const char *dir, const char *name)
{
	const char *s;

	output[0] = '\0';
  if(id && *id)
		sprintf(&output[strlen(output)], "%s:\\", id);
	if(dir && *dir) {
		s = dir;
		while(*s && ((*s == '\\') || (*s == '/')))
			s++;
		if(*s) {
			strcpy(&output[strlen(output)], s);
			while(strlen(output) && ((output[strlen(output)-1] == '\\')||(output[strlen(output)-1] == '/'))) {
				output[strlen(output)-1] = '\0';
			}
		}
	}
	if(name && *name) {
		s = name;
		while(*s && ((*s == '\\') || (*s == '/')))
			s++;
		if(*s) {
			if(output[strlen(output)-1] != '\\')
				strcpy(&output[strlen(output)], "\\");
			strcpy(&output[strlen(output)], s);
		}		
	}

  while ((s = strstr(output, "/")) != 0) {
		output[s-output] = '\\';
	}
	return 0;
}

/* Get file name from full-path */
int _fname_pos(const char *fullpath)
{
	const char *p;

	p = &fullpath[strlen(fullpath)];
	while((p != fullpath) && (*p != '\\')&& (*p != '/'))
		--p;
	return p-fullpath;
}


/*
 **  get_fattime
 **		The get_fattime function gets current time.
 **	Return Value
 **		Currnet time is returned with packed into a DWORD value.
 **			The bit field is as follows:
 **		bit31:25 - Year from 1980 (0..127)
 **		bit24:21 - Month (1..12)
 **		bit20:16 - Day in month(1..31)
 **		bit15:11 - Hour (0..23)
 **		bit10:5 - Minute (0..59)
 **		bit4:0 - Second / 2 (0..29)
 **	Description
 **		The get_fattime function must return any valid time even if the system does not support
 **		a real time clock. If a zero is returned, the file will not have a valid time.
 **		This fucntion is not required in read only configuration.
 */
DWORD get_fattime(void) {
    return ((DWORD)32<<25)
    | ((DWORD)9<<21)
    | ((DWORD)9<<16)
    | ((DWORD)19<<11)
    | ((DWORD)12<<5)
    | ((DWORD)25<<0);
}

/* Interrupt routine for SD card */
void SDIO_IRQHandler(void) {
    SD_ProcessIRQSrc();
}

void DMA2_Stream3_IRQHandler(void) {
    SD_ProcessDMAIRQ();
}

/*  fat file system
 */
void stdio_init(void) {
	/* Indicate init status */
	static uint8_t sdio_init_ready = 0;

	uint8_t i;
    NVIC_InitTypeDef NVIC_InitStructure;

	if(!sdio_init_ready) {    
		sdio_init_ready = 1; /* Activate ready state */

		// SDIO Interrupt ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
		#ifdef SD_DMA_MODE
		// DMA2 STREAMx Interrupt ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_Init(&NVIC_InitStructure);
		#endif

		/* Mount FAT */
		for (i=0; i<_VOLUMES; i++) {
			mount_vol(i);
		}
	}
}

