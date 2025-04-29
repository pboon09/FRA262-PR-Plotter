
#include "sprintf_s.h"
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
int sprintf_s(char *buffer, int buffer_size, const char *format,  ... )
{
  char c;
  int rem, count;
  char tmp[8];
  char tmp_output[256];
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
            p = (char*)va_arg(ap, char*); /* Ignore padding */
            count = strlen(p);
			if(count > 255)
				count = 255;
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
          /* Un-supported or invalid */
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
