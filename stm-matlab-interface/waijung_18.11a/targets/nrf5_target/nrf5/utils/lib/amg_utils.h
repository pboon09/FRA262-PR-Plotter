
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ff.h"
#include <stdint.h>


#ifndef AMG_UTILS_H_
#define AMG_UTILS_H_

int _ssplit(char *buffer, char **list, int list_count, char *splitter);
void _sstr_rtrim(char *buffer);

void stm32_ramdom_init(void);
uint32_t stm32_random(uint32_t value);
void print_file_error(void);
int _file_access(const char *zFilename, int mode);
int _freadln(FIL* f, char *buffer, int buffer_len, const char* eol);
int _fwritestr(FIL *f, const char* str);
int _dir_create(const char* dir);
FRESULT get_last_io_error(void);

/* ... */
FIL * _fopen(const char *filename, const char *mode);
int _fclose(FIL *file);
int _fextension_pos(const char *filename);
int _fname_pos(const char *fullpath);
int _fpath(char *output, int output_len, const char *id, const char *dir, const char *name);

void stdio_init(void);


#endif // RAPIDSTM32_UTILS_H_
