#ifndef __STM32F4_IO_CUSTOMCODE_H
#define __STM32F4_IO_CUSTOMCODE_H

#include "waijung_hwdrvlib.h"

#define uint8 short unsigned int
#define uint16 unsigned int


void enable_customio(void);

void output_customio(boolean_T in1, boolean_T in2, boolean_T * out1,  boolean_T * out2);

void disable_customio(void);

#endif
