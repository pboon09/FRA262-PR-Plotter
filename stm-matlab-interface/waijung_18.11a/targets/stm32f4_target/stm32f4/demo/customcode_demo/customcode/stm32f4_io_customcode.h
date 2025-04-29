#ifndef __STM32F4_IO_CUSTOMCODE_H
#define __STM32F4_IO_CUSTOMCODE_H

#include "waijung_hwdrvlib.h"
#include "stm32f4xx_gpio.h"

#define outD12 Peripheral_BB(GPIOD->ODR, 12)
#define outD13 Peripheral_BB(GPIOD->ODR, 13)

void start_customio(void);

void enable_customio(void);

void output_customio(const real_T *in1, boolean_T in2, boolean_T * out1,  boolean_T * out2);

void disable_customio(void);

#endif
