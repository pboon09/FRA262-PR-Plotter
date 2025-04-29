#include "stm32f4_io_customcode.h"

void start_customio(void)
{
  /* Initial once */
}

void enable_customio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void disable_customio(void)
{
	// do nothing
}

void output_customio(const real_T *in1, boolean_T in2, boolean_T * out1,  boolean_T * out2){
    *out1 = (in1[0] != 0);
    *out2 = !in2;
}
		
