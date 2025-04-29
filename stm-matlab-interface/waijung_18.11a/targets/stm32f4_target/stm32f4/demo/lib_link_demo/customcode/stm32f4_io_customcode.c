#include "stm32f4_io_customcode.h"
#include "Lib_header.h"

void enable_customio(void)
{
  
}

void disable_customio(void)
{
	// do nothing
}

void output_customio(boolean_T in1, boolean_T in2, boolean_T * out1,  boolean_T * out2){

  uint8 a,result;
  uint16 b;
  
    result=  ds_gkl_U8( a, b);
	* out1 = result;
	
}
		
