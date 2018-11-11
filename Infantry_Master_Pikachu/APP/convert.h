#ifndef __CONVERT_H_
#define __CONVERT_H_
#include <stdint.h>

float byte2float(unsigned char* bytes);
void float2bytes(float flt,unsigned char* buffer);
void uint16_t2byte(int16_t num,unsigned char* data);
int8_t byte2int8_t(unsigned char d);

#endif

 
