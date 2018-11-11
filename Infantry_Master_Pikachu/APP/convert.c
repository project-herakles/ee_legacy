#include "main.h"
#include "convert.h"
#include <stdint.h>

float byte2float(unsigned char* bytes)
{
	union{
		float f;
		unsigned char byte[4]; 
	}cv;
	uint8_t i;
	for(i=0;i<4;i++)
	{
		cv.byte[i] = bytes[i];//Order might change due to computer architecture. PC->stm32
	}
	return cv.f;
}

void float2bytes(float flt,unsigned char* buffer)
{
	union{
		float f;
		unsigned char byte[4];
	}cv;
	cv.f = flt;
	uint8_t i;
	for(i=0;i<4;i++)
	{
		buffer[i] = cv.byte[i]; //Order might change due to computer architecture. stm32->PC 
	}
}

uint8_t bytes2uint8_t(unsigned char byte)
{
	union{
		uint8_t i;
		unsigned char byte;
	}cv;
	cv.byte = byte;
	return cv.i;
}

int16_t byte2int16_t(unsigned char* data)
{
	union{
		int16_t i;
		unsigned char byte[2];
	}cv;
	cv.byte[0] = data[0];
	cv.byte[1] = data[1];
	return cv.i;
}

void uint16_t2byte(int16_t num,unsigned char* data)
{
	union{
		uint16_t i;
		unsigned char byte[2];
	}cv;
	cv.i = num;
	data[0] = cv.byte[0];
	data[1] = cv.byte[1];
}

int8_t byte2int8_t(unsigned char d)
{
	union{
		int8_t i;
		unsigned char data;
		}cv;
		cv.data = d;
		return cv.i;
}


