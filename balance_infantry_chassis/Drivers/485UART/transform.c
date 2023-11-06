#include "transform.h"


void inttouint8_t(uint8_t *u8Arry, int data,bool key)
{
    uint8_t farray[4];
    int *p;
		*p=data;
    *(int *)farray = *p;	
    if (key == true)
    {
        u8Arry[3] = farray[0];
        u8Arry[2] = farray[1];
        u8Arry[1] = farray[2];
        u8Arry[0] = farray[3];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
        u8Arry[2] = farray[2];
        u8Arry[3] = farray[3];
    }
}
void int16_ttouint8_t(uint8_t *u8Arry,int16_t data,bool key)
{
    uint8_t farray[2];
    int16_t *p;
		*p=data;
    *(int16_t *)farray = *p;
    if (key == true)
    {
        u8Arry[1] = farray[0];
        u8Arry[0] = farray[1];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
    }
}
void floattouint_8(uint8_t *u8Arry, float data, bool key)
{
    uint8_t farray[4];
		float *p;
		*p=data;
    *(float *)farray = *p;
    if (key == true)
    {
        u8Arry[3] = farray[0];
        u8Arry[2] = farray[1];
        u8Arry[1] = farray[2];
        u8Arry[0] = farray[3];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
        u8Arry[2] = farray[2];
        u8Arry[3] = farray[3];
    }
}
void uint32_ttouint8_t(uint8_t *u8Arry,uint32_t data,bool key)
{
		uint8_t farray[4];
    uint32_t *p;
		*p=data;
    *(uint32_t *)farray = *p;	
    if (key == true)
    {
        u8Arry[3] = farray[0];
        u8Arry[2] = farray[1];
        u8Arry[1] = farray[2];
        u8Arry[0] = farray[3];
    }
    else
    {
        u8Arry[0] = farray[0];
        u8Arry[1] = farray[1];
        u8Arry[2] = farray[2];
        u8Arry[3] = farray[3];
    }
}
float uint8tofloat(uint8_t *data, bool key)
{
    float fa = 0;
    uint8_t uc[4];
    if (key == true)
    {
        uc[3] = data[0];
        uc[2] = data[1];
        uc[1] = data[2];
        uc[0] = data[3];
    }
    else
    {
        uc[0] = data[0];
        uc[1] = data[1];
        uc[2] = data[2];
        uc[3] = data[3];
    }

    memcpy(&fa, uc, 4);
    return fa;
}
int16_t uint8toint16_t(uint8_t *data, bool key)
{
    int16_t fa = 0;
    uint8_t uc[2];
    if (key == true)
    {
        uc[1] = data[0];
        uc[0] = data[1];
    }
    else
    {
        uc[0] = data[0];
        uc[1] = data[1];
    }

    memcpy(&fa,uc,2);
    return fa;
}
int uint8toint(uint8_t *data, bool key)
{
    int fa = 0;
    uint8_t uc[4];
    if (key == true)
    {
        uc[3] = data[0];
        uc[2] = data[1];
				uc[1] = data[2];
				uc[0] = data[3];
    }
    else
    {
        uc[0] = data[0];
        uc[1] = data[1];
        uc[2] = data[2];
        uc[3] = data[3];
    }
    memcpy(&fa,uc,4);
    return fa;
}
uint32_t uint8touint32_t(uint8_t *data, bool key)
{
		uint32_t fa = 0;
    uint8_t uc[4];
    if (key == true)
    {
        uc[3] = data[0];
        uc[2] = data[1];
				uc[1] = data[2];
				uc[0] = data[3];
    }
    else
    {
        uc[0] = data[0];
        uc[1] = data[1];
        uc[2] = data[2];
        uc[3] = data[3];
    }
    memcpy(&fa,uc,4);
    return fa;
}
