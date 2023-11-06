#ifndef TRANSFORM_H
#define TRANSFORM_H
#include "struct_typedef.h"
#include <stdbool.h>
#include <string.h>

extern void inttouint8_t(uint8_t *u8Arry, int data,bool key);
extern void uint32_ttouint8_t(uint8_t *u8Arry,uint32_t data,bool key);
extern void int16_ttouint8_t(uint8_t *u8Arry,int16_t data,bool key);
extern void floattouint_8(uint8_t *u8Arry, float data, bool key);

extern int16_t uint8toint16_t(uint8_t *data, bool key);
extern uint32_t uint8touint32_t(uint8_t *data, bool key);
extern float uint8tofloat(uint8_t *data, bool key);
extern int uint8toint(uint8_t *data, bool key);

#endif

