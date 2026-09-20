#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h> //uint32_t, int32_t
#include <stddef.h> //size_t

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
// typedef unsigned int uint32_t;

typedef signed char int8_t;
typedef signed short int16_t;
// typedef signed int int32_t;

typedef volatile unsigned char vuint8_t;
typedef volatile unsigned short vuint16_t;
typedef volatile unsigned int vuint32_t;

typedef volatile signed char vint8_t;
typedef volatile signed short vint16_t;
typedef volatile signed int vint32_t;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

typedef signed char s8;
typedef signed short s16;
typedef signed int s32;
typedef signed long long s64;

typedef float fp32;
typedef double fp64;

#endif
