#ifndef BASIC_ENGINE_COMMON_H
#define BASIC_ENGINE_COMMON_H
#include <stdint.h>

typedef char s8;
typedef unsigned char u8;
typedef int16_t s16;
typedef uint16_t u16;
typedef int32_t s32;
typedef uint32_t u32;
typedef int64_t s64;
typedef uint64_t u64;
typedef int32_t boolean;
typedef float r32;
typedef double r64;

#define true 1
#define false 0

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#endif