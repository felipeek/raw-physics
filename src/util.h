#ifndef RAW_PHYSICS_UTIL_H
#define RAW_PHYSICS_UTIL_H
#include <common.h>

s8* util_read_file(const s8* path, s32* file_length);
void util_free_file(s8* file);
r64 util_random_float(r64 min, r64 max);

#endif