#ifndef RAW_PHYSICS_UTIL_H
#define RAW_PHYSICS_UTIL_H
#include <common.h>
#include <gm.h>
#include "quaternion.h"

s8* util_read_file(const s8* path, s32* file_length);
void util_free_file(s8* file);
r64 util_random_float(r64 min, r64 max);
mat4 util_get_model_matrix_no_scale(const Quaternion* rotation, vec3 translation);
void util_matrix_to_r32_array(const mat4* m, r32 out[16]);
vec4 util_pallete(u32 n);

// hash utils
int util_eid_compare(const void *key1, const void *key2);
int util_vec3_compare(const void *key1, const void *key2);
unsigned int util_vec3_hash(const void *key);
unsigned int util_eid_hash(const void *key);

#endif