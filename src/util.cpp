#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include "entity.h"
#include <limits.h>

r64 util_random_float(r64 min, r64 max) {
    r64 scale = rand() / (r64)RAND_MAX;
    return min + scale * (max - min);
}

s8* util_read_file(const s8* path, s32* _file_length) {
	FILE* file;
	s8* buffer;
	s32 file_length;

	file = fopen(path, "rb");

	if (file == NULL) {
		fprintf(stderr, "Error opening file [%s]: [%s]\n", path, strerror(errno));
		return NULL;
	}

	if (fseek(file, 0, SEEK_END)) {
		fprintf(stderr, "Error seeking file [%s]: [%s]\n", path, strerror(errno));
		return NULL;
	}

	file_length = ftell(file);
	rewind(file);

	buffer = (s8*)malloc((file_length + 1) * sizeof(s8));
	if (buffer == NULL) {
		fprintf(stderr, "Error allocating data for file [%s]: [%s]\n", path, strerror(errno));
		return NULL;
	}

	if (fread(buffer, 1, file_length, file) != file_length) {
		fprintf(stderr, "Error reading file [%s]: [%s]\n", path, strerror(errno));
		return NULL;
	}

	fclose(file);

	buffer[file_length] = '\0';

	if (_file_length) {
		*_file_length = file_length;
	}

	return buffer;
}

void util_free_file(s8* file) {
	free(file);
}

mat4 util_get_model_matrix_no_scale(const Quaternion* rotation, vec3 translation) {
	r64 s, c;

	mat4 rotation_matrix = quaternion_get_matrix(rotation);

	mat4 translation_matrix = (mat4) {
		1.0, 0.0, 0.0, translation.x,
		0.0, 1.0, 0.0, translation.y,
		0.0, 0.0, 1.0, translation.z,
		0.0, 0.0, 0.0, 1.0
	};

	mat4 model_matrix = gm_mat4_multiply(&translation_matrix, &rotation_matrix);
	return model_matrix;
}


void util_matrix_to_r32_array(const mat4* m, r32 out[16]) {
	out[0] =  (r32)m->data[0][0];
	out[1] =  (r32)m->data[0][1];
	out[2] =  (r32)m->data[0][2];
	out[3] =  (r32)m->data[0][3];
	out[4] =  (r32)m->data[1][0];
	out[5] =  (r32)m->data[1][1];
	out[6] =  (r32)m->data[1][2];
	out[7] =  (r32)m->data[1][3];
	out[8] =  (r32)m->data[2][0];
	out[9] =  (r32)m->data[2][1];
	out[10] = (r32)m->data[2][2];
	out[11] = (r32)m->data[2][3];
	out[12] = (r32)m->data[3][0];
	out[13] = (r32)m->data[3][1];
	out[14] = (r32)m->data[3][2];
	out[15] = (r32)m->data[3][3];
}

#define COLOR_PALETTE_MAX_NUM 17
static vec4 color_palette[COLOR_PALETTE_MAX_NUM];
static boolean is_color_palette_initialized;

vec4 util_pallete(u32 n) {
	if (!is_color_palette_initialized) {
		color_palette[0] =  (vec4){1.0, 0.0, 0.0, 1.0};
		color_palette[1] =  (vec4){0.0, 1.0, 0.0, 1.0};
		color_palette[2] =  (vec4){0.0, 0.0, 1.0, 1.0};
		color_palette[3] =  (vec4){1.0, 1.0, 0.0, 1.0};
		color_palette[4] =  (vec4){0.0, 1.0, 1.0, 1.0};
		color_palette[5] =  (vec4){1.0, 0.0, 1.0, 1.0};
		color_palette[6] =  (vec4){1.0, 1.0, 1.0, 1.0};
		color_palette[7] =  (vec4){0.0, 0.0, 0.0, 1.0};
		color_palette[8] =  (vec4){1.0, 0.5, 0.0, 1.0};
		color_palette[9] =  (vec4){0.0, 1.0, 0.5, 1.0};
		color_palette[10] = (vec4){0.5, 0.0, 1.0, 1.0};
		color_palette[11] = (vec4){0.5, 1.0, 0.0, 1.0};
		color_palette[12] = (vec4){0.0, 0.5, 1.0, 1.0};
		color_palette[13] = (vec4){1.0, 0.0, 0.5, 1.0};
		color_palette[14] = (vec4){1.0, 1.0, 0.5, 1.0};
		color_palette[15] = (vec4){1.0, 0.5, 1.0, 1.0};
		color_palette[16] = (vec4){0.5, 1.0, 1.0, 1.0};
		is_color_palette_initialized = true;
	}

	u32 idx = n % COLOR_PALETTE_MAX_NUM;
	return color_palette[idx];
}

// Hash Utils

int util_eid_compare(const void *key1, const void *key2) {
	eid id1 = *(eid*)key1;
	eid id2 = *(eid*)key2;
	return id1 == id2;
}

unsigned int util_eid_hash(const void *key) {
	eid id = *(eid*)key;
	return (unsigned int)id;
}

int util_vec3_compare(const void *key1, const void *key2) {
	vec3 v1 = *(vec3*)key1;
	vec3 v2 = *(vec3*)key2;
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

unsigned int util_vec3_hash(const void *key) {
	vec3 v = *(vec3*)key;
	unsigned long long x, y, z;
	assert(sizeof(r64) == sizeof(unsigned long long));
	memcpy(&x, &v.x, sizeof(r64));
	memcpy(&y, &v.y, sizeof(r64));
	memcpy(&z, &v.z, sizeof(r64));
	return (unsigned int)((x + y + z) % UINT_MAX);
}