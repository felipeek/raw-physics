#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

r64 util_random_float(r64 min, r64 max)
{
    r64 scale = rand() / (r64)RAND_MAX;
    return min + scale * (max - min);
}

s8* util_read_file(const s8* path, s32* _file_length)
{
	FILE* file;
	s8* buffer;
	s32 file_length;

	file = fopen(path, "rb");

	if (file == NULL) {
		fprintf(stderr, "Error opening file [%s]: [%s]\n", path, strerror(errno));
		return NULL;
	}
	fseek(file, 0, SEEK_END);
	file_length = ftell(file);
	rewind(file);

	buffer = (s8*)malloc((file_length + 1) * sizeof(s8));
	fread(buffer, file_length, 1, file);
	fclose(file);

	buffer[file_length] = '\0';

	if (_file_length)
		*_file_length = file_length;

	return buffer;
}

void util_free_file(s8* file)
{
	free(file);
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