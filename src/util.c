#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

extern r32 util_random_float(r32 min, r32 max)
{
    r32 scale = rand() / (r32)RAND_MAX;
    return min + scale * (max - min);
}

extern s8* util_read_file(const s8* path, s32* _file_length)
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

extern void util_free_file(s8* file)
{
	free(file);
}

#define COLOR_PALETTE_MAX_NUM 17
static vec4 color_palette[COLOR_PALETTE_MAX_NUM];
static boolean is_color_palette_initialized;

vec4 util_pallete(u32 n) {
	if (!is_color_palette_initialized) {
		color_palette[0] =  (vec4){1.0f, 0.0f, 0.0f, 1.0f};
		color_palette[1] =  (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		color_palette[2] =  (vec4){0.0f, 0.0f, 1.0f, 1.0f};
		color_palette[3] =  (vec4){1.0f, 1.0f, 0.0f, 1.0f};
		color_palette[4] =  (vec4){0.0f, 1.0f, 1.0f, 1.0f};
		color_palette[5] =  (vec4){1.0f, 0.0f, 1.0f, 1.0f};
		color_palette[6] =  (vec4){1.0f, 1.0f, 1.0f, 1.0f};
		color_palette[7] =  (vec4){0.0f, 0.0f, 0.0f, 1.0f};
		color_palette[8] =  (vec4){1.0f, 0.5f, 0.0f, 1.0f};
		color_palette[9] =  (vec4){0.0f, 1.0f, 0.5f, 1.0f};
		color_palette[10] = (vec4){0.5f, 0.0f, 1.0f, 1.0f};
		color_palette[11] = (vec4){0.5f, 1.0f, 0.0f, 1.0f};
		color_palette[12] = (vec4){0.0f, 0.5f, 1.0f, 1.0f};
		color_palette[13] = (vec4){1.0f, 0.0f, 0.5f, 1.0f};
		color_palette[14] = (vec4){1.0f, 1.0f, 0.5f, 1.0f};
		color_palette[15] = (vec4){1.0f, 0.5f, 1.0f, 1.0f};
		color_palette[16] = (vec4){0.5f, 1.0f, 1.0f, 1.0f};
		is_color_palette_initialized = true;
	}

	u32 idx = n % COLOR_PALETTE_MAX_NUM;
	return color_palette[idx];
}