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