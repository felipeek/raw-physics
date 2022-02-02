#ifndef RAW_PHYSICS_RENDER_OBJ_H
#define RAW_PHYSICS_RENDER_OBJ_H

#include <common.h>
#include "graphics.h"

int obj_parse(const char* obj_path, Vertex** vertices, u32** indices);

#endif