#ifndef RAW_PHYSICS_MESH_H
#define RAW_PHYSICS_MESH_H
#include <gm.h>

#pragma pack(push, 1)
typedef struct {
	vec3 position;
	vec3 normal;
	vec2 texture_coordinates;
} Vertex;
#pragma pack(pop)

typedef struct {
	u32 VAO, VBO, EBO;
	u32 num_indices;
} Mesh;

Mesh graphics_quad_create();
Mesh graphics_mesh_create(Vertex* vertices, u32* indices);
Mesh graphics_mesh_create_from_obj(const s8* obj_path);

#endif