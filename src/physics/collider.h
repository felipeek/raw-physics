#ifndef RAW_PHYSICS_COLLIDER_H
#define RAW_PHYSICS_COLLIDER_H
#include <gm.h>

typedef struct {
	u32* elements;
	vec3 normal;
} Collider_Convex_Hull_Face;

typedef struct {
	vec3* vertices;
	vec3* transformed_vertices;
	Collider_Convex_Hull_Face* faces;
	Collider_Convex_Hull_Face* transformed_faces;

	u32** vertex_to_faces;
	u32** vertex_to_neighbors;
	u32** face_to_neighbors;
} Collider_Convex_Hull;

typedef enum {
	COLLIDER_TYPE_CONVEX_HULL
} Collider_Type;

typedef struct {
	Collider_Type type;
	union {
		Collider_Convex_Hull convex_hull;
	};
} Collider;

Collider collider_create(const vec3* vertices, const u32* indices, Collider_Type type);
void collider_update(Collider* collider, mat4 model_matrix_no_scale);

#endif