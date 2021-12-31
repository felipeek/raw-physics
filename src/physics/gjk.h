#ifndef RAW_PHYSICS_GJK_H
#define RAW_PHYSICS_GJK_H

#include <common.h>
#include <gm.h>

typedef struct {
	vec3 a, b, c, d;
	u32 num;
} GJK_Simplex;

u32 gjk_get_support_point_index(vec3* shape, vec3 direction);
vec3 gjk_get_support_point(vec3* shape, vec3 direction);
vec3 gjk_get_support_point_of_minkowski_difference(vec3* shape1, vec3* shape2, vec3 direction);
boolean gjk_collides(vec3* shape1, vec3* shape2, GJK_Simplex* simplex);

#endif