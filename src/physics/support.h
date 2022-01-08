#ifndef RAW_PHYSICS_PHYSICS_SUPPORT_H
#define RAW_PHYSICS_PHYSICS_SUPPORT_H
#include "collider.h"

u32 support_point_get_index(Collider_Convex_Hull* convex_hull, vec3 direction);
vec3 support_point(Collider* collider, vec3 direction);
vec3 support_point_of_minkowski_difference(Collider* collider1, Collider* collider2, vec3 direction);

#endif