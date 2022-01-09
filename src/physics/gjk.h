#ifndef RAW_PHYSICS_PHYSICS_GJK_H
#define RAW_PHYSICS_PHYSICS_GJK_H

#include <common.h>
#include <gm.h>
#include "collider.h"

typedef struct {
	vec3 a, b, c, d;
	u32 num;
} GJK_Simplex;

boolean gjk_collides(Collider* collider1, Collider* collider2, GJK_Simplex* simplex);

#endif