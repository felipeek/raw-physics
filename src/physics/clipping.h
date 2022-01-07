#ifndef RAW_PHYSICS_PHYSICS_CLIPPING_H
#define RAW_PHYSICS_PHYSICS_CLIPPING_H
#include <gm.h>
#include "collider.h"

typedef struct {
	vec3 collision_point1;
	vec3 collision_point2;
} Clipping_Contact;

Clipping_Contact* clipping_get_contact_manifold(Collider_Convex_Hull* convex_hull1, Collider_Convex_Hull* convex_hull2, vec3 normal);

#endif