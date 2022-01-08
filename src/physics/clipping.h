#ifndef RAW_PHYSICS_PHYSICS_CLIPPING_H
#define RAW_PHYSICS_PHYSICS_CLIPPING_H
#include <gm.h>
#include "collider.h"

Collider_Contact* clipping_get_contact_manifold(Collider* collider1, Collider* collider2, vec3 normal, r64 penetration);

#endif