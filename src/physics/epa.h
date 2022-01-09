#ifndef RAW_PHYSICS_PHYSICS_EPA_H
#define RAW_PHYSICS_PHYSICS_EPA_H

#include <common.h>
#include <gm.h>
#include "gjk.h"

boolean epa(Collider* collider1, Collider* collider2, GJK_Simplex* simplex, vec3* normal, r64* penetration);

#endif