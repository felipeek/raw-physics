#ifndef RAW_PHYSICS_EPA_H
#define RAW_PHYSICS_EPA_H

#include <common.h>
#include <gm.h>
#include "gjk.h"

boolean epa(vec3* shape1, vec3* shape2, GJK_Simplex* simplex, vec3* normal, r64* penetration);

#endif