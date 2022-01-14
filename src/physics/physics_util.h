#ifndef RAW_PHYSICS_PHYSICS_PHYSICS_UTIL_H
#define RAW_PHYSICS_PHYSICS_PHYSICS_UTIL_H
#include "../entity.h"

vec3 calculate_external_force(Entity* e);
vec3 calculate_external_torque(Entity* e);
mat3 get_dynamic_inertia_tensor(Entity* e);
mat3 get_dynamic_inverse_inertia_tensor(Entity* e);

#endif