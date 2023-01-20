#ifndef RAW_PHYSICS_EXAMPLES_EXAMPLES_UTIL_H
#define RAW_PHYSICS_EXAMPLES_EXAMPLES_UTIL_H

#include "../render/mesh.h"
#include "../render/camera.h"
#include "../render/graphics.h"
#include "../physics/collider.h"

Collider* examples_util_create_single_convex_hull_collider_array(Vertex* vertices, u32* indices, vec3 scale);
Collider examples_util_create_convex_hull_collider(Vertex* vertices, u32* indices, vec3 scale);
void examples_util_throw_object(Perspective_Camera* camera, r64 velocity_norm);
Light* examples_util_create_lights();

#endif