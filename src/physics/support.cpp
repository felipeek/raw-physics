#include "support.h"
#include <float.h>
#include <light_array.h>

u32 support_point_get_index(Collider_Convex_Hull* convex_hull, vec3 direction) {
	u32 selected_index;
	r64 max_dot = -DBL_MAX;
	for (u32 i = 0; i < array_length(convex_hull->transformed_vertices); ++i) {
		r64 dot = gm_vec3_dot(convex_hull->transformed_vertices[i], direction);
		if (dot > max_dot) {
			selected_index = i;
			max_dot = dot;
		}
	}

	return selected_index;
}

vec3 support_point(Collider* collider, vec3 direction) {
	switch (collider->type) {
		case COLLIDER_TYPE_CONVEX_HULL: {
			u32 selected_index = support_point_get_index(&collider->convex_hull, direction);
			return collider->convex_hull.transformed_vertices[selected_index];
		} break;
		case COLLIDER_TYPE_SPHERE: {
			return gm_vec3_add(collider->sphere.center, gm_vec3_scalar_product(collider->sphere.radius, gm_vec3_normalize(direction)));
		} break;
	}
	
	assert(0);
	return (vec3){0.0, 0.0, 0.0};
}

vec3 support_point_of_minkowski_difference(Collider* collider1, Collider* collider2, vec3 direction) {
	vec3 support1 = support_point(collider1, direction);
	vec3 support2 = support_point(collider2, gm_vec3_scalar_product(-1.0, direction));

	return gm_vec3_subtract(support1, support2);
}
