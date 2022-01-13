#include "physics_util.h"
#include <light_array.h>

// Calculate the sum of all external forces acting on an entity
vec3 calculate_external_force(Entity* e) {
	const vec3 center_of_mass = (vec3){0.0, 0.0, 0.0};
	vec3 total_force = (vec3){0.0, 0.0, 0.0};
	for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
		total_force = gm_vec3_add(total_force, e->forces[i].force);
	}
	return total_force;
}

// Calculate the sum of all external torques acting on an entity
vec3 calculate_external_torque(Entity* e) {
	const vec3 center_of_mass = (vec3){0.0, 0.0, 0.0};
	vec3 total_torque = (vec3){0.0, 0.0, 0.0};
	for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
		vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
		total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, e->forces[i].force));
	}
	return total_torque;
}

// Calculate the dynamic inertia tensor of an entity, i.e., the inertia tensor transformed considering entity's rotation
mat3 get_dynamic_inertia_tensor(Entity* e) {
#if 1
	// Can only be used if the local->world matrix is orthogonal
	mat3 rotation_matrix = quaternion_get_matrix3(&e->world_rotation);
	mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix);
	mat3 aux = gm_mat3_multiply(&rotation_matrix, &e->inertia_tensor);
	return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
#else
	// Can always be used
	mat3 local_to_world = quaternion_get_matrix3(&e->world_rotation);
	mat3 inverse_local_to_world;
	assert(gm_mat3_inverse(&local_to_world, &inverse_local_to_world));
	mat3 transposed_inverse_local_to_world = gm_mat3_transpose(&inverse_local_to_world);
	mat3 aux = gm_mat3_multiply(&transposed_inverse_local_to_world, &e->inertia_tensor);
	return gm_mat3_multiply(&aux, &inverse_local_to_world);
#endif
}

// Calculate the dynamic inverse inertia tensor of an entity, i.e., the inverse inertia tensor transformed considering entity's rotation
mat3 get_dynamic_inverse_inertia_tensor(Entity* e) {
#if 1
	// Can only be used if the local->world matrix is orthogonal
	mat3 rotation_matrix = quaternion_get_matrix3(&e->world_rotation);
	mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix);
	mat3 aux = gm_mat3_multiply(&rotation_matrix, &e->inverse_inertia_tensor);
	return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
#else
	// Can always be used
	mat3 local_to_world = quaternion_get_matrix3(&e->world_rotation);
	mat3 inverse_local_to_world;
	assert(gm_mat3_inverse(&local_to_world, &inverse_local_to_world));
	mat3 transposed_inverse_local_to_world = gm_mat3_transpose(&inverse_local_to_world);
	mat3 aux = gm_mat3_multiply(&transposed_inverse_local_to_world, &e->inverse_inertia_tensor);
	return gm_mat3_multiply(&aux, &inverse_local_to_world);
#endif
}
