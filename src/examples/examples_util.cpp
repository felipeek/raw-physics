#include "examples_util.h"
#include <light_array.h>
#include "../render/obj.h"

Collider* examples_util_create_single_convex_hull_collider_array(Vertex* vertices, u32* indices, vec3 scale) {
	vec3* vertices_positions = array_new(vec3);
	for (u32 i = 0; i < array_length(vertices); ++i) {
		vec3 position = (vec3) {
			(r64)vertices[i].position.x,
			(r64)vertices[i].position.y,
			(r64)vertices[i].position.z
		};
		position.x *= scale.x;
		position.y *= scale.y;
		position.z *= scale.z;
		array_push(vertices_positions, position);
	}
	Collider collider = collider_convex_hull_create(vertices_positions, indices);
	array_free(vertices_positions);

	Collider* colliders = array_new(Collider);
	array_push(colliders, collider);
	return colliders;
}

Collider examples_util_create_convex_hull_collider(Vertex* vertices, u32* indices, vec3 scale) {
	vec3* vertices_positions = array_new(vec3);
	for (u32 i = 0; i < array_length(vertices); ++i) {
		vec3 position = (vec3) {
			(r64)vertices[i].position.x,
			(r64)vertices[i].position.y,
			(r64)vertices[i].position.z
		};
		position.x *= scale.x;
		position.y *= scale.y;
		position.z *= scale.z;
		array_push(vertices_positions, position);
	}
	Collider collider = collider_convex_hull_create(vertices_positions, indices);
	array_free(vertices_positions);
	return collider;
}

void examples_util_throw_object(Perspective_Camera* camera) {
	vec3 camera_z = camera_get_z_axis(camera);
	vec3 camera_pos = camera->position;
	r64 distance = 5.0;
	vec3 diff = gm_vec3_scalar_product(-distance, camera_z);
	vec3 cube_position = gm_vec3_add(camera_pos, diff);

	const char* mesh_name;
	int r = rand();
	if (r % 3 == 0) {
		mesh_name = "./res/cube.obj";
	} else if (r % 3 == 1) {
		mesh_name = "./res/ico.obj";
	} else {
		mesh_name = "./res/ico.obj";
	}
	Vertex* vertices;
	u32* indices;
	obj_parse(mesh_name, &vertices, &indices);
	Mesh m = graphics_mesh_create(vertices, indices);
	vec3 scale = (vec3){1.0, 1.0, 1.0};
	Collider* colliders = examples_util_create_single_convex_hull_collider_array(vertices, indices, scale);
	eid id = entity_create(m, cube_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 0.0),
		scale, (vec4){rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, 1.0}, 1.0, colliders);
	array_free(vertices);
	array_free(indices);

	Entity* e = entity_get_by_id(id);
	e->linear_velocity = gm_vec3_scalar_product(15.0, gm_vec3_scalar_product(-1.0, camera_z));
}