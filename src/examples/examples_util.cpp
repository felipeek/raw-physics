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

Collider* examples_util_create_sphere_convex_hull_array(r32 radius) {
	Collider collider = collider_sphere_create(radius);
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

void examples_util_throw_object(Perspective_Camera* camera, r64 velocity_norm) {
	vec3 camera_z = camera_get_z_axis(camera);
	vec3 camera_pos = camera->position;
	r64 distance = 5.0;
	vec3 diff = gm_vec3_scalar_product(-distance, camera_z);
	vec3 entity_position = gm_vec3_add(camera_pos, diff);

	const char* mesh_name;
	int r = rand();
	int is_sphere = 0;
	if (r % 4 == 0) {
		mesh_name = "./res/cube.obj";
	} else if (r % 4 == 1) {
		mesh_name = "./res/ico.obj";
	} else if (r % 4 == 2) {
		mesh_name = "./res/ico.obj";
	} else {
		is_sphere = 1;
		mesh_name = "./res/sphere.obj";
	}
	Vertex* vertices;
	u32* indices;
	obj_parse(mesh_name, &vertices, &indices);
	Mesh m = graphics_mesh_create(vertices, indices);

	vec3 scale;
	Collider* colliders;
	if (is_sphere) {
		r64 radius = 1.0;
		scale = (vec3){radius, radius, radius};
		colliders = examples_util_create_sphere_convex_hull_array(radius);
	} else {
		scale = (vec3){1.0, 1.0, 1.0};
		colliders = examples_util_create_single_convex_hull_collider_array(vertices, indices, scale);
	}

	eid id = entity_create(m, entity_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 0.0),
		scale, (vec4){rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, 1.0}, 1.0, colliders, 0.8, 0.8, 0.0);
	array_free(vertices);
	array_free(indices);

	Entity* e = entity_get_by_id(id);
	e->linear_velocity = gm_vec3_scalar_product(velocity_norm, gm_vec3_scalar_product(-1.0, camera_z));
}

Light* examples_util_create_lights() {
	Light light;
	Light* lights = array_new(Light);

	vec3 light_position = (vec3) {0.0, 15.0, 25.0};
	vec4 ambient_color = (vec4) {0.2, 0.2, 0.2, 1.0};
	vec4 diffuse_color = (vec4) {1.0, 1.0, 1.0, 1.0};
	vec4 specular_color = (vec4) {0.0, 0.0, 0.0, 1.0};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	return lights;
}