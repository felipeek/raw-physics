#ifndef RAW_PHYSICS_ENTITY_H
#define RAW_PHYSICS_ENTITY_H

#include <gm.h>
#include "render/mesh.h"
#include "physics/collider.h"
#include "quaternion.h"

typedef struct {
	vec3 position;
	vec3 force;
} Physics_Force;

typedef struct {
	Mesh mesh;
	vec3 world_position;
	Quaternion world_rotation;
	vec3 world_scale;
	vec4 color;

	// Physics Related
	Collider collider;
	Physics_Force* forces;
	r64 inverse_mass;
	mat3 inertia_tensor;
	mat3 inverse_inertia_tensor;
	vec3 angular_velocity;
	vec3 linear_velocity;
	boolean fixed;
	boolean active;
	r64 deactivation_time;
	r64 static_friction_coefficient;
	r64 dynamic_friction_coefficient;
	r64 restitution_coefficient;

	// PBD Auxilar
	vec3 previous_world_position;
	Quaternion previous_world_rotation;
	vec3 previous_linear_velocity;
	vec3 previous_angular_velocity;

	vec3 updated_linear_velocity;
	vec3 updated_angular_velocity;
} Entity;

mat4 entity_get_model_matrix(const Entity* entity);
mat4 entity_get_model_matrix_no_scale(const Entity* entity);
void entity_create(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r64 mass, Collider collider);
void entity_create_fixed(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, Collider collider);
void entity_destroy(Entity* entity);
void entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map);
void entity_set_position(Entity* entity, vec3 world_position);
void entity_set_rotation(Entity* entity, Quaternion world_rotation);
void entity_set_scale(Entity* entity, vec3 world_scale);

#endif