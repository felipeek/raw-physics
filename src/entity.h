#ifndef RAW_PHYSICS_ENTITY_H
#define RAW_PHYSICS_ENTITY_H

#include <gm.h>
#include "render/mesh.h"
#include "physics/collider.h"
#include "quaternion.h"

typedef u64 eid;

typedef struct {
	vec3 position;
	vec3 force;
	boolean local_coords;
} Physics_Force;

typedef struct {
	eid id;

	Mesh mesh;
	vec3 world_position;
	Quaternion world_rotation;
	vec3 world_scale;
	vec4 color;

	// Physics Related
	Collider* colliders;
	r64 bounding_sphere_radius;
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
} Entity;

void entity_module_init();
void entity_module_destroy();

eid entity_create(Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r64 mass, Collider* colliders,
		r64 static_friction_coefficient, r64 dynamic_friction_coefficient, r64 restitution_coefficient);
eid entity_create_fixed(Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, Collider* colliders,
		r64 static_friction_coefficient, r64 dynamic_friction_coefficient, r64 restitution_coefficient);
Entity* entity_get_by_id(eid id);
Entity** entity_get_all();
void entity_destroy(Entity* entity);
mat4 entity_get_model_matrix(const Entity* entity);
void entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map);
void entity_set_position(Entity* entity, vec3 world_position);
void entity_set_rotation(Entity* entity, Quaternion world_rotation);
void entity_set_scale(Entity* entity, vec3 world_scale);
void entity_activate(Entity* entity);
void entity_add_force(Entity* entity, vec3 position, vec3 force, boolean local_coords);
void entity_clear_forces(Entity* entity);

#endif