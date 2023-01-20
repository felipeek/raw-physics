#include "entity.h"
#include <light_array.h>
#include <GL/glew.h>
#include "hash_map.h"
#include "util.h"

Entity** entities;
Hash_Map entities_map;
eid eid_counter;

void entity_module_init() {
	entities = array_new(Entity*);
	assert(hash_map_create(&entities_map, 1024, sizeof(eid), sizeof(Entity*), util_eid_compare, util_eid_hash) == 0);
}

void entity_module_destroy() {
	for (u32 i = 0; i < array_length(entities); ++i) {
		entity_destroy(entities[i]);
	}
	array_free(entities);
	hash_map_destroy(&entities_map);
}

static eid entity_create_ex(Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r64 mass, Collider* colliders,
		r64 static_friction_coefficient, r64 dynamic_friction_coefficient, r64 restitution_coefficient, bool is_fixed) {
	Entity* entity = (Entity*)malloc(sizeof(Entity));
	entity->id = eid_counter++;
	entity->mesh = mesh;
	entity->color = color;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->previous_angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->previous_linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->bounding_sphere_radius = colliders_get_bounding_sphere_radius(colliders);
	if (is_fixed) {
		entity->inverse_mass = 0.0;
		entity->inertia_tensor = (mat3){0}; // this is not correct, but it shouldn't make a difference
		entity->inverse_inertia_tensor = (mat3){0};
	} else {
		entity->inverse_mass = 1.0 / mass;
		entity->inertia_tensor = colliders_get_default_inertia_tensor(colliders, mass);
		assert(gm_mat3_inverse(&entity->inertia_tensor, &entity->inverse_inertia_tensor));
	}
	entity->forces = array_new(Physics_Force);
	entity->fixed = is_fixed;
	entity->active = true;
	entity->deactivation_time = 0.0;
	entity->colliders = colliders;
	entity->static_friction_coefficient = static_friction_coefficient;
	entity->dynamic_friction_coefficient = dynamic_friction_coefficient;
	entity->restitution_coefficient = restitution_coefficient;
	assert(entity->static_friction_coefficient >= 0.0 && entity->static_friction_coefficient <= 1.0);
	assert(entity->dynamic_friction_coefficient >= 0.0 && entity->dynamic_friction_coefficient <= 1.0);
	assert(entity->restitution_coefficient >= 0.0 && entity->restitution_coefficient <= 1.0);
	if (entity->dynamic_friction_coefficient > entity->static_friction_coefficient) {
		printf("Warning: dynamic friction coefficient is greater than static friction coefficient\n");
	}

	array_push(entities, entity);
	assert(!hash_map_put(&entities_map, &entity->id, &entity));
	return entity->id;
}

eid entity_create(Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r64 mass, Collider* colliders,
		r64 static_friction_coefficient, r64 dynamic_friction_coefficient, r64 restitution_coefficient) {
	return entity_create_ex(mesh, world_position, world_rotation, world_scale, color, mass, colliders,
		static_friction_coefficient, dynamic_friction_coefficient, restitution_coefficient, false);
}

eid entity_create_fixed(Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, Collider* colliders,
		r64 static_friction_coefficient, r64 dynamic_friction_coefficient, r64 restitution_coefficient) {
	return entity_create_ex(mesh, world_position, world_rotation, world_scale, color, 0.0, colliders,
		static_friction_coefficient, dynamic_friction_coefficient, restitution_coefficient, true);
}

Entity* entity_get_by_id(eid id) {
	Entity* e;
	if (hash_map_get(&entities_map, &id, &e)) {
		return NULL;
	}

	return e;
}

Entity** entity_get_all() {
	return (Entity**)array_copy(entities);
}

void entity_destroy(Entity* entity) {
	array_free(entity->forces);

	// @TODO: avoid the need of this loop
	for (u32 i = 0; i < array_length(entities); ++i) {
		if (entities[i]->id == entity->id) {
			array_remove(entities, i);
			break;
		}
	}

	assert(hash_map_delete(&entities_map, &entity->id) == 0);
	free(entity);
}

static mat4 entity_get_model_matrix_no_translation(const Entity* entity) {
	r64 s, c;

	mat4 scale_matrix = (mat4) {
		entity->world_scale.x, 0.0, 0.0, 0.0,
		0.0, entity->world_scale.y, 0.0, 0.0,
		0.0, 0.0, entity->world_scale.z, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	mat4 rotation_matrix = quaternion_get_matrix(&entity->world_rotation);

	mat4 model_matrix = gm_mat4_multiply(&rotation_matrix, &scale_matrix);
	return model_matrix;
}

mat4 entity_get_model_matrix(const Entity* entity) {
	r64 s, c;

	mat4 scale_matrix = (mat4) {
		entity->world_scale.x, 0.0, 0.0, 0.0,
		0.0, entity->world_scale.y, 0.0, 0.0,
		0.0, 0.0, entity->world_scale.z, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	mat4 rotation_matrix = quaternion_get_matrix(&entity->world_rotation);

	mat4 translation_matrix = (mat4) {
		1.0, 0.0, 0.0, entity->world_position.x,
		0.0, 1.0, 0.0, entity->world_position.y,
		0.0, 0.0, 1.0, entity->world_position.z,
		0.0, 0.0, 0.0, 1.0
	};

	mat4 model_matrix = gm_mat4_multiply(&rotation_matrix, &scale_matrix);
	model_matrix = gm_mat4_multiply(&translation_matrix, &model_matrix);
	return model_matrix;
}

void entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map) {
	glDeleteBuffers(1, &entity->mesh.VBO);
	glDeleteBuffers(1, &entity->mesh.EBO);
	glDeleteVertexArrays(1, &entity->mesh.VAO);

	entity->mesh = mesh;
}

void entity_set_position(Entity* entity, vec3 world_position) {
	entity->world_position = world_position;
}

void entity_set_rotation(Entity* entity, Quaternion world_rotation) {
	entity->world_rotation = world_rotation;
}

void entity_set_scale(Entity* entity, vec3 world_scale) {
	entity->world_scale = world_scale;
}

void entity_activate(Entity* entity) {
	entity->active = true;
	entity->deactivation_time = 0.0;
}

// Add a force to an entity
// If local_coords is false, then the position and force are represented in world coordinates, assuming that the center of the
// world is the center of the entity. That is, the coordinate (0, 0, 0) corresponds to the center of the entity in world coords.
// If local_coords is true, then the position and force are represented in local coords.
void entity_add_force(Entity* entity, vec3 position, vec3 force, boolean local_coords) {
	if (local_coords) {
		// If the force and position are in local cords, we first convert them to world coords
		// (actually, we convert them to ~"world coords cenetered at entity"~)
		force = quaternion_apply_to_vec3(&entity->world_rotation, force);

		// note that we don't need translation since we want to be centered at entity anyway
		mat4 model_matrix = entity_get_model_matrix_no_translation(entity);
		position = gm_mat4_multiply_vec3(&model_matrix, position, true);
	}

	Physics_Force pf;
	pf.force = force;
	pf.position = position;
	array_push(entity->forces, pf);
}

void entity_clear_forces(Entity* entity) {
	array_clear(entity->forces);
}