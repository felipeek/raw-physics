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

mat3 get_default_inertia_tensor(Vertex* vertices, u32* indices, r64 mass) {
	// For now, the center of mass is always assumed to be at 0,0,0
	r64 mass_per_vertex = mass / array_length(vertices);

	mat3 result = {0};
	for (u32 i = 0; i < array_length(vertices); ++i) {
		fvec3 v = vertices[i].position;
		result.data[0][0] += (r64)(mass_per_vertex * (v.y * v.y + v.z * v.z));
		result.data[0][1] += (r64)(mass_per_vertex * v.x * v.y);
		result.data[0][2] += (r64)(mass_per_vertex * v.x * v.z);
		result.data[1][0] += (r64)(mass_per_vertex * v.x * v.y);
		result.data[1][1] += (r64)(mass_per_vertex * (v.x * v.x + v.z * v.z));
		result.data[1][2] += (r64)(mass_per_vertex * v.y * v.z);
		result.data[2][0] += (r64)(mass_per_vertex * v.x * v.z);
		result.data[2][1] += (r64)(mass_per_vertex * v.y * v.z);
		result.data[2][2] += (r64)(mass_per_vertex * (v.x * v.x + v.y * v.y));
	}

	return result;
}

eid entity_create(Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r64 mass, Vertex* vertices, u32* indices) {
	Entity* entity = (Entity*)malloc(sizeof(Entity));
	entity->id = eid_counter++;
	entity->mesh = mesh;
	entity->color = color;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->inverse_mass = 1.0 / mass;
	entity->inertia_tensor = get_default_inertia_tensor(vertices, indices, mass);
	assert(gm_mat3_inverse(&entity->inertia_tensor, &entity->inverse_inertia_tensor));
	entity->forces = array_new(Physics_Force);

	array_push(entities, entity);
	assert(!hash_map_put(&entities_map, &entity->id, &entity));
	return entity->id;
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

void entity_add_force(Entity* entity, vec3 position, vec3 force, boolean local_coords) {
    if (local_coords) {
        // If the force is in local cords, we first convert it to world coords
        position = quaternion_apply_to_vec3(&entity->world_rotation, position);
        force = quaternion_apply_to_vec3(&entity->world_rotation, force);
    }

    Physics_Force pf;
    pf.force = force;
    pf.position = position;
    array_push(entity->forces, pf);
}

void entity_clear_forces(Entity* entity) {
    array_clear(entity->forces);
}