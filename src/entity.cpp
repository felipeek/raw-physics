#include "entity.h"
#include <light_array.h>
#include <GL/glew.h>

mat4 entity_get_model_matrix(const Entity* entity)
{
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

void entity_create(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r64 mass, Collider collider)
{
	entity->mesh = mesh;
	entity->color = color;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->previous_angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->previous_linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->inverse_mass = 1.0 / mass;
	entity->inertia_tensor = collider_get_default_inertia_tensor(&collider, mass);
	assert(gm_mat3_inverse(&entity->inertia_tensor, &entity->inverse_inertia_tensor));
	entity->forces = array_new(Physics_Force);
	entity->fixed = false;
	entity->active = true;
	entity->deactivation_time = 0.0;
	entity->collider = collider;
	entity->static_friction_coefficient = 0.2;
	entity->dynamic_friction_coefficient = 1.0;
	entity->restitution_coefficient = 0.0;
	assert(entity->static_friction_coefficient >= 0.0 && entity->static_friction_coefficient <= 1.0);
	assert(entity->dynamic_friction_coefficient >= 0.0 && entity->dynamic_friction_coefficient <= 1.0);
	assert(entity->restitution_coefficient >= 0.0 && entity->restitution_coefficient <= 1.0);
}

void entity_create_fixed(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, Collider collider)
{
	entity->mesh = mesh;
	entity->color = color;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->previous_angular_velocity = (vec3){0.0, 0.0, 0.0};
	entity->previous_linear_velocity = (vec3){0.0, 0.0, 0.0};
	entity->inverse_mass = 0.0;
	entity->inertia_tensor = (mat3){0}; // this is not correct, but it shouldn't make a difference
	entity->inverse_inertia_tensor = (mat3){0};
	entity->forces = array_new(Physics_Force);
	entity->fixed = true;
	entity->active = true; // meaningless for fixed entities
	entity->deactivation_time = 0.0;
	entity->collider = collider;
	entity->static_friction_coefficient = 0.5;
	entity->dynamic_friction_coefficient = 0.5;
	entity->restitution_coefficient = 1.0;
	assert(entity->static_friction_coefficient >= 0.0 && entity->static_friction_coefficient <= 1.0);
	assert(entity->dynamic_friction_coefficient >= 0.0 && entity->dynamic_friction_coefficient <= 1.0);
	assert(entity->restitution_coefficient >= 0.0 && entity->restitution_coefficient <= 1.0);
}

void entity_destroy(Entity* entity)
{
	array_free(entity->forces);
}

void entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map)
{
	glDeleteBuffers(1, &entity->mesh.VBO);
	glDeleteBuffers(1, &entity->mesh.EBO);
	glDeleteVertexArrays(1, &entity->mesh.VAO);

	entity->mesh = mesh;
}

void entity_set_position(Entity* entity, vec3 world_position)
{
	entity->world_position = world_position;
}

void entity_set_rotation(Entity* entity, Quaternion world_rotation)
{
	entity->world_rotation = world_rotation;
}

void entity_set_scale(Entity* entity, vec3 world_scale)
{
	entity->world_scale = world_scale;
}

void entity_activate(Entity* entity)
{
	entity->active = true;
	entity->deactivation_time = 0.0;
}