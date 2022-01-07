#ifndef RAW_PHYSICS_RENDER_GRAPHICS_H
#define RAW_PHYSICS_RENDER_GRAPHICS_H
#include <gm.h>
#include "camera.h"
#include "mesh.h"
#include "../physics/collider.h"
#include "../entity.h"

typedef u32 Shader;

typedef struct {
	vec3 position;
	vec4 ambient_color;
	vec4 diffuse_color;
	vec4 specular_color;
} Light;

Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity);
void graphics_entity_render_phong_shader(const Perspective_Camera* camera, const Entity* entity, const Light* lights);
void graphics_light_create(Light* light, vec3 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color);

// Render primitives
void graphics_renderer_primitives_flush(const Perspective_Camera* camera);
void graphics_renderer_debug_points(vec3* points, int point_count, vec4 color);
void graphics_renderer_debug_vector(vec3 p1, vec3 p2, vec4 color);

#endif