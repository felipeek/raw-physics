#ifndef RAW_PHYSICS_GRAPHICS_H
#define RAW_PHYSICS_GRAPHICS_H
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

typedef struct {
	u8* data;
	s32 width, height, channels;
} Image_Data;

typedef struct {
	r32* data;
	s32 width, height, channels;
} Float_Image_Data;

Image_Data graphics_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data);
void graphics_image_free(Image_Data* image_data);
void graphics_float_image_Free(Float_Image_Data* image_data);
void graphics_image_save(const s8* image_path, const Image_Data* image_data);
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r32* memory);
Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_Data, u8* memory);
void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data);

Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity);
void graphics_entity_render_phong_shader(const Perspective_Camera* camera, const Entity* entity, const Light* lights);
void graphics_light_create(Light* light, vec3 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color);

u32 graphics_texture_create(const s8* texture_path);
u32 graphics_texture_create_from_data(const Image_Data* image_data);
u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data);
void graphics_texture_delete(u32 texture_id);

// Render primitives
void graphics_renderer_primitives_flush(const Perspective_Camera* camera);
void graphics_renderer_debug_points(vec3* points, int point_count, vec4 color);
void graphics_renderer_debug_vector(vec3 p1, vec3 p2, vec4 color);

#endif