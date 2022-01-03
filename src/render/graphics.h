#ifndef RAW_PHYSICS_GRAPHICS_H
#define RAW_PHYSICS_GRAPHICS_H
#include <gm.h>
#include "camera.h"
#include "../physics/collider.h"

typedef u32 Shader;

#pragma pack(push, 1)
typedef struct {
	fvec3 position;
	fvec3 normal;
	fvec2 texture_coordinates;
} Vertex;
#pragma pack(pop)

typedef struct {
	boolean use_diffuse_map;
	u32 diffuse_map;
	fvec4 diffuse_color;
} Diffuse_Info;

typedef struct {
	u32 VAO, VBO, EBO;
	u32 num_indices;
	Collider collider;
} Mesh;

typedef struct {
	vec3 position;
	vec3 force;
} Physics_Force;

typedef struct {
	Mesh mesh;
	vec3 world_position;
	Quaternion world_rotation;
	vec3 world_scale;
	Diffuse_Info diffuse_info;

	// Physics Related
	Physics_Force* forces;
	r64 inverse_mass;
	mat3 inertia_tensor;
	mat3 inverse_inertia_tensor;
	vec3 angular_velocity;
	vec3 linear_velocity;
	boolean fixed;

	// PBD Auxilar
	vec3 previous_world_position;
	Quaternion previous_world_rotation;
	vec3 previous_linear_velocity;
	vec3 previous_angular_velocity;
} Entity;

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
	r64* data;
	s32 width, height, channels;
} Float_Image_Data;

Image_Data graphics_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data);
void graphics_image_free(Image_Data* image_data);
void graphics_float_image_Free(Float_Image_Data* image_data);
void graphics_image_save(const s8* image_path, const Image_Data* image_data);
void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data);
Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
Mesh graphics_quad_create();
Mesh graphics_mesh_create(Vertex* vertices, u32* indices, Collider_Type collider_type);
Mesh graphics_mesh_create_from_obj(const s8* obj_path, Collider_Type collider_type);
void graphics_mesh_render(Shader shader, Mesh mesh);
void graphics_entity_create_with_color(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, fvec4 color, r64 mass);
void graphics_entity_create_with_color_fixed(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, fvec4 color);
void graphics_entity_create_with_texture(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, u32 texture);
void graphics_entity_destroy(Entity* entity);
// If entity already has a diffuse map, the older diffuse map will be deleted if delete_diffuse_map is true.
// If entity has a color instead of a diffuse map, the mesh will lose the color and be set to use the diffuse map.
void graphics_entity_change_diffuse_map(Entity* entity, u32 diffuse_map, boolean delete_diffuse_map);
// If the entity already has a color, the older color will be deleted.
// If entity has a diffuse map instead of a color, the diffuse map will be deleted if delete_diffuse_map is true
// The entity will be set to use the color.
void graphics_entity_change_color(Entity* entity, fvec4 color, boolean delete_diffuse_map);
void graphics_entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map);
void graphics_entity_set_position(Entity* entity, vec3 world_position);
void graphics_entity_set_rotation(Entity* entity, Quaternion world_rotation);
void graphics_entity_set_scale(Entity* entity, vec3 world_scale);
void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity);
void graphics_entity_render_phong_shader(const Perspective_Camera* camera, const Entity* entity, const Light* lights);
mat4 graphics_entity_get_model_matrix(const Entity* entity);
void graphics_light_create(Light* light, vec3 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color);
u32 graphics_texture_create(const s8* texture_path);
u32 graphics_texture_create_from_data(const Image_Data* image_data);
u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data);
void graphics_texture_delete(u32 texture_id);
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r64* memory);
Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_Data, u8* memory);

// Render primitives
void graphics_renderer_primitives_flush(const Perspective_Camera* camera);
void graphics_renderer_debug_points(vec3* points, int point_count, vec4 color);
void graphics_renderer_debug_vector(vec3 p1, vec3 p2, vec4 color);

#endif