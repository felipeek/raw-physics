#include "graphics.h"
#include "../util.h"
#include "obj.h"
#include <GL/glew.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <light_array.h>
#include <math.h>

#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define BASIC_VERTEX_SHADER_PATH "./shaders/basic_shader.vs"
#define BASIC_FRAGMENT_SHADER_PATH "./shaders/basic_shader.fs"

Image_Data graphics_image_load(const s8* image_path)
{
	Image_Data image_data;

	stbi_set_flip_vertically_on_load(1);
	image_data.data = stbi_load(image_path, &image_data.width, &image_data.height, &image_data.channels, 4);

	image_data.channels = 4;	// @temporary

	return image_data;
}

Float_Image_Data graphics_float_image_load(const s8* image_path)
{
	Image_Data image_data;

	stbi_set_flip_vertically_on_load(1);
	image_data.data = stbi_load(image_path, &image_data.width, &image_data.height, &image_data.channels, 4);

	image_data.channels = 4;	// @temporary

	Float_Image_Data fid = graphics_image_data_to_float_image_data(&image_data, 0);

	graphics_image_free(&image_data);

	return fid;
}

Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data)
{
	Float_Image_Data fid;

	fid = *image_data;

	fid.data = malloc(sizeof(r32) * fid.width * fid.height * fid.channels);
	memcpy(fid.data, image_data->data, sizeof(r32) * fid.width * fid.height * fid.channels);
	
	return fid;
}

void graphics_image_free(Image_Data* image_data)
{
	stbi_image_free(image_data->data);
}

void graphics_float_image_free(Float_Image_Data* image_data)
{
	free(image_data->data);
}

void graphics_image_save(const s8* image_path, const Image_Data* image_data)
{
	stbi_flip_vertically_on_write(1);
	stbi_write_bmp(image_path, image_data->width, image_data->height, image_data->channels, image_data->data);
}

void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data)
{
	Image_Data id = graphics_float_image_data_to_image_data(image_data, 0);
	graphics_image_save(image_path, &id);
	graphics_image_free(&id);
}

Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path)
{
	s8* vertex_shader_code = util_read_file(vertex_shader_path, 0);
	s8* fragment_shader_code = util_read_file(fragment_shader_path, 0);

	GLint success;
	GLchar info_log_buffer[1024];
	GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(vertex_shader, 1, (const GLchar *const*)&vertex_shader_code, 0);
	glShaderSource(fragment_shader, 1, (const GLchar *const*)&fragment_shader_code, 0);

	glCompileShader(vertex_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertex_shader, 1024, 0, info_log_buffer);
		printf("Error compiling vertex shader: %s\n", info_log_buffer);
	}

	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragment_shader, 1024, 0, info_log_buffer);
		printf("Error compiling fragment shader: %s\n", info_log_buffer);
	}

	GLuint shader_program = glCreateProgram();

	glAttachShader(shader_program, vertex_shader);
	glAttachShader(shader_program, fragment_shader);
	glLinkProgram(shader_program);

	glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(shader_program, 1024, 0, info_log_buffer);
		printf("Error linking program: %s\n", info_log_buffer);
	}

	free(vertex_shader_code);
	free(fragment_shader_code);
	return shader_program;
}

typedef struct {
	Shader phong_shader;
	Shader basic_shader;
	boolean initialized;
} Predefined_Shaders;

Predefined_Shaders predefined_shaders;

static void init_predefined_shaders()
{
	if (!predefined_shaders.initialized)
	{
		predefined_shaders.phong_shader = graphics_shader_create(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
		predefined_shaders.basic_shader = graphics_shader_create(BASIC_VERTEX_SHADER_PATH, BASIC_FRAGMENT_SHADER_PATH);
		predefined_shaders.initialized = true;
	}
}

Mesh graphics_quad_create()
{
	r32 size = 1.0f;
	Vertex* vertices = array_new(vertices);
	u32* indices = array_new(u32);

	Vertex v;
	v.position = (vec3) { 0.0f, 0.0f, 0.0f};
	v.normal = (vec3) { 0.0f, 0.0f, 1.0f};
	v.texture_coordinates = (vec2) { 0.0f, 0.0f };
	array_push(vertices, v);

	v.position = (vec3) { size, 0.0f, 0.0f };
	v.normal = (vec3) { 0.0f, 0.0f, 1.0f };
	v.texture_coordinates = (vec2) { 1.0f, 0.0f };
	array_push(vertices, v);

	v.position = (vec3) { 0.0f, size, 0.0f };
	v.normal = (vec3) { 0.0f, 0.0f, 1.0f };
	v.texture_coordinates = (vec2) { 0.0f, 1.0f };
	array_push(vertices, v);

	v.position = (vec3) { size, size, 0.0f };
	v.normal = (vec3) { 0.0f, 0.0f, 1.0f };
	v.texture_coordinates = (vec2) { 1.0f, 1.0f };
	array_push(vertices, v);

	array_push(indices, 0);
	array_push(indices, 1);
	array_push(indices, 2);
	array_push(indices, 1);
	array_push(indices, 3);
	array_push(indices, 2);

	return graphics_mesh_create(vertices, indices, 0);
}

Mesh graphics_mesh_create(Vertex* vertices, u32* indices, Collider_Type collider_type)
{
	Mesh mesh;
	GLuint VBO, EBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, array_length(vertices) * sizeof(Vertex), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, array_length(vertices) * sizeof(Vertex), vertices);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(6 * sizeof(GLfloat)));
	glEnableVertexAttribArray(2);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, array_length(indices) * sizeof(u32), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, array_length(indices) * sizeof(u32), indices);

	glBindVertexArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	mesh.VAO = VAO;
	mesh.VBO = VBO;
	mesh.EBO = EBO;
	mesh.num_indices = array_length(indices);

	vec3* vertices_positions = array_new(vec3);
	for (u32 i = 0; i < array_length(vertices); ++i) {
		vec3 position = vertices[i].position;
		array_push(vertices_positions, position);
	}
	mesh.collider = collider_create(vertices_positions, indices, collider_type);
	printf("Vertices positions: %ld\n", array_length(vertices_positions));
	array_free(vertices_positions);

	printf("Hull positions: %ld\n", array_length(mesh.collider.convex_hull.vertices));
	for (u32 i = 0; i < array_length(mesh.collider.convex_hull.faces); ++i) {
		Collider_Convex_Hull_Face face = mesh.collider.convex_hull.faces[i];
		printf("Face num elements: %ld\n", array_length(face.elements));
		printf("Face normal: <%.3f, %.3f, %.3f>\n", face.normal.x, face.normal.y, face.normal.z);
		//for (u32 j = 0; j < array_length(face.elements); ++j) {
		//	vec3 v = mesh.collider.convex_hull.vertices[face.elements[j]];
		//	printf("\tV: <%.3f, %.3f, %.3f> (elem: %d)\n", v.x, v.y, v.z, face.elements[j]);
		//}
	}

	return mesh;
}

static s8* build_light_uniform_name(s8* buffer, s32 index, const s8* property)
{
	sprintf(buffer, "lights[%d].%s", index, property);
	return buffer;
}

static void light_update_uniforms(const Light* lights, Shader shader)
{
	s32 number_of_lights = array_length(lights);
	s8 buffer[64];
	glUseProgram(shader);

	for (s32 i = 0; i < number_of_lights; ++i)
	{
		Light light = lights[i];
		GLint light_position_location = glGetUniformLocation(shader, build_light_uniform_name(buffer, i, "position"));
		GLint ambient_color_location = glGetUniformLocation(shader, build_light_uniform_name(buffer, i, "ambient_color"));
		GLint diffuse_color_location = glGetUniformLocation(shader, build_light_uniform_name(buffer, i, "diffuse_color"));
		GLint specular_color_location = glGetUniformLocation(shader, build_light_uniform_name(buffer, i, "specular_color"));
		glUniform3f(light_position_location, light.position.x, light.position.y, light.position.z);
		glUniform4f(ambient_color_location, light.ambient_color.x, light.ambient_color.y, light.ambient_color.z, light.ambient_color.w);
		glUniform4f(diffuse_color_location, light.diffuse_color.x, light.diffuse_color.y, light.diffuse_color.z, light.diffuse_color.w);
		glUniform4f(specular_color_location, light.specular_color.x, light.specular_color.y, light.specular_color.z, light.specular_color.w);
	}

	GLint light_quantity_location = glGetUniformLocation(shader, "light_quantity");
	glUniform1i(light_quantity_location, number_of_lights);
}

static void diffuse_update_uniforms(const Diffuse_Info* diffuse_info, Shader shader)
{
	glUseProgram(shader);
	GLint use_diffuse_map_location = glGetUniformLocation(shader, "diffuse_info.use_diffuse_map");
	GLint diffuse_map_location = glGetUniformLocation(shader, "diffuse_info.diffuse_map");
	GLint diffuse_color_location = glGetUniformLocation(shader, "diffuse_info.diffuse_color");
	glUniform1i(use_diffuse_map_location, diffuse_info->use_diffuse_map);
	if (diffuse_info->use_diffuse_map)
	{
		glUniform1i(diffuse_map_location, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, diffuse_info->diffuse_map);
	}
	else
		glUniform4f(diffuse_color_location, diffuse_info->diffuse_color.x, diffuse_info->diffuse_color.y,
			diffuse_info->diffuse_color.z, diffuse_info->diffuse_color.w);
}

void graphics_mesh_render(Shader shader, Mesh mesh)
{
	glBindVertexArray(mesh.VAO);
	glUseProgram(shader);
	glDrawElements(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0);
	glUseProgram(0);
	glBindVertexArray(0);
}

void graphics_entity_change_diffuse_map(Entity* entity, u32 diffuse_map, boolean delete_diffuse_map)
{
	if (delete_diffuse_map && entity->diffuse_info.use_diffuse_map)
		glDeleteTextures(1, &entity->diffuse_info.diffuse_map);

	entity->diffuse_info.diffuse_map = diffuse_map;
	entity->diffuse_info.use_diffuse_map = true;
}

void graphics_entity_change_color(Entity* entity, vec4 color, boolean delete_diffuse_map)
{
	if (delete_diffuse_map && entity->diffuse_info.use_diffuse_map)
		glDeleteTextures(1, &entity->diffuse_info.diffuse_map);

	entity->diffuse_info.use_diffuse_map = false;
	entity->diffuse_info.diffuse_color = color;
}

mat4 graphics_entity_get_model_matrix(const Entity* entity)
{
	r32 s, c;

	mat4 scale_matrix = (mat4) {
		entity->world_scale.x, 0.0f, 0.0f, 0.0f,
			0.0f, entity->world_scale.y, 0.0f, 0.0f,
			0.0f, 0.0f, entity->world_scale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 rotation_matrix = quaternion_get_matrix(&entity->world_rotation);

	mat4 translation_matrix = (mat4) {
		1.0f, 0.0f, 0.0f, entity->world_position.x,
			0.0f, 1.0f, 0.0f, entity->world_position.y,
			0.0f, 0.0f, 1.0f, entity->world_position.z,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 model_matrix = gm_mat4_multiply(&rotation_matrix, &scale_matrix);
	model_matrix = gm_mat4_multiply(&translation_matrix, &model_matrix);
	return model_matrix;
}

static mat3 get_symmetric_inertia_tensor_for_object(vec3* vertices, r32 mass, vec3 scale) {
    r32 mass_per_vertex = mass / array_length(vertices);
    mat3 result = {0};
    for (u32 i = 0; i < array_length(vertices); ++i) {
        vec3 v = vertices[i];
        r32 vx = v.x * scale.x;
        r32 vy = v.y * scale.y;
        r32 vz = v.z * scale.z;
        result.data[0][0] += mass_per_vertex * (vy * vy + vz * vz);
        result.data[0][1] += mass_per_vertex * vx * vy;
        result.data[0][2] += mass_per_vertex * vx * vz;
        result.data[1][0] += mass_per_vertex * vx * vy;
        result.data[1][1] += mass_per_vertex * (vx * vx + vz * vz);
        result.data[1][2] += mass_per_vertex * vy * vz;
        result.data[2][0] += mass_per_vertex * vx * vz;
        result.data[2][1] += mass_per_vertex * vy * vz;
        result.data[2][2] += mass_per_vertex * (vx * vx + vy * vy);
    }

    return result;
}

void graphics_entity_create_with_color(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color, r32 mass)
{
	entity->mesh = mesh;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->diffuse_info.diffuse_color = color;
	entity->diffuse_info.use_diffuse_map = false;
	entity->angular_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->linear_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->previous_angular_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->previous_linear_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->inverse_mass = 1.0f / mass;
	entity->inertia_tensor = get_symmetric_inertia_tensor_for_object(mesh.collider.convex_hull.vertices, mass, world_scale);
	assert(gm_mat3_inverse(&entity->inertia_tensor, &entity->inverse_inertia_tensor));
	entity->forces = array_new(Physics_Force);
	entity->fixed = false;
}

void graphics_entity_create_with_color_fixed(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color)
{
	entity->mesh = mesh;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->diffuse_info.diffuse_color = color;
	entity->diffuse_info.use_diffuse_map = false;
	entity->angular_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->linear_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->previous_angular_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->previous_linear_velocity = (vec3){0.0f, 0.0f, 0.0f};
	entity->inverse_mass = 0.0f;
	entity->inertia_tensor = (mat3){0}; // this is not correct, but it shouldn't make a difference
	entity->inverse_inertia_tensor = (mat3){0};
	entity->forces = array_new(Physics_Force);
	entity->fixed = true;
}

void graphics_entity_create_with_texture(Entity* entity, Mesh mesh, vec3 world_position, Quaternion world_rotation, vec3 world_scale, u32 texture)
{
	entity->mesh = mesh;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	entity->diffuse_info.diffuse_map = texture;
	entity->diffuse_info.use_diffuse_map = true;
}

void graphics_entity_destroy(Entity* entity)
{
	if (entity->diffuse_info.use_diffuse_map)
		glDeleteTextures(1, &entity->diffuse_info.diffuse_map);
}

void graphics_entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map)
{
	glDeleteBuffers(1, &entity->mesh.VBO);
	glDeleteBuffers(1, &entity->mesh.EBO);
	glDeleteVertexArrays(1, &entity->mesh.VAO);

	entity->mesh = mesh;
}

void graphics_entity_set_position(Entity* entity, vec3 world_position)
{
	entity->world_position = world_position;
}

void graphics_entity_set_rotation(Entity* entity, Quaternion world_rotation)
{
	entity->world_rotation = world_rotation;
}

void graphics_entity_set_scale(Entity* entity, vec3 world_scale)
{
	entity->world_scale = world_scale;
}

void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity)
{
	init_predefined_shaders();
	Shader shader = predefined_shaders.basic_shader;
	glUseProgram(shader);
	GLint model_matrix_location = glGetUniformLocation(shader, "model_matrix");
	GLint view_matrix_location = glGetUniformLocation(shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(shader, "projection_matrix");
	mat4 model_matrix = graphics_entity_get_model_matrix(entity);
	glUniformMatrix4fv(model_matrix_location, 1, GL_TRUE, (GLfloat*)model_matrix.data);
	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)camera->view_matrix.data);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)camera->projection_matrix.data);
	graphics_mesh_render(shader, entity->mesh);
	glUseProgram(0);
}

void graphics_entity_render_phong_shader(const Perspective_Camera* camera, const Entity* entity, const Light* lights)
{
	init_predefined_shaders();
	Shader shader = predefined_shaders.phong_shader;
	glUseProgram(shader);
	light_update_uniforms(lights, shader);
	GLint camera_position_location = glGetUniformLocation(shader, "camera_position");
	GLint shineness_location = glGetUniformLocation(shader, "object_shineness");
	GLint model_matrix_location = glGetUniformLocation(shader, "model_matrix");
	GLint view_matrix_location = glGetUniformLocation(shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(shader, "projection_matrix");
	glUniform3f(camera_position_location, camera->position.x, camera->position.y, camera->position.z);
	glUniform1f(shineness_location, 128.0f);
	mat4 model_matrix = graphics_entity_get_model_matrix(entity);
	glUniformMatrix4fv(model_matrix_location, 1, GL_TRUE, (GLfloat*)model_matrix.data);
	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)camera->view_matrix.data);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)camera->projection_matrix.data);
	diffuse_update_uniforms(&entity->diffuse_info, shader);
	graphics_mesh_render(shader, entity->mesh);
	glUseProgram(0);
}

u32 graphics_texture_create_from_data(const Image_Data* image_data)
{
	u32 texture_id;

	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	if (image_data->channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image_data->width, image_data->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data->data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, image_data->width, image_data->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data->data);

	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

	// Anisotropic Filtering
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0f);

	glBindTexture(GL_TEXTURE_2D, 0);

	return texture_id;
}

u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data)
{
	u32 texture_id;

	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	if (image_data->channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, image_data->width, image_data->height, 0, GL_RGBA, GL_FLOAT, image_data->data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, image_data->width, image_data->height, 0, GL_RGB, GL_FLOAT, image_data->data);
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

	// Anisotropic Filtering
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0f);

	glBindTexture(GL_TEXTURE_2D, 0);

	return texture_id;
}

u32 graphics_texture_create(const s8* texture_path)
{
	Image_Data image_data = graphics_image_load(texture_path);
	if (image_data.data == NULL) return -1;
	u32 texture_id = graphics_texture_create_from_data(&image_data);
	graphics_image_free(&image_data);

	return texture_id;
}

void graphics_texture_delete(u32 texture_id)
{
	glDeleteTextures(1, &texture_id);
}

void graphics_light_create(Light* light, vec3 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color)
{
	light->position = position;
	light->ambient_color = ambient_color;
	light->diffuse_color = diffuse_color;
	light->specular_color = specular_color;
}

// If memory is null, new memory will be allocated
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r32* memory)
{
	// @TODO: check WHY this is happening
	s32 image_channels = image_data->channels;

	if (!memory)
		memory = (r32*)malloc(sizeof(r32) * image_data->height * image_data->width * image_channels);

	for (s32 i = 0; i < image_data->height; ++i)
	{
		for (s32 j = 0; j < image_data->width; ++j)
		{
			memory[i * image_data->width * image_channels + j * image_channels] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels] / 255.0f;
			memory[i * image_data->width * image_channels + j * image_channels + 1] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels + 1] / 255.0f;
			memory[i * image_data->width * image_channels + j * image_channels + 2] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels + 2] / 255.0f;
			memory[i * image_data->width * image_channels + j * image_channels + 3] = 1.0f;
		}
	}

	Float_Image_Data fid;
	fid.width = image_data->width;
	fid.height = image_data->height;
	fid.channels = image_data->channels;
	fid.data = memory;

	return fid;
}

Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_data, u8* memory)
{
	// @TODO: check WHY this is happening
	s32 image_channels = float_image_data->channels;

	if (!memory)
		memory = (u8*)malloc(sizeof(u8) * float_image_data->height * float_image_data->width * image_channels);

	for (s32 i = 0; i < float_image_data->height; ++i)
	{
		for (s32 j = 0; j < float_image_data->width; ++j)
		{
			memory[i * float_image_data->width * image_channels + j * image_channels] = (u8)round(255.0f * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels]);
			memory[i * float_image_data->width * image_channels + j * image_channels + 1] = (u8)round(255.0f * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels + 1]);
			memory[i * float_image_data->width * image_channels + j * image_channels + 2] = (u8)round(255.0f * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels + 2]);
			if (float_image_data->channels > 3) memory[i * float_image_data->width * image_channels + j * image_channels + 3] = 255;
		}
	}

	Image_Data id;
	id.width = float_image_data->width;
	id.height = float_image_data->height;
	id.channels = float_image_data->channels;
	id.data = memory;

	return id;
}

Mesh graphics_mesh_create_from_obj(const s8* obj_path, Collider_Type collider_type)
{
	Vertex* vertices;
	u32* indices;
	obj_parse(obj_path, &vertices, &indices);
	Mesh m = graphics_mesh_create(vertices, indices, collider_type);
	array_free(vertices);
	array_free(indices);
	return m;
}

// Render primitives

typedef struct {
  u32 shader;
  // Vector rendering
  u32 vector_vao;
  u32 vector_vbo;

  void* data_ptr;
  int vertex_count;

  // Point rendering
  u32 point_vao;
  u32 point_vbo;
  void* point_data_ptr;
  int point_count;

  int initialized;
} Render_Primitives_Context;

typedef struct {
	vec3 position;
	vec4 color;
} Primitive_3D_Vertex;

static Render_Primitives_Context primitives_ctx;

void graphics_renderer_primitives_init()
{
    if (primitives_ctx.initialized) return;
    primitives_ctx.initialized = true;

	int batch_size = 1024 * 1024;

	primitives_ctx.shader = graphics_shader_create("shaders/debug.vs", "shaders/debug.fs");
	glUseProgram(primitives_ctx.shader);

	glGenVertexArrays(1, &primitives_ctx.vector_vao);
	glBindVertexArray(primitives_ctx.vector_vao);
	glGenBuffers(1, &primitives_ctx.vector_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.vector_vbo);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Primitive_3D_Vertex) * batch_size * 2, 0, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Primitive_3D_Vertex), &((Primitive_3D_Vertex *) 0)->position);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Primitive_3D_Vertex), &((Primitive_3D_Vertex *) 0)->color);

	glGenVertexArrays(1, &primitives_ctx.point_vao);
	glBindVertexArray(primitives_ctx.point_vao);
	glGenBuffers(1, &primitives_ctx.point_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.point_vbo);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Primitive_3D_Vertex) * batch_size, 0, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Primitive_3D_Vertex), &((Primitive_3D_Vertex *) 0)->position);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Primitive_3D_Vertex), &((Primitive_3D_Vertex *) 0)->color);

	glUseProgram(0);
}

void graphics_renderer_primitives_flush(const Perspective_Camera* camera)
{
    graphics_renderer_primitives_init();
	glUseProgram(primitives_ctx.shader);

	// Vector
	glDisable(GL_DEPTH_TEST);
	glBindVertexArray(primitives_ctx.vector_vao);
	glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.vector_vbo);
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	GLint view_matrix_location = glGetUniformLocation(primitives_ctx.shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(primitives_ctx.shader, "projection_matrix");

	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat *) camera->view_matrix.data);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat *) camera->projection_matrix.data);

	glDrawArrays(GL_LINES, 0, primitives_ctx.vertex_count);
	primitives_ctx.vertex_count = 0;
	primitives_ctx.data_ptr = 0;
	glEnable(GL_DEPTH_TEST);

	// Points
	glDisable(GL_DEPTH_TEST);
	glBindVertexArray(primitives_ctx.point_vao);
	glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.point_vbo);
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat *) camera->view_matrix.data);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat *) camera->projection_matrix.data);

	glPointSize(10.0f);
	glDrawArrays(GL_POINTS, 0, primitives_ctx.point_count);
	primitives_ctx.point_count = 0;
	primitives_ctx.point_data_ptr = 0;
	glEnable(GL_DEPTH_TEST);

	glUseProgram(0);
}

static void setup_primitives_render()
{
	if (primitives_ctx.data_ptr == 0)
	{
		glBindVertexArray(primitives_ctx.vector_vao);
		glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.vector_vbo);
		primitives_ctx.data_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	}
}

static void setup_primitives_point_render()
{
	if (primitives_ctx.point_data_ptr == 0)
	{
		glBindVertexArray(primitives_ctx.point_vao);
		glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.point_vbo);
		primitives_ctx.point_data_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	}
}

void graphics_renderer_debug_points(vec3* points, int point_count, vec4 color)
{
    graphics_renderer_primitives_init();
	setup_primitives_point_render();
	Primitive_3D_Vertex *verts = (Primitive_3D_Vertex *)primitives_ctx.point_data_ptr + primitives_ctx.point_count;

	for (s32 i = 0; i < point_count; ++i)
	{
		verts[i].position = points[i];
		verts[i].color = color;
	}

	primitives_ctx.point_count += point_count;
}

void graphics_renderer_debug_vector(vec3 p1, vec3 p2, vec4 color)
{
    graphics_renderer_primitives_init();
	setup_primitives_render();

	Primitive_3D_Vertex *verts = (Primitive_3D_Vertex *)primitives_ctx.data_ptr + primitives_ctx.vertex_count;

	verts[0].position = p1;
	verts[0].color = color;

	verts[1].position = p2;
	verts[1].color = color;

	primitives_ctx.vertex_count += 2;
}