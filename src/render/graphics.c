#include "graphics.h"
#include "../util.h"
#include "obj.h"
#include <GL/glew.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <light_array.h>
#include <math.h>
#include "../util.h"

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

	fid.data = malloc(sizeof(r64) * fid.width * fid.height * fid.channels);
	memcpy(fid.data, image_data->data, sizeof(r64) * fid.width * fid.height * fid.channels);
	
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
		glUniform3f(light_position_location, (r32)light.position.x, (r32)light.position.y, (r32)light.position.z);
		glUniform4f(ambient_color_location, (r32)light.ambient_color.x, (r32)light.ambient_color.y, (r32)light.ambient_color.z, (r32)light.ambient_color.w);
		glUniform4f(diffuse_color_location, (r32)light.diffuse_color.x, (r32)light.diffuse_color.y, (r32)light.diffuse_color.z, (r32)light.diffuse_color.w);
		glUniform4f(specular_color_location, (r32)light.specular_color.x, (r32)light.specular_color.y, (r32)light.specular_color.z, (r32)light.specular_color.w);
	}

	GLint light_quantity_location = glGetUniformLocation(shader, "light_quantity");
	glUniform1i(light_quantity_location, number_of_lights);
}

static void graphics_mesh_render(Shader shader, Mesh mesh)
{
	glBindVertexArray(mesh.VAO);
	glUseProgram(shader);
	glDrawElements(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0);
	glUseProgram(0);
	glBindVertexArray(0);
}

void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity)
{
	init_predefined_shaders();
	Shader shader = predefined_shaders.basic_shader;
	glUseProgram(shader);
	GLint model_matrix_location = glGetUniformLocation(shader, "model_matrix");
	GLint view_matrix_location = glGetUniformLocation(shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(shader, "projection_matrix");
	mat4 model_matrix = entity_get_model_matrix(entity);
	r32 model[16], view[16], proj[16];
	util_matrix_to_r32_array(&model_matrix, model);
	util_matrix_to_r32_array(&camera->view_matrix, view);
	util_matrix_to_r32_array(&camera->projection_matrix, proj);
	glUniformMatrix4fv(model_matrix_location, 1, GL_TRUE, (GLfloat*)model);
	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)view);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)proj);
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
	glUniform3f(camera_position_location, (r32)camera->position.x, (r32)camera->position.y, (r32)camera->position.z);
	glUniform1f(shineness_location, 128.0f);
	mat4 model_matrix = entity_get_model_matrix(entity);
	r32 model[16], view[16], proj[16];
	util_matrix_to_r32_array(&model_matrix, model);
	util_matrix_to_r32_array(&camera->view_matrix, view);
	util_matrix_to_r32_array(&camera->projection_matrix, proj);
	glUniformMatrix4fv(model_matrix_location, 1, GL_TRUE, (GLfloat*)model);
	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)view);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)proj);
	GLint diffuse_color_location = glGetUniformLocation(shader, "color");
	glUniform4f(diffuse_color_location, (r32)entity->color.x, (r32)entity->color.y, (r32)entity->color.z, (r32)entity->color.w);
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
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0);

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
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0);

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
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r64* memory)
{
	// @TODO: check WHY this is happening
	s32 image_channels = image_data->channels;

	if (!memory)
		memory = (r64*)malloc(sizeof(r64) * image_data->height * image_data->width * image_channels);

	for (s32 i = 0; i < image_data->height; ++i)
	{
		for (s32 j = 0; j < image_data->width; ++j)
		{
			memory[i * image_data->width * image_channels + j * image_channels] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels] / 255.0;
			memory[i * image_data->width * image_channels + j * image_channels + 1] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels + 1] / 255.0;
			memory[i * image_data->width * image_channels + j * image_channels + 2] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels + 2] / 255.0;
			memory[i * image_data->width * image_channels + j * image_channels + 3] = 1.0;
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
			memory[i * float_image_data->width * image_channels + j * image_channels] = (u8)round(255.0 * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels]);
			memory[i * float_image_data->width * image_channels + j * image_channels + 1] = (u8)round(255.0 * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels + 1]);
			memory[i * float_image_data->width * image_channels + j * image_channels + 2] = (u8)round(255.0 * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels + 2]);
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

	r32 view[16], proj[16];
	util_matrix_to_r32_array(&camera->view_matrix, view);
	util_matrix_to_r32_array(&camera->projection_matrix, proj);

	// Vector
	glDisable(GL_DEPTH_TEST);
	glBindVertexArray(primitives_ctx.vector_vao);
	glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.vector_vbo);
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

	GLint view_matrix_location = glGetUniformLocation(primitives_ctx.shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(primitives_ctx.shader, "projection_matrix");

	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)view);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)proj);

	glDrawArrays(GL_LINES, 0, primitives_ctx.vertex_count);
	primitives_ctx.vertex_count = 0;
	primitives_ctx.data_ptr = 0;
	glEnable(GL_DEPTH_TEST);

	// Points
	glDisable(GL_DEPTH_TEST);
	glBindVertexArray(primitives_ctx.point_vao);
	glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.point_vbo);
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)view);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)proj);

	glPointSize(10.0);
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