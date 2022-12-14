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

Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path) {
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
	if (!success) {
		glGetShaderInfoLog(vertex_shader, 1024, 0, info_log_buffer);
		printf("Error compiling vertex shader: %s\n", info_log_buffer);
	}

	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(fragment_shader, 1024, 0, info_log_buffer);
		printf("Error compiling fragment shader: %s\n", info_log_buffer);
	}

	GLuint shader_program = glCreateProgram();

	glAttachShader(shader_program, vertex_shader);
	glAttachShader(shader_program, fragment_shader);
	glLinkProgram(shader_program);

	glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
	if (!success) {
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

static void init_predefined_shaders() {
	if (!predefined_shaders.initialized) {
		predefined_shaders.phong_shader = graphics_shader_create(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
		predefined_shaders.basic_shader = graphics_shader_create(BASIC_VERTEX_SHADER_PATH, BASIC_FRAGMENT_SHADER_PATH);
		predefined_shaders.initialized = true;
	}
}

static s8* build_light_uniform_name(s8* buffer, s32 index, const s8* property) {
	sprintf(buffer, "lights[%d].%s", index, property);
	return buffer;
}

static void light_update_uniforms(const Light* lights, Shader shader) {
	s32 number_of_lights = array_length(lights);
	s8 buffer[64];
	glUseProgram(shader);

	for (s32 i = 0; i < number_of_lights; ++i) {
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

static void graphics_mesh_render(Shader shader, Mesh mesh) {
	glBindVertexArray(mesh.VAO);
	glUseProgram(shader);
	glDrawElements(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0);
	glUseProgram(0);
	glBindVertexArray(0);
}

void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity) {
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

void graphics_entity_render_phong_shader(const Perspective_Camera* camera, const Entity* entity, const Light* lights) {
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

void graphics_light_create(Light* light, vec3 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color) {
	light->position = position;
	light->ambient_color = ambient_color;
	light->diffuse_color = diffuse_color;
	light->specular_color = specular_color;
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
	fvec3 position;
	fvec4 color;
} Primitive_3D_Vertex;

static Render_Primitives_Context primitives_ctx;

void graphics_renderer_primitives_init() {
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

void graphics_renderer_primitives_flush(const Perspective_Camera* camera) {
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

static void setup_primitives_render() {
	if (primitives_ctx.data_ptr == 0) {
		glBindVertexArray(primitives_ctx.vector_vao);
		glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.vector_vbo);
		primitives_ctx.data_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	}
}

static void setup_primitives_point_render() {
	if (primitives_ctx.point_data_ptr == 0) {
		glBindVertexArray(primitives_ctx.point_vao);
		glBindBuffer(GL_ARRAY_BUFFER, primitives_ctx.point_vbo);
		primitives_ctx.point_data_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	}
}

void graphics_renderer_debug_points(vec3* points, int point_count, vec4 color) {
	graphics_renderer_primitives_init();
	setup_primitives_point_render();
	Primitive_3D_Vertex *verts = (Primitive_3D_Vertex *)primitives_ctx.point_data_ptr + primitives_ctx.point_count;

	for (s32 i = 0; i < point_count; ++i) {
		verts[i].position = (fvec3){(r32)points[i].x, (r32)points[i].y, (r32)points[i].z};
		verts[i].color = (fvec4){(r32)color.x, (r32)color.y, (r32)color.z, (r32)color.z};
	}

	primitives_ctx.point_count += point_count;
}

void graphics_renderer_debug_vector(vec3 p1, vec3 p2, vec4 color) {
	graphics_renderer_primitives_init();
	setup_primitives_render();

	Primitive_3D_Vertex *verts = (Primitive_3D_Vertex *)primitives_ctx.data_ptr + primitives_ctx.vertex_count;

	verts[0].position = (fvec3){(r32)p1.x, (r32)p1.y, (r32)p1.z};
	verts[0].color = (fvec4){(r32)color.x, (r32)color.y, (r32)color.z, (r32)color.w};

	verts[1].position = (fvec3){(r32)p2.x, (r32)p2.y, (r32)p2.z};
	verts[1].color = (fvec4){(r32)color.x, (r32)color.y, (r32)color.z, (r32)color.w};

	primitives_ctx.vertex_count += 2;
}