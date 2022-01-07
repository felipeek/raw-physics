#include "mesh.h"
#include <GL/glew.h>
#include <light_array.h>
#include "obj.h"

Mesh graphics_mesh_create_from_obj(const s8* obj_path)
{
	Vertex* vertices;
	u32* indices;
	obj_parse(obj_path, &vertices, &indices);
	Mesh m = graphics_mesh_create(vertices, indices);
	array_free(vertices);
	array_free(indices);
	return m;
}

Mesh graphics_quad_create()
{
	r32 size = 1.0f;
	Vertex* vertices = array_new(Vertex);
	u32* indices = array_new(u32);

	Vertex v;
	v.position = (fvec3) { 0.0f, 0.0f, 0.0f};
	v.normal = (fvec3) { 0.0f, 0.0f, 1.0f};
	v.texture_coordinates = (fvec2) { 0.0f, 0.0f };
	array_push(vertices, v);

	v.position = (fvec3) { size, 0.0f, 0.0f };
	v.normal = (fvec3) { 0.0f, 0.0f, 1.0f };
	v.texture_coordinates = (fvec2) { 1.0f, 0.0f };
	array_push(vertices, v);

	v.position = (fvec3) { 0.0f, size, 0.0f };
	v.normal = (fvec3) { 0.0f, 0.0f, 1.0f };
	v.texture_coordinates = (fvec2) { 0.0f, 1.0f };
	array_push(vertices, v);

	v.position = (fvec3) { size, size, 0.0f };
	v.normal = (fvec3) { 0.0f, 0.0f, 1.0f };
	v.texture_coordinates = (fvec2) { 1.0f, 1.0f };
	array_push(vertices, v);

	array_push(indices, 0);
	array_push(indices, 1);
	array_push(indices, 2);
	array_push(indices, 1);
	array_push(indices, 3);
	array_push(indices, 2);

	return graphics_mesh_create(vertices, indices);
}

Mesh graphics_mesh_create(Vertex* vertices, u32* indices)
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

	return mesh;
}

void mesh_destroy(Mesh* mesh) {
	// In theory this function is idempotent
	glDeleteVertexArrays(1, &mesh->VAO);
	glDeleteBuffers(1, &mesh->EBO);
	glDeleteBuffers(1, &mesh->VBO);
}