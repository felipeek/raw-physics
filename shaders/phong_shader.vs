#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_texture_coords;

out vec3 fragment_position;
out vec3 fragment_normal;
out vec2 fragment_texture_coords;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main()
{
	fragment_normal = mat3(inverse(transpose(model_matrix))) * vertex_normal;
	fragment_texture_coords = vertex_texture_coords;
	fragment_position = (model_matrix * vec4(vertex_position, 1.0)).xyz;
	gl_Position = projection_matrix * view_matrix * model_matrix * vec4(vertex_position, 1.0);
}