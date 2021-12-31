#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec4 vertex_color;

out vec4 f_color;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main()
{
	gl_Position = projection_matrix * view_matrix * vec4(vertex_position, 1.0);
	f_color = vertex_color;
}