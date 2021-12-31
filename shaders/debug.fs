#version 330 core

out vec4 final_color;
in vec4 f_color;

void main()
{
	final_color = f_color;
}