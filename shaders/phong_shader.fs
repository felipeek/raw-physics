#version 330 core

in vec3 fragment_position;
in vec3 fragment_normal;
in vec2 fragment_texture_coords;

// Light
struct Light
{
	vec3 position;
	vec4 ambient_color;
	vec4 diffuse_color;
	vec4 specular_color;
};

uniform mat4 model_matrix;
uniform Light lights[16];
uniform int light_quantity;
uniform vec3 camera_position;
uniform float object_shineness;
uniform vec4 color;
// Specular map will not be used (<1,1,1,1> assumed)

out vec4 final_color;

vec3 get_point_color_of_light(Light light)
{
	vec3 normal = normalize(fragment_normal);
	vec4 real_diffuse_color = color;

	vec3 fragment_to_point_light_vec = normalize(light.position - fragment_position);

	// Ambient Color
	vec4 point_ambient_color = light.ambient_color * real_diffuse_color;

	// Diffuse Color
	float point_diffuse_contribution = max(0, dot(fragment_to_point_light_vec, normal));
	vec4 point_diffuse_color = point_diffuse_contribution * light.diffuse_color * real_diffuse_color;
	
	// Specular Color
	vec3 fragment_to_camera_vec = normalize(camera_position - fragment_position);
	float point_specular_contribution = pow(max(dot(fragment_to_camera_vec, reflect(-fragment_to_point_light_vec, normal)), 0.0), object_shineness);
	vec4 point_specular_color = point_specular_contribution * light.specular_color * vec4(1.0, 1.0, 1.0, 1.0);

	// Attenuation
	float point_light_distance = length(light.position - fragment_position);
	float point_attenuation = 1.0 / (1.0 + 0.0014 * point_light_distance +
		0.000007 * point_light_distance * point_light_distance);

	point_ambient_color *= point_attenuation;
	point_diffuse_color *= point_attenuation;
	point_specular_color *= point_attenuation;

	vec4 point_color = point_ambient_color + point_diffuse_color;// + point_specular_color;
	return point_color.xyz;
	//final_color = vec4(point_color.xyz, 1.0);
}

void main()
{
	final_color = vec4(0.0, 0.0, 0.0, 1.0);

	for (int i = 0; i < light_quantity; ++i)
	{
		vec3 point_color = get_point_color_of_light(lights[i]);
		final_color.x += point_color.x;
		final_color.y += point_color.y;
		final_color.z += point_color.z;
	}
}