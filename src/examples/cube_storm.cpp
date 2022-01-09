#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "../vendor/imgui.h"
#include "../render/graphics.h"
#include "../render/obj.h"
#include "../physics/gjk.h"
#include "../physics/epa.h"
#include "../physics/clipping.h"
#include "../physics/pbd.h"
#include "../entity.h"
#include "../util.h"

#define GIM_ENTITY_COLOR (vec4) {1.0, 1.0, 1.0, 1.0}

static Perspective_Camera camera;
static Light* lights;
static Entity* entities;

static Collider create_collider(Vertex* vertices, u32* indices, vec3 scale) {
	vec3* vertices_positions = array_new(vec3);
	for (u32 i = 0; i < array_length(vertices); ++i) {
		vec3 position = (vec3) {
			(r64)vertices[i].position.x,
			(r64)vertices[i].position.y,
			(r64)vertices[i].position.z
		};
		position.x *= scale.x;
		position.y *= scale.y;
		position.z *= scale.z;
		array_push(vertices_positions, position);
	}
	return collider_convex_hull_create(vertices_positions, indices);
}

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { -15.0, 15.0, 25.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	camera.rotation = (Quaternion){0.210476, 0.000000, 0.000000, 0.977617};
	camera.yrotation = (Quaternion){0.000000, 0.303208, 0.000000, 0.952945};
	camera.position = (vec3){-16.194581, 15.336559, 25.110162};
	camera_force_matrix_recalculation(&camera);
	return camera;
}

static Light* create_lights() {
	Light light;
	Light* lights = array_new(Light);

	vec3 light_position = (vec3) {0.0, 0.0, 15.0};
	vec4 ambient_color = (vec4) {0.1, 0.1, 0.1, 1.0};
	vec4 diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0};
	vec4 specular_color = (vec4) {0.5, 0.5, 0.5, 1.0};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	return lights;
}

int ex_cube_storm_init() {
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	Entity e;
	entities = array_new(Entity);

	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 floor_scale = (vec3){50.0, 1.0, 50.0};
	Collider floor_collider = create_collider(cube_vertices, cube_indices, floor_scale);
	entity_create_fixed(&e, cube_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_collider);
	array_push(entities, e);

	const u32 N = 3;
	r64 y = 2.0;
	r64 gap = 2.01;
	for (u32 i = 0; i < N; ++i) {
		y += gap;

		r64 x = -2.0 * (N / 2.0);
		for (u32 j = 0; j < N; ++j) {
			x += gap;

			r64 z = -2.0 * (N / 2.0);
			for (u32 k = 0; k < N; ++k) {
				z += gap;

				vec3 cube_scale = (vec3){1.0, 1.0, 1.0};
				Collider cube_collider = create_collider(cube_vertices, cube_indices, cube_scale);
				entity_create(&e, cube_mesh, (vec3){x, y, z}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
					cube_scale, util_pallete(i + j + k), 1.0, cube_collider);
				e.static_friction_coefficient = 0.0f;
				array_push(entities, e);
			}
		}
	}

	array_free(cube_vertices);
	array_free(cube_indices);

	return 0;
}

void ex_cube_storm_destroy() {
	array_free(lights);

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		collider_destroy(&e->collider);
		mesh_destroy(&e->mesh);
		entity_destroy(e);
	}
	array_free(entities);
}

void ex_cube_storm_update(r64 delta_time) {
	//printf("(Quaternion){%f, %f, %f, %f}\n", camera.rotation.x, camera.rotation.y, camera.rotation.z, camera.rotation.w);
	//printf("(Quaternion){%f, %f, %f, %f}\n", camera.yrotation.x, camera.yrotation.y, camera.yrotation.z, camera.yrotation.w);
	//printf("(vec3){%f, %f, %f}\n", camera.position.x, camera.position.y, camera.position.z);
	delta_time = 0.016666667; // ~60fps

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		collider_update(&e->collider, e->world_position, &e->world_rotation);
	}

	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
		Physics_Force pf;
		pf.force = (vec3){0.0, -GRAVITY * 1.0 / entities[i].inverse_mass, 0.0};
		pf.position = (vec3){0.0, 0.0, 0.0};
		array_push(entities[i].forces, pf);
	}

	pbd_simulate(delta_time, entities);

	for (u32 i = 0; i < array_length(entities); ++i) {
		array_clear(entities[i].forces);
	}
}

void ex_cube_storm_render() {
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, &entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
}

void ex_cube_storm_input_process(boolean* key_state, r64 delta_time) {
	r64 movement_speed = 3.0;
	r64 rotation_speed = 300.0;

	if (key_state[GLFW_KEY_LEFT_SHIFT])
		movement_speed = 0.5;
	if (key_state[GLFW_KEY_RIGHT_SHIFT])
		movement_speed = 0.01;

	if (key_state[GLFW_KEY_W])
		camera_move_forward(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_S])
		camera_move_forward(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_A])
		camera_move_right(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_D])
		camera_move_right(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_L])
	{
		static boolean wireframe = false;

		if (wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		wireframe = !wireframe;
		key_state[GLFW_KEY_L] = false;
	}

	if (key_state[GLFW_KEY_SPACE]) {
		vec3 camera_z = camera_get_z_axis(&camera);
		vec3 camera_pos = camera.position;
		r64 distance = 5.0;
		vec3 diff = gm_vec3_scalar_product(-distance, camera_z);
		vec3 cube_position = gm_vec3_add(camera_pos, diff);

		Entity e;
		const char* mesh_name;
		int r = rand();
		if (r % 3 == 0) {
			mesh_name = "./res/ico.obj";
		} else if (r % 3 == 1) {
			mesh_name = "./res/ico.obj";
		} else {
			mesh_name = "./res/ico.obj";
		}
		Vertex* vertices;
		u32* indices;
		obj_parse(mesh_name, &vertices, &indices);
		Mesh m = graphics_mesh_create(vertices, indices);
		vec3 scale = (vec3){1.0, 1.0, 1.0};
		Collider collider = create_collider(vertices, indices, scale);
		entity_create(&e, m, cube_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 0.0),
			scale, (vec4){rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, 1.0}, 10.0, collider);
		array_free(vertices);
		array_free(indices);

		e.linear_velocity = gm_vec3_scalar_product(10.0, gm_vec3_scalar_product(-1.0, camera_z));
		array_push(entities, e);

		key_state[GLFW_KEY_SPACE] = false;
	}
}

void ex_cube_storm_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
	static r64 x_pos_old, y_pos_old;

	r64 x_difference = x_pos - x_pos_old;
	r64 y_difference = y_pos - y_pos_old;

	x_pos_old = x_pos;
	y_pos_old = y_pos;

	if (reset) return;

	static const r64 camera_mouse_speed = 0.1;
	camera_rotate_x(&camera, camera_mouse_speed * (r64)x_difference);
	camera_rotate_y(&camera, camera_mouse_speed * (r64)y_difference);
}

void ex_cube_storm_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_cube_storm_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_cube_storm_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_cube_storm_menu_update() {
	ImGui::Text("Cube Storm");
	ImGui::Separator();
	ImGui::TextWrapped("Press SPACE to throw objects!");
}