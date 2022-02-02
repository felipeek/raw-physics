#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <light_array.h>
#include "core.h"
#include "render/graphics.h"
#include "render/menu.h"
#include "render/obj.h"
#include "vendor/imgui.h"
#include "physics/physics.h"

#define MENU_TITLE "1 - Basics"

static Perspective_Camera camera;
static Light* lights;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0, 2.0, 10.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
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

static eid cube_id;

int core_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 cube_scale = (vec3){1.0, 1.0, 1.0};
	cube_id = entity_create(cube_mesh, (vec3){0.0, 2.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0),
		cube_scale, (vec4){1.0, 1.0, 1.0, 1.0}, 1.0, cube_vertices, cube_indices);

	array_free(cube_vertices);
	array_free(cube_indices);

	return 0;
}

void core_destroy() {
}

void core_update(r64 delta_time) {
	Entity** entities = entity_get_all();
	delta_time = 0.016666667; // ~60fps

	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
        //entity_add_force(entities[i], (vec3){0.0, 0.0, 0.0}, (vec3){0.0, -GRAVITY * 1.0 / entities[i]->inverse_mass}, 0.0);
	}

	physics_simulate(entities, delta_time);

	for (u32 i = 0; i < array_length(entities); ++i) {
		array_clear(entities[i]->forces);
	}

	array_free(entities);
}

void core_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void core_input_process(boolean* key_state, r64 delta_time) {
	r64 movement_speed = 15.0;
	r64 rotation_speed = 300.0;

	if (key_state[GLFW_KEY_LEFT_SHIFT]) {
		movement_speed = 0.5;
	}
	if (key_state[GLFW_KEY_RIGHT_SHIFT]) {
		movement_speed = 0.01;
	}

	if (key_state[GLFW_KEY_W]) {
		camera_move_forward(&camera, movement_speed * delta_time);
	}
	if (key_state[GLFW_KEY_S]) {
		camera_move_forward(&camera, -movement_speed * delta_time);
	}
	if (key_state[GLFW_KEY_A]) {
		camera_move_right(&camera, -movement_speed * delta_time);
	}
	if (key_state[GLFW_KEY_D]) {
		camera_move_right(&camera, movement_speed * delta_time);
	}
	if (key_state[GLFW_KEY_L]) {
		static boolean wireframe = false;

		if (wireframe) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		} else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}

		wireframe = !wireframe;
		key_state[GLFW_KEY_L] = false;
	}
	if (key_state[GLFW_KEY_M]) {
		Entity* e = entity_get_by_id(cube_id);
        entity_add_force(e, (vec3){1.0, 1.0, 1.0}, (vec3){0.0, -1.0, 0.0}, true);
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
	static const r64 camera_mouse_speed = 0.1;
	static r64 x_pos_old, y_pos_old;

	r64 x_difference = x_pos - x_pos_old;
	r64 y_difference = y_pos - y_pos_old;

	x_pos_old = x_pos;
	y_pos_old = y_pos;

	if (reset) return;

	camera_rotate_x(&camera, camera_mouse_speed * (r64)x_difference);
	camera_rotate_y(&camera, camera_mouse_speed * (r64)y_difference);
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {
}

void core_scroll_change_process(r64 x_offset, r64 y_offset) {
}

void core_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void core_menu_render() {
	if (ImGui::Begin(MENU_TITLE, NULL, 0)) {
	}
}
