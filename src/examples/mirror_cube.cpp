#include "mirror_cube.h"
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
#include "examples_util.h"

static Perspective_Camera camera;
static Light* lights;
static r64 thrown_objects_initial_linear_velocity_norm = 15.0;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0, 2.0, 10.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	return camera;
}

int ex_mirror_cube_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = examples_util_create_lights();
	
	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	Vertex* mirror_cube_vertices;
	u32* mirror_cube_indices;
	obj_parse("./res/mirror_cube.obj", &mirror_cube_vertices, &mirror_cube_indices);
	Mesh mirror_cube_mesh = graphics_mesh_create(mirror_cube_vertices, mirror_cube_indices);
	
	Vertex* mirror_cube_collider1_vertices;
	u32* mirror_cube_collider1_indices;
	obj_parse("./res/mirror_cube_collider1.obj", &mirror_cube_collider1_vertices, &mirror_cube_collider1_indices);

	Vertex* mirror_cube_collider2_vertices;
	u32* mirror_cube_collider2_indices;
	obj_parse("./res/mirror_cube_collider2.obj", &mirror_cube_collider2_vertices, &mirror_cube_collider2_indices);

	vec3 floor_scale = (vec3){50.0, 1.0, 50.0};
	Collider* floor_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, floor_scale);
	entity_create_fixed(cube_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_colliders, 0.5, 0.5, 0.0);

	vec3 mirror_cube_scale = (vec3){1.0, 1.0, 1.0};
	Collider mirror_cube_collider1 = examples_util_create_convex_hull_collider(mirror_cube_collider1_vertices, mirror_cube_collider1_indices, mirror_cube_scale);
	Collider mirror_cube_collider2 = examples_util_create_convex_hull_collider(mirror_cube_collider2_vertices, mirror_cube_collider2_indices, mirror_cube_scale);
	Collider* mirror_cube_colliders = array_new(Collider);
	array_push(mirror_cube_colliders, mirror_cube_collider1);
	array_push(mirror_cube_colliders, mirror_cube_collider2);
	entity_create(mirror_cube_mesh, (vec3){0.0, 2.0, 0.0}, quaternion_new((vec3){1.0, 1.0, 1.0}, 33.0),
		mirror_cube_scale, (vec4){1.0, 1.0, 1.0, 1.0}, 1.0, mirror_cube_colliders, 0.8, 0.8, 0.0);

	array_free(cube_vertices);
	array_free(cube_indices);
	array_free(mirror_cube_vertices);
	array_free(mirror_cube_indices);
	array_free(mirror_cube_collider1_vertices);
	array_free(mirror_cube_collider1_indices);
	array_free(mirror_cube_collider2_vertices);
	array_free(mirror_cube_collider2_indices);

	return 0;
}

void ex_mirror_cube_destroy() {
	array_free(lights);

	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		colliders_destroy(e->colliders);
		array_free(e->colliders);
		mesh_destroy(&e->mesh);
		entity_destroy(e);
	}
	array_free(entities);

	entity_module_destroy();
}

void ex_mirror_cube_update(r64 delta_time) {
	Entity** entities = entity_get_all();

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		colliders_update(e->colliders, e->world_position, &e->world_rotation);
	}

	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
		entity_add_force(entities[i], (vec3){0.0, 0.0, 0.0}, (vec3){0.0, -GRAVITY * 1.0 / entities[i]->inverse_mass, 0.0}, false);
	}

	pbd_simulate(delta_time, entities, 20, 1, true);

	for (u32 i = 0; i < array_length(entities); ++i) {
		entity_clear_forces(entities[i]);
	}
	array_free(entities);
}

void ex_mirror_cube_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_mirror_cube_input_process(boolean* key_state, r64 delta_time) {
	r64 movement_speed = 3.0;
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

	if (key_state[GLFW_KEY_SPACE]) {
		examples_util_throw_object(&camera, thrown_objects_initial_linear_velocity_norm);
		key_state[GLFW_KEY_SPACE] = false;
	}
}

void ex_mirror_cube_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_mirror_cube_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_mirror_cube_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_mirror_cube_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_mirror_cube_menu_update() {
	ImGui::Text("Mirror Cube");
	ImGui::Separator();

	ImGui::TextWrapped("Press SPACE to throw objects!");
	ImGui::TextWrapped("Thrown objects initial linear velocity norm:");
	r32 vel = (r32)thrown_objects_initial_linear_velocity_norm;
	ImGui::SliderFloat("Vel", &vel, 1.0f, 30.0f, "%.2f");
	thrown_objects_initial_linear_velocity_norm = vel;
}

Example_Scene mirror_cube_example_scene = (Example_Scene) {
	.name = "Mirror Cube",
	.init = ex_mirror_cube_init,
	.destroy = ex_mirror_cube_destroy,
	.input_process = ex_mirror_cube_input_process,
	.menu_properties_update = ex_mirror_cube_menu_update,
	.mouse_change_process = ex_mirror_cube_mouse_change_process,
	.mouse_click_process = ex_mirror_cube_mouse_click_process,
	.render = ex_mirror_cube_render,
	.scroll_change_process = ex_mirror_cube_scroll_change_process,
	.update = ex_mirror_cube_update,
	.window_resize_process = ex_mirror_cube_window_resize_process
};