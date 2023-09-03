#include "coin.h"
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
static eid coin_eid, floor_eid;
static r32 restitution_coefficient = 0.5;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0, 2.0, 10.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;

	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);

	camera.rotation = (Quaternion){0.240228, 0.000000, 0.000000, 0.970716};
	camera.yrotation = (Quaternion){0.000000, -0.298208, 0.000000, 0.954501};
	camera.position = (vec3){12.162289, 8.025262, 15.310513};
	camera_force_matrix_recalculation(&camera);

	return camera;
}

int ex_coin_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = examples_util_create_lights();

	// TODO: We don't have a collider of type CYLINDER yet, hence we are using a complex convex hull to represent the cylinder here.
	// Once that is added to the engine we should use the cylinder collider here.
	Vertex* coin_vertices;
	u32* coin_indices;
	obj_parse("./res/cylinder.obj", &coin_vertices, &coin_indices);
	Mesh coin_mesh = graphics_mesh_create(coin_vertices, coin_indices);

	vec3 coin_scale = (vec3){3.0, 0.1, 3.0};
	//vec3 coin_scale = (vec3){1.0, 1.0, 1.0}; // for debug
	Collider* coin_colliders = examples_util_create_single_convex_hull_collider_array(coin_vertices, coin_indices, coin_scale);
	coin_eid = entity_create(coin_mesh, (vec3){0.0, 4.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 1.0}, 30.0),
		coin_scale, (vec4){205.0 / 255.0, 127.0 / 255.0, 50.0 / 255.0, 1.0}, 1.0,
		coin_colliders, 0.5, 0.5, restitution_coefficient);

	Vertex* floor_vertices;
	u32* floor_indices;
	obj_parse("./res/floor.obj", &floor_vertices, &floor_indices);
	Mesh floor_mesh = graphics_mesh_create(floor_vertices, floor_indices);
	vec3 floor_scale = (vec3){1.0, 1.0, 1.0};
	Collider* floor_colliders = examples_util_create_single_convex_hull_collider_array(floor_vertices, floor_indices, floor_scale);
	floor_eid = entity_create_fixed(floor_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_colliders, 0.5, 0.5, restitution_coefficient);
	array_free(floor_vertices);
	array_free(floor_indices);

	array_free(coin_vertices);
	array_free(coin_indices);

	return 0;
}

void ex_coin_destroy() {
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

void ex_coin_update(r64 delta_time) {
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

void ex_coin_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_coin_input_process(boolean* key_state, r64 delta_time) {
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
}

void ex_coin_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_coin_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_coin_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_coin_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_coin_menu_update() {
	Entity* coin_entity = entity_get_by_id(coin_eid);
	Entity* floor_entity = entity_get_by_id(floor_eid);

	ImGui::Text("Coin");
	ImGui::Separator();

	ImGui::TextWrapped("The engine does not have a cylinder collider yet. The coin collider is being created as a generic convex hull. This is just an experiment :)");

	ImGui::TextWrapped("Coin and floor restitution coefficient:");
	if (ImGui::SliderFloat("rc", &restitution_coefficient, 0.0f, 0.8f, "%.3f")) {
		coin_entity->restitution_coefficient = (r64)restitution_coefficient;
		floor_entity->restitution_coefficient = (r64)restitution_coefficient;
		entity_activate(coin_entity);
	}

	ImGui::Separator();
}

Example_Scene coin_example_scene = (Example_Scene) {
	.name = "Coin",
	.init = ex_coin_init,
	.destroy = ex_coin_destroy,
	.input_process = ex_coin_input_process,
	.menu_properties_update = ex_coin_menu_update,
	.mouse_change_process = ex_coin_mouse_change_process,
	.mouse_click_process = ex_coin_mouse_click_process,
	.render = ex_coin_render,
	.scroll_change_process = ex_coin_scroll_change_process,
	.update = ex_coin_update,
	.window_resize_process = ex_coin_window_resize_process
};