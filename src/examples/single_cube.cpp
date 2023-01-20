#include "single_cube.h"
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
static eid cube_eid;
static r32 static_friction_coefficient = 1.0;
static r32 dynamic_friction_coefficient = 0.7;
static r32 restitution_coefficient = 0.0;

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

int ex_single_cube_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = examples_util_create_lights();
	
	Vertex* ramp_vertices;
	u32* ramp_indices;
	obj_parse("./res/ramp.obj", &ramp_vertices, &ramp_indices);
	Mesh ramp_mesh = graphics_mesh_create(ramp_vertices, ramp_indices);

	vec3 ramp_scale = (vec3){2.0, 4.0, 10.0};
	Collider* ramp_colliders = examples_util_create_single_convex_hull_collider_array(ramp_vertices, ramp_indices, ramp_scale);
	entity_create_fixed(ramp_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, -90.0),
		ramp_scale, (vec4){1.0, 1.0, 1.0, 1.0}, ramp_colliders, static_friction_coefficient, dynamic_friction_coefficient, restitution_coefficient);
	printf("Using %.3f, %.3f, %.3f\n", static_friction_coefficient, dynamic_friction_coefficient, restitution_coefficient);

	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 cube_scale = (vec3){1.0, 1.0, 1.0};
	Collider* cube_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, cube_scale);
	cube_eid = entity_create(cube_mesh, (vec3){-5.0, 4.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0),
		cube_scale, (vec4){0.8, 0.8, 1.0, 1.0}, 1.0, cube_colliders, static_friction_coefficient, dynamic_friction_coefficient, restitution_coefficient);

	array_free(cube_vertices);
	array_free(cube_indices);
	array_free(ramp_vertices);
	array_free(ramp_indices);

	return 0;
}

void ex_single_cube_destroy() {
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

void ex_single_cube_update(r64 delta_time) {
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

void ex_single_cube_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_single_cube_input_process(boolean* key_state, r64 delta_time) {
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
	if (key_state[GLFW_KEY_P]) {
		Entity* cube_entity = entity_get_by_id(cube_eid);
		entity_add_force(cube_entity, (vec3){0.0, 1.0, 0.0}, (vec3){10.0, 0.0, 0.0}, false);
		entity_activate(cube_entity);
	}
}

void ex_single_cube_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_single_cube_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_single_cube_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_single_cube_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_single_cube_menu_update() {
	Entity* cube_entity = entity_get_by_id(cube_eid);

	ImGui::Text("Single Cube");
	ImGui::Separator();

	ImGui::TextWrapped("Tweak the friction coefficients to see how the cube slide on the ramp.");
	ImGui::TextWrapped("Cube and ramp static friction coefficient:");
	if (ImGui::SliderFloat("fs", &static_friction_coefficient, 0.0f, 1.0f, "%.3f")) {
		cube_entity->static_friction_coefficient = (r64)static_friction_coefficient;
		entity_activate(cube_entity);
	}

	ImGui::TextWrapped("Cube and ramp dynamic friction coefficient:");
	bool changed = ImGui::SliderFloat("fd", &dynamic_friction_coefficient, 0.0f, 1.0f, "%.3f");
	if (changed || dynamic_friction_coefficient > static_friction_coefficient) {
		// clamp dynamic friction if it was set to be greater than static friction (to be 'physically' more accurate)
		dynamic_friction_coefficient = CLAMP(dynamic_friction_coefficient, 0.0, static_friction_coefficient);
		cube_entity->dynamic_friction_coefficient = (r64)dynamic_friction_coefficient;
		entity_activate(cube_entity);
	}

	ImGui::TextWrapped("Cube restitution coefficient:");
	if (ImGui::SliderFloat("rc", &restitution_coefficient, 0.0f, 1.0f, "%.3f")) {
		cube_entity->restitution_coefficient = (r64)restitution_coefficient;
		entity_activate(cube_entity);
	}
	ImGui::Separator();

	ImGui::TextWrapped("Press [P] to create an horizontal force (->) at the top of the cube.");
}

Example_Scene single_cube_example_scene = (Example_Scene) {
	.name = "Single Cube",
	.init = ex_single_cube_init,
	.destroy = ex_single_cube_destroy,
	.input_process = ex_single_cube_input_process,
	.menu_properties_update = ex_single_cube_menu_update,
	.mouse_change_process = ex_single_cube_mouse_change_process,
	.mouse_click_process = ex_single_cube_mouse_click_process,
	.render = ex_single_cube_render,
	.scroll_change_process = ex_single_cube_scroll_change_process,
	.update = ex_single_cube_update,
	.window_resize_process = ex_single_cube_window_resize_process
};