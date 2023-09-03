#include "brick_wall.h"
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
#include "examples_util.h"

static Perspective_Camera camera;
static Light* lights;
static r32 static_friction_coefficient = 0.5;
static r32 dynamic_friction_coefficient = 0.4;
static r32 restitution_coefficient = 0.0;
static r64 thrown_objects_initial_linear_velocity_norm = 15.0;
static eid* brick_eids;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { -15.0, 15.0, 25.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	camera.rotation = (Quaternion){0.134854, 0.000000, 0.000000, 0.990883};
	camera.yrotation = (Quaternion){0.000000, 0.262195, 0.000000, 0.965035};
	camera.position = (vec3){-6.386153, 7.327643, 20.298402};
	camera_force_matrix_recalculation(&camera);
	return camera;
}

int ex_brick_wall_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = examples_util_create_lights();
	
	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 floor_scale = (vec3){50.0, 1.0, 50.0};
	Collider* floor_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, floor_scale);
	entity_create_fixed(cube_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_colliders, 0.5, 0.5, 0.0);

	brick_eids = array_new(eid);
	const r64 brick_height = 0.35;
	const r64 brick_width = 0.8;
	r64 y = -1.0;
	r64 x;
	for (u32 i = 0; i < 6; ++i) {
		y += 2 * brick_height + 0.01;
		if (i % 2 == 0) {
			x = -2.0;
		} else {
			x = -2.0 + brick_width / 2;
		}
		for (u32 j = 0; j < 4; ++j) {
			vec3 cube_scale = (vec3){brick_width, brick_height, brick_height};
			Collider* cube_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, cube_scale);
			vec4 color = ((i + j) % 2 == 0) ?
				(vec4) { 188.0 / 255.0, 74.0 / 255.0, 60.0 / 255.0, 1.0 } :
				(vec4) { 168.0 / 255.0, 64.0 / 255.0, 50.0 / 255.0, 1.0 };
			eid brick_eid = entity_create(cube_mesh, (vec3){x, y, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
				cube_scale, color, 0.5, cube_colliders, static_friction_coefficient, dynamic_friction_coefficient, 0.0);
			array_push(brick_eids, brick_eid);
			x += 2 * brick_width + 0.01;
		}
		x = -2.0;
	}

	array_free(cube_vertices);
	array_free(cube_indices);

	return 0;
}

void ex_brick_wall_destroy() {
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
	array_free(brick_eids);
	entity_module_destroy();
}

void ex_brick_wall_update(r64 delta_time) {
	//printf("(Quaternion){%f, %f, %f, %f}\n", camera.rotation.x, camera.rotation.y, camera.rotation.z, camera.rotation.w);
	//printf("(Quaternion){%f, %f, %f, %f}\n", camera.yrotation.x, camera.yrotation.y, camera.yrotation.z, camera.yrotation.w);
	//printf("(vec3){%f, %f, %f}\n", camera.position.x, camera.position.y, camera.position.z);

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

void ex_brick_wall_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_brick_wall_input_process(boolean* key_state, r64 delta_time) {
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

void ex_brick_wall_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_brick_wall_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_brick_wall_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_brick_wall_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_brick_wall_menu_update() {
	ImGui::Text("Brick Wall");
	ImGui::Separator();

	ImGui::TextWrapped("Tweak the friction coefficients of the bricks that compose the wall.");
	ImGui::TextWrapped("Bricks static friction coefficient:");
	if (ImGui::SliderFloat("fs", &static_friction_coefficient, 0.0f, 1.0f, "%.3f")) {
		for (u32 i = 0; i < array_length(brick_eids); ++i) {
			Entity* brick_entity = entity_get_by_id(brick_eids[i]);
			brick_entity->static_friction_coefficient = (r64)static_friction_coefficient;
			entity_activate(brick_entity);
		}
	}

	ImGui::TextWrapped("Bricks dynamic friction coefficient:");
	bool changed = ImGui::SliderFloat("fd", &dynamic_friction_coefficient, 0.0f, 1.0f, "%.3f");
	if (changed || dynamic_friction_coefficient > static_friction_coefficient) {
		// clamp dynamic friction if it was set to be greater than static friction (to be 'physically' more accurate)
		dynamic_friction_coefficient = CLAMP(dynamic_friction_coefficient, 0.0, static_friction_coefficient);
		for (u32 i = 0; i < array_length(brick_eids); ++i) {
			Entity* brick_entity = entity_get_by_id(brick_eids[i]);
			brick_entity->dynamic_friction_coefficient = (r64)dynamic_friction_coefficient;
			entity_activate(brick_entity);
		}
	}
	ImGui::Separator();

	ImGui::TextWrapped("Press SPACE to throw objects!");
	ImGui::TextWrapped("Thrown objects initial linear velocity norm:");
	r32 vel = (r32)thrown_objects_initial_linear_velocity_norm;
	ImGui::SliderFloat("Vel", &vel, 1.0f, 30.0f, "%.2f");
	thrown_objects_initial_linear_velocity_norm = vel;
}

Example_Scene brick_wall_example_scene = (Example_Scene) {
	.name = "Brick Wall",
	.init = ex_brick_wall_init,
	.destroy = ex_brick_wall_destroy,
	.input_process = ex_brick_wall_input_process,
	.menu_properties_update = ex_brick_wall_menu_update,
	.mouse_change_process = ex_brick_wall_mouse_change_process,
	.mouse_click_process = ex_brick_wall_mouse_click_process,
	.render = ex_brick_wall_render,
	.scroll_change_process = ex_brick_wall_scroll_change_process,
	.update = ex_brick_wall_update,
	.window_resize_process = ex_brick_wall_window_resize_process
};