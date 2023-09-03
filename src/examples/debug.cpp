#include "debug.h"
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

static eid cube_eid, cube2_eid;
static Perspective_Camera camera;
static Light* lights;
static Constraint* constraints;
static r64 thrown_objects_initial_linear_velocity_norm = 15.0;

// Mouse binding to target positions
static boolean is_mouse_bound_to_entity_movement;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0, 5.0, 15.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	return camera;
}

static void reset_joint_distance(Entity* e1, Entity* e2, vec3 r1_lc, vec3 r2_lc) {
	mat3 e1_rot_matrix = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot_matrix = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1_wc = gm_mat3_multiply_vec3(&e1_rot_matrix, r1_lc);
	vec3 r2_wc = gm_mat3_multiply_vec3(&e2_rot_matrix, r2_lc);
	vec3 p1 = gm_vec3_add(e1->world_position, r1_wc);
	vec3 p2 = gm_vec3_add(e2->world_position, r2_wc);
	vec3 delta_r = gm_vec3_subtract(p1, p2);
	vec3 delta_x = delta_r;
	entity_set_position(e2, gm_vec3_add(e2->world_position, delta_x));
}

int ex_debug_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = examples_util_create_lights();
	
	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 cube_scale = (vec3){1.0, 1.0, 1.0};
	Collider* cube_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, cube_scale);
	cube_eid = entity_create(cube_mesh, (vec3){0.0, 2.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0),
		cube_scale, (vec4){1.0, 0.3, 0.3, 1.0}, 1.0, cube_colliders, 0.5, 0.5, 0.0);

	Mesh cube2_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 cube2_scale = (vec3){1.0, 1.0, 1.0};
	Collider* cube2_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, cube2_scale);
	cube2_eid = entity_create(cube2_mesh, (vec3){0.0, 3.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0),
		cube2_scale, (vec4){0.3, 1.0, 0.3, 1.0}, 1.0, cube2_colliders, 0.5, 0.5, 0.0);

	array_free(cube_vertices);
	array_free(cube_indices);

	return 0;
}

void ex_debug_destroy() {
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

Collider_Contact* contacts = NULL;

void ex_debug_update(r64 delta_time) {
	Entity* cube_entity = entity_get_by_id(cube_eid);
	Entity* cube2_entity = entity_get_by_id(cube2_eid);
	colliders_update(cube_entity->colliders, cube_entity->world_position, &cube_entity->world_rotation);
	colliders_update(cube2_entity->colliders, cube2_entity->world_position, &cube2_entity->world_rotation);

	contacts = colliders_get_contacts(cube_entity->colliders, cube2_entity->colliders);
}

void ex_debug_render() {
	Entity* cube_entity = entity_get_by_id(cube_eid);
	Entity* cube2_entity = entity_get_by_id(cube2_eid);

	graphics_entity_render_phong_shader(&camera, cube_entity, lights);
	graphics_entity_render_phong_shader(&camera, cube2_entity, lights);

	if (!contacts) {
		return;
	}	

	for (u32 i = 0; i < array_length(contacts); ++i) {
		Collider_Contact* contact = &contacts[i];
		vec3 cp1 = contact->collision_point1;
		vec3 cp2 = contact->collision_point2;
		vec3 normal = contact->normal;
		graphics_renderer_debug_points(&cp1, 1, (vec4){1.0, 1.0, 1.0, 1.0});
		graphics_renderer_debug_points(&cp2, 1, (vec4){1.0, 1.0, 1.0, 1.0});
		graphics_renderer_debug_vector(cp1, gm_vec3_add(cp1, normal), (vec4){1.0, 1.0, 1.0, 1.0});
		graphics_renderer_debug_vector(cp2, gm_vec3_add(cp2, normal), (vec4){1.0, 1.0, 1.0, 1.0});
	}

	graphics_renderer_primitives_flush(&camera);
}

void ex_debug_input_process(boolean* key_state, r64 delta_time) {
	r64 movement_speed = 30.0;
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

	if (key_state[GLFW_KEY_1]) {
		is_mouse_bound_to_entity_movement = true;
	} else {
		is_mouse_bound_to_entity_movement = false;
	}

	if (key_state[GLFW_KEY_SPACE]) {
		examples_util_throw_object(&camera, thrown_objects_initial_linear_velocity_norm);
		key_state[GLFW_KEY_SPACE] = false;
	}
}

void ex_debug_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
	static r64 x_pos_old, y_pos_old;

	r64 x_difference = x_pos - x_pos_old;
	r64 y_difference = y_pos - y_pos_old;

	x_pos_old = x_pos;
	y_pos_old = y_pos;

	if (reset) return;

	if (is_mouse_bound_to_entity_movement) {
		// MOVE TARGET POSITIONS!
		vec3 camera_y = camera_get_y_axis(&camera);
		vec3 camera_x = camera_get_x_axis(&camera);

		static const r64 target_point_move_speed = 0.001;
		vec3 y_diff = gm_vec3_scalar_product(-target_point_move_speed * (r64)y_difference, camera_y);
		vec3 x_diff = gm_vec3_scalar_product(target_point_move_speed * (r64)x_difference, camera_x);

		Entity** entities = entity_get_all();
		vec3 position = entities[1]->world_position;
		position = gm_vec3_add(position, y_diff);
		position = gm_vec3_add(position, x_diff);
		entity_set_position(entities[1], position);
		array_free(entities);
	} else {
		// NORMAL CAMERA MOVEMENT!
		static const r64 camera_mouse_speed = 0.1;
		camera_rotate_x(&camera, camera_mouse_speed * (r64)x_difference);
		camera_rotate_y(&camera, camera_mouse_speed * (r64)y_difference);
	}
}

void ex_debug_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_debug_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_debug_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_debug_menu_update() {
	ImGui::Text("Debug");
	ImGui::Separator();

	ImGui::TextWrapped("Press SPACE to throw objects!");
	ImGui::TextWrapped("Thrown objects initial linear velocity norm:");
	r32 vel = (r32)thrown_objects_initial_linear_velocity_norm;
	ImGui::SliderFloat("Vel", &vel, 1.0f, 30.0f, "%.2f");
	thrown_objects_initial_linear_velocity_norm = vel;
}

Example_Scene debug_example_scene = (Example_Scene) {
	.name = "Debug",
	.init = ex_debug_init,
	.destroy = ex_debug_destroy,
	.input_process = ex_debug_input_process,
	.menu_properties_update = ex_debug_menu_update,
	.mouse_change_process = ex_debug_mouse_change_process,
	.mouse_click_process = ex_debug_mouse_click_process,
	.render = ex_debug_render,
	.scroll_change_process = ex_debug_scroll_change_process,
	.update = ex_debug_update,
	.window_resize_process = ex_debug_window_resize_process
};