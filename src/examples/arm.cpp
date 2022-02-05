#include "arm.h"
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
static Constraint* constraints;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0, 5.0, 15.0 };
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

static Constraint* create_arm() {
	Vertex* cube_vertices;
	u32* cube_indices;

	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 support_position = (vec3){0.0, 15.0, 0.0};
	vec3 support_collider_scale = (vec3){0.2, 0.1, 0.1};
	Collider* support_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, support_collider_scale);
	eid support_id = entity_create_fixed(cube_mesh, support_position, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0), support_collider_scale,
		(vec4){0.0, 1.0, 0.0, 1.0}, support_colliders);

	vec3 upper_arm_collider_scale = (vec3){0.2, 1.0, 0.1};
	Collider* upper_arm_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, upper_arm_collider_scale);
	eid upper_arm_id = entity_create(cube_mesh, (vec3){0.0, 0.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0), upper_arm_collider_scale,
		(vec4){1.0, 1.0, 0.0, 1.0}, 1.0, upper_arm_colliders);

	vec3 lower_arm_collider_scale = (vec3){0.15, 1.0, 0.1};
	Collider* lower_arm_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, lower_arm_collider_scale);
	eid lower_arm_id = entity_create(cube_mesh, (vec3){0.0, 0.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0), lower_arm_collider_scale,
		(vec4){1.0, 1.0, 0.0, 1.0}, 1.0, lower_arm_colliders);

	vec3 hand_collider_scale = (vec3){0.3, 0.3, 0.1};
	Collider* hand_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, hand_collider_scale);
	eid hand_id = entity_create(cube_mesh, (vec3){0.0, 0.0, 0.0}, quaternion_new((vec3){1.0, 0.0, 0.0}, 0.0), hand_collider_scale,
		(vec4){1.0, 1.0, 0.0, 1.0}, 1.0, hand_colliders);

	array_free(cube_vertices);
	array_free(cube_indices);

	Constraint* constraints = array_new(Constraint);
	Constraint constraint;

	Entity* support_entity = entity_get_by_id(support_id);
	Entity* upper_arm_entity = entity_get_by_id(upper_arm_id);
	Entity* lower_arm_entity = entity_get_by_id(lower_arm_id);
	Entity* hand_entity = entity_get_by_id(hand_id);

	vec3 r1_lc, r2_lc;

	// Support - Upper Arm Joint (Shoulder)
	r1_lc = (vec3){0.0, -10.0, 0.0};
	r2_lc = (vec3){0.0, 1.0, 0.0};
	reset_joint_distance(support_entity, upper_arm_entity, r1_lc, r2_lc);
	pbd_spherical_joint_constraint_init(&constraint, support_id, upper_arm_id, r1_lc, r2_lc, PBD_POSITIVE_X_AXIS, PBD_POSITIVE_X_AXIS, PBD_POSITIVE_Y_AXIS, PBD_POSITIVE_Y_AXIS,
		-PI_F, PI_F, -PI_F, PI_F);
	array_push(constraints, constraint);

	// Upper Arm - Lower Arm Joint (Elbow)
	r1_lc = (vec3){0.0, -1.2, 0.0};
	r2_lc = (vec3){0.0, 1.2, 0.0};
	reset_joint_distance(upper_arm_entity, lower_arm_entity, r1_lc, r2_lc);
	pbd_hinge_joint_constraint_limited_init(&constraint, upper_arm_id, lower_arm_id, r1_lc, r2_lc, 0.0, PBD_POSITIVE_X_AXIS, PBD_POSITIVE_X_AXIS, PBD_POSITIVE_Y_AXIS, PBD_POSITIVE_Y_AXIS,
		0.0, 0.9 * PI_F);
	array_push(constraints, constraint);

	// Lower Arm - Hand Arm Joint (Wrist)
	r1_lc = (vec3){0.0, -1.2, 0.0};
	r2_lc = (vec3){0.0, 0.5, 0.0};
	reset_joint_distance(lower_arm_entity, hand_entity, r1_lc, r2_lc);
	pbd_spherical_joint_constraint_init(&constraint, lower_arm_id, hand_id, r1_lc, r2_lc, PBD_POSITIVE_X_AXIS, PBD_POSITIVE_X_AXIS, PBD_POSITIVE_Y_AXIS, PBD_POSITIVE_Y_AXIS,
		-0.0 * PI_F, 0.0 * PI_F, -0.3 * PI_F, 0.05 * PI_F);
	array_push(constraints, constraint);

	return constraints;
}

int ex_arm_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	//Vertex* floor_vertices;
	//u32* floor_indices;
	//obj_parse("./res/floor.obj", &floor_vertices, &floor_indices);
	//Mesh floor_mesh = graphics_mesh_create(floor_vertices, floor_indices);
	//vec3 floor_scale = (vec3){1.0, 1.0, 1.0};
	//Collider* floor_colliders = examples_util_create_single_convex_hull_collider_array(floor_vertices, floor_indices, floor_scale);
	//entity_create_fixed(floor_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
	//	floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_collider);
	//array_free(floor_vertices);
	//array_free(floor_indices);

	constraints = create_arm();

	return 0;
}

void ex_arm_destroy() {
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

void ex_arm_update(r64 delta_time) {
#ifdef LIMIT_60FPS
	delta_time = 0.016666667; // ~60fps
#endif

	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		colliders_update(e->colliders, e->world_position, &e->world_rotation);
		//printf("e%d: <%.50f, %.50f, %.50f>\n", i, e->world_position.x, e->world_position.y, e->world_position.z);
		//printf("e%d: rot: <%.50f, %.50f, %.50f, %.50f>\n", i, e->world_rotation.x, e->world_rotation.y, e->world_rotation.z, e->world_rotation.w);
	}

	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
		entity_add_force(entities[i], (vec3){0.0, 0.0, 0.0}, (vec3){0.0, -GRAVITY * 1.0 / entities[i]->inverse_mass, 0.0}, false);
	}

	pbd_simulate_with_constraints(delta_time, entities, constraints, 20, 1, true);

	for (u32 i = 0; i < array_length(entities); ++i) {
		entity_clear_forces(entities[i]);
	}
	array_free(entities);
}

void ex_arm_render() {
	Entity** entities = entity_get_all();

	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_arm_input_process(boolean* key_state, r64 delta_time) {
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

	if (key_state[GLFW_KEY_SPACE]) {
		examples_util_throw_object(&camera);
		key_state[GLFW_KEY_SPACE] = false;
	}
}

void ex_arm_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_arm_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_arm_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_arm_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_arm_menu_update() {
	ImGui::Text("Arm");
	ImGui::Separator();
	ImGui::TextWrapped("Press SPACE to throw objects!");
}

Example_Scene arm_example_scene = (Example_Scene) {
	.name = "Arm",
	.init = ex_arm_init,
	.destroy = ex_arm_destroy,
	.input_process = ex_arm_input_process,
	.menu_properties_update = ex_arm_menu_update,
	.mouse_change_process = ex_arm_mouse_change_process,
	.mouse_click_process = ex_arm_mouse_click_process,
	.render = ex_arm_render,
	.scroll_change_process = ex_arm_scroll_change_process,
	.update = ex_arm_update,
	.window_resize_process = ex_arm_window_resize_process
};