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

static Perspective_Camera camera;
static Light* lights;
static Constraint* static_constraints;

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

eid support_id, lever_id;
vec3 e1_a = (vec3){1.0, 0.0, 0.0};
vec3 e2_a = (vec3){1.0, 0.0, 0.0};

int ex_debug_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	Vertex* floor_vertices;
	u32* floor_indices;
	obj_parse("./res/floor.obj", &floor_vertices, &floor_indices);
	Mesh floor_mesh = graphics_mesh_create(floor_vertices, floor_indices);
	vec3 floor_scale = (vec3){1.0, 1.0, 1.0};
	Collider* floor_colliders = examples_util_create_single_convex_hull_collider_array(floor_vertices, floor_indices, floor_scale);
	//entity_create_fixed(floor_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
	//	floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_collider);
	array_free(floor_vertices);
	array_free(floor_indices);

	Vertex* cube_vertices;
	u32* cube_indices;

	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);
	vec3 cube_scale = (vec3){1.0, 1.0, 1.0};

	vec3 support_collider_scale = (vec3){0.25, 0.25, 0.25};
	Collider* support_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, support_collider_scale);
	#if 0
	support_id = entity_create(cube_mesh, (vec3){0.0, 1.0, 0.0}, quaternion_new((vec3){0.0, -1.0, 0.0}, 0.0),
		support_collider_scale, (vec4){0.0, 1.0, 0.0, 1.0}, 1.0, support_colliders);
	#else
	support_id = entity_create_fixed(cube_mesh, (vec3){0.0, 5.0, 0.0}, quaternion_new((vec3){0.0, -1.0, 0.0}, 0.0),
		support_collider_scale, (vec4){0.0, 1.0, 0.0, 1.0}, support_colliders);
	#endif

	vec3 lever_collider_scale = (vec3){0.2, 1.0, 0.2};
	Collider* lever_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, lever_collider_scale);
	lever_id = entity_create(cube_mesh, (vec3){0.0, 0.5, 0.0}, quaternion_new((vec3){0.0, -1.0, 0.0}, 0.0),
		lever_collider_scale, (vec4){1.0, 1.0, 0.0, 1.0}, 1.0, lever_colliders);

	array_free(cube_vertices);
	array_free(cube_indices);

	Entity* support_entity = entity_get_by_id(support_id);
	Entity* lever_entity = entity_get_by_id(lever_id);

	static_constraints = array_new(Constraint);
	Constraint static_constraint;

	// Make sure that support and lever starts with the correct distance, otherwise simulation will explode in the 1st frame
	vec3 r1_lc = (vec3){0.0, 0.0, 0.0};
	vec3 r1_wc = r1_lc; // considering no rotation yet
	vec3 r2_lc = (vec3){0.0, 1.5, 0.0};
	vec3 r2_wc = r2_lc; // considering no rotation yet
	vec3 p1 = gm_vec3_add(support_entity->world_position, r1_wc);
	vec3 p2 = gm_vec3_add(lever_entity->world_position, r2_wc);
	vec3 delta_r = gm_vec3_subtract(p1, p2);
	vec3 delta_x = delta_r;
	entity_set_position(lever_entity, gm_vec3_add(lever_entity->world_position, delta_x));

	static_constraint.type = HINGE_JOINT_STATIC_CONSTRAINT;
	static_constraint.hinge_joint_constraint.e1_id = support_id;
	static_constraint.hinge_joint_constraint.e2_id = lever_id;
	static_constraint.hinge_joint_constraint.compliance = 0.0;
	static_constraint.hinge_joint_constraint.r1_lc = r1_lc;
	static_constraint.hinge_joint_constraint.r2_lc = r2_lc;
	static_constraint.hinge_joint_constraint.e1_a = e1_a;
	static_constraint.hinge_joint_constraint.e2_a = e2_a;
	array_push(static_constraints, static_constraint);

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

boolean paused = false;

void ex_debug_update(r64 delta_time) {
	delta_time = 0.016666667; // ~60fps

	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		colliders_update(e->colliders, e->world_position, &e->world_rotation);
		//printf("e%d: <%.50f, %.50f, %.50f>\n", i, e->world_position.x, e->world_position.y, e->world_position.z);
		//printf("e%d: rot: <%.50f, %.50f, %.50f, %.50f>\n", i, e->world_rotation.x, e->world_rotation.y, e->world_rotation.z, e->world_rotation.w);
	}

	if (paused) {
		return;
	}

#if 1
	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
		Physics_Force pf;
		pf.force = (vec3){0.0, -GRAVITY * 1.0 / entities[i]->inverse_mass, 0.0};
		pf.position = (vec3){0.0, 0.0, 0.0};
		//array_push(entities[i]->forces, pf);
	}

	pbd_simulate_with_static_constraints(delta_time, entities, static_constraints);

	for (u32 i = 0; i < array_length(entities); ++i) {
		array_clear(entities[i]->forces);
	}
#endif

	array_free(entities);
}

void ex_debug_render() {
	Entity** entities = entity_get_all();

	#if 0
	for (u32 i = 0; i < array_length(entities); ++i) {
		for (u32 j = i + 1; j < array_length(entities); ++j) {
			Entity* e1 = entities[i];
			Entity* e2 = entities[j];
			GJK_Simplex simplex;
			vec3 normal;
			boolean collision = false;
			Collider_Contact* contacts = collider_get_contacts(&e1->collider, &e2->collider, &normal);

			if (contacts && array_length(contacts) > 0) {
				for (u32 i = 0; i < array_length(contacts); ++i) {
					Collider_Contact* contact = &contacts[i];

					vec3 cp1 = contact->collision_point1;
					vec3 cp2 = contact->collision_point2;
					graphics_renderer_debug_points(&cp1, 1, (vec4){1.0, 1.0, 1.0, 1.0});
					graphics_renderer_debug_points(&cp2, 1, (vec4){1.0, 1.0, 1.0, 1.0});
					graphics_renderer_debug_vector(cp1, gm_vec3_add(cp1, normal), (vec4){1.0, 1.0, 1.0, 1.0});
					graphics_renderer_debug_vector(cp2, gm_vec3_add(cp2, normal), (vec4){1.0, 1.0, 1.0, 1.0});
				}

				e1->color = (vec4){0.0, 1.0, 0.0, 1.0};
				e2->color = (vec4){0.0, 1.0, 0.0, 1.0};
			} else {
				e1->color = (vec4){1.0, 0.0, 0.0, 1.0};
				e2->color = (vec4){1.0, 0.0, 0.0, 1.0};
			}
		}
	}
	#endif

	Entity* support_entity = entity_get_by_id(support_id);
	Entity* lever_entity = entity_get_by_id(lever_id);
	mat3 support_rot = quaternion_get_matrix3(&support_entity->world_rotation);
	mat3 lever_rot = quaternion_get_matrix3(&lever_entity->world_rotation);
	vec3 e1_a_wc = gm_mat3_multiply_vec3(&support_rot, e1_a);
	vec3 e2_a_wc = gm_mat3_multiply_vec3(&lever_rot, e2_a);
	graphics_renderer_debug_vector(support_entity->world_position, gm_vec3_add(support_entity->world_position, e1_a_wc),
		(vec4){1.0, 0.0, 0.0, 1.0});
	graphics_renderer_debug_vector(lever_entity->world_position, gm_vec3_add(lever_entity->world_position, e2_a_wc),
		(vec4){0.0, 0.0, 0.0, 1.0});

	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_debug_input_process(boolean* key_state, r64 delta_time) {
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

	Entity* e = entity_get_by_id(lever_id);

	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			entity_set_rotation(e, quaternion_product(&rotation, &e->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			entity_set_rotation(e, quaternion_product(&rotation, &e->world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			entity_set_rotation(e, quaternion_product(&rotation, &e->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			entity_set_rotation(e, quaternion_product(&rotation, &e->world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			entity_set_rotation(e, quaternion_product(&rotation, &e->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			entity_set_rotation(e, quaternion_product(&rotation, &e->world_rotation));
		}
	}

	if (key_state[GLFW_KEY_1]) {
		is_mouse_bound_to_entity_movement = true;
	} else {
		is_mouse_bound_to_entity_movement = false;
	}

	if (key_state[GLFW_KEY_SPACE]) {
		examples_util_throw_object(&camera);
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