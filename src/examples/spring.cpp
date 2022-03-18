#include "spring.h"
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
static Constraint* constraints;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0, 3.0, 10.0 };
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

int ex_spring_init() {
	entity_module_init();

	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	Vertex* cube_vertices;
	u32* cube_indices;
	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);

	vec3 floor_scale = (vec3){50.0, 1.0, 50.0};
	Collider* floor_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, floor_scale);
	entity_create_fixed(cube_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_colliders);

	vec3 attachment_scale = (vec3){0.1, 0.1, 0.1};
	Collider* attachment_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, attachment_scale);
	eid attachment_eid = entity_create_fixed(cube_mesh, (vec3){0.0, 6.0, 0.0}, quaternion_new((vec3){1.0, 1.0, 1.0}, 33.0),
		attachment_scale, (vec4){1.0, 1.0, 1.0, 1.0}, attachment_colliders);

	vec3 cube_scale = (vec3){1.0, 1.0, 1.0};
	Collider* cube_colliders = examples_util_create_single_convex_hull_collider_array(cube_vertices, cube_indices, cube_scale);
	eid cube_eid = entity_create(cube_mesh, (vec3){0.0, 2.0, 0.0}, quaternion_new((vec3){1.0, 1.0, 1.0}, 33.0),
		cube_scale, (vec4){1.0, 1.0, 1.0, 1.0}, 1.0, cube_colliders);

	array_free(cube_vertices);
	array_free(cube_indices);

	constraints = array_new(Constraint);
	Constraint constraint;
	pbd_positional_constraint_init(&constraint, cube_eid, attachment_eid, (vec3) { 0.0, 0.0, 0.0 }, (vec3) { 0.0, 0.0, 0.0 }, 0.001, (vec3) { 0.0, -3.0, 0.0 });
	array_push(constraints, constraint);

	return 0;
}

void ex_spring_destroy() {
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
	array_free(constraints);

	entity_module_destroy();
}

void ex_spring_update(r64 delta_time) {
	Entity** entities = entity_get_all();

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		colliders_update(e->colliders, e->world_position, &e->world_rotation);
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

void ex_spring_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_spring_input_process(boolean* key_state, r64 delta_time) {
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
		examples_util_throw_object(&camera);
		key_state[GLFW_KEY_SPACE] = false;
	}
}

void ex_spring_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_spring_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_spring_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_spring_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_spring_menu_update() {
	ImGui::Text("Spring");

	r32 compliance = (r32)constraints[0].positional_constraint.compliance;
	ImGui::SliderFloat("Compliance", &compliance, 0.0f, 1.0f, "%.4f");
	constraints[0].positional_constraint.compliance = (r64)compliance;
	ImGui::Separator();
	ImGui::TextWrapped("Press SPACE to throw objects!");
}

Example_Scene spring_example_scene = (Example_Scene) {
	.name = "Spring",
	.init = ex_spring_init,
	.destroy = ex_spring_destroy,
	.input_process = ex_spring_input_process,
	.menu_properties_update = ex_spring_menu_update,
	.mouse_change_process = ex_spring_mouse_change_process,
	.mouse_click_process = ex_spring_mouse_click_process,
	.render = ex_spring_render,
	.scroll_change_process = ex_spring_scroll_change_process,
	.update = ex_spring_update,
	.window_resize_process = ex_spring_window_resize_process
};