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

static Perspective_Camera camera;
static Light* lights;

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
	Collider collider = collider_convex_hull_create(vertices_positions, indices);
	array_free(vertices_positions);
	return collider;
}

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

int ex_stack_init() {
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
	Collider floor_collider = create_collider(cube_vertices, cube_indices, floor_scale);
	entity_create_fixed(cube_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_collider);

	const r64 brick_height = 0.35;
	const r64 brick_width = 0.8;
	r64 y = -1.0;
	r64 x;
	for (u32 i = 0; i < 8; ++i) {
		y += 2 * brick_height + 0.01;
		if (i % 2 == 0) {
			x = -2.0;
		} else {
			x = -2.0 + brick_width / 2;
		}
		for (u32 j = 0; j < 8; ++j) {
			vec3 cube_scale = (vec3){brick_width, brick_height, brick_height};
			Collider cube_collider = create_collider(cube_vertices, cube_indices, cube_scale);
			vec4 color = ((i + j) % 2 == 0) ?
				(vec4) { 188.0 / 255.0, 74.0 / 255.0, 60.0 / 255.0, 1.0 } :
				(vec4) { 168.0 / 255.0, 64.0 / 255.0, 50.0 / 255.0, 1.0 };
			entity_create(cube_mesh, (vec3){x, y, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
				cube_scale, color, 1.0, cube_collider);
			x += 2 * brick_width + 0.01;
		}
		x = -2.0;
	}

	array_free(cube_vertices);
	array_free(cube_indices);

	return 0;
}

void ex_stack_destroy() {
	array_free(lights);

	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		collider_destroy(&e->collider);
		mesh_destroy(&e->mesh);
		entity_destroy(e);
	}
	array_free(entities);
	entity_module_destroy();
}

void ex_stack_update(r64 delta_time) {
	//printf("(Quaternion){%f, %f, %f, %f}\n", camera.rotation.x, camera.rotation.y, camera.rotation.z, camera.rotation.w);
	//printf("(Quaternion){%f, %f, %f, %f}\n", camera.yrotation.x, camera.yrotation.y, camera.yrotation.z, camera.yrotation.w);
	//printf("(vec3){%f, %f, %f}\n", camera.position.x, camera.position.y, camera.position.z);
	delta_time = 0.016666667; // ~60fps

	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = entities[i];
		collider_update(&e->collider, e->world_position, &e->world_rotation);
	}

	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
		Physics_Force pf;
		pf.force = (vec3){0.0, -GRAVITY * 1.0 / entities[i]->inverse_mass, 0.0};
		pf.position = (vec3){0.0, 0.0, 0.0};
		array_push(entities[i]->forces, pf);
	}

	pbd_simulate(delta_time, entities);

	for (u32 i = 0; i < array_length(entities); ++i) {
		array_clear(entities[i]->forces);
	}

	array_free(entities);
}

void ex_stack_render() {
	Entity** entities = entity_get_all();
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
	array_free(entities);
}

void ex_stack_input_process(boolean* key_state, r64 delta_time) {
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
		eid id = entity_create(m, cube_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 0.0),
			scale, (vec4){rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, 1.0}, 10.0, collider);
		array_free(vertices);
		array_free(indices);

		Entity* e = entity_get_by_id(id);
		e->linear_velocity = gm_vec3_scalar_product(10.0, gm_vec3_scalar_product(-1.0, camera_z));

		key_state[GLFW_KEY_SPACE] = false;
	}
}

void ex_stack_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

void ex_stack_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void ex_stack_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void ex_stack_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}

void ex_stack_menu_update() {
	ImGui::Text("Brick Wall");
	ImGui::Separator();
	ImGui::TextWrapped("Press SPACE to throw objects!");
}

Example_Scene stack_example_scene = (Example_Scene) {
	.name = "Brick Wall",
	.init = ex_stack_init,
	.destroy = ex_stack_destroy,
	.input_process = ex_stack_input_process,
	.menu_properties_update = ex_stack_menu_update,
	.mouse_change_process = ex_stack_mouse_change_process,
	.mouse_click_process = ex_stack_mouse_click_process,
	.render = ex_stack_render,
	.scroll_change_process = ex_stack_scroll_change_process,
	.update = ex_stack_update,
	.window_resize_process = ex_stack_window_resize_process
};