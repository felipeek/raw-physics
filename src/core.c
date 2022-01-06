#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "render/graphics.h"
#include "render/obj.h"
#include "render/menu.h"
#include "physics/gjk.h"
#include "physics/epa.h"
#include "physics/clipping.h"
#include "physics/pbd.h"

#define GIM_ENTITY_COLOR (vec4) {1.0, 1.0, 1.0, 1.0}

static Perspective_Camera camera;
static Light* lights;
static Entity* entities;

// Mouse binding to target positions
static boolean is_mouse_bound_to_entity_movement;

static Perspective_Camera create_camera() {
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 5000.0, 3.0, 5010.0 };
	r64 camera_near_plane = -0.01;
	r64 camera_far_plane = -1000.0;
	r64 camera_fov = 45.0;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	return camera;
}

static Light* create_lights() {
	Light light;
	Light* lights = array_new(Light);

	vec3 light_position = (vec3) {5000.0, 2.0, 5005.0};
	vec4 ambient_color = (vec4) {0.1, 0.1, 0.1, 1.0};
	vec4 diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0};
	vec4 specular_color = (vec4) {0.5, 0.5, 0.5, 1.0};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	light_position = (vec3) {0.0, 0.0, 15.0};
	ambient_color = (vec4) {0.1, 0.1, 0.1, 1.0};
	diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0};
	specular_color = (vec4) {0.5, 0.5, 0.5, 1.0};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	return lights;
}

static void menu_dummy_callback() {
	printf("dummy callback called!\n");
}

int core_init() {
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	entities = array_new(Entity);

	Entity e;
	Mesh m = graphics_mesh_create_from_obj("./res/floor.obj", COLLIDER_TYPE_CONVEX_HULL);
	graphics_entity_create_with_color_fixed(&e, m, (vec3){5000.0, -2.0, 5000.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		(vec3){1.0, 1.0, 1.0}, (fvec4){1.0f, 1.0f, 1.0f, 1.0f});
	array_push(entities, e);

	r64 y = -2.0;

	for (u32 i = 0; i < 1; ++i) {
		y += 3.0;
		Mesh m2 = graphics_mesh_create_from_obj("./res/cube.obj", COLLIDER_TYPE_CONVEX_HULL);
		graphics_entity_create_with_color(&e, m2, (vec3){5000.0, y, 5000.0}, quaternion_new((vec3){1.0, 1.0, 1.0}, 20.0),
			(vec3){1.0, 1.0, 1.0}, (fvec4){1.0f, 0.0f, 0.0f, 1.0f}, 1.0);
		//e.world_position = (vec3){-0.000944963249, -0.0000153643723, -0.00111608475};
		//e.world_rotation = (Quaternion){0.00000165915253, -0.00146911771, -9.74293812E-7, 0.999998986};
		array_push(entities, e);
	}

	menu_register_dummy_callback(menu_dummy_callback);

	return 0;
}

void core_destroy() {
	array_free(lights);
}

boolean paused = false;

void core_update(r64 delta_time) {
	delta_time = 0.02;
	//delta_time = 0.01;
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		mat4 model_matrix = graphics_entity_get_model_matrix(e);
		collider_update(&e->mesh.collider, model_matrix);

		//printf("e%d: <%f, %f, %f>\n", i, e->world_position.x, e->world_position.y, e->world_position.z);
		//printf("e%d: rot: <%f, %f, %f, %f>\n", i, e->world_rotation.x, e->world_rotation.y, e->world_rotation.z, e->world_rotation.w);
	}

	if (paused) {
		return;
	}

#if 1
	const r64 GRAVITY = 10.0;
	for (u32 i = 0; i < array_length(entities); ++i) {
		Physics_Force pf;
		pf.force = (vec3){0.0, -GRAVITY * 1.0 / entities[i].inverse_mass, 0.0};
		pf.position = (vec3){0.0, 0.0, 0.0};
		array_push(entities[i].forces, pf);
	}

	pbd_simulate(delta_time, entities);

	for (u32 i = 0; i < array_length(entities); ++i) {
		array_clear(entities[i].forces);
	}
#endif
}

extern Temporary_Contact* tmp_contacts;

void core_render() {
	#if 1
	if (tmp_contacts) {
		for (u32 i = 0; i < array_length(tmp_contacts); ++i) {
			Temporary_Contact* tmp = &tmp_contacts[i];

			vec3 point1 = gm_vec3_add(tmp->e1->world_position, tmp->r1_wc);
			vec3 point2 = gm_vec3_add(tmp->e2->world_position, tmp->r2_wc);
			vec3 normal = tmp->normal;

			graphics_renderer_debug_points(&point1, 1, (vec4){1.0, 1.0, 1.0, 1.0});
			graphics_renderer_debug_points(&point2, 1, (vec4){1.0, 1.0, 1.0, 1.0});
			graphics_renderer_debug_vector(point1, gm_vec3_add(point1, normal), (vec4){1.0, 1.0, 1.0, 1.0});
			graphics_renderer_debug_vector(point2, gm_vec3_add(point2, normal), (vec4){1.0, 1.0, 1.0, 1.0});
		}
	}
	#else
	Entity* e1 = &entities[0];
	Entity* e2 = &entities[1];
	GJK_Simplex simplex;
	Clipping_Contact* contacts;
	vec3 normal;
	boolean collision = false;
	if (gjk_collides(e1->mesh.collider.convex_hull.transformed_vertices, e2->mesh.collider.convex_hull.transformed_vertices, &simplex)) {
		r64 pen;
		if (epa(e1->mesh.collider.convex_hull.transformed_vertices, e2->mesh.collider.convex_hull.transformed_vertices, &simplex, &normal, &pen)) {
			e1->diffuse_info.diffuse_color = (vec4){0.0, 1.0, 0.0, 1.0};
			e2->diffuse_info.diffuse_color = (vec4){0.0, 1.0, 0.0, 1.0};

			contacts = clipping_get_contact_manifold(&e1->mesh.collider.convex_hull, &e2->mesh.collider.convex_hull, normal);
			collision = true;
		}
	} else {
		e1->diffuse_info.diffuse_color = (vec4){1.0, 0.0, 0.0, 1.0};
		e2->diffuse_info.diffuse_color = (vec4){1.0, 0.0, 0.0, 1.0};
	}
	if (collision) {
		for (u32 i = 0; i < array_length(contacts); ++i) {
			Clipping_Contact* contact = &contacts[i];

			vec3 cp1 = contact->collision_point1;
			vec3 cp2 = contact->collision_point2;
			graphics_renderer_debug_points(&cp1, 1, (vec4){1.0, 1.0, 1.0, 1.0});
			graphics_renderer_debug_points(&cp2, 1, (vec4){1.0, 1.0, 1.0, 1.0});
			graphics_renderer_debug_vector(cp1, gm_vec3_add(cp1, normal), (vec4){1.0, 1.0, 1.0, 1.0});
			graphics_renderer_debug_vector(cp2, gm_vec3_add(cp2, normal), (vec4){1.0, 1.0, 1.0, 1.0});
		}
	}
	#endif

	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, &entities[i], lights);
	}

    //graphics_renderer_primitives_flush(&camera);
}

void core_input_process(boolean* key_state, r64 delta_time) {
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
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0, 0.0, 0.0}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0, 0.0, 0.0}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 1.0, 0.0}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 1.0, 0.0}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 0.0, 1.0}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 0.0, 1.0}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
	}
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
	if (key_state[GLFW_KEY_1]) {
		is_mouse_bound_to_entity_movement = true;
	} else {
		is_mouse_bound_to_entity_movement = false;
	}

	if (key_state[GLFW_KEY_SPACE]) {
		vec3 camera_z = camera_get_z_axis(&camera);
		vec3 camera_pos = camera.position;
		r64 distance = 5.0;
		vec3 diff = gm_vec3_scalar_product(-distance, camera_z);
		vec3 cube_position = gm_vec3_add(camera_pos, diff);

		Entity e;
		char* mesh_name;
		int r = rand();
		if (r % 3 == 0) {
			mesh_name = "./res/ico.obj";
		} else if (r % 3 == 1) {
			mesh_name = "./res/ico.obj";
		} else {
			mesh_name = "./res/ico.obj";
		}
		Mesh m = graphics_mesh_create_from_obj(mesh_name, 0);
		//graphics_entity_create_with_color(&e, m, cube_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 33.0),
		//	(vec3){1.0, 1.0, 1.0}, (vec4){rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, 1.0}, 10.0);
		graphics_entity_create_with_color(&e, m, cube_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 0.0),
			(vec3){1.0, 1.0, 1.0}, (fvec4){rand() / (r32)RAND_MAX, rand() / (r32)RAND_MAX, rand() / (r32)RAND_MAX, 1.0f}, 1.0);

		//Physics_Force force;
		//force.force = gm_vec3_scalar_product(15000.0, gm_vec3_scalar_product(-1.0, camera_z));
		//force.position = (vec3) {0.0, 0.0, 0.0};
		//array_push(e.forces, force);

		e.linear_velocity = gm_vec3_scalar_product(10.0, gm_vec3_scalar_product(-1.0, camera_z));

		//if (array_length(entities) == 2) {
		//	entities[1] = e;
		//} else {
			array_push(entities, e);
		//}

		key_state[GLFW_KEY_SPACE] = false;
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
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

		vec3 position = entities[1].world_position;
		position = gm_vec3_add(position, y_diff);
		position = gm_vec3_add(position, x_diff);
		graphics_entity_set_position(&entities[1], position);
	} else {
		// NORMAL CAMERA MOVEMENT!
		static const r64 camera_mouse_speed = 0.1;
		camera_rotate_x(&camera, camera_mouse_speed * (r64)x_difference);
		camera_rotate_y(&camera, camera_mouse_speed * (r64)y_difference);
	}
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {

}

void core_scroll_change_process(r64 x_offset, r64 y_offset) {

}

void core_window_resize_process(s32 width, s32 height) {
	camera_force_matrix_recalculation(&camera);
}