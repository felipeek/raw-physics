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

#define GIM_ENTITY_COLOR (vec4) {1.0, 1.0, 1.0, 1.0}

static Perspective_Camera camera;
static Light* lights;
static Entity* entities;

// Mouse binding to target positions
static boolean is_mouse_bound_to_entity_movement;

static Collider create_convex_collider(Vertex* vertices, u32* indices, vec3 scale) {
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
	printf("Vertices positions: %ld\n", array_length(vertices_positions));
	array_free(vertices_positions);

	printf("Hull positions: %ld\n", array_length(collider.convex_hull.vertices));
	for (u32 i = 0; i < array_length(collider.convex_hull.faces); ++i) {
		Collider_Convex_Hull_Face face = collider.convex_hull.faces[i];
		printf("Face num elements: %ld\n", array_length(face.elements));
		printf("Face normal: <%.3f, %.3f, %.3f>\n", face.normal.x, face.normal.y, face.normal.z);
		//for (u32 j = 0; j < array_length(face.elements); ++j) {
		//	vec3 v = mesh.collider.convex_hull.vertices[face.elements[j]];
		//	printf("\tV: <%.3f, %.3f, %.3f> (elem: %d)\n", v.x, v.y, v.z, face.elements[j]);
		//}
	}

	return collider;
}


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

int ex_debug_init() {
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();
	
	Entity e;
	entities = array_new(Entity);

	Vertex* floor_vertices;
	u32* floor_indices;
	obj_parse("./res/floor.obj", &floor_vertices, &floor_indices);
	Mesh floor_mesh = graphics_mesh_create(floor_vertices, floor_indices);
	vec3 floor_scale = (vec3){1.0, 1.0, 1.0};
	Collider floor_collider = create_convex_collider(floor_vertices, floor_indices, floor_scale);
	entity_create_fixed(&e, floor_mesh, (vec3){0.0, -2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.0}, 0.0),
		floor_scale, (vec4){1.0, 1.0, 1.0, 1.0}, floor_collider);
	array_push(entities, e);
	array_free(floor_vertices);
	array_free(floor_indices);

	Vertex* cube_vertices;
	u32* cube_indices;
	Vertex* sphere_vertices;
	u32* sphere_indices;

	obj_parse("./res/cube.obj", &cube_vertices, &cube_indices);
	Mesh cube_mesh = graphics_mesh_create(cube_vertices, cube_indices);
	vec3 cube_scale = (vec3){1.0, 1.0, 1.0};

	obj_parse("./res/sphere.obj", &sphere_vertices, &sphere_indices);
	Mesh sphere_mesh = graphics_mesh_create(sphere_vertices, sphere_indices);
	vec3 sphere_scale = (vec3){1.0, 1.0, 1.0};

	//Collider cube_collider1 = create_convex_collider(cube_vertices, cube_indices, cube_scale);
	//entity_create(&e, cube_mesh, (vec3){0.0, 0.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.5}, 0.0),
	//	cube_scale, (vec4){1.0, 0.0, 0.0, 1.0}, 1.0, cube_collider1);
	//array_push(entities, e);

	//Collider cube_collider2 = create_convex_collider(cube_vertices, cube_indices, cube_scale);
	//entity_create(&e, cube_mesh, (vec3){0.0, 2.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.5}, 0.0),
	//	cube_scale, (vec4){0.5, 0.0, 0.0, 1.0}, 1.0, cube_collider2);
	//array_push(entities, e);

	Collider sphere_collider = collider_sphere_create(1.0);
	entity_create(&e, sphere_mesh, (vec3){0.0, 2.1, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.5}, 0.0),
		sphere_scale, (vec4){0.5, 0.0, 0.0, 1.0}, 1.0, sphere_collider);
	array_push(entities, e);

	Collider sphere_collider1 = collider_sphere_create(1.0);
	entity_create(&e, sphere_mesh, (vec3){0.0, 0.0, 0.0}, quaternion_new((vec3){0.0, 1.0, 0.5}, 0.0),
		sphere_scale, (vec4){1.0, 0.0, 0.0, 1.0}, 1.0, sphere_collider1);
	array_push(entities, e);

	array_free(cube_vertices);
	array_free(cube_indices);
	array_free(sphere_vertices);
	array_free(sphere_indices);

	return 0;
}

void ex_debug_destroy() {
	array_free(lights);
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		collider_destroy(&e->collider);
		mesh_destroy(&e->mesh);
		entity_destroy(e);
	}
	array_free(entities);
}

boolean paused = false;

void ex_debug_update(r64 delta_time) {
	delta_time = 0.016666667; // ~60fps

	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		collider_update(&e->collider, e->world_position, &e->world_rotation);
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

void ex_debug_render() {
	#if 1
	//vec3 point1 = _p1;
	//vec3 point2 = _p2;
	//vec3 normal = _n;

	//graphics_renderer_debug_points(&point1, 1, (vec4){1.0, 1.0, 1.0, 1.0});
	//graphics_renderer_debug_points(&point2, 1, (vec4){1.0, 1.0, 1.0, 1.0});
	//graphics_renderer_debug_vector(point1, gm_vec3_add(point1, normal), (vec4){1.0, 1.0, 1.0, 1.0});
	//graphics_renderer_debug_vector(point2, gm_vec3_add(point2, normal), (vec4){1.0, 1.0, 1.0, 1.0});
	#else
	Entity* e1 = &entities[0];
	Entity* e2 = &entities[1];
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
	#endif

	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, &entities[i], lights);
	}

	graphics_renderer_primitives_flush(&camera);
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
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0, 0.0, 0.0}, rotation_speed * delta_time);
			entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0, 0.0, 0.0}, -rotation_speed * delta_time);
			entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 1.0, 0.0}, rotation_speed * delta_time);
			entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 1.0, 0.0}, -rotation_speed * delta_time);
			entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 0.0, 1.0}, rotation_speed * delta_time);
			entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0, 0.0, 1.0}, -rotation_speed * delta_time);
			entity_set_rotation(&entities[1], quaternion_product(&rotation, &entities[1].world_rotation));
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
		Collider collider = create_convex_collider(vertices, indices, scale);
		entity_create(&e, m, cube_position, quaternion_new((vec3){0.35, 0.44, 0.12}, 0.0),
			scale, (vec4){rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, rand() / (r64)RAND_MAX, 1.0}, 1.0, collider);
		array_free(vertices);
		array_free(indices);

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

		vec3 position = entities[1].world_position;
		position = gm_vec3_add(position, y_diff);
		position = gm_vec3_add(position, x_diff);
		entity_set_position(&entities[1], position);
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
	ImGui::TextWrapped("Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. ");
}