#include "camera.h"
#include <math.h>

extern s32 window_width;
extern s32 window_height;

static void recalculate_view_matrix(Perspective_Camera* camera) {
	mat4 trans = (mat4) {
		1.0, 0.0, 0.0, -camera->position.x,
		0.0, 1.0, 0.0, -camera->position.y,
		0.0, 0.0, 1.0, -camera->position.z,
		0.0, 0.0, 0.0, 1.0
	};

	Quaternion f = quaternion_product(&camera->rotation, &camera->yrotation);

	mat4 rotation = quaternion_get_matrix(&f);
	camera->view_matrix = gm_mat4_multiply(&rotation, &trans);
}

static void recalculate_projection_matrix(Perspective_Camera* camera) {
	r64 near = camera->near_plane;
	r64 far = camera->far_plane;
	r64 top = (r64)fabs(near) * atan(gm_radians(camera->fov) / 2.0);
	r64 bottom = -top;
	r64 right = top * ((r64)window_width / (r64)window_height);
	r64 left = -right;

	mat4 p = (mat4) {
		near, 0, 0, 0,
		0, near, 0, 0,
		0, 0, near + far, -near * far,
		0, 0, 1, 0
	};

	mat4 m = (mat4) {
		2.0 / (right - left), 0, 0, -(right + left) / (right - left),
		0, 2.0 / (top - bottom), 0, -(top + bottom) / (top - bottom),
		0, 0, 2.0 / (far - near), -(far + near) / (far - near),
		0, 0, 0, 1
	};

	// Need to transpose when sending to shader
	mat4 mp = gm_mat4_multiply(&m, &p);
	camera->projection_matrix = gm_mat4_scalar_product(-1, &mp);
}

void camera_init(Perspective_Camera* camera, vec3 position, r64 near_plane, r64 far_plane, r64 fov) {
	camera->position = position;
	camera->near_plane = near_plane;
	camera->far_plane = far_plane;
	camera->fov = fov;
	camera->rotation = (Quaternion) {0.0, 0.0, 0.0, 1.0};
	camera->yrotation = (Quaternion) {0.0, 0.0, 0.0, 1.0};
	recalculate_view_matrix(camera);
	recalculate_projection_matrix(camera);
}

void camera_set_position(Perspective_Camera* camera, vec3 position) {
	camera->position = position;
	recalculate_view_matrix(camera);
}

void camera_set_near_plane(Perspective_Camera* camera, r64 near_plane) {
	camera->near_plane = near_plane;
	recalculate_projection_matrix(camera);
}

void camera_set_far_plane(Perspective_Camera* camera, r64 far_plane) {
	camera->far_plane = far_plane;
	recalculate_projection_matrix(camera);
}

void camera_rotate_x(Perspective_Camera* camera, r64 x_difference) {
	Quaternion y_axis = quaternion_new((vec3) { 0.0, 1.0, 0.0 }, x_difference);
	camera->yrotation = quaternion_product(&y_axis, &camera->yrotation);
	camera->yrotation = quaternion_normalize(&camera->yrotation);
	recalculate_view_matrix(camera);
}

void camera_rotate_y(Perspective_Camera* camera, r64 y_difference) {
	vec3 right = quaternion_get_right_inverted(&camera->rotation);
	right = gm_vec3_normalize(right);
	Quaternion x_axis = quaternion_new(right, y_difference);
	camera->rotation = quaternion_product(&camera->rotation, &x_axis);
	camera->rotation = quaternion_normalize(&camera->rotation);
	recalculate_view_matrix(camera);
}

void camera_move_forward(Perspective_Camera* camera, r64 amount) {
	Quaternion f = quaternion_product(&camera->rotation, &camera->yrotation);

	vec3 forward = quaternion_get_forward_inverted(&f);
	forward = gm_vec3_scalar_product(amount, gm_vec3_normalize(forward));
	camera->position = gm_vec3_add((vec3) {-forward.x, -forward.y, -forward.z}, camera->position);

	recalculate_view_matrix(camera);
}

void camera_move_right(Perspective_Camera* camera, r64 amount) {
	Quaternion f = quaternion_product(&camera->rotation, &camera->yrotation);

	vec3 right = quaternion_get_right_inverted(&f);
	right = gm_vec3_scalar_product(amount, gm_vec3_normalize(right));
	camera->position = gm_vec3_add((vec3) {right.x, right.y, right.z}, camera->position);

	recalculate_view_matrix(camera);
}

vec3 camera_get_x_axis(const Perspective_Camera* camera) {
	Quaternion q = quaternion_product(&camera->rotation, &camera->yrotation);
	vec3 right = quaternion_get_right_inverted(&q);
	right = gm_vec3_normalize(right);
	return (vec3) {right.x, right.y, right.z};
}

vec3 camera_get_y_axis(const Perspective_Camera* camera) {
	Quaternion q = quaternion_product(&camera->rotation, &camera->yrotation);
	vec3 up = quaternion_get_up_inverted(&q);
	up = gm_vec3_normalize(up);
	return (vec3) {up.x, up.y, up.z};
}

vec3 camera_get_z_axis(const Perspective_Camera* camera) {
	Quaternion q = quaternion_product(&camera->rotation, &camera->yrotation);
	vec3 forward = quaternion_get_forward_inverted(&q);
	forward = gm_vec3_normalize(forward);
	return (vec3) {forward.x, forward.y, forward.z};
}

void camera_set_fov(Perspective_Camera* camera, r64 fov) {
	camera->fov = fov;
	recalculate_projection_matrix(camera);
}

void camera_force_matrix_recalculation(Perspective_Camera* camera) {
	recalculate_view_matrix(camera);
	recalculate_projection_matrix(camera);
}