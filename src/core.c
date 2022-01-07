#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "core.h"
#include "examples/examples.h"
#include "examples/single_cube.h"
#include "examples/debug.h"
#include "render/menu.h"

Example_Scene selected_scene;

static int load_selected_scene() {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			return ex_debug_init();
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			return ex_single_cube_init();
		} break;
		case NONE_EXAMPLE_SCENE: {
			return 0;
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void destroy_selected_scene() {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_destroy();
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_destroy();
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void update_selected_scene(r64 delta_time) {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_update(delta_time);
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_update(delta_time);
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void render_selected_scene() {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_render();
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_render();
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void input_process_selected_scene(boolean* key_state, r64 delta_time) {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_input_process(key_state, delta_time);
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_input_process(key_state, delta_time);
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void mouse_change_process_selected_scene(boolean reset, r64 x_pos, r64 y_pos) {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_mouse_change_process(reset, x_pos, y_pos);
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_mouse_change_process(reset, x_pos, y_pos);
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void mouse_click_process_selected_scene(s32 button, s32 action, r64 x_pos, r64 y_pos) {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_mouse_click_process(button, action, x_pos, y_pos);
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_mouse_click_process(button, action, x_pos, y_pos);
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void scroll_change_process_selected_scene(r64 x_offset, r64 y_offset) {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_scroll_change_process(x_offset, y_offset);
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_scroll_change_process(x_offset, y_offset);
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void window_resize_process_selected_scene(s32 width, s32 height) {
	switch (selected_scene) {
		case DEBUG_EXAMPLE_SCENE: {
			ex_debug_window_resize_process(width, height);
		} break;
		case SINGLE_CUBE_EXAMPLE_SCENE: {
			ex_single_cube_window_resize_process(width, height);
		} break;
		case NONE_EXAMPLE_SCENE: {
		} break;
		default: {
			assert(0);
		} break;
	}
}

static void menu_selected_scene_callback(Example_Scene scene) {
	destroy_selected_scene();
	selected_scene = scene;
	load_selected_scene();
}

int core_init() {
	menu_register_selected_scene_callback(menu_selected_scene_callback);
	selected_scene = EXAMPLE_SCENE_INITIAL;
	return load_selected_scene(selected_scene);
}

void core_destroy() {
	destroy_selected_scene();
}

void core_update(r64 delta_time) {
	update_selected_scene(delta_time);
}

void core_render() {
	render_selected_scene();
}

void core_input_process(boolean* key_state, r64 delta_time) {
	input_process_selected_scene(key_state, delta_time);
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
	mouse_change_process_selected_scene(reset, x_pos, y_pos);
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {
	mouse_click_process_selected_scene(button, action, x_pos, y_pos);
}

void core_scroll_change_process(r64 x_offset, r64 y_offset) {
	scroll_change_process_selected_scene(x_offset, y_offset);
}

void core_window_resize_process(s32 width, s32 height) {
	window_resize_process_selected_scene(width, height);
}