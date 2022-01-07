#ifndef RAW_PHYSICS_CORE_H
#define RAW_PHYSICS_CORE_H

#include <common.h>

typedef enum {
	NONE_EXAMPLE_SCENE,
	DEBUG_EXAMPLE_SCENE,
	SINGLE_CUBE_EXAMPLE_SCENE,
	CUBE_STORM_EXAMPLE_SCENE,
	SEESAW_EXAMPLE_SCENE
} Example_Scene;

#define EXAMPLE_SCENE_INITIAL SINGLE_CUBE_EXAMPLE_SCENE

int core_init();
void core_destroy();
void core_update(r64 delta_time);
void core_render();
void core_input_process(boolean* key_state, r64 delta_time);
void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void core_scroll_change_process(r64 x_offset, r64 y_offset);
void core_window_resize_process(s32 width, s32 height);
void core_switch_scene(Example_Scene scene);
void core_menu_properties_update();

#endif