#ifndef RAW_PHYSICS_EXAMPLES_CUBE_STORM_H
#define RAW_PHYSICS_EXAMPLES_CUBE_STORM_H

#include <common.h>

int  ex_cube_storm_init();
void ex_cube_storm_destroy();
void ex_cube_storm_update(r64 delta_time);
void ex_cube_storm_render();
void ex_cube_storm_input_process(boolean* key_state, r64 delta_time);
void ex_cube_storm_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void ex_cube_storm_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void ex_cube_storm_scroll_change_process(r64 x_offset, r64 y_offset);
void ex_cube_storm_window_resize_process(s32 width, s32 height);
void ex_cube_storm_menu_update();

#endif