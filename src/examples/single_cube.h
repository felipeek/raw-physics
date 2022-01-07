#ifndef RAW_PHYSICS_EXAMPLES_SINGLE_CUBE_H
#define RAW_PHYSICS_EXAMPLES_SINGLE_CUBE_H

#include <common.h>

int ex_single_cube_init();
void ex_single_cube_destroy();
void ex_single_cube_update(r64 delta_time);
void ex_single_cube_render();
void ex_single_cube_input_process(boolean* key_state, r64 delta_time);
void ex_single_cube_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void ex_single_cube_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void ex_single_cube_scroll_change_process(r64 x_offset, r64 y_offset);
void ex_single_cube_window_resize_process(s32 width, s32 height);

#endif