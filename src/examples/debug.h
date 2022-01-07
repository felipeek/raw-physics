#ifndef RAW_PHYSICS_EXAMPLES_DEBUG_H
#define RAW_PHYSICS_EXAMPLES_DEBUG_H

#include <common.h>

int ex_debug_init();
void ex_debug_destroy();
void ex_debug_update(r64 delta_time);
void ex_debug_render();
void ex_debug_input_process(boolean* key_state, r64 delta_time);
void ex_debug_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void ex_debug_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void ex_debug_scroll_change_process(r64 x_offset, r64 y_offset);
void ex_debug_window_resize_process(s32 width, s32 height);

#endif