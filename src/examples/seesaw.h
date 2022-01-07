#ifndef RAW_PHYSICS_EXAMPLES_SEESAW_H
#define RAW_PHYSICS_EXAMPLES_SEESAW_H

#include <common.h>

int ex_seesaw_init();
void ex_seesaw_destroy();
void ex_seesaw_update(r64 delta_time);
void ex_seesaw_render();
void ex_seesaw_input_process(boolean* key_state, r64 delta_time);
void ex_seesaw_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void ex_seesaw_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void ex_seesaw_scroll_change_process(r64 x_offset, r64 y_offset);
void ex_seesaw_window_resize_process(s32 width, s32 height);
void ex_seesaw_menu_update();

#endif