#ifndef RAW_PHYSICS_EXAMPLES_CHAIN_H
#define RAW_PHYSICS_EXAMPLES_CHAIN_H

#include <common.h>

int  ex_chain_init();
void ex_chain_destroy();
void ex_chain_update(r64 delta_time);
void ex_chain_render();
void ex_chain_input_process(boolean* key_state, r64 delta_time);
void ex_chain_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void ex_chain_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void ex_chain_scroll_change_process(r64 x_offset, r64 y_offset);
void ex_chain_window_resize_process(s32 width, s32 height);
void ex_chain_menu_update();

#endif