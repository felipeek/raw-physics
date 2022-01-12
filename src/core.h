#ifndef RAW_PHYSICS_CORE_H
#define RAW_PHYSICS_CORE_H

#include <common.h>

typedef int  (*Example_Init_Func)(void);
typedef void (*Example_Destroy_Func)(void);
typedef void (*Example_Update_Func)(r64);
typedef void (*Example_Render_Func)(void);
typedef void (*Example_Input_Process_Func)(boolean*, r64);
typedef void (*Example_Mouse_Change_Process_Func)(boolean, r64, r64);
typedef void (*Example_Mouse_Click_Process_Func)(s32, s32, r64, r64);
typedef void (*Example_Scroll_Change_Process_Func)(r64, r64);
typedef void (*Example_Window_Resize_Process_Func)(s32, s32);
typedef void (*Example_Menu_Properties_Update_Func)();

typedef struct {
	const char* name;
	Example_Init_Func init;
	Example_Destroy_Func destroy;
	Example_Input_Process_Func input_process;
	Example_Menu_Properties_Update_Func menu_properties_update;
	Example_Mouse_Change_Process_Func mouse_change_process;
	Example_Mouse_Click_Process_Func mouse_click_process;
	Example_Render_Func render;
	Example_Scroll_Change_Process_Func scroll_change_process;
	Example_Update_Func update;
	Example_Window_Resize_Process_Func window_resize_process;
} Example_Scene;

typedef enum {
	DEBUG_EXAMPLE_SCENE,
	SINGLE_CUBE_EXAMPLE_SCENE,
	CUBE_STORM_EXAMPLE_SCENE,
	SEESAW_EXAMPLE_SCENE,
	CHAIN_EXAMPLE_SCENE,
	BRICK_WALL_EXAMPLE_SCENE,
	END_EXAMPLE_SCENE
} Example_Scene_Type;

//#define EXAMPLE_SCENE_INITIAL SINGLE_CUBE_EXAMPLE_SCENE
#define EXAMPLE_SCENE_INITIAL DEBUG_EXAMPLE_SCENE
//#define EXAMPLE_SCENE_INITIAL CHAIN_EXAMPLE_SCENE
//#define EXAMPLE_SCENE_INITIAL CUBE_STORM_EXAMPLE_SCENE

int core_init();
void core_destroy();
void core_update(r64 delta_time);
void core_render();
void core_input_process(boolean* key_state, r64 delta_time);
void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos);
void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos);
void core_scroll_change_process(r64 x_offset, r64 y_offset);
void core_window_resize_process(s32 width, s32 height);
void core_switch_scene(Example_Scene_Type scene);
void core_menu_render();

#endif
