#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "core.h"
#include "examples/cube_and_ramp.h"
#include "examples/debug.h"
#include "examples/cube_storm.h"
#include "examples/seesaw.h"
#include "examples/spring.h"
#include "examples/brick_wall.h"
#include "examples/mirror_cube.h"
#include "examples/hinge_joints.h"
#include "examples/arm.h"
#include "examples/stack.h"
#include "examples/rott_pendulum.h"
#include "examples/triple_pendula.h"
#include "examples/spot_storm.h"
#include "examples/coin.h"
#include "render/menu.h"
#include "vendor/imgui.h"

#define MENU_TITLE "Examples"
#define FPS_MAX 144
#define FPS_MIN 30

static Example_Scene example_scenes[END_EXAMPLE_SCENE];
static Example_Scene_Type selected_scene;

// Used when fps is fixed
static bool fix_fps = true;
static int fps = 60;

static int load_selected_scene() {
	return example_scenes[selected_scene].init();
}

static void core_destroy_selected_scene() {
	example_scenes[selected_scene].destroy();
}

int core_init() {
	example_scenes[DEBUG_EXAMPLE_SCENE] = debug_example_scene;
	example_scenes[CUBE_AND_RAMP_EXAMPLE_SCENE] = cube_and_ramp_example_scene;
	example_scenes[CUBE_STORM_EXAMPLE_SCENE] = cube_storm_example_scene;
	example_scenes[SEESAW_EXAMPLE_SCENE] = seesaw_example_scene;
	example_scenes[SPRING_EXAMPLE_SCENE] = spring_example_scene;
	example_scenes[BRICK_WALL_EXAMPLE_SCENE] = brick_wall_example_scene;
	example_scenes[MIRROR_CUBE_EXAMPLE_SCENE] = mirror_cube_example_scene;
	example_scenes[HINGE_JOINTS_EXAMPLE_SCENE] = hinge_joints_example_scene;
	example_scenes[ARM_EXAMPLE_SCENE] = arm_example_scene;
	example_scenes[STACK_EXAMPLE_SCENE] = stack_example_scene;
	example_scenes[ROTT_PENDULUM_EXAMPLE_SCENE] = rott_pendulum_example_scene;
	example_scenes[TRIPLE_PENDULA_EXAMPLE_SCENE] = triple_pendula_example_scene;
	example_scenes[SPOT_STORM_EXAMPLE_SCENE] = spot_storm_example_scene;
	example_scenes[COIN_EXAMPLE_SCENE] = coin_example_scene;

	selected_scene = EXAMPLE_SCENE_INITIAL;
	return load_selected_scene();
}

void core_destroy() {
	core_destroy_selected_scene();
}

void core_update(r64 delta_time) {
	if (fix_fps) {
		fps = CLAMP(fps, FPS_MIN, FPS_MAX);
		delta_time = 1.0 / fps;
	}
	example_scenes[selected_scene].update(delta_time);
}

void core_render() {
	example_scenes[selected_scene].render();
}

void core_input_process(boolean* key_state, r64 delta_time) {
	if (key_state[GLFW_KEY_2]) {
		core_switch_scene(CUBE_AND_RAMP_EXAMPLE_SCENE);
		key_state[GLFW_KEY_2] = 0;
	}
	else {
		example_scenes[selected_scene].input_process(key_state, delta_time);
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos) {
	example_scenes[selected_scene].mouse_change_process(reset, x_pos, y_pos);
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos) {
	example_scenes[selected_scene].mouse_click_process(button, action, x_pos, y_pos);
}

void core_scroll_change_process(r64 x_offset, r64 y_offset) {
	example_scenes[selected_scene].scroll_change_process(x_offset, y_offset);
}

void core_window_resize_process(s32 width, s32 height) {
	example_scenes[selected_scene].window_resize_process(width, height);
}

void core_menu_render() {
    if (ImGui::Begin(MENU_TITLE, NULL, 0)) {
        ImGui::Text("Press [ESC] to open/close the menu");

        ImGui::Checkbox("Fix timestep for FPS", &fix_fps);
        if (fix_fps) {
            ImGui::DragInt("FPS", &fps, 0.1f, FPS_MIN, FPS_MAX);
        } else {
            ImGui::Text("Timestep will be calculated per frame");
            ImGui::Text("Simulation might become a bit unstable.");
        }

        // left
        static Example_Scene_Type selected = EXAMPLE_SCENE_INITIAL;
        ImGui::BeginChild("left pane", ImVec2(150, 0), true);
		for (u32 i = 0; i < END_EXAMPLE_SCENE; ++i) {
			Example_Scene* example_scene = &example_scenes[i];

			if (ImGui::Selectable(example_scene->name, selected == i)) {
				selected = static_cast<Example_Scene_Type>(i);
				core_switch_scene(selected);
			}
		}
        ImGui::EndChild();
        ImGui::SameLine();

        // right
        ImGui::BeginGroup();
		ImGui::BeginChild("item view", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
		example_scenes[selected_scene].menu_properties_update();

		ImGui::EndChild();
		//if (ImGui::Button("Revert")) {}
		//ImGui::SameLine();
		//if (ImGui::Button("Save")) {}
        ImGui::EndGroup();
    }
}

void core_switch_scene(Example_Scene_Type scene) {
	core_destroy_selected_scene();
	selected_scene = scene;
	load_selected_scene();

	// Force scene's camera to reset
	extern GLFWwindow* main_window;
	double xpos, ypos;
	glfwGetCursorPos(main_window, &xpos, &ypos);
	core_mouse_change_process(true, xpos, ypos);
}