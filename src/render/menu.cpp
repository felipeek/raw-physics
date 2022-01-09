#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "../vendor/imgui.h"
#include "../vendor/imgui_impl_glfw.h"
#include "../vendor/imgui_impl_opengl3.h"
#include "../core.h"
#include <common.h>
#include <stdio.h>

#include <dirent.h>
#include <light_array.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLSL_VERSION "#version 330"
#define MENU_TITLE "Examples"

void menu_char_click_process(GLFWwindow* window, u32 c)
{
	ImGui_ImplGlfw_CharCallback(window, c);
}

void menu_key_click_process(GLFWwindow* window, s32 key, s32 scan_code, s32 action, s32 mods)
{
	ImGui_ImplGlfw_KeyCallback(window, key, scan_code, action, mods);
}

void menu_mouse_click_process(GLFWwindow* window, s32 button, s32 action, s32 mods)
{
	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
}

void menu_scroll_change_process(GLFWwindow* window, s64 x_offset, s64 y_offset)
{
	ImGui_ImplGlfw_ScrollCallback(window, x_offset, y_offset);
}

void menu_init(GLFWwindow* window)
{
	// Setup Dear ImGui binding
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;

	ImGui_ImplGlfw_InitForOpenGL(window, false);
	ImGui_ImplOpenGL3_Init(GLSL_VERSION);

	// Setup style
	ImGui::StyleColorsDark();
}

static void draw_main_window()
{
    ImGui::SetNextWindowSize(ImVec2(500, 440), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(MENU_TITLE, NULL, 0))
    {
        // left
        static Example_Scene selected = EXAMPLE_SCENE_INITIAL;
        ImGui::BeginChild("left pane", ImVec2(150, 0), true);
		if (ImGui::Selectable("Single Cube", selected == SINGLE_CUBE_EXAMPLE_SCENE)) {
			selected = SINGLE_CUBE_EXAMPLE_SCENE;
			core_switch_scene(SINGLE_CUBE_EXAMPLE_SCENE);
		}
		if (ImGui::Selectable("Cube Storm", selected == CUBE_STORM_EXAMPLE_SCENE)) {
			selected = CUBE_STORM_EXAMPLE_SCENE;
			core_switch_scene(CUBE_STORM_EXAMPLE_SCENE);
		}
		if (ImGui::Selectable("Seesaw", selected == SEESAW_EXAMPLE_SCENE)) {
			selected = SEESAW_EXAMPLE_SCENE;
			core_switch_scene(SEESAW_EXAMPLE_SCENE);
		}
		if (ImGui::Selectable("Chain", selected == CHAIN_EXAMPLE_SCENE)) {
			selected = CHAIN_EXAMPLE_SCENE;
			core_switch_scene(CHAIN_EXAMPLE_SCENE);
		}
		if (ImGui::Selectable("Debug", selected == DEBUG_EXAMPLE_SCENE)) {
			selected = DEBUG_EXAMPLE_SCENE;
			core_switch_scene(DEBUG_EXAMPLE_SCENE);
		}
        ImGui::EndChild();
        ImGui::SameLine();

        // right
        ImGui::BeginGroup();
		ImGui::BeginChild("item view", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); // Leave room for 1 line below us
		core_menu_properties_update();
		ImGui::EndChild();
		//if (ImGui::Button("Revert")) {}
		//ImGui::SameLine();
		//if (ImGui::Button("Save")) {}
        ImGui::EndGroup();
    }
    ImGui::End();
}

void menu_render()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	
	ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

	draw_main_window();
	//ImGui::ShowDemoWindow();

	// Rendering
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void menu_destroy()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}