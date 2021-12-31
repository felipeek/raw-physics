#define DYNAMIC_ARRAY_IMPLEMENT
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define GRAPHICS_MATH_IMPLEMENT
#define C_FEK_HASH_MAP_IMPLEMENT
#include <light_array.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <hash_map.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include "core.h"
#include <gm.h>
#include "render/menu.h"

#define WINDOW_TITLE "basic-engine"

s32 window_width = 1366;
s32 window_height = 768;
GLFWwindow* main_window;

static boolean key_state[1024];	// @TODO: Check range.
static boolean is_menu_visible = false;

static void glfw_key_callback(GLFWwindow* window, s32 key, s32 scanCode, s32 action, s32 mods)
{
	if (action == GLFW_PRESS)
		key_state[key] = true;
	if (action == GLFW_RELEASE)
		key_state[key] = false;
	if (key_state[GLFW_KEY_ESCAPE])
	{
		if (is_menu_visible)
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		else
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

		is_menu_visible = !is_menu_visible;
		key_state[GLFW_KEY_ESCAPE] = !key_state[GLFW_KEY_ESCAPE];
	}
}

static void glfw_cursor_callback(GLFWwindow* window, r64 x_pos, r64 y_pos)
{
	static boolean reset_core_mouse_movement = true;

	if (!is_menu_visible)
	{
		core_mouse_change_process(reset_core_mouse_movement, x_pos, y_pos);
		reset_core_mouse_movement = false;
	}
	else
		reset_core_mouse_movement = true;
}

static void glfw_mouse_button_callback(GLFWwindow* window, s32 button, s32 action, s32 mods)
{
	r64 x_pos, y_pos;
	glfwGetCursorPos(window, &x_pos, &y_pos);
	y_pos = window_height - y_pos;
	if (is_menu_visible)
		menu_mouse_click_process(window, button, action, mods);
	else
		core_mouse_click_process(button, action, x_pos, y_pos);
}

static void glfw_scroll_callback(GLFWwindow* window, r64 x_offset, r64 y_offset)
{
	if (is_menu_visible)
		menu_scroll_change_process(window, x_offset, y_offset);
	else
		core_scroll_change_process(x_offset, y_offset);
}

static void glfw_resize_callback(GLFWwindow* window, s32 width, s32 height)
{
	window_width = width;
	window_height = height;
	glViewport(0, 0, width, height);
	core_window_resize_process(width, height);
}

static void glfw_char_callback(GLFWwindow* window, u32 c)
{
	if (is_menu_visible)
		menu_char_click_process(window, c);
}

static GLFWwindow* init_glfw()
{
	glfwInit();
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	GLFWwindow* window = glfwCreateWindow(window_width, window_height, WINDOW_TITLE, 0, 0);
	glfwSetWindowPos(window, 50, 50);
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, glfw_key_callback);
	glfwSetCursorPosCallback(window, glfw_cursor_callback);
	glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
	glfwSetScrollCallback(window, glfw_scroll_callback);
	glfwSetWindowSizeCallback(window, glfw_resize_callback);
	glfwSetCharCallback(window, glfw_char_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	return window;
}

static void init_glew()
{
	glewExperimental = true;
	glewInit();
}

s32 main(s32 argc, s8** argv)
{
	r32 delta_time = 0.0f;
	main_window = init_glfw();
	init_glew();

	if (core_init())
		return -1;

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	r64 last_frame = glfwGetTime();
	s32 frame_number = (s32)last_frame;
	u32 fps = 0;

	menu_init(main_window);

	while (!glfwWindowShouldClose(main_window))
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.2074f, 0.3168f, 0.3615f, 1.0f);

		core_update(delta_time);
		core_render();
		if (!is_menu_visible)
			core_input_process(key_state, delta_time);
		else
			menu_render();

		glfwPollEvents();
		glfwSwapBuffers(main_window);

		r64 current_frame = glfwGetTime();
		if ((s32)current_frame > frame_number)
		{
			//printf("FPS: %u\n", fps);
			fps = 0;
			frame_number++;
		}
		else
			++fps;

		delta_time = (r32)(current_frame - last_frame);

		last_frame = current_frame;
	}

	menu_destroy();
	core_destroy();
	glfwTerminate();
}
