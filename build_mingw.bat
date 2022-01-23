set BUILD_OPTS=-O2 -ffast-math
gcc -c src/physics/broad.cpp -o gccbuild/broad.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/clipping.cpp -o gccbuild/clipping.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/collider.cpp -o gccbuild/collider.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/epa.cpp -o gccbuild/epa.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/gjk.cpp -o gccbuild/gjk.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/pbd.cpp -o gccbuild/pbd.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/pbd_base_constraints.cpp -o gccbuild/pbd_base_constraints.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/physics_util.cpp -o gccbuild/physics_util.o -Iinclude %BUILD_OPTS%
gcc -c src/physics/support.cpp -o gccbuild/support.o -Iinclude %BUILD_OPTS%
gcc -c src/render/camera.cpp -o gccbuild/camera.o -Iinclude %BUILD_OPTS%
gcc -c src/render/graphics.cpp -o gccbuild/graphics.o -Iinclude %BUILD_OPTS%
gcc -c src/render/menu.cpp -o gccbuild/menu.o -Iinclude %BUILD_OPTS%
gcc -c src/render/mesh.cpp -o gccbuild/mesh.o -Iinclude %BUILD_OPTS%
gcc -c src/render/obj.cpp -o gccbuild/obj.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/arm.cpp -o gccbuild/arm.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/brick_wall.cpp -o gccbuild/brick_wall.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/cube_storm.cpp -o gccbuild/cube_storm.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/debug.cpp -o gccbuild/debug.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/examples_util.cpp -o gccbuild/examples_util.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/hinge_joints.cpp -o gccbuild/hinge_joints.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/mirror_cube.cpp -o gccbuild/mirror_cube.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/seesaw.cpp -o gccbuild/seesaw.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/single_cube.cpp -o gccbuild/single_cube.o -Iinclude %BUILD_OPTS%
gcc -c src/examples/spring.cpp -o gccbuild/spring.o -Iinclude %BUILD_OPTS%
gcc -c src/vendor/imgui.cpp -o gccbuild/imgui.o -Iinclude %BUILD_OPTS%
gcc -c src/vendor/imgui_demo.cpp -o gccbuild/imgui_demo.o -Iinclude %BUILD_OPTS%
gcc -c src/vendor/imgui_draw.cpp -o gccbuild/imgui_draw.o -Iinclude %BUILD_OPTS%
gcc -c src/vendor/imgui_impl_glfw.cpp -o gccbuild/imgui_impl_glfw.o -Iinclude %BUILD_OPTS%
gcc -c src/vendor/imgui_impl_opengl3.cpp -o gccbuild/imgui_impl_opengl3.o -Iinclude %BUILD_OPTS%
gcc -c src/vendor/imgui_widgets.cpp -o gccbuild/imgui_widgets.o -Iinclude %BUILD_OPTS%
gcc -c src/core.cpp -o gccbuild/core.o -Iinclude %BUILD_OPTS%
gcc -c src/entity.cpp -o gccbuild/entity.o -Iinclude %BUILD_OPTS%
gcc -c src/main.cpp -o gccbuild/main.o -Iinclude %BUILD_OPTS%
gcc -c src/quaternion.cpp -o gccbuild/quaternion.o -Iinclude %BUILD_OPTS%
gcc -c src/util.cpp -o gccbuild/util.o -Iinclude %BUILD_OPTS%
g++ -o gccbuild/raw-physics -Llib/win64 gccbuild/*.o -lws2_32 -luser32 -lole32 -lShell32 -lgdi32 -lwinmm -lkernel32 -lopengl32 -lglew32 -lglfw3dll
cp lib/win64/glew32.dll gccbuild/
cp lib/win64/glfw3.dll gccbuild/
