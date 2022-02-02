@echo off

set COMPILER_FLAGS=/MT /nologo /D_CRT_SECURE_NO_WARNINGS /I../../../include /Zi /Feraw-physics.exe /O2 /wd4576 /EHsc /std:c++latest /fp:fast
set LIBRARIES=opengl32.lib ws2_32.lib user32.lib ole32.lib Shell32.lib gdi32.lib winmm.lib kernel32.lib ../../../lib/win64/glew32.lib ../../../lib/win64/glfw3dll.lib
set FILES=../src/*.cpp ../src/physics/*.cpp ../src/render/*.cpp ../src/vendor/*.cpp

mkdir bin
pushd bin
call cl %COMPILER_FLAGS% %FILES% /link %LIBRARIES%
cp ../../../lib/win64/glew32.dll .
cp ../../../lib/win64/glfw3.dll .
popd
