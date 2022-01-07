CCX=g++
IDIR=include
CPPFLAGS=-I$(IDIR) -g -O0

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	LDFLAGS=-framework OpenGL -lm -lglfw -lglew
else
	LDFLAGS=-lm -lglfw -lGLEW -lGL -lpng -lz
endif

# Final binary
BIN = raw-physics
# Put all auto generated stuff to this build dir.
BUILD_DIR = ./bin/raw-physics

# List of all .cpp source files.
CPP = $(wildcard src/*.cpp) $(wildcard src/examples/*.cpp) $(wildcard src/render/*.cpp) \
	$(wildcard src/physics/*.cpp) $(wildcard src/vendor/*.cpp)

# All .o files go to build dir.
OBJ = $(CPP:%.cpp=$(BUILD_DIR)/%.o)
# Gcc/Clang will create these .d files containing dependencies.
DEP = $(OBJ:%.o=%.d)

# Default target named after the binary.
$(BIN) : $(BUILD_DIR)/$(BIN)

# Actual target of the binary - depends on all .o files.
$(BUILD_DIR)/$(BIN) : $(OBJ)
	# Create build directories - same structure as sources.
	mkdir -p $(@D)
	# Just link all the object files.
	$(CCX) $(CFLAGS) $^ $(LDFLAGS) -o $@

# Include all .d files
-include $(DEP)

# Build target for every single object file.
# The potential dependency on header files is covered
# by calling `-include $(DEP)`.
$(BUILD_DIR)/%.o : %.cpp
	mkdir -p $(@D)
	# The -MMD flags additionaly creates a .d file with
	# the same name as the .o file.
	$(CCX) $(CPPFLAGS) -MMD -c $< -o $@

.PHONY : clean
clean :
	# This should remove all generated files.
	-rm $(BUILD_DIR)/$(BIN) $(OBJ) $(DEP)