CCX=g++
IDIR=include
CPPFLAGS_RELEASE=-I$(IDIR) -O2 -ffast-math
CPPFLAGS_DEBUG=-I$(IDIR) -O0 -g

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	LDFLAGS=-framework OpenGL -lm -lglfw -lglew
else
	LDFLAGS=-lm -lglfw -lGLEW -lGL -lpng -lz
endif

# Final binary
BIN = raw-physics
# Put all auto generated stuff to this build dir.
BUILD_DIR=./bin
BUILD_DIR_DEBUG = $(BUILD_DIR)/debug
BUILD_DIR_RELEASE = $(BUILD_DIR)/release

# List of all .cpp source files.
CPP = $(wildcard src/*.cpp) $(wildcard src/examples/*.cpp) $(wildcard src/render/*.cpp) \
	$(wildcard src/physics/*.cpp) $(wildcard src/vendor/*.cpp)

# All .o files go to build dir.
RELEASE_OBJ = $(CPP:%.cpp=$(BUILD_DIR_RELEASE)/%.o)
DEBUG_OBJ = $(CPP:%.cpp=$(BUILD_DIR_DEBUG)/%.o)
# Gcc/Clang will create these .d files containing dependencies.
DEP = $(OBJ:%.o=%.d)

# Default target named after the binary.
release : $(BUILD_DIR_RELEASE)/$(BIN)

debug : $(BUILD_DIR_DEBUG)/$(BIN)

# Actual target of the binary - depends on all .o files.
$(BUILD_DIR_RELEASE)/$(BIN) : $(RELEASE_OBJ)
	# Create build directories - same structure as sources.
	mkdir -p $(@D)
	# Just link all the object files.
	$(CCX) $(CPPFLAGS_RELEASE) $^ $(LDFLAGS) -o $@

$(BUILD_DIR_DEBUG)/$(BIN) : $(DEBUG_OBJ)
	# Create build directories - same structure as sources.
	mkdir -p $(@D)
	# Just link all the object files.
	$(CCX) $(CPPFLAGS_DEBUG) $^ $(LDFLAGS) -o $@

# Include all .d files
-include $(DEP)

# Build target for every single object file.
# The potential dependency on header files is covered
# by calling `-include $(DEP)`.
$(BUILD_DIR_RELEASE)/%.o : %.cpp
	mkdir -p $(@D)
	# The -MMD flags additionaly creates a .d file with
	# the same name as the .o file.
	$(CCX) $(CPPFLAGS_RELEASE) -MMD -c $< -o $@

$(BUILD_DIR_DEBUG)/%.o : %.cpp
	mkdir -p $(@D)
	# The -MMD flags additionaly creates a .d file with
	# the same name as the .o file.
	$(CCX) $(CPPFLAGS_DEBUG) -MMD -c $< -o $@

.PHONY : clean
clean :
	# This should remove all generated files.
	-rm $(BUILD_DIR)/$(BIN) $(OBJ) $(DEP)