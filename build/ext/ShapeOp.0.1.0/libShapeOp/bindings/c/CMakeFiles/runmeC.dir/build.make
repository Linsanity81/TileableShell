# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/linsanity/Documents/GitHub/2022_TileableShell_public

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build

# Include any dependencies generated for this target.
include ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/compiler_depend.make

# Include the progress variables for this target.
include ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/progress.make

# Include the compile flags for this target's objects.
include ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/flags.make

ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.o: ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/flags.make
ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runme.c
ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.o: ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.o -MF CMakeFiles/runmeC.dir/runme.c.o.d -o CMakeFiles/runmeC.dir/runme.c.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runme.c

ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/runmeC.dir/runme.c.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runme.c > CMakeFiles/runmeC.dir/runme.c.i

ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/runmeC.dir/runme.c.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c && /usr/bin/clang $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runme.c -o CMakeFiles/runmeC.dir/runme.c.s

# Object files for target runmeC
runmeC_OBJECTS = \
"CMakeFiles/runmeC.dir/runme.c.o"

# External object files for target runmeC
runmeC_EXTERNAL_OBJECTS =

ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runmeC: ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/runme.c.o
ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runmeC: ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/build.make
ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runmeC: ext/ShapeOp.0.1.0/libShapeOp/libShapeOp.0.1.0.dylib
ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runmeC: ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable runmeC"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/runmeC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/build: ext/ShapeOp.0.1.0/libShapeOp/bindings/c/runmeC
.PHONY : ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/build

ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/clean:
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c && $(CMAKE_COMMAND) -P CMakeFiles/runmeC.dir/cmake_clean.cmake
.PHONY : ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/clean

ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/depend:
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/linsanity/Documents/GitHub/2022_TileableShell_public /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/ShapeOp.0.1.0/libShapeOp/bindings/c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/ShapeOp.0.1.0/libShapeOp/bindings/c/CMakeFiles/runmeC.dir/depend

