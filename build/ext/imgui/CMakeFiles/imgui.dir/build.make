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
include ext/imgui/CMakeFiles/imgui.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ext/imgui/CMakeFiles/imgui.dir/compiler_depend.make

# Include the progress variables for this target.
include ext/imgui/CMakeFiles/imgui.dir/progress.make

# Include the compile flags for this target's objects.
include ext/imgui/CMakeFiles/imgui.dir/flags.make

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o: ext/imgui/CMakeFiles/imgui.dir/flags.make
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui.cpp
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o: ext/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o -MF CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o.d -o CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui.cpp

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/__/imgui/imgui.cpp.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui.cpp > CMakeFiles/imgui.dir/__/imgui/imgui.cpp.i

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/__/imgui/imgui.cpp.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui.cpp -o CMakeFiles/imgui.dir/__/imgui/imgui.cpp.s

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o: ext/imgui/CMakeFiles/imgui.dir/flags.make
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_demo.cpp
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o: ext/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o -MF CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o.d -o CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_demo.cpp

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_demo.cpp > CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.i

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_demo.cpp -o CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.s

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o: ext/imgui/CMakeFiles/imgui.dir/flags.make
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_draw.cpp
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o: ext/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o -MF CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o.d -o CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_draw.cpp

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_draw.cpp > CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.i

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_draw.cpp -o CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.s

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o: ext/imgui/CMakeFiles/imgui.dir/flags.make
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_widgets.cpp
ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o: ext/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o -MF CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o.d -o CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_widgets.cpp

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_widgets.cpp > CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.i

ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/imgui_widgets.cpp -o CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.s

ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o: ext/imgui/CMakeFiles/imgui.dir/flags.make
ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_glfw.cpp
ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o: ext/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o -MF CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o.d -o CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_glfw.cpp

ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_glfw.cpp > CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.i

ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_glfw.cpp -o CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.s

ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o: ext/imgui/CMakeFiles/imgui.dir/flags.make
ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_opengl3.cpp
ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o: ext/imgui/CMakeFiles/imgui.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o -MF CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o.d -o CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o -c /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_opengl3.cpp

ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.i"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_opengl3.cpp > CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.i

ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.s"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/imgui/examples/imgui_impl_opengl3.cpp -o CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.s

# Object files for target imgui
imgui_OBJECTS = \
"CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o" \
"CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o" \
"CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o" \
"CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o" \
"CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o" \
"CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o"

# External object files for target imgui
imgui_EXTERNAL_OBJECTS =

ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui.cpp.o
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_demo.cpp.o
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_draw.cpp.o
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/__/imgui/imgui_widgets.cpp.o
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_glfw.cpp.o
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/__/imgui/examples/imgui_impl_opengl3.cpp.o
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/build.make
ext/imgui/libimgui.dylib: ext/glad/libglad.dylib
ext/imgui/libimgui.dylib: ext/glfw/src/libglfw.3.4.dylib
ext/imgui/libimgui.dylib: ext/imgui/CMakeFiles/imgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libimgui.dylib"
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imgui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ext/imgui/CMakeFiles/imgui.dir/build: ext/imgui/libimgui.dylib
.PHONY : ext/imgui/CMakeFiles/imgui.dir/build

ext/imgui/CMakeFiles/imgui.dir/clean:
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui && $(CMAKE_COMMAND) -P CMakeFiles/imgui.dir/cmake_clean.cmake
.PHONY : ext/imgui/CMakeFiles/imgui.dir/clean

ext/imgui/CMakeFiles/imgui.dir/depend:
	cd /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/linsanity/Documents/GitHub/2022_TileableShell_public /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/libigl-imgui /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui /Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/imgui/CMakeFiles/imgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/imgui/CMakeFiles/imgui.dir/depend

