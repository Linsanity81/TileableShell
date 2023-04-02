cmake_minimum_required(VERSION 3.1)

set(OPENGL_VIEWER OFF CACHE BOOL "OpenGL Viewer")
if(OPENGL_VIEWER)
  message("Opengl viewer activated.")
  add_subdirectory(opengl)
endif()
