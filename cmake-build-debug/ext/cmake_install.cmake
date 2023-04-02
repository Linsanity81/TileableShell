# Install script for directory: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/ShapeOp.0.1.0/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/OpenMesh/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/pugixml/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/glad/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/glfw/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/imgui/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/stb_image/cmake_install.cmake")
  include("/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/matplotlib/cmake_install.cmake")

endif()

