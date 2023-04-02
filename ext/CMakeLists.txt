cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#ShapeOp
find_package(ShapeOp)

#OpenMesh
find_package(OpenMesh)

#PUGIXML
find_package(PugiXML)

#LIBIGL
find_package(LIBIGL REQUIRED QUIET)
if(LIBIGL_FOUND)
    add_definitions(-DLIBIGL_FOUND=1)
endif()

#matplotlib_cpp
find_package(matplotlib_cpp)

set(ext_include
        ${PUGIXML_INCLUDE_DIR}
        ${LIBIGL_INCLUDE_DIR}
        ${OPENMESH_INCLUDE_DIR}
        ${ShapeOp_INCLUDE_DIR}
        ${matplotlib_cpp_INCLUDE_DIR}
        PARENT_SCOPE)

set(ext_lib pugixml
        igl::core
        igl::cgal
        igl::opengl_glfw
        igl::opengl_glfw_imgui
        igl::png
        OpenMeshCore
        OpenMeshTools
        ShapeOp
        matplotlib_cpp
        PARENT_SCOPE)