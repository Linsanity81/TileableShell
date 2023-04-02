unset (ShapeOp_LIBRARY CACHE)
unset (ShapeOp_INCLUDE_DIR CACHE)

set(BUILD_SHARED_LIBS ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ShapeOp.0.1.0)
set(ShapeOp_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ShapeOp.0.1.0)
include_directories(${ShapeOp_INCLUDE_DIR})
set(ShapeOp_LIBRARY ShapeOp)