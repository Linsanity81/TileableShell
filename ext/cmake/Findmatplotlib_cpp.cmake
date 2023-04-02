unset (matplotlib_cpp_LIBRARY CACHE)
unset (matplotlib_cpp_INCLUDE_DIR CACHE)

set(BUILD_SHARED_LIBS ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/matplotlib)
set(matplotlib_cpp_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/matplotlib)
include_directories(${matplotlib_cpp_INCLUDE_DIR})
set(matplotlib_cpp_LIBRARY matplotlib_cpp)