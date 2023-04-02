# Find the pugixml XML parsing library.
#
# Sets the usual variables expected for find_package scripts:
#
# PUGIXML_INCLUDE_DIR - header location
# PUGIXML_LIBRARIES - library to link against
# PUGIXML_FOUND - true if pugixml was found.

unset (OPENMESH_LIBRARY CACHE)
unset (OPENMESH_INCLUDE_DIR CACHE)

set(BUILD_SHARED_LIBS ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/OpenMesh)
set(OPENMESH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/OpenMesh/src)
include_directories(${OPENMESH_INCLUDE_DIR})
set(OPENMESH_LIBRARY OpenMesh)