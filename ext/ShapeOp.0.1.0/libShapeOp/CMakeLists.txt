cmake_minimum_required(VERSION 3.1)
project(ShapeOp)

set(CMAKE_CONFIGURATION_TYPES Release Debug CACHE TYPE INTERNAL FORCE)
  
# Tell CMake he'll be able to include ".cmake" configurations 
# files in the folder where the current file is located
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_FOUND)
    message(ERROR " EIGEN not found.")
else()
    message("EIGEN found.")
endif() 

# Where are the headers found?
include_directories("src/")
include_directories("api/")

# h files
file(GLOB SHAPEOP_HEADERS "src/*.h")
# cpp files
file(GLOB SHAPEOP_SOURCES "src/*.cpp")
# h files
file(GLOB API_HEADERS "api/*.h")
# cpp files
file(GLOB API_SOURCES "api/*.cpp")

# Shared library
set(HEADERS ${SHAPEOP_HEADERS} ${API_HEADERS})
set(SOURCES ${SHAPEOP_SOURCES} ${API_SOURCES})
add_library(ShapeOp SHARED ${HEADERS} ${SOURCES})
set_target_properties(ShapeOp PROPERTIES VERSION ${SHAPEOP_VERSION_STRING} SOVERSION ${SHAPEOP_VERSION_MAJOR})



# Rename executable to contain version number on windows. On mac, a so-chain is generated from the SOVERSION specified above.
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
	set_target_properties(ShapeOp PROPERTIES OUTPUT_NAME ShapeOp.${SHAPEOP_VERSION_STRING})


endif()

#Adding subdirectories
add_subdirectory(bindings)
