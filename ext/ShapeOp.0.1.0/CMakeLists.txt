cmake_minimum_required(VERSION 3.1)

project(ShapeOp)

#Versioning
set(SHAPEOP_VERSION_MAJOR 0)
set(SHAPEOP_VERSION_MINOR 1)
set(SHAPEOP_VERSION_PATCH 0)
set(SHAPEOP_VERSION_STRING ${SHAPEOP_VERSION_MAJOR}.${SHAPEOP_VERSION_MINOR}.${SHAPEOP_VERSION_PATCH})

# Make a release library build by default. This does not set the default build type for IDE'separate_arguments
# like Visual Studio and Xcode. There seems to be no way to set default types in the CMakeList. See discussion here:
# http://stackoverflow.com/questions/24460486/cmake-build-type-not-being-used-in-cmakelists-txt
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

message(${CMAKE_BUILD_TYPE})

# Configure a header file to pass some of the CMake settings to the Doxygen documentation
configure_file("${PROJECT_SOURCE_DIR}/doc/DoxyShapeOp.in" "${PROJECT_BINARY_DIR}/doc/DoxyShapeOp")
configure_file("${PROJECT_SOURCE_DIR}/doc/DoxyShapeOp.bib" "${PROJECT_BINARY_DIR}/doc/DoxyShapeOp.bib" COPYONLY)


# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang") #Tests for Clang and AppleClang which is defined in CMake 3.0 and higher
    message("Clang.")
    set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++ -Wall -g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
#    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    add_definitions(-DSHAPEOP_CLANG)
    add_definitions(-DSHAPEOP_EXPORT)
	set(OPENMP OFF CACHE BOOL "OpenMP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    message("GCC.")
    set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g")
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
#    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    add_definitions(-DSHAPEOP_GCC)
    add_definitions(-DSHAPEOP_EXPORT)
	set(OPENMP ON CACHE BOOL "OpenMP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    message("ICC.")
    if(WIN32)
        set(CMAKE_CXX_FLAGS "/Qstd=c++11")
        set(CMAKE_CXX_FLAGS_RELEASE "/O3 /DNDEBUG")
        add_definitions(/DSHAPEOP_ICC)
        add_definitions(/DSHAPEOP_EXPORT)
    else()
        set(CMAKE_CXX_FLAGS "-std=c++11")
        set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
        add_definitions(-DSHAPEOP_ICC)
        add_definitions(-DSHAPEOP_EXPORT)
    endif()
	set(OPENMP OFF CACHE BOOL "OpenMP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message("MSVC.")
    set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
    set(CMAKE_CXX_FLAGS_RELEASE "/MT /Ox /Ot /fp:fast /GS- /GL /DNDEBUG")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /INCREMENTAL:NO /OPT:REF")
    add_definitions(/DSHAPEOP_MSVC)
    add_definitions(/DSHAPEOP_EXPORT)
	set(OPENMP ON CACHE BOOL "OpenMP")
else()
    message( FATAL_ERROR "CXX compiler not recognized. CMake will quit." )
endif()

#OpenMP
if(OPENMP)
  FIND_PACKAGE(OpenMP QUIET)
  if(OPENMP_FOUND)
      message("OpenMP found. OpenMP activated in release.")
      set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
      set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
      set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${OpenMP_EXE_LINKER_FLAGS}")
      add_definitions(-DSHAPEOP_OPENMP)
  else()
      message("OpenMP not found.")
  endif()
endif()

add_subdirectory(libShapeOp)
add_subdirectory(applications)
