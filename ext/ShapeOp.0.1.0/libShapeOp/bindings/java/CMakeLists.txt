cmake_minimum_required(VERSION 3.1)
find_package(SWIG QUIET)
if(SWIG_FOUND)
  message("Swig found. Building Java bindings.")
  project(libShapeOpJava)
  include(${SWIG_USE_FILE})

  find_package(JNI)
  if(JNI_FOUND)
    message("Java found.")
    include_directories(${JAVA_INCLUDE_PATH})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    set(CMAKE_SWIG_FLAGS "")

    set_source_files_properties(shapeop.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(shapeop.i PROPERTIES SWIG_FLAGS "-includeall")

    swig_add_module(shapeopJava java shapeop.i ${SOURCES})
    swig_link_libraries(shapeopJava ${JAVA_LIBRARIES})

    configure_file(runme.java runme.java COPYONLY)
  else()
    message("Java not found.")
  endif()
else(SWIG_FOUND)
        message("Swig not found. Not building Java bindings.")
endif(SWIG_FOUND)
