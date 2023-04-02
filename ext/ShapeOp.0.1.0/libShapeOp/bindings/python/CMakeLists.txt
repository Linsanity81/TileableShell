cmake_minimum_required(VERSION 3.1)
find_package(SWIG QUIET)
if(SWIG_FOUND)
  message("Swig found. Building Python bindings.")
  project(libShapeOpPython)
  include(${SWIG_USE_FILE})

  find_package(PythonLibs)
  if(PYTHONLIBS_FOUND)
    message("Python found.")
    include_directories(${PYTHON_INCLUDE_PATH})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    set(CMAKE_SWIG_FLAGS "")

    set_source_files_properties(shapeop.i PROPERTIES CPLUSPLUS ON)
    set_property(SOURCE shapeop.i PROPERTY SWIG_FLAGS "-includeall") #add "-py3" here to generate python 3 code

    swig_add_module(shapeopPython python shapeop.i ${SOURCES})
    swig_link_libraries(shapeopPython ${PYTHON_LIBRARIES})

    configure_file(runme.py runme.py COPYONLY)
  else()
    message("Python not found.")
  endif()
else(SWIG_FOUND)
        message("Swig not found. Not building Python bindings.")
endif(SWIG_FOUND)
