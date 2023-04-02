# Install script for directory: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/Geometry" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/Config.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/EigenVectorT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/LoopSchemeMaskT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/MathDefs.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/NormalConeT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/NormalConeT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/Plane3d.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/QuadricT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/Vector11T.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/VectorT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Geometry/VectorT_inc.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/IO" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/BinaryHelper.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/IOInstances.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/IOManager.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/MeshIO.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/OFFFormat.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/OMFormat.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/OMFormatT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/Options.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/SR_binary.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/SR_binary_spec.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/SR_binary_vector_of_bool.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/SR_rbo.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/SR_store.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/SR_types.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/StoreRestore.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/IO/importer" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/importer/BaseImporter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/importer/ImporterT.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/IO/exporter" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/exporter/BaseExporter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/exporter/ExporterT.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/IO/reader" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/reader/BaseReader.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/reader/OBJReader.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/reader/OFFReader.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/reader/OMReader.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/reader/PLYReader.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/reader/STLReader.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/IO/writer" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/BaseWriter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/OBJWriter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/OFFWriter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/OMWriter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/PLYWriter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/STLWriter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/IO/writer/VTKWriter.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/Mesh" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/ArrayItems.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/ArrayKernel.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/ArrayKernelT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/AttribKernelT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/Attributes.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/BaseKernel.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/BaseMesh.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/Casts.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/CirculatorsT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/DefaultPolyMesh.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/DefaultTriMesh.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/FinalMeshItemsT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/Handles.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/IteratorsT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/PolyConnectivity.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/PolyConnectivity_inline_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/PolyMeshT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/PolyMeshT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/SmartHandles.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/SmartRange.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/Status.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/Tags.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/Traits.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/TriConnectivity.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/TriMeshT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/TriMeshT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/Mesh/gen" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/gen/circulators_header.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/gen/circulators_template.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/gen/footer.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/gen/iterators_header.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Mesh/gen/iterators_template.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/System" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/System/OpenMeshDLLMacros.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/System/compiler.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/System/config.h"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/System/config.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/System/mostream.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/System/omstream.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Core/Utils" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/AutoPropertyHandleT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/BaseProperty.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/Endian.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/GenProg.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/HandleToPropHandle.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/Noncopyable.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/Predicates.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/Property.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/PropertyContainer.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/PropertyCreator.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/PropertyManager.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/RandomNumberGenerator.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/SingletonT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/SingletonT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/color_cast.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/typename.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/vector_cast.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/Utils/vector_traits.hh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/OpenMesh/src/OpenMesh/Core/libOpenMeshCore.9.1.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.9.1.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.9.1.dylib")
    execute_process(COMMAND /Users/linsanity/opt/anaconda3/bin/install_name_tool
      -add_rpath "@executable_path/../lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.9.1.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.9.1.dylib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/OpenMesh/src/OpenMesh/Core/libOpenMeshCore.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.dylib")
    execute_process(COMMAND /Users/linsanity/opt/anaconda3/bin/install_name_tool
      -add_rpath "@executable_path/../lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshCore.dylib")
    endif()
  endif()
endif()

