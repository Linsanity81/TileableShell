# Install script for directory: /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools

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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Decimater" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/BaseDecimaterT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/BaseDecimaterT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/CollapseInfoT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/DecimaterT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/DecimaterT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/McDecimaterT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/McDecimaterT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/MixedDecimaterT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/MixedDecimaterT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModAspectRatioT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModAspectRatioT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModBaseT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModEdgeLengthT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModEdgeLengthT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModHausdorffT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModHausdorffT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModIndependentSetsT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModNormalDeviationT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModNormalFlippingT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModProgMeshT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModProgMeshT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModQuadricT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModQuadricT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/ModRoundnessT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Decimater/Observer.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Dualizer" TYPE FILE FILES "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Dualizer/meshDualT.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Kernel_OSG" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/ArrayKernelT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/AttribKernelT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/PropertyKernel.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/PropertyT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/Traits.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/TriMesh_OSGArrayKernelT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/VectorAdapter.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/bindT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Kernel_OSG/color_cast.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Smoother" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/LaplaceSmootherT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/LaplaceSmootherT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/SmootherT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/SmootherT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Smoother/smooth_mesh.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Subdivider/Adaptive/Composite" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeTraits.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/RuleInterfaceT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/RulesT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/RulesT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Adaptive/Composite/Traits.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Subdivider/Uniform" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/CompositeLoopT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/CompositeSqrt3T.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/LongestEdgeT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/LoopT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/MidpointT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/ModifiedButterFlyT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/Sqrt3InterpolatingSubdividerLabsikGreinerT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/Sqrt3T.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/SubdividerT.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Subdivider/Uniform/Composite" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/Composite/CompositeT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/Composite/CompositeT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Subdivider/Uniform/Composite/CompositeTraits.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/Utils" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/Config.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/GLConstAsString.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/Gnuplot.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/HeapT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/MeshCheckerT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/MeshCheckerT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/NumLimitsT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/StripifierT.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/StripifierT_impl.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/TestingFramework.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/Timer.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/conio.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/Utils/getopt.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenMesh/Tools/VDPM" TYPE FILE FILES
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/MeshTraits.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/StreamingDef.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/VFront.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/VHierarchy.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/VHierarchyNode.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/VHierarchyNodeIndex.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/VHierarchyWindow.hh"
    "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/VDPM/ViewingParameters.hh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/OpenMesh/src/OpenMesh/Tools/libOpenMeshTools.9.1.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.9.1.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.9.1.dylib")
    execute_process(COMMAND /Users/linsanity/opt/anaconda3/bin/install_name_tool
      -delete_rpath "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/OpenMesh/src/OpenMesh/Core"
      -add_rpath "@executable_path/../lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.9.1.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.9.1.dylib")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/OpenMesh/src/OpenMesh/Tools/libOpenMeshTools.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.dylib")
    execute_process(COMMAND /Users/linsanity/opt/anaconda3/bin/install_name_tool
      -delete_rpath "/Users/linsanity/Documents/GitHub/2022_TileableShell_public/build/ext/OpenMesh/src/OpenMesh/Core"
      -add_rpath "@executable_path/../lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenMeshTools.dylib")
    endif()
  endif()
endif()

