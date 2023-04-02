#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenMeshCore" for configuration "Release"
set_property(TARGET OpenMeshCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMeshCore PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libOpenMeshCore.9.1.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libOpenMeshCore.9.1.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMeshCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMeshCore "${_IMPORT_PREFIX}/lib/libOpenMeshCore.9.1.dylib" )

# Import target "OpenMeshTools" for configuration "Release"
set_property(TARGET OpenMeshTools APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenMeshTools PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libOpenMeshTools.9.1.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libOpenMeshTools.9.1.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenMeshTools )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenMeshTools "${_IMPORT_PREFIX}/lib/libOpenMeshTools.9.1.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
