include (VCICommon)

set( UNITTEST_SRC
unittests.cc
unittests_add_face.cc
unittests_boundary.cc
unittests_centroid_calculations.cc
unittests_convert_meshes.cc
unittests_cpp_11_features.cc
unittests_decimater.cc
unittests_delete_face.cc
unittests_eigen3_type.cc
unittests_faceless_mesh.cc
unittests_mc_decimater.cc
unittests_mesh_cast.cc
unittests_mesh_dual.cc
unittests_mesh_type.cc
unittests_mixed_decimater.cc
unittests_normal_calculations.cc
unittests_polymesh_collapse.cc
unittests_polymesh_vec2i.cc
unittests_property.cc
unittests_propertymanager.cc
unittests_randomNumberGenerator.cc
unittests_read_write_OBJ.cc
unittests_read_write_OFF.cc
unittests_read_write_OM.cc
unittests_read_write_PLY.cc
unittests_read_write_STL.cc
unittests_set_positions_directly.cc
unittests_smart_handles.cc
unittests_smart_ranges.cc
unittests_smarttagger.cc
unittests_smoother.cc
unittests_split_copy.cc
unittests_split_edge_copy.cc
unittests_sr_binary.cc
unittests_stripifier.cc
unittests_subdivider_adaptive.cc
unittests_subdivider_uniform.cc
unittests_trimesh_circulator_current_halfedge_handle_replacement.cc
unittests_trimesh_circulator_face_edge.cc
unittests_trimesh_circulator_face_face.cc
unittests_trimesh_circulator_face_halfedge.cc
unittests_trimesh_circulator_face_vertex.cc
unittests_trimesh_circulator_halfedge_loop.cc
unittests_trimesh_circulator_vertex_edge.cc
unittests_trimesh_circulator_vertex_face.cc
unittests_trimesh_circulator_vertex_ihalfedge.cc
unittests_trimesh_circulator_vertex_ohalfedge.cc
unittests_trimesh_circulator_vertex_vertex.cc
unittests_trimesh_collapse.cc
unittests_trimesh_garbage_collection.cc
unittests_trimesh_iterators.cc
unittests_trimesh_others.cc
unittests_trimesh_ranges.cc
unittests_trimesh_split.cc
unittests_trimesh_vec2i.cc
unittests_tutorials.cc
unittests_vdpm.cc
unittests_vector_type.cc
)



if ( NOT DEFINED OPENMESH_BUILD_UNIT_TESTS)
    set( OPENMESH_BUILD_UNIT_TESTS false CACHE BOOL "Enable or disable unit test builds in OpenMesh." )
endif()

if ( OPENMESH_BUILD_UNIT_TESTS )
  # Search for gtest headers and libraries
  find_package(GTest)

  if(GTEST_FOUND)

    enable_testing()
   
    find_package(Eigen3)

    # Set correct include paths so that the compiler can find the headers
    include_directories(${GTEST_INCLUDE_DIRS} )


    # set additional link directories
    link_directories(${GTEST_LIBRARY_DIR} )

    
    if (TARGET Eigen3::Eigen)
      add_definitions( -DENABLE_EIGEN3_TEST ) 
      link_libraries(Eigen3::Eigen)
    else()
      message(WARNING "Eigen3 not found! This will skip the Eigen3 Unittests. You can point cmake to Eigen3 by setting Eigen3_DIR to the cmake files of Eigen3")
    endif()

    # Create new target named unittests_hexmeshing
    # Create unittest executable
    vci_add_executable(unittests ${UNITTEST_SRC})
    vci_add_executable(unittests_customvec ${UNITTEST_SRC})
    vci_add_executable(unittests_doublevec ${UNITTEST_SRC})
    target_compile_definitions(unittests_customvec PRIVATE TEST_CUSTOM_TRAITS)
    target_compile_definitions(unittests_doublevec PRIVATE TEST_DOUBLE_TRAITS)
 
    # For the unittest we don't want the install rpath as set by vci_add_executable 
    set_target_properties ( unittests PROPERTIES  BUILD_WITH_INSTALL_RPATH 0 )
    set_target_properties ( unittests_customvec PROPERTIES  BUILD_WITH_INSTALL_RPATH 0 )
    set_target_properties ( unittests_doublevec PROPERTIES  BUILD_WITH_INSTALL_RPATH 0 )

   # Set output directory to ${BINARY_DIR}/Unittests
    set (OUTPUT_DIR "${CMAKE_BINARY_DIR}/Unittests")
      set_target_properties(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
      set_target_properties(unittests_customvec PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
      set_target_properties(unittests_doublevec PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
      foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${CONFIG} UPCONFIG)
        set_target_properties(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${UPCONFIG} ${OUTPUT_DIR})
        set_target_properties(unittests_customvec PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${UPCONFIG} ${OUTPUT_DIR})
        set_target_properties(unittests_doublevec PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${UPCONFIG} ${OUTPUT_DIR})
    endforeach()

	
	if ( NOT WIN32)
           # Link against all necessary libraries
           target_link_libraries(unittests OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
           target_link_libraries(unittests_customvec OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
           target_link_libraries(unittests_doublevec OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} pthread)
	  
	  
	
       else()
	    # Link against all necessary libraries
		if ( OPENMESH_BUILD_SHARED )
			add_definitions( -DOPENMESHDLL )

		endif()
      target_link_libraries(unittests OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
      target_link_libraries(unittests_customvec OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
      target_link_libraries(unittests_doublevec OpenMeshCore OpenMeshTools ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    endif()
	

    
	if ( NOT WIN32 )
	    # Set compiler flags
      set_target_properties(unittests PROPERTIES COMPILE_FLAGS "-g -pedantic -Wno-long-long")
      set_target_properties(unittests_customvec PROPERTIES COMPILE_FLAGS "-g -pedantic -Wno-long-long")
      set_target_properties(unittests_doublevec PROPERTIES COMPILE_FLAGS "-g -pedantic -Wno-long-long")
	else()
	    # Set compiler flags
      set_target_properties(unittests PROPERTIES COMPILE_FLAGS "" )
      set_target_properties(unittests_customvec PROPERTIES COMPILE_FLAGS "" )
      set_target_properties(unittests_doublevec PROPERTIES COMPILE_FLAGS "" )
    endif()
    
    if ( OPENMESH_BUILD_SHARED )
	
      #-------- copy dlls to unittests --------
      SET(OPENMESH_TARGETS "OpenMeshTools" "OpenMeshCore")
    
       foreach(TAR ${OPENMESH_TARGETS} )
         add_custom_command(TARGET unittests POST_BUILD 
                            COMMAND "${CMAKE_COMMAND}" -E copy 
                            "$<TARGET_FILE:${TAR}>"
                            "${CMAKE_BINARY_DIR}/Unittests/$<TARGET_FILE_NAME:${TAR}>" 
                            COMMENT "Copying OpenMesh targets to unittests directory")
         add_custom_command(TARGET unittests_customvec POST_BUILD
                            COMMAND "${CMAKE_COMMAND}" -E copy
                            "$<TARGET_FILE:${TAR}>"
                            "${CMAKE_BINARY_DIR}/Unittests/$<TARGET_FILE_NAME:${TAR}>"
                            COMMENT "Copying OpenMesh targets to unittests directory")
         add_custom_command(TARGET unittests_doublevec POST_BUILD
                            COMMAND "${CMAKE_COMMAND}" -E copy
                            "$<TARGET_FILE:${TAR}>"
                            "${CMAKE_BINARY_DIR}/Unittests/$<TARGET_FILE_NAME:${TAR}>"
                            COMMENT "Copying OpenMesh targets to unittests directory")
       endforeach(TAR)
     endif()
	
    vci_copy_after_build(unittests ${CMAKE_CURRENT_SOURCE_DIR}/TestFiles ${CMAKE_BINARY_DIR}/Unittests/)
    vci_copy_after_build(unittests_customvec ${CMAKE_CURRENT_SOURCE_DIR}/TestFiles ${CMAKE_BINARY_DIR}/Unittests/)
    vci_copy_after_build(unittests_doublevec ${CMAKE_CURRENT_SOURCE_DIR}/TestFiles ${CMAKE_BINARY_DIR}/Unittests/)

    add_test(NAME AllTestsIn_OpenMesh_tests WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Unittests" COMMAND "${CMAKE_BINARY_DIR}/Unittests/unittests")
    add_test(NAME AllTestsIn_OpenMesh_tests_with_minimal_vector WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Unittests" COMMAND "${CMAKE_BINARY_DIR}/Unittests/unittests_customvec")
    add_test(NAME AllTestsIn_OpenMesh_tests_with_double_vector WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/Unittests" COMMAND "${CMAKE_BINARY_DIR}/Unittests/unittests_doublevec")

  else(GTEST_FOUND)
      message(STATUS "Google testing framework was not found, unittests disabled.")
  endif(GTEST_FOUND)
endif()
