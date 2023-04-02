# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# compile CXX with /Library/Developer/CommandLineTools/usr/bin/c++
CXX_FLAGS = -O3 -DNDEBUG -isysroot /Library/Developer/CommandLineTools/SDKs/MacOSX13.0.sdk -mmacosx-version-min=12.5 -fPIE   -Wno-deprecated-declarations -std=gnu++14

CXX_DEFINES = -DBOOST_ATOMIC_DYN_LINK -DBOOST_ATOMIC_NO_LIB -DBOOST_FILESYSTEM_DYN_LINK -DBOOST_FILESYSTEM_NO_LIB -DBOOST_SYSTEM_DYN_LINK -DBOOST_SYSTEM_NO_LIB -DBOOST_THREAD_DYN_LINK -DBOOST_THREAD_NO_LIB -DCGAL_HEADER_ONLY=1 -DCGAL_USE_CORE=1 -DCGAL_USE_GMPXX=1 -DIMGUI_IMPL_OPENGL_LOADER_GLAD

CXX_INCLUDES = -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/pugixml -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/matplotlib_cpp -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/src -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/SYSTEM -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/pugixml/src -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/ShapeOp.0.1.0 -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/matplotlib -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../AABB_tree/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Advancing_front_surface_reconstruction/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Algebraic_foundations/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Algebraic_kernel_d/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Algebraic_kernel_for_circles/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Algebraic_kernel_for_spheres/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Alpha_shapes_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Alpha_shapes_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Apollonius_graph_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Arithmetic_kernel/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Arrangement_on_surface_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../BGL/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Barycentric_coordinates_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Boolean_set_operations_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Bounding_volumes/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Box_intersection_d/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../CGAL_Core/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../CGAL_ImageIO/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../CGAL_ipelets/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Cartesian_kernel/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Circular_kernel_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Circular_kernel_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Circulator/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Classification/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Combinatorial_map/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Cone_spanners_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Convex_decomposition_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Convex_hull_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Convex_hull_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Convex_hull_d/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Distance_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Distance_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Envelope_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Envelope_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Filtered_kernel/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Generalized_map/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Generator/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Geomview/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../GraphicsView/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../HalfedgeDS/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Hash_map/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Homogeneous_kernel/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Inscribed_areas/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Installation/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Interpolation/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Intersections_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Intersections_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Interval_skip_list/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Interval_support/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Inventor/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Jet_fitting_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Kernel_23/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Kernel_d/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../LEDA/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Linear_cell_complex/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Matrix_search/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Mesh_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Mesh_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Mesher_level/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Minkowski_sum_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Minkowski_sum_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Modifier/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Modular_arithmetic/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Nef_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Nef_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Nef_S2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../NewKernel_d/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Number_types/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../OpenNL/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Operations_on_polyhedra/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Optimal_transportation_reconstruction_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Optimisation_basic/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Partition_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Periodic_2_triangulation_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Periodic_3_triangulation_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Point_set_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Point_set_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Point_set_processing_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Point_set_shape_detection_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Poisson_surface_reconstruction_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polygon/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polygon_mesh_processing/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polyhedron/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polyhedron_IO/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polyline_simplification_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polynomial/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Polytope_distance_d/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Principal_component_analysis/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Principal_component_analysis_LGPL/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Profiling_tools/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Property_map/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../QP_solver/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Random_numbers/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Ridges_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../STL_Extension/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Scale_space_reconstruction_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../SearchStructures/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Segment_Delaunay_graph_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Segment_Delaunay_graph_Linf_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Set_movable_separability_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Skin_surface_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Snap_rounding_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Solver_interface/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Spatial_searching/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Spatial_sorting/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Straight_skeleton_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Stream_lines_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Stream_support/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Subdivision_method_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh_deformation/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh_parameterization/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh_segmentation/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh_shortest_path/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh_simplification/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesh_skeletonization/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_mesher/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Surface_sweep_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../TDS_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../TDS_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Testsuite/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Three/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Triangulation/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Triangulation_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Triangulation_3/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Union_find/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Visibility_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/cgal/Installation/lib/cmake/CGAL/../../../../Voronoi_diagram_2/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/glad/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/glfw/include -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/libigl-imgui/../imgui -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/libigl-imgui/. -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/libigl-imgui/.. -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/libigl-imgui/../imgui/examples -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/external/stb -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/cmake-build-debug/ext/stb_image -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Core/../.. -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/OpenMesh/src/OpenMesh/Tools/../.. -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/matplotlib/examples -I/Users/linsanity/Documents/GitHub/2022_TileableShell_public/src/Render -isystem /usr/local/include/boost/include -isystem /Users/linsanity/Documents/GitHub/2022_TileableShell_public/ext/libigl/cmake/../include -isystem /opt/homebrew/include/eigen3 -isystem /usr/local/include -iframework /Library/Developer/CommandLineTools/SDKs/MacOSX13.0.sdk/System/Library/Frameworks -isystem /Users/linsanity/opt/anaconda3/include/python3.9 -isystem /Users/linsanity/opt/anaconda3/lib/python3.9/site-packages/numpy/core/include 

