///////////////////////////////////////////////////////////////
//
// HelpTypedef.h
//
//   Rename some datatypes that are too long, e.g., those in libigl
//
// by Peng SONG
//
// 03/Dec/2020
//
//
///////////////////////////////////////////////////////////////


#ifndef _HELP_TYPEDEF_H
#define _HELP_TYPEDEF_H

#include <igl/arap.h>
#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/readOBJ.h>
#include <igl/lscm.h>
#include <igl/rotation_matrix_from_directions.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/ViewerData.h>
#include <igl/jet.h>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

///////////////////////////////////////////////////////////////////////


typedef igl::opengl::glfw::Viewer      iglViewer;
typedef igl::opengl::ViewerData        iglViewerData;

typedef OpenMesh::PolyMesh_ArrayKernelT<>  PolyMesh;
typedef OpenMesh::TriMesh_ArrayKernelT<>   TriMesh;

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef K::Plane_3 Plane_3;

///////////////////////////////////////////////////////////////////////


#endif //_HELP_TYPEDEF_H
