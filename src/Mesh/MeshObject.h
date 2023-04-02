///////////////////////////////////////////////////////////////
//
// MeshObject.h
//
//   An object represented as a group of meshes, mainly for rendering
//
// by Peng Song
//
// 29/Nov/2020
//
//
///////////////////////////////////////////////////////////////


#ifndef _MESH_OBJECT_H
#define _MESH_OBJECT_H

#include <igl/opengl/ViewerData.h>

using namespace std;
using namespace Eigen;

class Mesh;

class MeshObject
{

public:
    MeshObject() {};
    ~MeshObject() {};

    /// Ground
    vector<igl::opengl::ViewerData> CreateGround(Vector3d origin, double size, int gridNum, int sampNum);

    /// Axes
    vector<igl::opengl::ViewerData> CreateAxes(Vector3d origin, double size, int sampNum);

    /// Obj Model

    // General
    vector<igl::opengl::ViewerData> CreateObjModel(MatrixX3d verM, MatrixX3i triM, RowVector3d color);

    // Given surface
    vector<igl::opengl::ViewerData> CreateObjModel(MatrixX3d verM, MatrixX3i triM);
    vector<igl::opengl::ViewerData> CreateObjModelWithEdgeCluster(MatrixX3d verM, MatrixX3i triM,
                                                                MatrixX3d e1_mat, MatrixX3d e2_mat,
                                                                VectorXd edgeLabelList);

    // Remeshed surface
    vector<igl::opengl::ViewerData> CreateRemeshedSurface(MatrixX3d verM, MatrixX3i triM,
                                                          MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                          MatrixX3d e1_mat, MatrixX3d e2_mat,
                                                          VectorXd planarityList, bool isShowEdgeNormal);

    // Texture
    vector<igl::opengl::ViewerData> CreateTexture(MatrixX3d verM, MatrixX3i triM,
                                                  MatrixX3d p1_mat, MatrixX3d p2_mat, bool isShowLine);

    // Optimized surface
    vector<igl::opengl::ViewerData> CreateOptimizedSurface(MatrixX3d verM, MatrixX3i triM,
                                                           MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                           MatrixX3d e1_mat, MatrixX3d e2_mat,
                                                           VectorXd edgeLabelList, VectorXd polygonLabelList,
                                                           bool isShowEdgeClusterColor = false, bool isShowPolygonColor = true, bool isShowEdgeNormal = false);

    // Replaced surface
    vector<igl::opengl::ViewerData> CreateReplacedSurfaceModel(MatrixX3d verM, MatrixX3i triM,
                                                               MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                               MatrixX3d fp1_mat, MatrixX3d fp2_mat,
                                                               VectorXd fabLabelList);

    // Shell structure
    vector<igl::opengl::ViewerData> CreateShellStructure(MatrixX3d verM, MatrixX3i triM,
                                                           MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                           VectorXd polygonLabelList);

    /// Volume
    vector<igl::opengl::ViewerData> CreateVolume(MatrixXd voxelMinPts,
                                                 MatrixXd voxelMaxPts,MatrixXd voxelCenPts);

    /// Fuction for testing MeshCreator
    vector<igl::opengl::ViewerData> Function_Test();

};


#endif //_MESH_OBJECT_H
