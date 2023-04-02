///////////////////////////////////////////////////////
//HelpFunc.h
//
// Additional Functions
//
// Created by Rulin Chen on 2022/04/26.
//
////////////////////////////////////////////////////////

#ifndef HELP_FUNC_H
#define HELP_FUNC_H

#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "Utility/dbg.h"
#include "Utility/HelpFunc_Internal.h"

#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>

typedef OpenMesh::PolyMesh_ArrayKernelT<>  PolyMesh;
typedef OpenMesh::TriMesh_ArrayKernelT<>   TriMesh;

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////
// Name Declaration
///////////////////////////////////////////////////////////////

#define CASE_FAB_READY    0
#define CASE_PENETRATION  1
#define CASE_GAP          2

#define DIR_UNDEFINED     0
#define DIR_NEG           1
#define DIR_POS           2

#define FREE_ANGLE   0
#define ERROR_ANGLE -1

#define SHOW_WHITE_FACE 0
#define SHOW_POLYGON_CLUSTER_COLOR 1
#define SHOW_BLOCK_CLUSTER_COLOR 2

///////////////////////////////////////////////////////////////
// Function Declaration
///////////////////////////////////////////////////////////////

// Color interpolation
void RedColorInterpolation(VectorXd planarityList, MatrixXd & colorMap);

// Debug
void PrintVerList(vector<Vector3d> & verList, string str);

// String Operations
void GetFolderPath(string filePath, string &folderPath);
void GetFolderPath(const char filePath[], char folderPath[]);
string GetFileType(const char filePath[]);
void GetFileName(string filePath, string & fileName);

// Mesh Volume and Polygon matching
Vector3d ComputePolygonCenter(vector<Vector3d> & verList);
void MoveToCenter(MatrixXd & verMat);
void MoveToCenter_ShapeOp(ShapeOp::Matrix3X & verMat);
void MoveToSameCenter_ShapeOp(ShapeOp::Matrix3X & referP, ShapeOp::Matrix3X & currP);
bool IsVerListEqual(vector<Vector3d> & verList_1, vector<Vector3d> & verList_2);
int ComputeMatchingNum(vector<int> v1, vector<int> v2);
int IsCurrListInExistingList(vector< vector<int> > existingList, vector<int> currList);
double ComputeAverageLength(const vector<Vector3d> & verList);
vector<Vector3d> CombineTwoVector(const vector<Vector3d> & upperList, const vector<Vector3d> & lowerList);
void SplitTwoVector(const vector<Vector3d> & combinedVector, vector<Vector3d> & upperList, vector<Vector3d> & lowerList);
Vector3d PolyMeshPoint2EigenVector3d(const PolyMesh::Point & p1);
vector<Vector3d> PolyMeshPointList2EigenVector3dList(const vector<PolyMesh::Point> & p1_list);
PolyMesh::Point EigenVector3d2PolyMeshPoint(const Vector3d & p1);
void Matrix2VectorList(const MatrixXd & V, vector<Vector3d> & verList);
bool IsTwoBlockMatrixEqual(const MatrixXd & V_1, const MatrixXd & V_2);
vector<Vector3d> ComputeMiddlePlane(const vector<Vector3d> & verList_1, const vector<Vector3d> & verList_2);
vector<Vector3d> CombineTwoBlockVerLists(const vector<Vector3d> & verList_1, const vector<Vector3d> & verList_2);
vector<Vector3d> CombineTwoBlockVerLists_clockwise(const vector<Vector3d> & verList_1, const vector<Vector3d> & verList_2);
Vector3d ComputePolygonCentroid(const vector<Vector3d> & verList);
Vector3d ComputePolygonCentroid(const MatrixXd & p);
Vector3d ComputeMatrixCentroid(const MatrixXd & p);
double AngleBetweenVectors(const Eigen::Vector3d & a, const Eigen::Vector3d & b);

// Mesh Volume Calculation
double ComputeSignedVolumeOfTriangle(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3);
double ComputeVolumeOfMesh(const MatrixXd & V_intersection, const MatrixXi & F_intersection);

// Utility Solver
double SolveEqua(double a, double b, double c);
double SolveEqua2(double a, double b, double c);
double SolvePointToCircleK1(double Pointx, double Pointy, double Cx, double Cy, double r);
double SolvePointToCircleK2(double Pointx, double Pointy, double Cx, double Cy, double r);
void UpIntersectionOfCircles(double ans[2], double O1_x, double O1_y, double O1_R, double O2_x, double O2_y, double O2_R);
void DownIntersectionOfCircles(double ans[2], double O1_x, double O1_y, double O1_R, double O2_x, double O2_y, double O2_R);
bool isIntersectOfCircles(double O1_x, double O1_y, double O1_R, double O2_x, double O2_y, double O2_R);

// PolyMesh data to libigl rendering data
void Matrix2PolyMesh(PolyMesh & polyMesh, MatrixXd V, MatrixXi F);
void PolyMesh2Matrix(PolyMesh polyMesh, MatrixXd & V, MatrixXi & F);
void PolyMesh2Edge(PolyMesh polyMesh, MatrixXd & p1_mat, MatrixXd & p2_mat);
void PolyMesh2Planarity(PolyMesh polyMesh, VectorXd & planarityList);
void PolyMesh2ClusteringLabel(PolyMesh polyMesh, VectorXd & labelList);
void PolyMesh2BlockClusterLabel(PolyMesh polyMesh, VectorXd & labelList);
void PolyMesh2EdgeClusteringLabel(PolyMesh polyMesh, VectorXd & labelList);
void PolyMesh2EdgeNormal(PolyMesh polyMesh, MatrixXd & p1_mat, MatrixXd & p2_mat);
void PolyMesh2FabTestLabel(PolyMesh polyMesh, MatrixXd & p1_mat, MatrixXd & p2_mat, VectorXd & fabLabelList);

// PolyMesh data and ShapeOp matrix data transfer
void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat, vector< vector<int> > & faceVerList);
void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat,
                            vector< vector<int> > & faceVerList, vector< vector<int> > & edgeVerList,
                            vector<double> & targetLengthList, vector< vector<int> > & angleVerList,
                            vector<double> & targetScaleList);
void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat,
                            vector< vector<int> > & faceVerList,
                            vector< vector<int> > & edgeVerList,vector<double> & targetLengthList,
                            vector< vector<int> > & angleVerList, vector<double> & targetScaleList,
                            vector< vector<int> > & diagonalVerList, vector<double> & targetDiagonalList);
void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat,
                            vector< vector<int> > & faceVerList,
                            vector< vector<int> > & edgeVerList,vector<double> & targetLengthList,
                            vector< vector<int> > & angleVerList, vector<double> & targetScaleList,
                            vector< vector<int> > & diagonalVerList, vector<double> & targetDiagonalList,
                            int & worstFabVer, vector< vector<int> > & worstPolygonEdgeList, vector< vector<int> > & worstDiagonal,
                            vector<int> & worstPlanarityVerList, vector< vector<int> > & worstDihedralAngleVerList);
void ShapeOpMatrix2PolyMesh(PolyMesh & polyMesh, Matrix3Xd verMat, vector< vector<int> > faceVerList);
void ShapeOpMatrix2VerList(Matrix3Xd verMat, vector<Vector3d> & verList);
void ShapeOpMatrix2OriPolyMesh(PolyMesh & polyMesh, Matrix3Xd verMat);

// PolyMesh property transfer
PolyMesh::HalfedgeHandle FindHalfedge(const PolyMesh & m, PolyMesh::VertexHandle v1, PolyMesh::VertexHandle v2);
PolyMesh::EdgeHandle FindEdge(const PolyMesh& m, PolyMesh::VertexHandle v1, PolyMesh::VertexHandle v2);
void NormalListPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh);
void EdgeNormalPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh);
void PlanarityPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh);
void ClusteringLabelPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh);
void BlockClusteringLabelPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh);

// PolyMesh Get Operation
void GetFaceVerList(PolyMesh * polyMesh, PolyMesh::FaceIter fh, vector<Vector3d> & verList);

// Least Squares Method
void FindFittedPlane_LSM(MatrixXd points, Vector3d & normal, double & d);
void FindFittedPlane_LSM(vector<Vector3d> verList, Vector3d & normal);

// Add Property to PolyMesh
void AddVertexTextureCoord2PolyMesh(PolyMesh & triMesh);

// Random Stuff
vector<int> GetRandomObjIndexList(vector<float> possibList, float alpha, int objNum);
int GetRandomObjIndex(vector<float> possibList, float alpha);
vector<float> PossibExpMapping(vector<float> possibList, float alpha);
float GetRandomNumber(int seedIndex);

#endif //HELPFUNC_H

