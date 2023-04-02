//
// Created by Linsanity on 18/4/22.
//

#ifndef TILEABLE_STRUCTURE_SurfaceOptimizer_H
#define TILEABLE_STRUCTURE_SurfaceOptimizer_H

#include <Eigen/Eigen>
#include <vector>
#include <numeric>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>
#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"
#include "TileableStructure/BaseSurface.h"
#include "TileableStructure/ShellStructure.h"
#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/DefaultTriMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

using namespace std;
using namespace Eigen;

class SurfaceOptimizer {

public:
    BaseSurface * baseSurface;

    /// Linked to Shell Structure
    ShellStructure * shellStructure;

    // Status
    int status;

    // Edge Clustering
    int edgeClusteringNum_start;
    int edgeClusteringNum_end;
    float edgeErrorThreshold;

    float w1_edge;
    float w1_planarity;
    int iter_1;

    // Dihedral Angle Clustering
    int dihedralClusterNum_start;
    int dihedralClusterNum_end;
    float dihedralAngleErrorThreshold;

    float w2_edge;
    float w2_dihedralAngle;
    float w2_planarity;
    int iter_2;

    // Polygon Clustering
    int polygonClusterNum;
    int polygonClusterOpIter;

    float w3_edge;
    float w3_dihedralAngle;
    float w3_diagonal;
    float w3_planarity;
    int iter_3;

    // Cluster Merging
    int miniPolygonClusterNumThreshold;
    int mergingCandidateNum;

    // Worst-case Optimization
    int iter_worst;

    float worstPlanarityScalar;
    float worstDiagonalScalar;
    float worstEdgeScalar;
    float worstDihedralAngleScalar;

    // Optimization time
    float surfaceOpTime;

    // FolderName
    string savingFullPath;

public:
    SurfaceOptimizer(BaseSurface * _baseSurface, ShellStructure * _shellStructure);
    ~SurfaceOptimizer();
    void ClearSurfaceOptimizerData();

    /// Polygon Optimization
    void PolygonOptimization();
    void PolygonOptimization_Merging();

    /// Clustering and optimization for edge of baseSurface
    void EdgeClusteringAndOptimization();
    void EdgeClustering_HierarchicalClustering(int clusteringNum);
    void EdgeClustering_kMeans(int clusteringNum);
    void ShapeOptimization_Edge();

    /// Clustering and optimization for edge and dihedral angle of baseSurface
    void EdgeDihedralAngleClusteringAndOptimization();
    void DihedralAngleClustering_HierarchicalClustering(int clusteringNum);
    void DihedralAngleClustering_kMeans(int clusteringNum);
    void ShapeOptimization_Edge_DihedralAngle();

    /// Clustering and optimization for edge and dihedral angle and polygon
    void PolygonClusteringAndOptimization();
    void PolygonClustering(bool isShowInfo = true);
    void ShapeOptimization_PolygonClustering();

    /// Cluster merging
    void ClusterMerging();
    void FindSmallClusterIndexList(vector<int> & indexList);
    void FindSmallestClusterIndex(int & index, int & minClusterNum);
    void FindMergingCandidates(int index, vector<int> & mergingCandidateIndexList);
    bool MergeCluster(int index, vector<int> mergingCandidateIndexList);

    /// Outlier optimization
    void WorstCaseOptimization();
    void ShapeOptimization_WorstCaseOptimization();

    /// Helper Functions
    void ComputePolygonSimilarity(vector<Vector3d> referP, vector<Vector3d> currP, double & similarity);

    /// Save files
    void SaveInput(string folderPath);
    void SaveSurface(string folderPath);
    void SavePolygonClusteringFiles();
    void SaveBlockClusteringFiles();
    void CreateFolderName(string folderPath, string surfacePath, string tilePath);
    void SaveEdgeDihedralAngleClusteringAnalysisFiles(string folderPath);
    void SavePolygonClusteringAnalysisFiles(string folderPath);
    void SaveParameter2File(string folderPath);
    void SaveFinalResult2File(string folderPath);
    void SaveSurfaceOptimizationResult(string folderPath);
    void SaveSurfaceOBJs(string folderPath);
    void SaveAllClusteringPolygonOBJs(string folderPath, string templatePath);
    void SaveClusterPolygonOBJs(string folderPath, string templatePath, int clusterID);
    void SaveBlockClusteringAnalysisFiles(string folderPath);
    void SaveShellStructureOBJs(string folderPath);
};

#endif //TILEABLE_STRUCTURE_SurfaceOptimizer_H
