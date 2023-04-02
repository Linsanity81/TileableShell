//
// Created by Linsanity on 18/4/22.
//

#ifndef TILEABLE_STRUCTURE_BaseSurface_H
#define TILEABLE_STRUCTURE_BaseSurface_H

#include <Eigen/Eigen>
#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>
#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"

using namespace std;
using namespace Eigen;

class Surface;
class TilePattern;

class BaseSurface
{

public:
    /// Input Surface
    Surface *inputSurface;

    /// Input Pattern
    TilePattern *inputTilePattern;

    /// Input data
    PolyMesh remeshedPolySurface;
    PolyMesh remeshedTriSurface;

//    PolyMesh remeshedPolySurface_without_shapeop;
//    PolyMesh remeshedTriSurface_without_shapeop;

    /// Data that need to be processed
    PolyMesh baseSurface;
    PolyMesh triBaseSurface;

    /// replacedSurface
    PolyMesh replacedSurface;
    PolyMesh replacedTriSurface;

    /// Fabrication Constraint
    float tolerance;
    float fabToleranceScalar;
    float absoluteTolerance;
    float fabricationThreshold;

    /// Planarity Constraint
    float planarityThreshold;

    /// Dihedral Angle Cluster Error Constraint
    float dihedralAngleClusterErrorThreshold;

    /// Worst-case Analysiss
    float worstFabApproximation;

    float avgPolygonSimilarity;
    float worstPolygonSimilarity;
    float E_polygon;

    float avgPlanarity;
    float worstPlanarity;
    float E_planar;

    float avgDihedralAngleError;
    float worstDihedralAngleError;
    float E_dihed;

    float avgEdgeLengthError;
    float worstEdgeLengthError;
    float E_edge;

    /// Mesh term
    float closeness;
    float smoothness;

public:
    /// Initialization for baseSurface
    BaseSurface(Surface *_inputSurface, TilePattern *_inputTilePattern);
    ~BaseSurface();
    void ClearSurface();

    /// Intialization for replacedSurface
    void InitReplacedSurfaceAndToleranceParameters();
    void InitReplacedSurface();
    void InitToleranceParameters();

    /// Read and Write
    void ReadBaseSurface(char *fileName);
    void WriteBaseSurface(char *fileName);
    void WriteRemeshedBaseSurface(string filePath);

    /// Update Operation
    // 1) for edge clustering
    void UpdateEdgeLength();
    void UpdateEdgeClusterCentroidList();

    // 2) for dihedral angle clustering
    void UpdateDihedralAngle();
    void UpdateDihedralAngleClusterCentroidList();

    // 3) for polygon clustering
    void UpdateDiagnonalList();
    void UpdatePolygonSimilarityTable_LSM();
    void ComputeClusterSimilarityList(int clusterID, vector<double> &similarityList,
                                      vector<Vector3d> &centroidPolygonVerList);
    void ComputeClusterCentroidPolygon_LSM(int clusterID, vector<Vector3d> &centroidPolygonVerList,
                                           vector<vector<Vector3d>> &clusteredAlignedPolygons, vector<PolyMesh::FaceHandle> &currClusterFaceHandleList);

    // 4) others
    void UpdateEdgeNormal(PolyMesh *myMesh, PolyMesh *myTriMesh);
    void UpdatePolygonPlanarity_LSM(PolyMesh *myMesh, PolyMesh *myTriMesh);

    // 5) for block clustering
    void UpdateBlockSimilarityTable_LSM();
    void ComputeClusterSimilarityList(int clusterID, vector<double> &similarityList,
                                      tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock,
                                      tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock_combined);
    void ComputeClusterCentroidBlock_LSM(int clusterID, tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock,
                                         vector<tuple<vector<Vector3d>, vector<Vector3d>>> &clusteredAlignedBlocks);

    void UpdateBlockSimilarityTable_LSM_planar();
    void ComputeClusterSimilarityList_planar(int clusterID, vector<double> &similarityList,
                                      tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock_planar);
    void ComputeClusterCentroidBlock_LSM_planar(int clusterID, tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock_planar,
                                         vector<tuple<vector<Vector3d>, vector<Vector3d>>> &clusteredAlignedBlocks_planar);

    /// Worst-case Analysis
    void WorstCaseAnalysis();

    // Check worst case: 1) optimized surface meets fabrication requirement (penetration / gap)
    void ComputeWorstCaseSurfaceMeetFabRequirement();
    int IsVertexMeetFabRequirement(Vector3d &origVec, Vector3d &movedVec, double &result);

    // Check worst case: 2) polygon similarity
    void ComputeWorstPolygonSimilarityCase();

    // Check worst case: 3) planarity
    void ComputeWorstPlanarityCase();

    // Check worst case: 4) dihedral angle
    void ComputeWorstDihedralAngleError();

    // Check worst case: 5) edge length
    void ComputeWorstEdgeLengthError();

    /// Surface analysis
    void SurfaceAnalysis();
    void ComputeSurfaceCloseness();
    void ComputeSurfaceSmoothness();

    /// Get Edge Operation
    void GetBaseSurfaceEdgeLengthList(vector<double> &edgeLengthList);
    void GetRemeshedSurfaceEdgeLengthList(vector<double> &edgeLengthList);
    void GetEdgeClusterList(vector<vector<double>> &edgeClusterList, int clusterNum);
    void GetEdgeTupleList(vector<std::tuple<PolyMesh::Point, PolyMesh::Point, double>> &edgeTupleList);
    void GetEdgeClusterErrorList(vector<vector<double>> &edgeClusterList, vector<std::tuple<double, double, double>> &edgeClusterErrorList, float threshold);

    /// Get dihedral angle operation
    void GetBaseSurfaceDihedralAngleList(vector<double> &angleList);
    void GetRemeshedSurfaceDihedralAngleList(vector<double> &angleList);
    void GetDihedralAngleClusterList(vector<vector<double>> &dihedralAngleClusterList, int clusterNum);
    void GetDihedralAngleClusterErrorList(vector<vector<double>> &dihedralAngleClusterList, vector<std::tuple<double, double, double>> &dihedralAngleClusterErrorList, float threshold);

    /// Get polygon cluster operation
    int GetPolygonClusterNum();
    void GetDiagonalLengthList(vector<double> &diagonalLengthList);
    void GetPlanarityList(vector<double> &planarityList);
    int ComputePolygonClusterNumByEdgeLabel();

    /// Get block cluster operation
    int GetBlockClusterNum();
    vector<int> GetBlockClusterBasedOnPolygonCluster(int polygonClusterID);

    /// Disassembly Planning
    void ComputeDisassemblyPlan();
    bool IsRemovableFace(const PolyMesh::FaceHandle &currFH);

    /// Helper Function
    void AlignBlock_LSM(const vector<Vector3d> &referBlockVerList, vector<Vector3d> &currUpperList, vector<Vector3d> &currLowerList, MatrixXd &p);
    void FindBestMatchingVerListWithFlipping(const vector<Vector3d> &currReferVerList, vector<Vector3d> &currList, vector<Vector3d> &bestMatchingVertexList);
    void FindBestMatchingVerListWithFlipping(const vector<Vector3d> &currReferVerList, vector<Vector3d> &currList, vector<Vector3d> &bestMatchingVertexList, double & simi);
    void FindBlockBestMatchingTransMatWithoutFlipping(const vector<Vector3d> &currReferVerList, vector<Vector3d> &currList, vector<Vector3d> &bestMatchingVerList, MatrixXd &transMat);
    void FindBestMatchingBlockVerListWithoutFlipping(const vector<Vector3d> &referCombinedVerList, vector<Vector3d> &currUpperList, vector<Vector3d> &currLowerList, vector<Vector3d> &bestMatchingVertexList);
    void ComputePolygonSimilarity_ShapeOp_FixedConfig(const vector<Vector3d> &referP, const vector<Vector3d> &currP, double &similarity);
    void ComputePolygonSimilarity_LSM_FixedConfig(const vector<Vector3d> &referP, const vector<Vector3d> &currP, double &similarity);
    void ComputePolygonBestTransMat_LSM_FixedConfig(const vector<Vector3d> &referP, const vector<Vector3d> &currP, double &similarity, MatrixXd &transMat);
    void AlignPolygon_ShapeOp_FixedConfig(const vector<Vector3d> &referP, const vector<Vector3d> &currP, MatrixXd &alignedP_mat);
    void AlignPolygon_LSM_FixedConfig(const vector<Vector3d> &referP, const vector<Vector3d> &currP, MatrixXd &alignedP_mat);

    /// Polygon Alignment Experiment

};

#endif // TILEABLE_STRUCTURE_BaseSurface_H
