//
// Created by Linsanity on 2/10/22.
//

#ifndef TILEABLE_STRUCTURE_CUTTINGPLANEOPTIMIZER_H
#define TILEABLE_STRUCTURE_CUTTINGPLANEOPTIMIZER_H

#include "TileableStructure/BaseSurface.h"
#include "TileableStructure/CuttingPlane.h"

class CuttingPlane;

class CuttingPlaneOptimizer {

public:

    BaseSurface * baseSurface;

    float angleErrorThreshold;

    int miniBlockClusterThreshold;

    int targetBlockClusterNum;

    int blockClusterNum;

public:
    /// Initialization
    CuttingPlaneOptimizer(BaseSurface * _baseSurface);
    ~CuttingPlaneOptimizer();
    void InitCuttingPlanes();

    /// Block clustering
    int BlockClustering(PolyMesh * mySurface, bool isShowInfo = true);
    void CuttingPlaneAngleClustering(PolyMesh * mySurface);
    void BuildBlockClusterConnectionGraph(PolyMesh * mySurface);

    /// CuttingPlane Optimization
    void CuttingPlaneOptimization();
    void CuttingPlaneOptimization_mini();
    void CuttingPlaneOptimization_mini_without_boundary();
    void CuttingPlaneOptimization_mini_boundary();
    void CuttingPlaneOptimization_mini_boundary_only();
    void FindMiniBlockClusterIDList_without_boundary(PolyMesh * mySurface, int minBlockClusterThreshold, vector<int> & candiList, vector<float> & possList);
    void FindMiniBlockClusterIDList_with_boundary(PolyMesh * mySurface, int minBlockClusterThreshold, vector<int> & candiList, vector<float> & possList);
    void FindMiniBlockClusterIDList_boundary_only(PolyMesh * mySurface, int minBlockClusterThreshold, vector<int> & candiList, vector<float> & possList);
    void FindTargetPolygonClusterCandidates(int candiNum, vector<int> & candiPolygonClusterIndexList, vector<float> & possList);
    void CuttingPlaneOptimizationInPolygonCluster(int polygonClusterID);
    void FindBigBlockClusterCandidates(int polygonClusterID, int candiNum, vector<int> & candiBlockClusterIndexList, vector<float> & possList);
    void FindSmallBlockClusterCandidates(int polygonClusterID, int candiNum, vector<int> & candiBlockClusterIndexList, vector<float> & possList);
    void GenerateSelectedBlockIDPairs(const vector<int> & candiBlockClusterIndexList_big, const vector<float> & possList_big,
                                      const vector<int> & candiBlockClusterIndexList_small, const vector<float> & possList_small,
                                      vector< vector<int> > & selectedBlockIDPairs, vector<float> & possList_BlockIDPairs);
    bool MergeMiniBlockClusters_without_boundary(PolyMesh * mySurface, vector<int> & candiList, vector<float> & possList);
    bool MergeMiniBlockClusters_with_boundary(PolyMesh * mySurface, vector<int> & candiList, vector<float> & possList);
    bool MergeMiniBlockClusters_with_boundary(PolyMesh * mySurface, vector<int> & candiList, vector<float> & possList, vector< tuple<float, int> > & freeAnglePairList);
    bool MergeTwoBlockClusters(vector< vector<int> > & selectedBlockIDPairs, vector<float> & possList_BlockIDPairs);
    bool MergeTwoBlockClusters(vector< vector<int> > & selectedBlockIDPairs, vector<float> & possList_BlockIDPairs, vector< tuple<float, int> > & freeAnglePairList);
    bool MergeTwoBlockClusters(PolyMesh * mySurface, const vector<int> & currSelectedPair);
    bool MergeTwoBlockClusters(PolyMesh * mySurface, const vector<int> & currSelectedPair, vector< tuple<float, int> > & freeAnglePairList);
    void MatchingTwoEdgeLabelPairLists(vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big,
                                       vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small,
                                       vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big_matched,
                                       vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small_matched);
    void MatchingTwoEdgeLabelPairLists(vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big,
                                       vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small,
                                       vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big_matched,
                                       vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small_matched,
                                       vector< tuple<float, int> > & freeAnglePairList);
    void AdjustBlockAugmentedVector(PolyMesh * mySurface, PolyMesh::FaceHandle blockFH, vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList);
    void FindBestMatchingEdgeLabelPairList(const vector< tuple<float, int> > & origPairList, vector< tuple<float, int> > & matchedPairList);
    void FindOrigFreeAnglePairList(vector< tuple<float, int> > & freeAnglePairList);

    /// Helper Function
    bool IsTwoCuttingPlaneSame(OpenMesh::EdgeHandle inputEdgeHandle_1, OpenMesh::FaceHandle inputFaceHandle_1,
                               OpenMesh::EdgeHandle inputEdgeHandle_2, OpenMesh::FaceHandle inputFaceHandle_2);
};


#endif //TILEABLE_STRUCTURE_CUTTINGPLANEOPTIMIZER_H
