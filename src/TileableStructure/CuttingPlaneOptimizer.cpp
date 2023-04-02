//
// Created by Linsanity on 2/10/22.
//

#include "CuttingPlaneOptimizer.h"

///=========================================================================================///
///                                        Initialization
///=========================================================================================///

CuttingPlaneOptimizer::CuttingPlaneOptimizer(BaseSurface * _baseSurface)
{
    baseSurface = _baseSurface;

    angleErrorThreshold = 0.1;

    miniBlockClusterThreshold = 10;

    targetBlockClusterNum = 1;
}

CuttingPlaneOptimizer::~CuttingPlaneOptimizer()
{
    baseSurface = nullptr;
}

void CuttingPlaneOptimizer::InitCuttingPlanes()
{
    // Update edge normal
    baseSurface->UpdateEdgeNormal(&baseSurface->baseSurface, &baseSurface->triBaseSurface);

    // Add cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface->baseSurface.get_property_handle( cuttingPlane ,"cuttingPlane");

    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        CuttingPlane currCuttingPlane(*e_it, baseSurface);

        baseSurface->baseSurface.property(cuttingPlane, *e_it) = currCuttingPlane;

//        baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(10);

//        if (baseSurface->baseSurface.is_boundary(*e_it))
//        {
//            baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(baseSurface->baseSurface.property(cuttingPlane, *e_it).positiveFaceHandle, 100.0f);
//        }

//        baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(baseSurface->baseSurface.property(cuttingPlane, *e_it).positiveFaceHandle, 100.0f);

//        baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(30);
    }

//    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
//    {
//
//        baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(baseSurface->baseSurface.property(cuttingPlane, *e_it).negativeFaceHandle, 90.0f);
//
// //    }

//    int i = 0;
//    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
//    {
//        if (i % 2 != 0)
//            baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(-30);
//
//        ++i;
//    }
//
//    i = 0;
//    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
//    {
//        if (i % 3 != 0)
//            baseSurface->baseSurface.property(cuttingPlane, *e_it).UpdateCuttingPlane(5);
//
//        ++i;
//    }

    cout << "Init Cutting Planes Done. " << endl;
}




///=========================================================================================///
///                                      Block clustering
///=========================================================================================///

int CuttingPlaneOptimizer::BlockClustering(PolyMesh * mySurface, bool isShowInfo)
{
    CuttingPlaneAngleClustering(mySurface);

    // Get cuttingAngleCentroidList
    OpenMesh::MPropHandleT< vector<double> > cuttingAngleCentroidList;
    mySurface->get_property_handle(cuttingAngleCentroidList, "cuttingAngleCentroidList");
    mySurface->property(cuttingAngleCentroidList).clear();

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    mySurface->get_property_handle( cuttingPlane ,"cuttingPlane");

    // Get cuttingAngleLabelList;
    OpenMesh::FPropHandleT< vector<int> > cuttingAngleLabelList;
    mySurface->get_property_handle(cuttingAngleLabelList, "cuttingAngleLabelList");

    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    mySurface->get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");
    mySurface->property(blockCuttingAngleLabelList).clear();

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");
    mySurface->property(blockCuttingAngleLabelList_full).clear();

    // Get blockCuttingAngleList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleList_full;
    mySurface->get_property_handle(blockCuttingAngleList_full, "blockCuttingAngleList_full");
    mySurface->property(blockCuttingAngleList_full).clear();

    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    mySurface->get_property_handle(polygonClusterList, "polygonClusterList");

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    mySurface->get_property_handle(clusteringLabel, "clusteringLabel");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    mySurface->get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get blockClusterFaceHandleList to mesh
    OpenMesh::MPropHandleT< vector< vector<PolyMesh::FaceHandle> > > blockClusterFaceHandleList;
    mySurface->get_property_handle(blockClusterFaceHandleList, "blockClusterFaceHandleList");
    mySurface->property(blockClusterFaceHandleList).clear();

    // Get cuttingAngleAndEdgeLabelPairs to mesh
    OpenMesh::MPropHandleT< vector< tuple<float, int> > > surfaceLabelPairs;
    mySurface->get_property_handle(surfaceLabelPairs, "cuttingAngleAndEdgeLabelPairs");

    // Init blockCuttingAngleLabelList
    for (int i = 0; i < mySurface->property(polygonClusterList).size(); ++i)
    {
        vector< vector<int> > currLabelList;
        mySurface->property(blockCuttingAngleLabelList).push_back(currLabelList);
    }

    for (PolyMesh::FaceIter f_it=mySurface->faces_begin(); f_it!=mySurface->faces_end(); ++f_it)
    {
        int currFaceLabel = mySurface->property(clusteringLabel, *f_it);

        if (IsCurrListInExistingList(mySurface->property(blockCuttingAngleLabelList)[currFaceLabel], mySurface->property(cuttingAngleLabelList, *f_it)) == -1)
        {
            mySurface->property(blockCuttingAngleLabelList)[currFaceLabel].push_back(mySurface->property(cuttingAngleLabelList, *f_it));
        }
    }

    for (PolyMesh::FaceIter f_it=mySurface->faces_begin(); f_it!=mySurface->faces_end(); ++f_it)
    {
        int count = 0;
//        dbg(baseSurface->baseSurface.property(cuttingAngleLabelList, *f_it));
        for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList).size(); ++i)
        {
//            dbg(baseSurface->baseSurface.property(blockCuttingAngleLabelList)[i]);
            if (i == mySurface->property(clusteringLabel, *f_it))
            {
                int pos = IsCurrListInExistingList(mySurface->property(blockCuttingAngleLabelList)[i], mySurface->property(cuttingAngleLabelList, *f_it));
                mySurface->property(blockClusteringLabel, *f_it) = count + pos;
//                dbg(baseSurface->baseSurface.property(blockClusteringLabel, *f_it));
                break;
            }

            count += int(mySurface->property(blockCuttingAngleLabelList)[i].size());
        }
    }

    int currBlockClusterNum = 0;

    for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList).size(); ++i)
    {
        currBlockClusterNum += (mySurface->property(blockCuttingAngleLabelList)[i].size());
    }

    for (int i = 0; i < currBlockClusterNum; ++i)
    {
        vector< vector<int> > currVecList;
        mySurface->property(blockCuttingAngleLabelList_full).push_back(currVecList);

        vector<PolyMesh::FaceHandle> currFaceHandleList;
        mySurface->property(blockClusterFaceHandleList).push_back(currFaceHandleList);
    }

    for (PolyMesh::FaceIter f_it=mySurface->faces_begin(); f_it!=mySurface->faces_end(); ++f_it)
    {
        int currFaceLabel = mySurface->property(blockClusteringLabel, *f_it);

        mySurface->property(blockCuttingAngleLabelList_full)[currFaceLabel].push_back(mySurface->property(cuttingAngleLabelList, *f_it));
        mySurface->property(blockClusterFaceHandleList)[currFaceLabel].push_back(*f_it);
    }

//    for (int i = 0; i < blockClusterNum; ++i)
//    {
//        dbg(baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[i]);
//    }

    if (isShowInfo)
    {
        cout << "Unique block number: " << currBlockClusterNum << endl;
        cout << endl << "Unique block number of each polygon cluster: ";
        for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList).size(); ++i)
        {
            if (i % 10  ==  0)
                cout << endl;

            cout << mySurface->property(blockCuttingAngleLabelList)[i].size() << " ";
        }
        cout << endl << endl;

        cout << endl << "Instance block number of each block cluster: ";
        for (int i = 0; i < baseSurface->baseSurface.property(blockCuttingAngleLabelList_full).size(); ++i)
        {
            if (i % 10  ==  0)
                cout << endl;

            cout << mySurface->property(blockCuttingAngleLabelList_full)[i].size() << " ";
        }
        cout << endl << endl;

         for (int i = 0; i < baseSurface->baseSurface.property(blockCuttingAngleLabelList_full).size(); ++i)
         {
             vector< tuple<float, int> > curr;

             for (int j = 0; j < baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[i][0].size(); ++j)
             {
                 int currIndex = baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[i][0][j];
                 curr.push_back(mySurface->property(surfaceLabelPairs)[currIndex]);
             }

//             dbg(curr);
         }

//         dbg(mySurface->property(surfaceLabelPairs));
    }

//    for (int i = 0; i < baseSurface->baseSurface.property(polygonClusterList).size(); ++i)
//    {
//        dbg(baseSurface->GetBlockClusterBasedOnPolygonCluster(i));
//    }

//    for (int i = 0; i < baseSurface->baseSurface.property(blockCuttingAngleLabelList_full).size(); ++i)
//    {
//        dbg(baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[i]);
//    }
//
//    for (int i = 0; i < baseSurface->baseSurface.property(blockCuttingAngleLabelList).size(); ++i)
//    {
//        dbg(baseSurface->baseSurface.property(blockCuttingAngleLabelList)[i]);
//    }

    return currBlockClusterNum;
}

void CuttingPlaneOptimizer::CuttingPlaneAngleClustering(PolyMesh * mySurface)
{
    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairs;

    // Get cuttingAngleCentroidList
    OpenMesh::MPropHandleT< vector<double> > cuttingAngleCentroidList;
    mySurface->get_property_handle(cuttingAngleCentroidList, "cuttingAngleCentroidList");
    mySurface->property(cuttingAngleCentroidList).clear();

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    mySurface->get_property_handle( cuttingPlane ,"cuttingPlane");

    // Get cuttingAngleLabelList;
    OpenMesh::FPropHandleT< vector<int> > cuttingAngleLabelList;
    mySurface->get_property_handle(cuttingAngleLabelList, "cuttingAngleLabelList");

    // Get cuttingAngleList;
    OpenMesh::FPropHandleT< vector<double> > cuttingAngleList;
    mySurface->get_property_handle(cuttingAngleList, "cuttingAngleList");

    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    mySurface->get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get cuttingAngleAndEdgeLabelPairs to mesh
    OpenMesh::MPropHandleT< vector< tuple<float, int> > > surfaceLabelPairs;
    mySurface->get_property_handle(surfaceLabelPairs, "cuttingAngleAndEdgeLabelPairs");
    mySurface->property(surfaceLabelPairs).clear();

    for (PolyMesh::FaceIter f_it=mySurface->faces_begin(); f_it!=mySurface->faces_end(); ++f_it)
    {
        PolyMesh::FaceEdgeCCWIter fe_it;

        for (fe_it = mySurface->fe_ccwiter(*f_it); fe_it.is_valid(); ++fe_it)
        {
            float currCuttingAngle = mySurface->property(cuttingPlane, *fe_it).GetCuttingPlaneAngle(*f_it);
            int currLabel = mySurface->property(edgeClusteringLabel, *fe_it);

            tuple<float, int> currTuple(currCuttingAngle, currLabel);

            if (find(cuttingAngleAndEdgeLabelPairs.begin(), cuttingAngleAndEdgeLabelPairs.end(), currTuple) == cuttingAngleAndEdgeLabelPairs.end())
            {
                cuttingAngleAndEdgeLabelPairs.push_back(currTuple);
            }
        }
    }

    for (PolyMesh::FaceIter f_it=mySurface->faces_begin(); f_it!=mySurface->faces_end(); ++f_it)
    {
        PolyMesh::FaceEdgeCCWIter fe_it;
        vector<int> currCuttingAngleLabelList;
        vector<double> currCuttingAngleList;

        for (fe_it = mySurface->fe_ccwiter(*f_it); fe_it.is_valid(); ++fe_it)
        {
            float currCuttingAngle = mySurface->property(cuttingPlane, *fe_it).GetCuttingPlaneAngle(*f_it);
            currCuttingAngleList.push_back(currCuttingAngle);

            int currLabel = mySurface->property(edgeClusteringLabel, *fe_it);

            tuple<float, int> currTuple(currCuttingAngle, currLabel);

            currCuttingAngleLabelList.push_back(find(cuttingAngleAndEdgeLabelPairs.begin(), cuttingAngleAndEdgeLabelPairs.end(), currTuple) - cuttingAngleAndEdgeLabelPairs.begin());
        }

        mySurface->property(cuttingAngleLabelList, *f_it) = currCuttingAngleLabelList;
        mySurface->property(cuttingAngleList, *f_it)  = currCuttingAngleList;
    }

    mySurface->property(surfaceLabelPairs) = cuttingAngleAndEdgeLabelPairs;

//    cout << "Cutting Plane Clustering Done. " << endl;
}

void CuttingPlaneOptimizer::BuildBlockClusterConnectionGraph(PolyMesh * mySurface)
{
    // Get blockClusterConnectionGraph
    OpenMesh::MPropHandleT< vector< vector<int> > > blockClusterConnectionGraph;
    mySurface->get_property_handle(blockClusterConnectionGraph, "blockClusterConnectionGraph");
    mySurface->property(blockClusterConnectionGraph).clear();

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    mySurface->get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");
    mySurface->property(isBoundaryBlockList).clear();

    // Get blockSingleNeighbourNumList
    OpenMesh::MPropHandleT< vector<int> > blockSingleNeighbourNumList;
    mySurface->get_property_handle(blockSingleNeighbourNumList, "blockSingleNeighbourNumList");
    mySurface->property(blockSingleNeighbourNumList).clear();

    // Get isBoundaryFace to each face
    OpenMesh::FPropHandleT< bool > isBoundaryFace;
    mySurface->get_property_handle(isBoundaryFace, "isBoundaryFace");

    for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList_full).size(); ++i)
    {
        vector<int> curr;

        mySurface->property(blockClusterConnectionGraph).push_back(curr);
        mySurface->property(isBoundaryBlockList).push_back(0);
        mySurface->property(blockSingleNeighbourNumList).push_back(0);
    }

    for (PolyMesh::FaceIter f_it=mySurface->faces_begin(); f_it!=mySurface->faces_end(); ++f_it)
    {
        PolyMesh::FaceFaceCCWIter ff_it;
        int currBlockLabel = mySurface->property(blockClusteringLabel, *f_it);

        for (ff_it = mySurface->ff_ccwiter(*f_it); ff_it.is_valid(); ++ff_it)
        {
            int currNeighbourBlockLabel = mySurface->property(blockClusteringLabel, *ff_it);

            if (find(mySurface->property(blockClusterConnectionGraph)[currBlockLabel].begin(), mySurface->property(blockClusterConnectionGraph)[currBlockLabel].end(),
                    currNeighbourBlockLabel) == mySurface->property(blockClusterConnectionGraph)[currBlockLabel].end())
            {
                mySurface->property(blockClusterConnectionGraph)[currBlockLabel].push_back(currNeighbourBlockLabel);
            }
        }

        if (mySurface->property(isBoundaryFace, *f_it))
        {
            mySurface->property(isBoundaryBlockList)[currBlockLabel] = 1;
        }
    }

    for (int i = 0; i < mySurface->property(blockClusterConnectionGraph).size(); ++i)
    {
        for (int j = 0; j < mySurface->property(blockClusterConnectionGraph)[i].size(); ++j)
        {
            int currLabel = mySurface->property(blockClusterConnectionGraph)[i][j];

            if (mySurface->property(blockCuttingAngleLabelList_full)[currLabel].size() == 1)
            {
                mySurface->property(blockSingleNeighbourNumList)[i] ++;
            }
        }
    }

//    dbg(mySurface->property(blockClusterConnectionGraph));
//    dbg(mySurface->property(isBoundaryBlockList));
//    dbg(mySurface->property(blockSingleNeighbourNumList));
}




///=========================================================================================///
///                                 CuttingPlane Optimization
///=========================================================================================///

void CuttingPlaneOptimizer::CuttingPlaneOptimization()
{
    InitCuttingPlanes();
    blockClusterNum = BlockClustering(&baseSurface->baseSurface);

    int count = 0;
    while (count < 1000)
    {
        vector<int> candiPolygonClusterIndexList;
        vector<float> possList;

        FindTargetPolygonClusterCandidates(5, candiPolygonClusterIndexList, possList);
        int selectedPolygonIndex = GetRandomObjIndex(possList, 3);

        CuttingPlaneOptimizationInPolygonCluster(candiPolygonClusterIndexList[selectedPolygonIndex]);

        count++;
    }

    blockClusterNum = BlockClustering(&baseSurface->baseSurface);
}

void CuttingPlaneOptimizer::CuttingPlaneOptimization_mini()
{
    InitCuttingPlanes();
    blockClusterNum = BlockClustering(&baseSurface->baseSurface, false);
    BuildBlockClusterConnectionGraph(&baseSurface->baseSurface);

    CuttingPlaneOptimization_mini_without_boundary();

    CuttingPlaneOptimization_mini_boundary();
}

void CuttingPlaneOptimizer::CuttingPlaneOptimization_mini_without_boundary()
{
    int count = 0;

    while (count < 1000)
    {
        if (blockClusterNum <= targetBlockClusterNum)
            break;

        vector<int> miniBlockCandiList;
        vector<float> miniPossList;

        FindMiniBlockClusterIDList_without_boundary(&baseSurface->baseSurface, miniBlockClusterThreshold, miniBlockCandiList, miniPossList);

//        dbg(miniBlockCandiList);

        if (miniBlockCandiList.empty())
            break;

        if (!MergeMiniBlockClusters_without_boundary(&baseSurface->baseSurface, miniBlockCandiList, miniPossList))
            break;

        count ++;
    }

    blockClusterNum = BlockClustering(&baseSurface->baseSurface, true);
    BuildBlockClusterConnectionGraph(&baseSurface->baseSurface);
}

void CuttingPlaneOptimizer::CuttingPlaneOptimization_mini_boundary()
{
    int count = 0;

    while (count < 1000)
    {
        if (blockClusterNum <= targetBlockClusterNum)
            break;

        vector<int> miniBlockCandiList;
        vector<float> miniPossList;

        FindMiniBlockClusterIDList_with_boundary(&baseSurface->baseSurface, miniBlockClusterThreshold, miniBlockCandiList, miniPossList);

        if (miniBlockCandiList.empty())
            break;

        if (!MergeMiniBlockClusters_with_boundary(&baseSurface->baseSurface, miniBlockCandiList, miniPossList))
            break;

//        cout << "Succeed. " << endl;

        count ++;
    }

    blockClusterNum = BlockClustering(&baseSurface->baseSurface, true);
    BuildBlockClusterConnectionGraph(&baseSurface->baseSurface);
}

void CuttingPlaneOptimizer::CuttingPlaneOptimization_mini_boundary_only()
{
    int count = 0;
    int minBlockClusterThreshold = 5;

    vector< tuple<float, int> > freeAnglePairList;
    FindOrigFreeAnglePairList(freeAnglePairList);

    // dbg(freeAnglePairList);

    while (count < 1000)
    {
        vector<int> miniBlockCandiList;
        vector<float> miniPossList;

        FindMiniBlockClusterIDList_boundary_only(&baseSurface->baseSurface, minBlockClusterThreshold, miniBlockCandiList, miniPossList);

        if (!MergeMiniBlockClusters_with_boundary(&baseSurface->baseSurface, miniBlockCandiList, miniPossList))
            break;

//        cout << "Succeed. " << endl;

        count ++;
    }

    blockClusterNum = BlockClustering(&baseSurface->baseSurface, true);
    BuildBlockClusterConnectionGraph(&baseSurface->baseSurface);
}

void CuttingPlaneOptimizer::FindMiniBlockClusterIDList_without_boundary(PolyMesh * mySurface, int minBlockClusterThreshold, vector<int> & candiList, vector<float> & possList)
{

    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");

    // Get blockSingleNeighbourNumList
    OpenMesh::MPropHandleT< vector<int> > blockSingleNeighbourNumList;
    mySurface->get_property_handle(blockSingleNeighbourNumList, "blockSingleNeighbourNumList");

    // Get blockClusterConnectionGraph
    OpenMesh::MPropHandleT< vector< vector<int> > > blockClusterConnectionGraph;
    mySurface->get_property_handle(blockClusterConnectionGraph, "blockClusterConnectionGraph");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList_full).size(); ++i)
    {
//        dbg(mySurface->property(blockCuttingAngleLabelList_full)[i]);
        if (mySurface->property(blockCuttingAngleLabelList_full)[i].size() <= minBlockClusterThreshold
            and mySurface->property(isBoundaryBlockList)[i] == 0)
        {
            candiList.push_back(i);

            possList.push_back( mySurface->property(blockSingleNeighbourNumList)[i] / float(mySurface->property(blockClusterConnectionGraph)[i].size()) + 0.1 );
        }
    }
}

void CuttingPlaneOptimizer::FindMiniBlockClusterIDList_with_boundary(PolyMesh * mySurface, int minBlockClusterThreshold, vector<int> & candiList, vector<float> & possList)
{
    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");

    // Get blockSingleNeighbourNumList
    OpenMesh::MPropHandleT< vector<int> > blockSingleNeighbourNumList;
    mySurface->get_property_handle(blockSingleNeighbourNumList, "blockSingleNeighbourNumList");

    // Get blockClusterConnectionGraph
    OpenMesh::MPropHandleT< vector< vector<int> > > blockClusterConnectionGraph;
    mySurface->get_property_handle(blockClusterConnectionGraph, "blockClusterConnectionGraph");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList_full).size(); ++i)
    {
        if (mySurface->property(blockCuttingAngleLabelList_full)[i].size() <= minBlockClusterThreshold)
        {
            candiList.push_back(i);

            float currPoss = mySurface->property(blockSingleNeighbourNumList)[i] / float(mySurface->property(blockClusterConnectionGraph)[i].size()) + 0.1;

            if ( mySurface->property(isBoundaryBlockList)[i] != 0)
                currPoss += 0.5;

            possList.push_back( currPoss );

//            cout << "Pushing " << i << " to miniCandisList. " << endl;
        }
    }
}

void CuttingPlaneOptimizer::FindMiniBlockClusterIDList_boundary_only(PolyMesh * mySurface, int minBlockClusterThreshold, vector<int> & candiList, vector<float> & possList)
{
    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");

    // Get blockSingleNeighbourNumList
    OpenMesh::MPropHandleT< vector<int> > blockSingleNeighbourNumList;
    mySurface->get_property_handle(blockSingleNeighbourNumList, "blockSingleNeighbourNumList");

    // Get blockClusterConnectionGraph
    OpenMesh::MPropHandleT< vector< vector<int> > > blockClusterConnectionGraph;
    mySurface->get_property_handle(blockClusterConnectionGraph, "blockClusterConnectionGraph");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    for (int i = 0; i < mySurface->property(blockCuttingAngleLabelList_full).size(); ++i)
    {
        if (mySurface->property(blockCuttingAngleLabelList_full)[i].size() <= minBlockClusterThreshold
            and mySurface->property(isBoundaryBlockList)[i] == 1)
        {
            candiList.push_back(i);

            possList.push_back( mySurface->property(blockSingleNeighbourNumList)[i] / float(mySurface->property(blockClusterConnectionGraph)[i].size()) + 0.1 );
        }
    }
}

bool CuttingPlaneOptimizer::MergeMiniBlockClusters_without_boundary(PolyMesh * mySurface, vector<int> & candiList, vector<float> & possList)
{
    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    mySurface->get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");

//    int count = 0;

    while(1)
    {
//        cout << "Start: " << endl;
        auto result = max_element(possList.begin(), possList.end());
        float currMax = *result;
        int i = distance(possList.begin(), result);

//        dbg(possList);
//        dbg(currMax);

        if (currMax <= 0)
            break;

        possList[i] = 0;

        vector<int> currblockCuttingAngleLabelList = mySurface->property(blockCuttingAngleLabelList_full)[candiList[i]][0];

//        dbg(currblockCuttingAngleLabelList);

        vector< vector<int> > selectedBlockIDPairs;
        vector<float> currPossList;

        for (int j = 0; j < mySurface->property(blockCuttingAngleLabelList).size(); ++j)
        {
            auto _result = find(mySurface->property(blockCuttingAngleLabelList)[j].begin(), mySurface->property(blockCuttingAngleLabelList)[j].end(),
                               currblockCuttingAngleLabelList);
            if (_result != mySurface->property(blockCuttingAngleLabelList)[j].end() and mySurface->property(blockCuttingAngleLabelList)[j].size() > 1)
            {
                vector<int> toMergeBlockClusterList = baseSurface->GetBlockClusterBasedOnPolygonCluster(j);

                for (int k = 0; k < toMergeBlockClusterList.size(); ++k)
                {
                    if (toMergeBlockClusterList[k] != candiList[i] and mySurface->property(isBoundaryBlockList)[toMergeBlockClusterList[k]] == 0)
                    {
                        vector<int> currSelectedIDPair;

                        currSelectedIDPair.push_back(toMergeBlockClusterList[k]);
                        currSelectedIDPair.push_back(candiList[i]);

                        selectedBlockIDPairs.push_back(currSelectedIDPair);
                        currPossList.push_back(mySurface->property(blockCuttingAngleLabelList_full)[toMergeBlockClusterList[k]].size());

//                        if (selectedBlockIDPairs.size() > 10)
//                            break;
                    }
                }
            }

//            if (selectedBlockIDPairs.size() > 10)
//                break;
        }

//       dbg(selectedBlockIDPairs);
//       dbg(currPossList);

        if (currPossList.empty())
            continue;

        if (MergeTwoBlockClusters(selectedBlockIDPairs, currPossList))
        {
            return true;
        }

//        count ++;
//
//        if (count > 10)
//            break;
    }

    return false;
}

bool CuttingPlaneOptimizer::MergeMiniBlockClusters_with_boundary(PolyMesh * mySurface, vector<int> & candiList, vector<float> & possList)
{
    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    mySurface->get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");

    while(1)
    {
//        cout << "Start: " << endl;
        auto result = max_element(possList.begin(), possList.end());
        float currMax = *result;
        int i = distance(possList.begin(), result);

//        dbg(possList);
//        dbg(currMax);

        if (currMax <= 0)
            break;

        possList[i] = 0;

        vector<int> currblockCuttingAngleLabelList = mySurface->property(blockCuttingAngleLabelList_full)[candiList[i]][0];

//        dbg(currblockCuttingAngleLabelList);

        vector< vector<int> > selectedBlockIDPairs;
        vector<float> currPossList;

        for (int j = 0; j < mySurface->property(blockCuttingAngleLabelList).size(); ++j)
        {
            auto _result = find(mySurface->property(blockCuttingAngleLabelList)[j].begin(), mySurface->property(blockCuttingAngleLabelList)[j].end(),
                                currblockCuttingAngleLabelList);
            if (_result != mySurface->property(blockCuttingAngleLabelList)[j].end() and mySurface->property(blockCuttingAngleLabelList)[j].size() > 1)
            {
                vector<int> toMergeBlockClusterList = baseSurface->GetBlockClusterBasedOnPolygonCluster(j);

                for (int k = 0; k < toMergeBlockClusterList.size(); ++k)
                {
                    if (toMergeBlockClusterList[k] != candiList[i])
                    {
                        vector<int> currSelectedIDPair;

                        currSelectedIDPair.push_back(toMergeBlockClusterList[k]);
                        currSelectedIDPair.push_back(candiList[i]);

                        selectedBlockIDPairs.push_back(currSelectedIDPair);
                        currPossList.push_back(mySurface->property(blockCuttingAngleLabelList_full)[toMergeBlockClusterList[k]].size());

//                        if (selectedBlockIDPairs.size() > 10)
//                            break;
                    }
                }
            }

//            if (selectedBlockIDPairs.size() > 10)
//                break;
        }

//        dbg(selectedBlockIDPairs);
//        dbg(currPossList);

        if (currPossList.empty())
            continue;

        if (MergeTwoBlockClusters(selectedBlockIDPairs, currPossList))
        {
            return true;
        }

    }

    return false;
}

bool CuttingPlaneOptimizer::MergeMiniBlockClusters_with_boundary(PolyMesh * mySurface, vector<int> & candiList, vector<float> & possList, vector< tuple<float, int> > & freeAnglePairList)
{
    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    mySurface->get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    mySurface->get_property_handle(isBoundaryBlockList, "isBoundaryBlockList");

    while(1)
    {
//        cout << "Start: " << endl;
        auto result = max_element(possList.begin(), possList.end());
        float currMax = *result;
        int i = distance(possList.begin(), result);

//        dbg(possList);
//        dbg(currMax);

        if (currMax <= 0)
            break;

        possList[i] = 0;

        vector<int> currblockCuttingAngleLabelList = mySurface->property(blockCuttingAngleLabelList_full)[candiList[i]][0];

//        dbg(currblockCuttingAngleLabelList);

        vector< vector<int> > selectedBlockIDPairs;
        vector<float> currPossList;

        for (int j = 0; j < mySurface->property(blockCuttingAngleLabelList).size(); ++j)
        {
            auto _result = find(mySurface->property(blockCuttingAngleLabelList)[j].begin(), mySurface->property(blockCuttingAngleLabelList)[j].end(),
                                currblockCuttingAngleLabelList);
            if (_result != mySurface->property(blockCuttingAngleLabelList)[j].end() and mySurface->property(blockCuttingAngleLabelList)[j].size() > 1)
            {
                vector<int> toMergeBlockClusterList = baseSurface->GetBlockClusterBasedOnPolygonCluster(j);

                for (int k = 0; k < toMergeBlockClusterList.size(); ++k)
                {
                    if (toMergeBlockClusterList[k] != candiList[i])
                    {
                        vector<int> currSelectedIDPair;

                        currSelectedIDPair.push_back(toMergeBlockClusterList[k]);
                        currSelectedIDPair.push_back(candiList[i]);

                        selectedBlockIDPairs.push_back(currSelectedIDPair);
                        currPossList.push_back(mySurface->property(blockCuttingAngleLabelList_full)[toMergeBlockClusterList[k]].size());
                    }
                }
            }
        }

        if (currPossList.empty())
            continue;

        if (MergeTwoBlockClusters(selectedBlockIDPairs, currPossList, freeAnglePairList))
        {
            return true;
        }
    }

    return false;
}

void CuttingPlaneOptimizer::FindTargetPolygonClusterCandidates(int candiNum, vector<int> & candiPolygonClusterIndexList, vector<float> & possList)
{
    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    baseSurface->baseSurface.get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    vector<int> polygonBlockClusterNumList;

    for (int i = 0; i < baseSurface->baseSurface.property(blockCuttingAngleLabelList).size(); ++i)
    {
        polygonBlockClusterNumList.push_back(baseSurface->baseSurface.property(blockCuttingAngleLabelList)[i].size());
    }

    for (int i = 0; i < candiNum; ++i)
    {
        auto result = max_element(polygonBlockClusterNumList.begin(), polygonBlockClusterNumList.end());
        int currMax = *result;
        int currDist = distance(polygonBlockClusterNumList.begin(), result);

        if (currMax <= 1)
            break;

        candiPolygonClusterIndexList.push_back(currDist);
        possList.push_back(currMax);

        polygonBlockClusterNumList[currDist] = 0;
    }
}

void CuttingPlaneOptimizer::CuttingPlaneOptimizationInPolygonCluster(int polygonClusterID)
{
    vector<int> candiBigBlockClusterIndexList;
    vector<float> possList_big;
    FindBigBlockClusterCandidates(polygonClusterID, 10, candiBigBlockClusterIndexList, possList_big);

    vector<int> candiSmallBlockClusterIndexList;
    vector<float> possList_small;
    FindSmallBlockClusterCandidates(polygonClusterID, 10, candiSmallBlockClusterIndexList, possList_small);

    vector< vector<int> > selectedBlockIDPairs;
    vector<float> possList_BlockIDPair;

    GenerateSelectedBlockIDPairs(candiBigBlockClusterIndexList, possList_big,
                                     candiSmallBlockClusterIndexList, possList_small,
                                     selectedBlockIDPairs, possList_BlockIDPair);

    bool flag = MergeTwoBlockClusters(selectedBlockIDPairs, possList_BlockIDPair);
}

void CuttingPlaneOptimizer::FindBigBlockClusterCandidates(int polygonClusterID, int candiNum, vector<int> & candiBlockClusterIndexList, vector<float> & possList)
{
    vector<int> blockClusterIDList = baseSurface->GetBlockClusterBasedOnPolygonCluster(polygonClusterID);

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    baseSurface->baseSurface.get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    vector<int> blockClusterNumOfEachPolygonCluster;

    for (int i = 0; i < blockClusterIDList.size(); ++i)
    {
        blockClusterNumOfEachPolygonCluster.push_back(baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[blockClusterIDList[i]].size());
    }

//    dbg(blockClusterIDList);
//    dbg(blockClusterNumOfEachPolygonCluster);

    for (int i = 0; i < candiNum; ++i)
    {
        auto result = max_element(blockClusterNumOfEachPolygonCluster.begin(), blockClusterNumOfEachPolygonCluster.end());
        int currMax = *result;
        int currDist = distance(blockClusterNumOfEachPolygonCluster.begin(), result);

        if (currMax < 1)
            break;

        candiBlockClusterIndexList.push_back(blockClusterIDList[currDist]);
        possList.push_back(blockClusterNumOfEachPolygonCluster[currDist]);

        blockClusterNumOfEachPolygonCluster[currDist] = 0;
    }

//    dbg(candiBlockClusterIndexList);
//    dbg(possList);

}

void CuttingPlaneOptimizer::FindSmallBlockClusterCandidates(int polygonClusterID, int candiNum, vector<int> & candiBlockClusterIndexList, vector<float> & possList)
{
    vector<int> blockClusterIDList = baseSurface->GetBlockClusterBasedOnPolygonCluster(polygonClusterID);

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    baseSurface->baseSurface.get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    vector<int> blockClusterNumOfEachPolygonCluster;

    for (int i = 0; i < blockClusterIDList.size(); ++i)
    {
        blockClusterNumOfEachPolygonCluster.push_back(baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[blockClusterIDList[i]].size());
    }

//    dbg(blockClusterIDList);
//    dbg(blockClusterNumOfEachPolygonCluster);

    for (int i = 0; i < candiNum; ++i)
    {
        auto result = min_element(blockClusterNumOfEachPolygonCluster.begin(), blockClusterNumOfEachPolygonCluster.end());
        int currMin = *result;
        int currDist = distance(blockClusterNumOfEachPolygonCluster.begin(), result);

        if (currMin >= 1000000)
            break;

        candiBlockClusterIndexList.push_back(blockClusterIDList[currDist]);
        possList.push_back(1.0 / float(blockClusterNumOfEachPolygonCluster[currDist]));

        blockClusterNumOfEachPolygonCluster[currDist] = 1000000;
    }

//    dbg(candiBlockClusterIndexList);
//    dbg(possList);
}

void CuttingPlaneOptimizer::GenerateSelectedBlockIDPairs(const vector<int> & candiBlockClusterIndexList_big, const vector<float> & possList_big,
                                  const vector<int> & candiBlockClusterIndexList_small, const vector<float> & possList_small,
                                  vector< vector<int> > & selectedBlockIDPairs, vector<float> & possList_BlockIDPairs)
{
    for (int i = 0; i < candiBlockClusterIndexList_big.size(); ++i)
    {
        for (int j = 0; j < candiBlockClusterIndexList_small.size(); ++j)
        {
            if (candiBlockClusterIndexList_big[i] == candiBlockClusterIndexList_small[j])
                continue;

            vector<int> curr;
            curr.push_back(candiBlockClusterIndexList_big[i]);
            curr.push_back(candiBlockClusterIndexList_small[j]);

            selectedBlockIDPairs.push_back(curr);
            possList_BlockIDPairs.push_back(possList_big[i] * possList_small[j]);
        }
    }

//    dbg(selectedBlockIDPairs);
//    dbg(possList_BlockIDPairs);
}

bool CuttingPlaneOptimizer::MergeTwoBlockClusters(vector< vector<int> > & selectedBlockIDPairs, vector<float> & possList_BlockIDPairs)
{
    // Get normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface->baseSurface.get_property_handle( normal ,"normal");

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface->baseSurface.get_property_handle( cuttingPlane ,"cuttingPlane");

    while(1)
    {
        // Backup
        PolyMesh tempSurface = baseSurface->baseSurface;

        vector<Vector3d> origAngleList;
        vector<PolyMesh::EdgeHandle> origEHList;
        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            origAngleList.push_back(baseSurface->baseSurface.property(normal, *e_it));
            origEHList.push_back(*e_it);
        }

        auto result = max_element(possList_BlockIDPairs.begin(), possList_BlockIDPairs.end());
        float currMax = *result;
        int i = distance(possList_BlockIDPairs.begin(), result);
        possList_BlockIDPairs[i] = 0;

        if (currMax == 0)
            break;

        vector<int> currSelectedPair = selectedBlockIDPairs[i];

//        dbg(currSelectedPair);

        bool flag = MergeTwoBlockClusters(&tempSurface, currSelectedPair);

        if (flag)
        {
            baseSurface->baseSurface = tempSurface;
            blockClusterNum = BlockClustering(&baseSurface->baseSurface, false);
            BuildBlockClusterConnectionGraph(&baseSurface->baseSurface);

            for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
            {
                baseSurface->baseSurface.property(normal, *e_it) = baseSurface->baseSurface.property(cuttingPlane, *e_it).edgeNormal;
            }

            return true;
        }

        for (int j = 0; j < origEHList.size(); ++j)
        {
            if (baseSurface->baseSurface.property(normal, origEHList[j]) != origAngleList[j])
                baseSurface->baseSurface.property(normal, origEHList[j]) = origAngleList[j];
        }
    }

    return false;
}

bool CuttingPlaneOptimizer::MergeTwoBlockClusters(vector< vector<int> > & selectedBlockIDPairs, vector<float> & possList_BlockIDPairs, vector< tuple<float, int> > & freeAnglePairList)
{
    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface->baseSurface.get_property_handle( cuttingPlane ,"cuttingPlane");

    while(1)
    {
        // Backup
        PolyMesh tempSurface = baseSurface->baseSurface;
        vector< tuple<float, int> >  tempFreeAnglePairList = freeAnglePairList;

        vector<double> origAngleList;
        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            origAngleList.push_back(baseSurface->baseSurface.property(cuttingPlane, *e_it).GetCuttingPlaneAngle(baseSurface->baseSurface.property(cuttingPlane, *e_it).positiveFaceHandle));
        }

        auto result = max_element(possList_BlockIDPairs.begin(), possList_BlockIDPairs.end());
        float currMax = *result;
        int i = distance(possList_BlockIDPairs.begin(), result);
        possList_BlockIDPairs[i] = 0;

        if (currMax <= 0)
            break;

        vector<int> currSelectedPair = selectedBlockIDPairs[i];

        bool flag = MergeTwoBlockClusters(&tempSurface, currSelectedPair, tempFreeAnglePairList);

        if (flag)
        {
            baseSurface->baseSurface = tempSurface;
            freeAnglePairList = tempFreeAnglePairList;
            blockClusterNum = BlockClustering(&baseSurface->baseSurface, false);
            BuildBlockClusterConnectionGraph(&baseSurface->baseSurface);

            return true;
        }

        vector<double> afterAngleList;
        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            afterAngleList.push_back(baseSurface->baseSurface.property(cuttingPlane, *e_it).GetCuttingPlaneAngle(baseSurface->baseSurface.property(cuttingPlane, *e_it).positiveFaceHandle));
        }

        if (origAngleList != afterAngleList)
            cout << "Angle changed. " << endl;
    }

    return false;
}

bool CuttingPlaneOptimizer::MergeTwoBlockClusters(PolyMesh * mySurface, const vector<int> & currSelectedPair)
{
    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get cuttingAngleAndEdgeLabelPairs to mesh
    OpenMesh::MPropHandleT< vector< tuple<float, int> > > surfaceLabelPairs;
    mySurface->get_property_handle(surfaceLabelPairs, "cuttingAngleAndEdgeLabelPairs");

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    mySurface->get_property_handle( cuttingPlane ,"cuttingPlane");

    // Get blockClusterFaceHandleList to mesh
    OpenMesh::MPropHandleT< vector< vector<PolyMesh::FaceHandle> > > blockClusterFaceHandleList;
    mySurface->get_property_handle(blockClusterFaceHandleList, "blockClusterFaceHandleList");

    int blockID_big = currSelectedPair[0];
    int blockID_small = currSelectedPair[1];

    vector<int> blockCuttingAngleLabelList_big = mySurface->property(blockCuttingAngleLabelList_full)[blockID_big][0];
    vector<int> blockCuttingAngleLabelList_small = mySurface->property(blockCuttingAngleLabelList_full)[blockID_small][0];

//    dbg(blockCuttingAngleLabelList_big);
//    dbg(blockCuttingAngleLabelList_small);

    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_big;
    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_small;

    for (int i = 0; i < blockCuttingAngleLabelList_big.size(); ++i)
    {
        cuttingAngleAndEdgeLabelPairList_big.push_back(mySurface->property(surfaceLabelPairs)[blockCuttingAngleLabelList_big[i]]);
        cuttingAngleAndEdgeLabelPairList_small.push_back(mySurface->property(surfaceLabelPairs)[blockCuttingAngleLabelList_small[i]]);
    }

//    dbg(cuttingAngleAndEdgeLabelPairList_big);
//    dbg(cuttingAngleAndEdgeLabelPairList_small);

    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_big_matched;
    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_small_matched;

    MatchingTwoEdgeLabelPairLists(cuttingAngleAndEdgeLabelPairList_big,
                                  cuttingAngleAndEdgeLabelPairList_small,
                                  cuttingAngleAndEdgeLabelPairList_big_matched,
                                  cuttingAngleAndEdgeLabelPairList_small_matched);

//   dbg(cuttingAngleAndEdgeLabelPairList_big_matched);
//   dbg(cuttingAngleAndEdgeLabelPairList_small_matched);

    if (cuttingAngleAndEdgeLabelPairList_big != cuttingAngleAndEdgeLabelPairList_big_matched)
    {
        for (int i = 0; i < mySurface->property(blockClusterFaceHandleList)[blockID_big].size(); ++i)
        {
            AdjustBlockAugmentedVector(mySurface, mySurface->property(blockClusterFaceHandleList)[blockID_big][i], cuttingAngleAndEdgeLabelPairList_big_matched);
        }
    }

    for (int i = 0; i < mySurface->property(blockClusterFaceHandleList)[blockID_small].size(); ++i)
    {
        AdjustBlockAugmentedVector(mySurface, mySurface->property(blockClusterFaceHandleList)[blockID_small][i], cuttingAngleAndEdgeLabelPairList_small_matched);
    }

    int currBlockClusterNum = BlockClustering(mySurface, false);

//    dbg(currBlockClusterNum);

    if (currBlockClusterNum < blockClusterNum)
    {
        cout << "Merging Succeed. " << endl;
        cout << "Block Cluster Number: " << blockClusterNum << " -> " << currBlockClusterNum << endl;

        return true;
    }
    else
    {
//        cout << "Merging Failed. " << endl;
//        cout << "Block Cluster Number: " << blockClusterNum << " -> " << currBlockClusterNum << endl;

        return false;
    }
}

bool CuttingPlaneOptimizer::MergeTwoBlockClusters(PolyMesh * mySurface, const vector<int> & currSelectedPair, vector< tuple<float, int> > & freeAnglePairList)
{
    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    mySurface->get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get cuttingAngleAndEdgeLabelPairs to mesh
    OpenMesh::MPropHandleT< vector< tuple<float, int> > > surfaceLabelPairs;
    mySurface->get_property_handle(surfaceLabelPairs, "cuttingAngleAndEdgeLabelPairs");

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    mySurface->get_property_handle( cuttingPlane ,"cuttingPlane");

    // Get blockClusterFaceHandleList to mesh
    OpenMesh::MPropHandleT< vector< vector<PolyMesh::FaceHandle> > > blockClusterFaceHandleList;
    mySurface->get_property_handle(blockClusterFaceHandleList, "blockClusterFaceHandleList");

    int blockID_big = currSelectedPair[0];
    int blockID_small = currSelectedPair[1];

    vector<int> blockCuttingAngleLabelList_big = mySurface->property(blockCuttingAngleLabelList_full)[blockID_big][0];
    vector<int> blockCuttingAngleLabelList_small = mySurface->property(blockCuttingAngleLabelList_full)[blockID_small][0];

//    dbg(blockCuttingAngleLabelList_big);
//    dbg(blockCuttingAngleLabelList_small);

    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_big;
    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_small;

    for (int i = 0; i < blockCuttingAngleLabelList_big.size(); ++i)
    {
        cuttingAngleAndEdgeLabelPairList_big.push_back(mySurface->property(surfaceLabelPairs)[blockCuttingAngleLabelList_big[i]]);
        cuttingAngleAndEdgeLabelPairList_small.push_back(mySurface->property(surfaceLabelPairs)[blockCuttingAngleLabelList_small[i]]);
    }

//    dbg(cuttingAngleAndEdgeLabelPairList_big);
//    dbg(cuttingAngleAndEdgeLabelPairList_small);

    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_big_matched;
    vector< tuple<float, int> > cuttingAngleAndEdgeLabelPairList_small_matched;

    vector< tuple<float, int> > tempFreeAngleList = freeAnglePairList;

    MatchingTwoEdgeLabelPairLists(cuttingAngleAndEdgeLabelPairList_big,
                                  cuttingAngleAndEdgeLabelPairList_small,
                                  cuttingAngleAndEdgeLabelPairList_big_matched,
                                  cuttingAngleAndEdgeLabelPairList_small_matched,
                                  tempFreeAngleList);

//    dbg(cuttingAngleAndEdgeLabelPairList_big);
//    dbg(cuttingAngleAndEdgeLabelPairList_small);
//    dbg(cuttingAngleAndEdgeLabelPairList_big_matched);
//    dbg(cuttingAngleAndEdgeLabelPairList_small_matched);

    if (cuttingAngleAndEdgeLabelPairList_big != cuttingAngleAndEdgeLabelPairList_big_matched)
    {
        for (int i = 0; i < mySurface->property(blockClusterFaceHandleList)[blockID_big].size(); ++i)
        {
            AdjustBlockAugmentedVector(mySurface, mySurface->property(blockClusterFaceHandleList)[blockID_big][i], cuttingAngleAndEdgeLabelPairList_big_matched);
        }
    }

    for (int i = 0; i < mySurface->property(blockClusterFaceHandleList)[blockID_small].size(); ++i)
    {
        AdjustBlockAugmentedVector(mySurface, mySurface->property(blockClusterFaceHandleList)[blockID_small][i], cuttingAngleAndEdgeLabelPairList_small_matched);
    }

    int currBlockClusterNum = BlockClustering(mySurface, false);

//    dbg(currBlockClusterNum);

    if (currBlockClusterNum < blockClusterNum)
    {
        cout << "Merging Succeed. " << endl;
        cout << "Block Cluster Number: " << blockClusterNum << " -> " << currBlockClusterNum << endl;

        freeAnglePairList = tempFreeAngleList;

//        dbg(freeAnglePairList);

        return true;
    }

    cout << "Merging Failed. " << endl;
    cout << "Block Cluster Number: " << blockClusterNum << " -> " << currBlockClusterNum << endl;

    return false;
}

void CuttingPlaneOptimizer::MatchingTwoEdgeLabelPairLists(vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big,
                                                          vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small,
                                                          vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big_matched,
                                                          vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small_matched)
                                                
{
    int count = 0;
    int unMatchedNum = 10000;
    double diffTotalAngle = 10000;
    double diffMaxAngle = 10000;

    while (count < cuttingAngleAndEdgeLabelPairList_big.size())
    {
        bool isSame = true;

        for (int i = 0; i < cuttingAngleAndEdgeLabelPairList_big.size(); ++i)
        {
            tuple<float, int> currTuple_big = cuttingAngleAndEdgeLabelPairList_big[i];
            tuple<float, int> currTuple_small = cuttingAngleAndEdgeLabelPairList_small[i];

            if (get<1>(currTuple_big) != get<1>(currTuple_small))
            {
                isSame = false;
                break;
            }
        }

        if (isSame)
        {
            int currUnMatchedNum = 0;
            double currDiffAngle = 0;
            double currDiffMaxAngle = 0;

            vector< tuple<float, int> > currList_big = cuttingAngleAndEdgeLabelPairList_big;
            vector< tuple<float, int> > currList_small = cuttingAngleAndEdgeLabelPairList_small;

            for (int i = 0; i < currList_big.size(); ++i)
            {
                if (currList_big[i] == currList_small[i])
                    continue;

                if (get<0>(currList_big[i]) == 0 and get<0>(currList_small[i]) == 0)
                {
                    get<0>(currList_big[i]) = 80;
                    get<0>(currList_small[i]) = 80;
                }

                if (get<0>(currList_big[i]) == 0 and get<0>(currList_small[i]) != 0)
                {
                    get<0>(currList_big[i]) = get<0>(currList_small[i]);
                }

                if (get<0>(currList_big[i]) != 0 and get<0>(currList_small[i]) == 0)
                {
                    get<0>(currList_small[i]) = get<0>(currList_big[i]);
                }

                if (get<0>(currList_big[i]) != 0 and get<0>(currList_small[i]) != 0)
                {
                    get<0>(currList_small[i]) = get<0>(currList_big[i]);

                    currUnMatchedNum ++;
                    currDiffAngle += fabs(get<0>(currList_small[i]) - get<0>(currList_big[i]));

                    if (fabs(get<0>(currList_small[i]) - get<0>(currList_big[i])) > currDiffMaxAngle)
                    {
                        currDiffMaxAngle = fabs(get<0>(currList_small[i]) - get<0>(currList_big[i]));
                    }
                }

            }

            if (currUnMatchedNum < unMatchedNum)
            {
                unMatchedNum = currUnMatchedNum;
                diffTotalAngle = currDiffAngle;
                diffMaxAngle = currDiffMaxAngle;

                cuttingAngleAndEdgeLabelPairList_big_matched = currList_big;
                cuttingAngleAndEdgeLabelPairList_small_matched = currList_small;
            }

            if (currUnMatchedNum == unMatchedNum and currDiffAngle < diffTotalAngle)
            {
                unMatchedNum = currUnMatchedNum;
                diffTotalAngle = currDiffAngle;
                diffMaxAngle = currDiffMaxAngle;

                cuttingAngleAndEdgeLabelPairList_big_matched = currList_big;
                cuttingAngleAndEdgeLabelPairList_small_matched = currList_small;
            }

        }

        cuttingAngleAndEdgeLabelPairList_small.push_back(cuttingAngleAndEdgeLabelPairList_small[0]);
        cuttingAngleAndEdgeLabelPairList_small.erase(cuttingAngleAndEdgeLabelPairList_small.begin());

        count++;
    }
}

void CuttingPlaneOptimizer::MatchingTwoEdgeLabelPairLists(vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big,
                                                          vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small,
                                                          vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_big_matched,
                                                          vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList_small_matched,
                                                          vector< tuple<float, int> > & freeAnglePairList)
{
    int count = 0;
    int unMatchedNum = 10000;
    double diffTotalAngle = 10000;
    double diffMaxAngle = 10000;

    while (count <= cuttingAngleAndEdgeLabelPairList_big.size())
    {
        bool isSame = true;

        for (int i = 0; i < cuttingAngleAndEdgeLabelPairList_big.size(); ++i)
        {
            tuple<float, int> currTuple_big = cuttingAngleAndEdgeLabelPairList_big[i];
            tuple<float, int> currTuple_small = cuttingAngleAndEdgeLabelPairList_small[i];

            if (get<1>(currTuple_big) != get<1>(currTuple_small))
            {
                isSame = false;
                break;
            }
        }

        if (isSame)
        {
            int currUnMatchedNum = 0;
            double currDiffAngle = 0;
            double currDiffMaxAngle = 0;

            vector< tuple<float, int> > currList_big = cuttingAngleAndEdgeLabelPairList_big;
            vector< tuple<float, int> > currList_small = cuttingAngleAndEdgeLabelPairList_small;

            for (int i = 0; i < currList_big.size(); ++i)
            {
                if (currList_big[i] == currList_small[i] and find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]) == freeAnglePairList.end()
                    and find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) == freeAnglePairList.end())
                    continue;

                if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]) != freeAnglePairList.end()
                    and find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) != freeAnglePairList.end())
                {
                    auto currResult = find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]);
                    int k = distance(freeAnglePairList.begin(), currResult);
                    freeAnglePairList.erase(freeAnglePairList.begin() + k);

                    get<0>(currList_small[i]) = get<0>(currList_big[i]);

                    if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]) == freeAnglePairList.end())
                        freeAnglePairList.push_back(currList_big[i]);
                }

                if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]) != freeAnglePairList.end()
                    and find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) == freeAnglePairList.end())
                {
                    auto currResult = find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]);
                    int k = distance(freeAnglePairList.begin(), currResult);
                    freeAnglePairList.erase(freeAnglePairList.begin() + k);

                    get<0>(currList_big[i]) = get<0>(currList_small[i]);

                    if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) == freeAnglePairList.end())
                        freeAnglePairList.push_back(currList_small[i]);
                }

                if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]) == freeAnglePairList.end()
                    and find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) != freeAnglePairList.end())
                {
                    auto currResult = find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]);
                    int k = distance(freeAnglePairList.begin(), currResult);
                    freeAnglePairList.erase(freeAnglePairList.begin() + k);

                    get<0>(currList_small[i]) = get<0>(currList_big[i]);

                    if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) == freeAnglePairList.end())
                        freeAnglePairList.push_back(currList_small[i]);
                }

                if (find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_big[i]) == freeAnglePairList.end()
                and find(freeAnglePairList.begin(), freeAnglePairList.end(), currList_small[i]) == freeAnglePairList.end())
                {
                    get<0>(currList_small[i]) = get<0>(currList_big[i]);

                    currUnMatchedNum ++;
                    currDiffAngle += fabs(get<0>(currList_small[i]) - get<0>(currList_big[i]));

                    if (fabs(get<0>(currList_small[i]) - get<0>(currList_big[i])) > currDiffMaxAngle)
                    {
                        currDiffMaxAngle = fabs(get<0>(currList_small[i]) - get<0>(currList_big[i]));
                    }
                }

            }

            if (currUnMatchedNum < unMatchedNum)
            {
                unMatchedNum = currUnMatchedNum;
                diffTotalAngle = currDiffAngle;
                diffMaxAngle = currDiffMaxAngle;

                cuttingAngleAndEdgeLabelPairList_big_matched = currList_big;
                cuttingAngleAndEdgeLabelPairList_small_matched = currList_small;
            }

            if (currUnMatchedNum == unMatchedNum and currDiffAngle < diffTotalAngle)
            {
                unMatchedNum = currUnMatchedNum;
                diffTotalAngle = currDiffAngle;
                diffMaxAngle = currDiffMaxAngle;

                cuttingAngleAndEdgeLabelPairList_big_matched = currList_big;
                cuttingAngleAndEdgeLabelPairList_small_matched = currList_small;
            }

        }

        cuttingAngleAndEdgeLabelPairList_small.push_back(cuttingAngleAndEdgeLabelPairList_small[0]);
        cuttingAngleAndEdgeLabelPairList_small.erase(cuttingAngleAndEdgeLabelPairList_small.begin());

        count++;
    }
}

void CuttingPlaneOptimizer::AdjustBlockAugmentedVector(PolyMesh * mySurface, PolyMesh::FaceHandle blockFH, vector< tuple<float, int> > & cuttingAngleAndEdgeLabelPairList)
{
    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    mySurface->get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    mySurface->get_property_handle( cuttingPlane ,"cuttingPlane");

    PolyMesh::FaceEdgeCCWIter fe_it;

    vector< tuple<float, int> > currList;

    for (fe_it = mySurface->fe_ccwiter(blockFH); fe_it.is_valid(); ++fe_it)
    {
        float currCuttingAngle = mySurface->property(cuttingPlane, *fe_it).GetCuttingPlaneAngle(blockFH);

        int currLabel = mySurface->property(edgeClusteringLabel, *fe_it);

        tuple<float, int> currTuple(currCuttingAngle, currLabel);

        currList.push_back(currTuple);
    }

    FindBestMatchingEdgeLabelPairList(currList, cuttingAngleAndEdgeLabelPairList);

//    dbg(cuttingAngleAndEdgeLabelPairList);

    int i = 0;
    for (fe_it = mySurface->fe_ccwiter(blockFH); fe_it.is_valid(); ++fe_it)
    {
        mySurface->property(cuttingPlane, *fe_it).UpdateCuttingPlane(blockFH, get<0>(cuttingAngleAndEdgeLabelPairList[i]));

        ++i;
    }

//    dbg(currList);
//    dbg(cuttingAngleAndEdgeLabelPairList);
}

void CuttingPlaneOptimizer::FindBestMatchingEdgeLabelPairList(const vector< tuple<float, int> > & origPairList, vector< tuple<float, int> > & matchedPairList)
{
    vector< tuple<float, int> > result;

    int unMatchedNum = 10000;
    double diffTotalAngle = 10000;
    double diffMaxAngle = 10000;

    for (int i = 0; i < origPairList.size(); ++i)
    {
        bool isSame = true;

        for (int j = 0; j < origPairList.size(); ++j) {
            if (get<1>(origPairList[j]) != get<1>(matchedPairList[j]))
            {
                isSame = false;
                break;
            }
        }

        if (isSame)
        {
            int currUnMatchedNum = 0;
            double currDiffAngle = 0;
            double currDiffMaxAngle = 0;

            for (int j = 0; j < origPairList.size(); ++j)
            {
                if (get<0>(origPairList[j]) == 0)
                    continue;

                if (get<0>(origPairList[j]) != 0 and get<0>(origPairList[j]) != get<0>(matchedPairList[j]))
                {
                    currUnMatchedNum ++;
                    currDiffAngle += fabs(get<0>(origPairList[i]) - get<0>(matchedPairList[i]));

                    if (fabs(get<0>(origPairList[i]) - get<0>(matchedPairList[i])) > currDiffMaxAngle)
                    {
                        currDiffMaxAngle = fabs(get<0>(origPairList[i]) - get<0>(matchedPairList[i]));
                    }
                }
            }

            if (currUnMatchedNum < unMatchedNum)
            {
                unMatchedNum = currUnMatchedNum;
                diffTotalAngle = currDiffAngle;
                diffMaxAngle = currDiffMaxAngle;

                result = matchedPairList;
            }

            if (currUnMatchedNum == unMatchedNum and currDiffAngle < diffTotalAngle)
            {
                unMatchedNum = currUnMatchedNum;
                diffTotalAngle = currDiffAngle;
                diffMaxAngle = currDiffMaxAngle;

                result = matchedPairList;
            }
        }

        matchedPairList.push_back(matchedPairList[0]);
        matchedPairList.erase(matchedPairList.begin());
    }

    matchedPairList = result;
}

void CuttingPlaneOptimizer::FindOrigFreeAnglePairList(vector< tuple<float, int> > & freeAnglePairList)
{
    // Get cuttingAngleAndEdgeLabelPairs to mesh
    OpenMesh::MPropHandleT< vector< tuple<float, int> > > cuttingAngleAndEdgeLabelPairs;
    baseSurface->baseSurface.get_property_handle(cuttingAngleAndEdgeLabelPairs, "cuttingAngleAndEdgeLabelPairs");

    for (int i = 0; i < baseSurface->baseSurface.property(cuttingAngleAndEdgeLabelPairs).size(); ++i)
    {
        if (get<0>(baseSurface->baseSurface.property(cuttingAngleAndEdgeLabelPairs)[i]) == 0)
        {
            freeAnglePairList.push_back(baseSurface->baseSurface.property(cuttingAngleAndEdgeLabelPairs)[i]);
        }
    }
}


///=========================================================================================///
///                                     Helper Function
///=========================================================================================///

bool CuttingPlaneOptimizer::IsTwoCuttingPlaneSame(OpenMesh::EdgeHandle inputEdgeHandle_1, OpenMesh::FaceHandle inputFaceHandle_1,
                                                       OpenMesh::EdgeHandle inputEdgeHandle_2, OpenMesh::FaceHandle inputFaceHandle_2)
{
    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface->baseSurface.get_property_handle( cuttingPlane ,"cuttingPlane");

    CuttingPlane cuttingPlane_1 = baseSurface->baseSurface.property(cuttingPlane, inputEdgeHandle_1);
    CuttingPlane cuttingPlane_2 = baseSurface->baseSurface.property(cuttingPlane, inputEdgeHandle_2);

    if (cuttingPlane_1.edgeLabel != cuttingPlane_2.edgeLabel)
        return false;

    float cuttingPlaneAngle_1 = cuttingPlane_1.GetCuttingPlaneAngle(inputFaceHandle_1);
    float cuttingPlaneAngle_2 = cuttingPlane_2.GetCuttingPlaneAngle(inputFaceHandle_2);

    if (fabs(cuttingPlaneAngle_1 - cuttingPlaneAngle_2) > angleErrorThreshold)
    {
        return false;
    }
    else
    {
        return true;
    }
}



