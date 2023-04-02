//
// Created by Linsanity on 18/4/22.
//

#include "SurfaceOptimizer.h"
#include <Clustering/fastcluster.h>
#include "TileableStructure/Initialization.h"
#include <matplotlibcpp.h>
#include <map>
#include "Clustering/dkm.hpp"
#include "Utility/dbg.h"

namespace plt = matplotlibcpp;

//**************************************************************************************//
//                                   Helper Function
//**************************************************************************************//

void PrintPoints(const ShapeOp::Matrix3X &p)
{
    for(int i = 0; i < p.cols(); ++ i){
        std::cout << "Point " << i << " : ( ";
        ShapeOp::Vector3 current_pt = p.col(i);
        std::cout << current_pt.transpose();
        std::cout << " )" << std::endl;
    }

    cout << endl;
}




//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

SurfaceOptimizer::SurfaceOptimizer(BaseSurface * _baseSurface, ShellStructure * _shellStructure)
{
    baseSurface = _baseSurface;

    shellStructure = _shellStructure;

    // Edge Clustering
    edgeClusteringNum_start = 4;
    edgeClusteringNum_end = 4;
    edgeErrorThreshold = 0.005;

    w1_edge = 10;
    w1_planarity = 1;
    iter_1 = 0;

    // Dihedral Angle Clustering
    dihedralClusterNum_start = 6;
    dihedralClusterNum_end = 6;
    dihedralAngleErrorThreshold = 0.5;

    w2_edge = 5;
    w2_dihedralAngle = 5;
    w2_planarity = 10;
    iter_2 = 0;

    // Polygon Clustering
    polygonClusterOpIter = 10;

    w3_edge = 5;
    w3_dihedralAngle = 5;
    w3_diagonal = 5;
    w3_planarity = 10;
    iter_3 = 0;

    // Cluster Merging
    miniPolygonClusterNumThreshold = 0;
    mergingCandidateNum = 10;

    // Worst-case Optimization
    iter_worst = 5;

    worstPlanarityScalar = 1.5;
    worstDiagonalScalar = 2;
    worstEdgeScalar = 2;
    worstDihedralAngleScalar = 3;

    surfaceOpTime = 0;
}

SurfaceOptimizer::~SurfaceOptimizer()
{
    ClearSurfaceOptimizerData();
}

void SurfaceOptimizer::ClearSurfaceOptimizerData()
{
    baseSurface = nullptr;
    shellStructure = nullptr;

    // Edge Clustering
    edgeClusteringNum_start = 4;
    edgeClusteringNum_end = 4;
    edgeErrorThreshold = 0.005;

    w1_edge = 1;
    w1_planarity = 0.1;
    iter_1 = 0;

    // Dihedral Angle Clustering
    dihedralClusterNum_start = 6;
    dihedralClusterNum_end = 6;
    dihedralAngleErrorThreshold = 0.5;

    w2_edge = 1;
    w2_dihedralAngle =1;
    w2_planarity = 1;
    iter_2 = 0;

    // Polygon Clustering
    polygonClusterOpIter = 10;

    w3_edge = 1;
    w3_dihedralAngle = 1;
    w3_diagonal = 1;
    w3_planarity = 1;
    iter_3 = 0;

    // Cluster Merging
    miniPolygonClusterNumThreshold = 0;
    mergingCandidateNum = 10;

    // Worst-case Optimization
    iter_worst = 5;

    worstPlanarityScalar = 1.5;
    worstDiagonalScalar = 2;
    worstEdgeScalar = 2;
    worstDihedralAngleScalar = 3;

    surfaceOpTime = 0;
}




//**************************************************************************************//
//                                 Polygon Optimization
//**************************************************************************************//

void SurfaceOptimizer::PolygonOptimization()
{
    // Polygon Optimization
    cout << "Start Surface Optimization. " << endl;

    EdgeClusteringAndOptimization();

    cout << "Edge Clustering And Optimization Done. " << endl;

    EdgeDihedralAngleClusteringAndOptimization();

    cout << "Edge Dihedral Angle Clustering And Optimization Done. " << endl;

    PolygonClusteringAndOptimization();

    cout << "Polygon Clustering And Optimization Done. " << endl;

    ClusterMerging();

    // Polygon Replacement
    baseSurface->InitReplacedSurfaceAndToleranceParameters();

    cout << "Init Replaced Surface" << endl;

    // Worst-case Analysis
    baseSurface->WorstCaseAnalysis();

    // Surface Analysis
    baseSurface->SurfaceAnalysis();
}

void SurfaceOptimizer::PolygonOptimization_Merging()
{
    ClusterMerging();

    // Polygon Replacement
    baseSurface->InitReplacedSurfaceAndToleranceParameters();

    // Worst-case Analysis
    baseSurface->WorstCaseAnalysis();
}




//**************************************************************************************//
//                             Edge Clustering and Optimization
//**************************************************************************************//

void SurfaceOptimizer::EdgeClusteringAndOptimization()
{
    // K-means clustering
    for (int i = edgeClusteringNum_start; i >= edgeClusteringNum_end; --i)
    {
        EdgeClustering_kMeans(i);

//        DihedralAngleClustering_kMeans(dihedralClusterNum_end);

        ShapeOptimization_Edge();
    }

    // Fix the cluster label and do the iteration
    for (int i = 0; i < iter_1; ++i)
    {
        ShapeOptimization_Edge();
    }

//    EdgeClustering_kMeans(edgeClusteringNum_end);
//    DihedralAngleClustering_kMeans(dihedralClusterNum_end);
//
//    baseSurface->UpdateEdgeClusterCentroidList();
//    baseSurface->UpdateDihedralAngleClusterCentroidList();

    EdgeClustering_kMeans(edgeClusteringNum_end);
    baseSurface->UpdateEdgeClusterCentroidList();
}

void SurfaceOptimizer::EdgeClustering_HierarchicalClustering(int clusteringNum)
{
    OpenMesh::MPropHandleT< vector<vector<double>> > similarityTable;
    baseSurface->baseSurface.get_property_handle(similarityTable, "similarityTable");

    if (baseSurface->baseSurface.property(similarityTable).size() == 0)
        return;

    int edgeNum = baseSurface->baseSurface.property(similarityTable).size();
    int tableElementNum = (edgeNum * (edgeNum - 1) / 2);

    double * disMatrix = new double[tableElementNum];

    int k,m,n;
    k = 0;
    for (m=k=0; m<edgeNum; m++) {
        for (n=m+1; n<edgeNum; n++) {
            disMatrix[k] = baseSurface->baseSurface.property(similarityTable)[m][n];
            k++;
        }
    }

    int* merge = new int[2*(edgeNum-1)];
    double* height = new double[edgeNum-1];
    hclust_fast(edgeNum, disMatrix, HCLUST_METHOD_SINGLE, merge, height);

    int* labels = new int[edgeNum];

    // partitioning into nclust clusters
    cutree_k(edgeNum, merge, clusteringNum, labels);

    OpenMesh::EPropHandleT< int > currLabel;
    baseSurface->baseSurface.get_property_handle(currLabel, "edgeClusteringLabel");

    OpenMesh::EPropHandleT< double > edgeCentroid;
    baseSurface->baseSurface.get_property_handle(edgeCentroid, "edgeClusterCentroid");

    int f = 0;
    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        baseSurface->baseSurface.property(currLabel, *e_it) = labels[f];
        ++f;
    }

    // clean up
    delete[] disMatrix;
    delete[] merge;
    delete[] height;
    delete[] labels;

    // Compute edge clustering centroids
    for (int i = 0; i < clusteringNum; ++i)
    {
        double totalLength = 0;
        int totalNum = 0;
        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface->baseSurface.property(currLabel, *e_it) != i)
                continue;

            const PolyMesh::Point to   = baseSurface->baseSurface.point(baseSurface->baseSurface.to_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));
            const PolyMesh::Point from = baseSurface->baseSurface.point(baseSurface->baseSurface.from_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));

            totalLength += (to - from).norm();
            totalNum ++;
        }

        double currCentroid = totalLength / totalNum;

        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface->baseSurface.property(currLabel, *e_it) != i)
                continue;

            baseSurface->baseSurface.property(edgeCentroid, *e_it) = currCentroid;
        }
    }
}

void SurfaceOptimizer::EdgeClustering_kMeans(int clusteringNum)
{
    // Extract data from baseSurface
    vector<array<float, 1>> data;

    // Get edgeLength for each edge
    OpenMesh::EPropHandleT< double > edgeLength;
    baseSurface->baseSurface.get_property_handle(edgeLength, "edgeLength");

    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        array<float, 1> currArray;

        const PolyMesh::Point to   = baseSurface->baseSurface.point(baseSurface->baseSurface.to_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface->baseSurface.point(baseSurface->baseSurface.from_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));

        currArray[0] = (to - from).norm();

        data.push_back(currArray);
    }

    // K-means clustering
    auto cluster_data = dkm::kmeans_lloyd(data, clusteringNum);

    // Update labels
    OpenMesh::EPropHandleT< int > currLabel;
    baseSurface->baseSurface.get_property_handle(currLabel, "edgeClusteringLabel");

    auto labelList = std::get<1>(cluster_data);

//    cout << "edge label list: " << endl;
    int f = 0;
    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        baseSurface->baseSurface.property(currLabel, *e_it) = int(labelList[f]);

//        cout << baseSurface->baseSurface.property(currLabel, *e_it) << ", ";
        ++f;
    }
//    cout << endl;

    // Update centroids
    OpenMesh::EPropHandleT< double > edgeCentroid;
    baseSurface->baseSurface.get_property_handle(edgeCentroid, "edgeClusterCentroid");

    auto centroidList = std::get<0>(cluster_data);

    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        int index = baseSurface->baseSurface.property(currLabel, *e_it);
        baseSurface->baseSurface.property(edgeCentroid, *e_it) = centroidList[index][0];
    }

    // Update centroidList
    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface->baseSurface.get_property_handle(edgeClusterCentroidList, "edgeClusterCentroidList");
    baseSurface->baseSurface.property(edgeClusterCentroidList).clear();

//    cout << "edge centroid list: " << endl;
    for (auto i : centroidList)
    {
//        cout << i[0] << ", ";
        baseSurface->baseSurface.property(edgeClusterCentroidList).push_back(i[0]);
    }
//    cout << endl;

//    cout << "Edge: " << endl;
//    for (int i = 0; i < baseSurface->baseSurface.property(edgeClusterCentroidList).size(); ++i)
//    {
//        vector<double> currList;
//
//        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
//        {
//            if (baseSurface->baseSurface.property(currLabel, *e_it) == i)
//            {
//                const PolyMesh::Point currTo   = baseSurface->baseSurface.point(baseSurface->baseSurface.to_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));
//                const PolyMesh::Point currFrom = baseSurface->baseSurface.point(baseSurface->baseSurface.from_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));
//
//                currList.push_back((currTo - currFrom).norm());
//            }
//        }
//
//        for (int j = 0; j < currList.size(); ++j)
//        {
//            cout << currList[j] << ", " ;
//        }
//        cout << endl;
//    }
//
//    cout << endl << endl;
}

void SurfaceOptimizer::ShapeOptimization_Edge()
{
    ShapeOp::Matrix3X verMat; //column major

    vector< vector<int> > faceVerticeList;
    vector< vector<int> > edgeVerList;
    vector< double > targetLengthList;
    vector< vector<int> > angleVerList;
    vector< double > targetScaleList;

    PolyMesh2ShapeOpMatrix(baseSurface->baseSurface, verMat, faceVerticeList, edgeVerList, targetLengthList, angleVerList, targetScaleList);

    ShapeOp::Solver s;
//    ShapeOp::LSSolver
    s.setPoints(verMat);
    ShapeOp::Scalar weight = 1.0;

    // Add a plane constraint to all the vertices.
    {
        for (int i = 0; i < faceVerticeList.size(); ++i)
        {
            vector<int> id_vector = faceVerticeList[i];

            // Plan constraint only applicable for the face with more than 3 vertices.
            if (id_vector.size() > 3)
            {
                auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, w1_planarity, s.getPoints());

                s.addConstraint(c);
            }
        }
    }

//    cout << "Add a plane constraint to all the vertices. " << endl;

    // Add a angle constraint to all the faces.
    // The constraints have no effect to final result. Putting here for avoiding optimization error.
    {
        for (int i = 0; i < faceVerticeList.size(); ++i)
        {
            vector<int> id_vector = faceVerticeList[i];

            if (id_vector.size() == 3)
            {
                vector<int> angle1; angle1.push_back(id_vector[0]); angle1.push_back(id_vector[2]); angle1.push_back(id_vector[1]);
                vector<int> angle2; angle2.push_back(id_vector[1]); angle2.push_back(id_vector[0]); angle2.push_back(id_vector[2]);
                vector<int> angle3; angle3.push_back(id_vector[2]); angle3.push_back(id_vector[0]); angle3.push_back(id_vector[1]);

                ShapeOp::Scalar minAngle = 0 / 180 * M_PI;
                ShapeOp::Scalar maxAngle = 180 / 180 * M_PI;

                auto c1 = std::make_shared<ShapeOp::AngleConstraint>(angle1, weight, s.getPoints()); s.addConstraint(c1);
                auto c2 = std::make_shared<ShapeOp::AngleConstraint>(angle2, weight, s.getPoints()); s.addConstraint(c2);
                auto c3 = std::make_shared<ShapeOp::AngleConstraint>(angle3, weight, s.getPoints()); s.addConstraint(c3);
            }
        }
    }

//    cout << "Add a angle constraint to all the faces." << endl;

    // Add edge constraint
    {
        for (int i = 0; i < edgeVerList.size(); ++i)
        {
            vector<int> id_vector = edgeVerList[i];

            if (edgeVerList[i].size() == 2)
            {
                auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, ShapeOp::Scalar(w1_edge), s.getPoints(), 1, 1);
                c->setEdgeLength(targetLengthList[i]);
                s.addConstraint(c);
            }
        }
    }

//    cout << "Add edge constraint. " << endl;

    s.initialize();
    s.solve(100);
    verMat = s.getPoints();

//    cout << "ShapeOp done. " << endl;

    // Move the model to the center (0, 0, 0)
    Vector3d minPoint = verMat.rowwise().minCoeff();
    Vector3d maxPoint = verMat.rowwise().maxCoeff();
    Vector3d currCenter = (maxPoint + minPoint) / 2;
    Vector3d offset = Vector3d(0,0,0) - currCenter;

    for (int i = 0; i < verMat.cols(); ++i)
    {
        verMat(0, i) += offset(0);
        verMat(1, i) += offset(1);
        verMat(2, i) += offset(2);
    }

    baseSurface->triBaseSurface.clear();

    ShapeOpMatrix2OriPolyMesh(baseSurface->baseSurface, verMat);

    baseSurface->triBaseSurface = baseSurface->baseSurface;
    baseSurface->triBaseSurface.triangulate();

    // Update Properties
    baseSurface->UpdateEdgeLength();
    baseSurface->UpdateEdgeClusterCentroidList();
}


//**************************************************************************************//
//                             Dihedral Angle Clustering
//**************************************************************************************//

void SurfaceOptimizer::EdgeDihedralAngleClusteringAndOptimization()
{
    // K-means clustering
    for (int i = dihedralClusterNum_start; i >= dihedralClusterNum_end; --i)
    {
        EdgeClustering_kMeans(edgeClusteringNum_end);
        DihedralAngleClustering_kMeans(i);
        ShapeOptimization_Edge_DihedralAngle();
    }

    for (int i = 0; i < iter_2; ++i)
    {
        ShapeOptimization_Edge_DihedralAngle();
    }

    EdgeClustering_kMeans(edgeClusteringNum_end);
    DihedralAngleClustering_kMeans(dihedralClusterNum_end);

    baseSurface->UpdateEdgeClusterCentroidList();
    baseSurface->UpdateDihedralAngleClusterCentroidList();
}

void SurfaceOptimizer::DihedralAngleClustering_HierarchicalClustering(int clusteringNum)
{
    OpenMesh::MPropHandleT< vector<vector<double>> > angleSimiTable;
    baseSurface->baseSurface.get_property_handle(angleSimiTable, "angleSimiTable");

    if (baseSurface->baseSurface.property(angleSimiTable).size() == 0)
        return;

    int edgeNum = baseSurface->baseSurface.property(angleSimiTable).size();

    int tableElementNum = (edgeNum * (edgeNum - 1) / 2);

    double * disMatrix = new double[tableElementNum];

    int k,m,n;
    k = 0;
    for (m=k=0; m<edgeNum; m++) {
        for (n=m+1; n<edgeNum; n++) {
            disMatrix[k] = baseSurface->baseSurface.property(angleSimiTable)[m][n];
            k++;
        }
    }

    int* merge = new int[2*(edgeNum-1)];
    double* height = new double[edgeNum-1];
    hclust_fast(edgeNum, disMatrix, HCLUST_METHOD_SINGLE, merge, height);

    int* labels = new int[edgeNum];

    // partitioning into nclust clusters
    cutree_k(edgeNum, merge, clusteringNum, labels);

    OpenMesh::EPropHandleT< int > currLabel;
    baseSurface->baseSurface.get_property_handle(currLabel, "dihedralAngleLabel");

    OpenMesh::EPropHandleT< double > currAngle;
    baseSurface->baseSurface.get_property_handle(currAngle, "dihedralAngle");

    OpenMesh::EPropHandleT< double > edgeCentroid;
    baseSurface->baseSurface.get_property_handle(edgeCentroid, "angleClusterCentroid");

    int f = 0;
    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface->baseSurface.property(currLabel, *e_it) == -1)
            continue;

        baseSurface->baseSurface.property(currLabel, *e_it) = labels[f];
        ++f;
    }

    // clean up
    delete[] disMatrix;
    delete[] merge;
    delete[] height;
    delete[] labels;

    // Compute angle clustering centroids
    for (int i = 0; i < clusteringNum; ++i)
    {
        double totalAngle = 0;
        int totalNum = 0;

        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface->baseSurface.property(currLabel, *e_it) != i)
                continue;

            totalAngle += baseSurface->baseSurface.property(currAngle, *e_it);
            totalNum ++;
        }

        double currCentroid = totalAngle / totalNum;

        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface->baseSurface.property(currLabel, *e_it) != i)
                continue;

            baseSurface->baseSurface.property(edgeCentroid, *e_it) = currCentroid;
        }
    }
}

void SurfaceOptimizer::DihedralAngleClustering_kMeans(int clusteringNum)
{
    // Extract data from baseSurface
    vector<array<float, 1>> data;

    // Get edgeLength for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface->baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface->baseSurface.property(dihedralAngle, *e_it) > 0)
        {
            array<float, 1> currArray;

            currArray[0] = baseSurface->baseSurface.property(dihedralAngle, *e_it);

            data.push_back(currArray);
        }
    }

    // K-means clustering
    auto cluster_data = dkm::kmeans_lloyd(data, clusteringNum);

    // Update labels
    OpenMesh::EPropHandleT< int > currLabel;
    baseSurface->baseSurface.get_property_handle(currLabel, "dihedralAngleLabel");

    auto labelList = std::get<1>(cluster_data);

    int f = 0;
    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface->baseSurface.property(dihedralAngle, *e_it) > 0)
        {
            baseSurface->baseSurface.property(currLabel, *e_it) = int(labelList[f]);
            ++f;
        }
//        dbg(baseSurface->baseSurface.property(currLabel, *e_it));
    }

    // Update centroids
    OpenMesh::EPropHandleT< double > dihedralAngleCentroid;
    baseSurface->baseSurface.get_property_handle(dihedralAngleCentroid, "angleClusterCentroid");

    auto centroidList = std::get<0>(cluster_data);

    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface->baseSurface.property(dihedralAngle, *e_it) > 0)
        {
            int index = baseSurface->baseSurface.property(currLabel, *e_it);
            baseSurface->baseSurface.property(dihedralAngleCentroid, *e_it) = centroidList[index][0];
        }
        else
        {
            baseSurface->baseSurface.property(dihedralAngleCentroid, *e_it) = -1;
        }
//        dbg(baseSurface->baseSurface.property(dihedralAngleCentroid, *e_it));
    }

    // Update centroidList
    OpenMesh::MPropHandleT< vector<double> > angleClusterCentroidList;
    baseSurface->baseSurface.get_property_handle(angleClusterCentroidList, "angleClusterCentroidList");
    baseSurface->baseSurface.property(angleClusterCentroidList).clear();

    for (auto i : centroidList)
    {
        baseSurface->baseSurface.property(angleClusterCentroidList).push_back(i[0]);
    }

//    cout << "Dihedral Angle: " << endl;
//    for (int i = 0; i < baseSurface->baseSurface.property(angleClusterCentroidList).size(); ++i)
//    {
//        vector<double> currList;
//
//        for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
//        {
//            if (baseSurface->baseSurface.property(currLabel, *e_it) == i and !baseSurface->baseSurface.is_boundary(*e_it))
//            {
//                currList.push_back(180 - baseSurface->baseSurface.calc_dihedral_angle(*e_it) / M_PI * 180.0f);
//            }
//        }
//
//        for (int j = 0; j < currList.size(); ++j)
//        {
//            cout << currList[j] << ", " ;
//        }
//        cout << endl;
//    }
//
//    cout << endl << endl;
}

void SurfaceOptimizer::ShapeOptimization_Edge_DihedralAngle()
{
    ShapeOp::Matrix3X verMat; //column major

    vector< vector<int> > faceVerticeList;
    vector< vector<int> > edgeVerList;
    vector< double > targetLengthList;
    vector< vector<int> > angleVerList;
    vector< double > targetScaleList;

    PolyMesh2ShapeOpMatrix(baseSurface->baseSurface, verMat, faceVerticeList, edgeVerList, targetLengthList, angleVerList, targetScaleList);

    ShapeOp::Solver s;
    s.setPoints(verMat);
    ShapeOp::Scalar weight = 1.0;

    // Add a plane constraint to all the vertices.
    {
        for (int i = 0; i < faceVerticeList.size(); ++i)
        {
            vector<int> id_vector = faceVerticeList[i];

            // Plane constraint only applicable for the face with more than 3 vertices.
            if (id_vector.size() > 3)
            {
                auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, w2_planarity, s.getPoints());

                s.addConstraint(c);
            }
        }
    }

    // Add a angle constraint to all the faces.
    // The constraints have no effect to final result. Putting here for avoiding optimization error.
    {
        for (int i = 0; i < faceVerticeList.size(); ++i)
        {
            vector<int> id_vector = faceVerticeList[i];

            if (id_vector.size() == 3)
            {
                vector<int> angle1; angle1.push_back(id_vector[0]); angle1.push_back(id_vector[2]); angle1.push_back(id_vector[1]);
                vector<int> angle2; angle2.push_back(id_vector[1]); angle2.push_back(id_vector[0]); angle2.push_back(id_vector[2]);
                vector<int> angle3; angle3.push_back(id_vector[2]); angle3.push_back(id_vector[0]); angle3.push_back(id_vector[1]);

                ShapeOp::Scalar minAngle = 0 / 180 * M_PI;
                ShapeOp::Scalar maxAngle = 180 / 180 * M_PI;

                auto c1 = std::make_shared<ShapeOp::AngleConstraint>(angle1, weight, s.getPoints()); s.addConstraint(c1);
                auto c2 = std::make_shared<ShapeOp::AngleConstraint>(angle2, weight, s.getPoints()); s.addConstraint(c2);
                auto c3 = std::make_shared<ShapeOp::AngleConstraint>(angle3, weight, s.getPoints()); s.addConstraint(c3);
            }
        }
    }

    // Add edge constraint
    {
        for (int i = 0; i < edgeVerList.size(); ++i)
        {
            vector<int> id_vector = edgeVerList[i];

            if (edgeVerList[i].size() == 2)
            {
                auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w2_edge, s.getPoints(), 1, 1);
                c->setEdgeLength(targetLengthList[i]);
                s.addConstraint(c);
            }
        }
    }

    // Add Dihedral Angle constraint
    {
        for (int i = 0; i < angleVerList.size(); ++i)
        {
            vector<int> id_vector = angleVerList[i];

            if (angleVerList[i].size() == 4)
            {
                auto c = std::make_shared<ShapeOp::BendingConstraint>(id_vector, ShapeOp::Scalar(w2_dihedralAngle), s.getPoints(), targetScaleList[i], targetScaleList[i]);
                s.addConstraint(c);
            }
        }
    }

    s.initialize();
    s.solve(100);
    verMat = s.getPoints();

    // Move the model to the center (0, 0, 0)
    Vector3d minPoint = verMat.rowwise().minCoeff();
    Vector3d maxPoint = verMat.rowwise().maxCoeff();
    Vector3d currCenter = (maxPoint + minPoint) / 2;
    Vector3d offset = Vector3d(0,0,0) - currCenter;

    for (int i = 0; i < verMat.cols(); ++i)
    {
        verMat(0, i) += offset(0);
        verMat(1, i) += offset(1);
        verMat(2, i) += offset(2);
    }

    baseSurface->triBaseSurface.clear();

    ShapeOpMatrix2OriPolyMesh(baseSurface->baseSurface, verMat);

    baseSurface->triBaseSurface = baseSurface->baseSurface;
    baseSurface->triBaseSurface.triangulate();

    // Update Properties
    baseSurface->UpdateEdgeClusterCentroidList();
    baseSurface->UpdateDihedralAngleClusterCentroidList();
}




//**************************************************************************************//
//           Clustering and optimization for edge and dihedral angle and polygon
//**************************************************************************************//

void SurfaceOptimizer::PolygonClusteringAndOptimization()
{
    PolygonClustering();

//    baseSurface->UpdatePolygonSimilarityTable_LSM();

    ShapeOptimization_PolygonClustering();

    for (int i = 0; i < iter_3; ++i)
    {
        ShapeOptimization_PolygonClustering();
    }

    baseSurface->UpdatePolygonSimilarityTable_LSM();
}

void SurfaceOptimizer::PolygonClustering(bool isShowInfo)
{
    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    baseSurface->baseSurface.get_property_handle(clusteringLabel, "clusteringLabel");

    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface->baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");
    baseSurface->baseSurface.property(polygonClusterList).clear();

    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface->baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get polygonClusterFullList to mesh
    OpenMesh::MPropHandleT< vector<vector<vector<int>>> > polygonClusterFullList;
    baseSurface->baseSurface.get_property_handle(polygonClusterFullList, "polygonClusterFullList");
    baseSurface->baseSurface.property(polygonClusterFullList).clear();

    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        PolyMesh::FaceEdgeCWIter fe_it;

        vector<int> currEdgeClusterLabelList;

        for (fe_it = baseSurface->baseSurface.fe_cwiter(*f_it); fe_it.is_valid(); ++fe_it)
        {
            currEdgeClusterLabelList.push_back(baseSurface->baseSurface.property(edgeClusteringLabel, *fe_it));
        }

        if (IsCurrListInExistingList(baseSurface->baseSurface.property(polygonClusterList), currEdgeClusterLabelList) == -1)
        {
            baseSurface->baseSurface.property(polygonClusterList).push_back(currEdgeClusterLabelList);
            baseSurface->baseSurface.property(clusteringLabel, *f_it) = baseSurface->baseSurface.property(polygonClusterList).size() - 1;

            vector< vector<int> > currList;
            currList.push_back(currEdgeClusterLabelList);
            baseSurface->baseSurface.property(polygonClusterFullList).push_back(currList);
        }
        else
        {
            baseSurface->baseSurface.property(clusteringLabel, *f_it) = IsCurrListInExistingList(baseSurface->baseSurface.property(polygonClusterList), currEdgeClusterLabelList);
            baseSurface->baseSurface.property(polygonClusterFullList)[baseSurface->baseSurface.property(clusteringLabel, *f_it)].push_back(currEdgeClusterLabelList);
        }
    }

    polygonClusterNum = baseSurface->baseSurface.property(polygonClusterFullList).size();

    if (isShowInfo)
    {
        // Print clustering Info
        cout << "Unique polygon number: " << baseSurface->baseSurface.property(polygonClusterFullList).size() << endl;
        cout << "Unique polygon number of each cluster: ";
        for (int i = 0; i < baseSurface->baseSurface.property(polygonClusterFullList).size(); ++i)
        {
            if (i % 10 == 0)
                cout << endl;

            cout << baseSurface->baseSurface.property(polygonClusterFullList)[i].size() << " ";
        }
        cout << endl << endl;
    }


    // Update diagonalList and diagonalClusterCentroidList
    baseSurface->UpdateDiagnonalList();
}

void SurfaceOptimizer::ShapeOptimization_PolygonClustering()
{
    // Update Properties
    baseSurface->UpdateEdgeClusterCentroidList();
    baseSurface->UpdateDihedralAngleClusterCentroidList();
    baseSurface->UpdateDiagnonalList();

    ShapeOp::Matrix3X verMat; //column major

    vector< vector<int> > faceVerticeList;
    vector< vector<int> > edgeVerList;
    vector< double > targetLengthList;
    vector< vector<int> > angleVerList;
    vector< double > targetScaleList;
    vector< vector<int> > diagonalVerList;
    vector<double> targetDiagonalList;

    PolyMesh2ShapeOpMatrix(baseSurface->baseSurface, verMat, faceVerticeList,
                           edgeVerList, targetLengthList,
                           angleVerList, targetScaleList,
                           diagonalVerList, targetDiagonalList);

    ShapeOp::Solver s;
    s.setPoints(verMat);
    ShapeOp::Scalar weight = 1.0;

    // Add a plane constraint to all the vertices.
    {
        for (int i = 0; i < faceVerticeList.size(); ++i)
        {
            vector<int> id_vector = faceVerticeList[i];

            // Plane constraint only applicable for the face with more than 3 vertices.
            if (id_vector.size() > 3)
            {
                auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, w3_planarity, s.getPoints());

                s.addConstraint(c);
            }
        }
    }

    // Add diagonal constraint
    {
        for (int i = 0; i < diagonalVerList.size(); ++i)
        {
            vector<int> id_vector = diagonalVerList[i];

            if (id_vector.size() == 2)
            {
                auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w3_diagonal, s.getPoints(), 1, 1);
                c->setEdgeLength(targetDiagonalList[i]);
                s.addConstraint(c);
            }
        }
    }

    // Add edge constraint
    {
        for (int i = 0; i < edgeVerList.size(); ++i)
        {
            vector<int> id_vector = edgeVerList[i];

            if (id_vector.size() == 2)
            {
                auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w3_edge, s.getPoints(), 1, 1);
                c->setEdgeLength(targetLengthList[i]);
                s.addConstraint(c);
            }
        }
    }

    // Add Dihedral Angle constraint
    {
        for (int i = 0; i < angleVerList.size(); ++i)
        {
            vector<int> id_vector = angleVerList[i];

            if (id_vector.size() == 4)
            {
                auto c = std::make_shared<ShapeOp::BendingConstraint>(id_vector, w3_dihedralAngle ,s.getPoints(), targetScaleList[i], targetScaleList[i]);
                s.addConstraint(c);
            }
        }
    }

    s.initialize();
    s.solve(100);
    verMat = s.getPoints();

    // Move the model to the center (0, 0, 0)
    Vector3d minPoint = verMat.rowwise().minCoeff();
    Vector3d maxPoint = verMat.rowwise().maxCoeff();
    Vector3d currCenter = (maxPoint + minPoint) / 2;
    Vector3d offset = Vector3d(0,0,0) - currCenter;

    for (int i = 0; i < verMat.cols(); ++i)
    {
        verMat(0, i) += offset(0);
        verMat(1, i) += offset(1);
        verMat(2, i) += offset(2);
    }

    baseSurface->triBaseSurface.clear();

    ShapeOpMatrix2OriPolyMesh(baseSurface->baseSurface, verMat);

    // Update Properties
    baseSurface->UpdateEdgeClusterCentroidList();
    baseSurface->UpdateDihedralAngleClusterCentroidList();
    baseSurface->UpdateDiagnonalList();

    baseSurface->triBaseSurface = baseSurface->baseSurface;
    baseSurface->triBaseSurface.triangulate();

    ClusteringLabelPropertyTransfer(&baseSurface->baseSurface, &baseSurface->triBaseSurface);
}




//**************************************************************************************//
//                                  Cluster Merging
//**************************************************************************************//

void SurfaceOptimizer::ClusterMerging()
{
    vector<int> smallIndexList;
    FindSmallClusterIndexList(smallIndexList);

    int count = 0;
    int j = 0;
    while (count < 100 and j < smallIndexList.size() - 1 and !smallIndexList.empty())
    {
        vector<int> currMergingCandidates;
        FindMergingCandidates(smallIndexList[j], currMergingCandidates);

        if (MergeCluster(smallIndexList[j], currMergingCandidates))
        {
            PolygonClustering();

            baseSurface->UpdatePolygonSimilarityTable_LSM();

            smallIndexList.clear();
            FindSmallClusterIndexList(smallIndexList);

            j = 0;
        }
        else
        {
//            PolygonClustering(false);
            ++j;
        }

        ++count;
    }

    cout << "Merging finished. " << endl;

//    PolygonClustering();
    ShapeOptimization_PolygonClustering();

    baseSurface->UpdatePolygonSimilarityTable_LSM();
    baseSurface->UpdatePolygonPlanarity_LSM(&baseSurface->baseSurface, &baseSurface->triBaseSurface);

    cout << "Clustering Merging Done. " << endl;
}

void SurfaceOptimizer::FindSmallClusterIndexList(vector<int> & indexList)
{
    indexList.clear();

    // Get polygonClusterFullList to mesh
    OpenMesh::MPropHandleT< vector<vector<vector<int>>> > polygonClusterFullList;
    baseSurface->baseSurface.get_property_handle(polygonClusterFullList, "polygonClusterFullList");

    for (int i = 0; i < baseSurface->baseSurface.property(polygonClusterFullList).size(); ++i)
    {
        if (baseSurface->baseSurface.property(polygonClusterFullList)[i].size() <= miniPolygonClusterNumThreshold)
        {
            indexList.push_back(i);
        }
    }
}

void SurfaceOptimizer::FindSmallestClusterIndex(int & index, int & minClusterNum)
{
    // Get polygonClusterFullList to mesh
    OpenMesh::MPropHandleT< vector<vector<vector<int>>> > polygonClusterFullList;
    baseSurface->baseSurface.get_property_handle(polygonClusterFullList, "polygonClusterFullList");

    vector<int> clusterNumList;
    for (int i = 0; i < baseSurface->baseSurface.property(polygonClusterFullList).size(); ++i)
    {
        clusterNumList.push_back(baseSurface->baseSurface.property(polygonClusterFullList)[i].size());
    }

    index = (min_element(clusterNumList.begin(), clusterNumList.end()) - clusterNumList.begin());
    minClusterNum = clusterNumList[index];
}

void SurfaceOptimizer::FindMergingCandidates(int index, vector<int> & mergingCandidateIndexList)
{
    // Get centroidPolygonList to mesh
    OpenMesh::MPropHandleT< vector<vector<Vector3d>> > centroidPolygonList;
    baseSurface->baseSurface.get_property_handle(centroidPolygonList, "centroidPolygonList");

    vector<Vector3d> currSmallClusterCentroid = baseSurface->baseSurface.property(centroidPolygonList)[index];

    vector<double> simiList;
    for (int i = 0; i < baseSurface->baseSurface.property(centroidPolygonList).size(); ++i)
    {
        if (i ==index)
        {
            simiList.push_back(10000);
            continue;
        }

        if (baseSurface->baseSurface.property(centroidPolygonList)[i].size() != currSmallClusterCentroid.size())
        {
            simiList.push_back(10000);
        }
        else
        {
            double currSimi;

            ComputePolygonSimilarity(currSmallClusterCentroid, baseSurface->baseSurface.property(centroidPolygonList)[i], currSimi);

            simiList.push_back(currSimi);
        }
    }

    mergingCandidateIndexList.clear();
    for (int i = 0; i < mergingCandidateNum; ++i)
    {
        auto currMinLocation = min_element(simiList.begin(), simiList.end()) - simiList.begin();

        if (simiList[currMinLocation] == 10000)
            break;

        mergingCandidateIndexList.push_back(currMinLocation);

        simiList[currMinLocation] = 10000;
    }
}

bool SurfaceOptimizer::MergeCluster(int index, vector<int> mergingCandidateIndexList)
{
    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface->baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");

    int origPolygonClusterNum = baseSurface->baseSurface.property(polygonClusterList).size();

    // Make a copy for relabeling
    BaseSurface tempSurface = *baseSurface;

    // Get clusterLabel to each face temp
    OpenMesh::FPropHandleT< int > tempClusteringLabel;
    tempSurface.baseSurface.get_property_handle(tempClusteringLabel, "clusteringLabel");

    // Get edgeClusteringLabel for each edge temp
    OpenMesh::EPropHandleT< int > tempEdgeClusteringLabel;
    tempSurface.baseSurface.get_property_handle(tempEdgeClusteringLabel, "edgeClusteringLabel");

    vector<PolyMesh::FaceHandle> processedFHList;

    for (int i = 0; i < mergingCandidateIndexList.size(); ++i)
    {
        for (PolyMesh::FaceIter f_it=tempSurface.baseSurface.faces_begin(); f_it!=tempSurface.baseSurface.faces_end(); ++f_it)
        {
            if (tempSurface.baseSurface.property(tempClusteringLabel, *f_it) != index or find(processedFHList.begin(), processedFHList.end(), *f_it) != processedFHList.end())
                continue;

            // Get currSmallClusterEdgeLabelList
            PolyMesh::FaceEdgeCWIter fe_it;
            vector<int> currSmallClusterEdgeLabelList;

            for (fe_it = tempSurface.baseSurface.fe_cwiter(*f_it); fe_it.is_valid(); ++fe_it)
            {
                currSmallClusterEdgeLabelList.push_back(tempSurface.baseSurface.property(tempEdgeClusteringLabel, *fe_it));
            }

            // Get currMergingClusterEdgeLabelList
            vector<int> currMergingClusterEdgeLabelList = baseSurface->baseSurface.property(polygonClusterList)[mergingCandidateIndexList[i]];

            // Find bestMatchingList
            int j = 0;
            int currMatchingNum = -1;

            vector<int> bestMatchingList = currMergingClusterEdgeLabelList;

            while(j < currSmallClusterEdgeLabelList.size())
            {
                if (ComputeMatchingNum(currMergingClusterEdgeLabelList, currSmallClusterEdgeLabelList) > currMatchingNum)
                {
                    currMatchingNum = ComputeMatchingNum(currMergingClusterEdgeLabelList, currSmallClusterEdgeLabelList);
                    bestMatchingList = currMergingClusterEdgeLabelList;
                }

                currMergingClusterEdgeLabelList.push_back(currMergingClusterEdgeLabelList[0]);
                currMergingClusterEdgeLabelList.erase(currMergingClusterEdgeLabelList.begin());

                ++j;
            }

            // Reassign the edge label
            j = 0;
            for (fe_it = tempSurface.baseSurface.fe_cwiter(*f_it); fe_it.is_valid(); ++fe_it)
            {
                tempSurface.baseSurface.property(tempEdgeClusteringLabel, *fe_it) = bestMatchingList[j];
                ++j;
            }

            // Compute polygon cluster number after reassign the edge label
            if (tempSurface.ComputePolygonClusterNumByEdgeLabel() > origPolygonClusterNum)
            {
                // Revert the edge label
                j = 0;
                for (fe_it = tempSurface.baseSurface.fe_cwiter(*f_it); fe_it.is_valid(); ++fe_it)
                {
                    tempSurface.baseSurface.property(tempEdgeClusteringLabel, *fe_it) = currSmallClusterEdgeLabelList[j];
                    ++j;
                }
            }

            else if (tempSurface.ComputePolygonClusterNumByEdgeLabel() == origPolygonClusterNum)
            {
                processedFHList.push_back(*f_it);
            }

            else
            {
                cout << "Merging successful. " << endl;
                baseSurface->baseSurface = tempSurface.baseSurface;

                baseSurface->UpdateEdgeClusterCentroidList();
                baseSurface->UpdateDihedralAngleClusterCentroidList();
                baseSurface->UpdateDiagnonalList();

                return true;
            }
        }
    }

    cout << "Merging failed. " << endl;
    return false;
}




//**************************************************************************************//
//                                Outlier Optimization
//**************************************************************************************//

void SurfaceOptimizer::WorstCaseOptimization()
{
    for (int i = 0; i < iter_worst; i++)
    {
        ShapeOptimization_WorstCaseOptimization();

        // Polygon Replacement
        baseSurface->InitReplacedSurfaceAndToleranceParameters();

        // Worst-case Analysis
        baseSurface->WorstCaseAnalysis();
    }
}

void SurfaceOptimizer::ShapeOptimization_WorstCaseOptimization()
{
    ShapeOp::Matrix3X verMat; //column major

    vector< vector<int> > faceVerticeList;
    vector< vector<int> > edgeVerList;
    vector< double > targetLengthList;
    vector< vector<int> > angleVerList;
    vector< double > targetScaleList;
    vector< vector<int> > diagonalVerList;
    vector<double> targetDiagonalList;

    int worstFabVer;
    vector< vector<int> > worstPolygonEdgeList;
    vector< vector<int> > worstDiagonal;
    vector<int> worstPlanarityVerList;
    vector< vector<int> > worstDihedralAngleVerList;

    PolyMesh2ShapeOpMatrix(baseSurface->baseSurface, verMat,
                           faceVerticeList, edgeVerList, targetLengthList,
                           angleVerList, targetScaleList,
                           diagonalVerList, targetDiagonalList,
                           worstFabVer, worstPolygonEdgeList, worstDiagonal,
                           worstPlanarityVerList, worstDihedralAngleVerList);

//    dbg(worstFabVer);
//    dbg(worstPolygonEdgeList);
//    dbg(worstDiagonal);
//    dbg(worstPlanarityVerList);
//    dbg(worstDihedralAngleVerList);

    ShapeOp::Solver s;
    s.setPoints(verMat);
    ShapeOp::Scalar weight = 1.0;

    // Add a plane constraint to all the vertices.
    {
        for (int i = 0; i < faceVerticeList.size(); ++i)
        {
            vector<int> id_vector = faceVerticeList[i];

            // Plane constraint only applicable for the face with more than 3 vertices.
            if (id_vector.size() > 3)
            {
                if (id_vector == worstPlanarityVerList)
                {
                    auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, w3_planarity * worstPlanarityScalar, s.getPoints());

                    s.addConstraint(c);
                }
                else
                {
                    auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, w3_planarity, s.getPoints());

                    s.addConstraint(c);
                }
            }
        }
    }

    // Add diagonal constraint
    {
        for (int i = 0; i < diagonalVerList.size(); ++i)
        {
            vector<int> id_vector = diagonalVerList[i];

            if (id_vector.size() == 2)
            {
                if (find(worstDiagonal.begin(), worstDiagonal.end(), id_vector) == worstDiagonal.end())
                {
                    auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w3_diagonal, s.getPoints(), 1, 1);
                    c->setEdgeLength(targetDiagonalList[i]);
                    s.addConstraint(c);
                }
                else
                {
                    auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w3_diagonal * worstDiagonalScalar, s.getPoints(), 1, 1);
                    c->setEdgeLength(targetDiagonalList[i]);
                    s.addConstraint(c);
                }
            }
        }
    }

    // Add edge constraint
    {
        for (int i = 0; i < edgeVerList.size(); ++i)
        {
            vector<int> id_vector = edgeVerList[i];

            if (id_vector.size() == 2)
            {
                if (find(worstPolygonEdgeList.begin(), worstPolygonEdgeList.end(), id_vector) == worstPolygonEdgeList.end()
                    and find(id_vector.begin(), id_vector.end(), worstFabVer) == id_vector.end())
                {
                    auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w3_edge, s.getPoints(), 1, 1);
                    c->setEdgeLength(targetLengthList[i]);
                    s.addConstraint(c);
                }
                else
                {
                    auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id_vector, w3_edge * worstEdgeScalar, s.getPoints(), 1, 1);
                    c->setEdgeLength(targetLengthList[i]);
                    s.addConstraint(c);
                }
            }
        }
    }

    // Add Dihedral Angle constraint
    {
        for (int i = 0; i < angleVerList.size(); ++i)
        {
            vector<int> id_vector = angleVerList[i];

            if (id_vector.size() == 4)
            {
                if (find(worstDihedralAngleVerList.begin(), worstDihedralAngleVerList.end(), id_vector) == worstDihedralAngleVerList.end())
                {
                    auto c = std::make_shared<ShapeOp::BendingConstraint>(id_vector, w3_dihedralAngle ,s.getPoints(), targetScaleList[i], targetScaleList[i]);
                    s.addConstraint(c);
                }
                else
                {
                    auto c = std::make_shared<ShapeOp::BendingConstraint>(id_vector, w3_dihedralAngle * worstDihedralAngleScalar, s.getPoints(), targetScaleList[i], targetScaleList[i]);
                    s.addConstraint(c);
                }
            }
        }
    }

    s.initialize();
    s.solve(100);
    verMat = s.getPoints();

    // Move the model to the center (0, 0, 0)
    Vector3d minPoint = verMat.rowwise().minCoeff();
    Vector3d maxPoint = verMat.rowwise().maxCoeff();
    Vector3d currCenter = (maxPoint + minPoint) / 2;
    Vector3d offset = Vector3d(0,0,0) - currCenter;

    for (int i = 0; i < verMat.cols(); ++i)
    {
        verMat(0, i) += offset(0);
        verMat(1, i) += offset(1);
        verMat(2, i) += offset(2);
    }

    baseSurface->triBaseSurface.clear();

    ShapeOpMatrix2OriPolyMesh(baseSurface->baseSurface, verMat);

    // Update Properties
    baseSurface->UpdateEdgeClusterCentroidList();
    baseSurface->UpdateDihedralAngleClusterCentroidList();
    baseSurface->UpdateDiagnonalList();

    baseSurface->triBaseSurface = baseSurface->baseSurface;
    baseSurface->triBaseSurface.triangulate();

    ClusteringLabelPropertyTransfer(&baseSurface->baseSurface, &baseSurface->triBaseSurface);

    baseSurface->UpdatePolygonSimilarityTable_LSM();
    baseSurface->UpdatePolygonPlanarity_LSM(&baseSurface->baseSurface, &baseSurface->triBaseSurface);
}




//**************************************************************************************//
//                                  Helper Functions
//**************************************************************************************//

void SurfaceOptimizer::ComputePolygonSimilarity(vector<Vector3d> referP, vector<Vector3d> currP, double & similarity)
{
    // Set a big value for similarity
    similarity = 10000;

    // ShapeOp Matrix
    ShapeOp::Matrix3X referenceP_mat, currP_mat;

    // Prepare referenceP_mat
    referenceP_mat.resize(3, currP.size());
    for (int i = 0; i < currP.size(); ++i)
    {
        referenceP_mat(0, i) = referP[i][0];
        referenceP_mat(1, i) = referP[i][1];
        referenceP_mat(2, i) = referP[i][2];
    }

    // Move the reference model to the center (0, 0, 0)
    {
        Vector3d minPoint = referenceP_mat.rowwise().minCoeff();
        Vector3d maxPoint = referenceP_mat.rowwise().maxCoeff();
        Vector3d currCenter = (maxPoint + minPoint) / 2;
        Vector3d offset = Vector3d(0,0,0) - currCenter;

        for (int j = 0; j < referenceP_mat.cols(); ++j)
        {
            referenceP_mat(0, j) += offset(0);
            referenceP_mat(1, j) += offset(1);
            referenceP_mat(2, j) += offset(2);
        }
    }

    // Resize currP_mat
    currP_mat.resize(3, currP.size());

    for (int i = 0; i < currP.size(); ++i)
    {
        // Prepare currP_mat
        for (int j = 0; j < currP.size(); ++j)
        {
            currP_mat(0, j) = currP[j][0];
            currP_mat(1, j) = currP[j][1];
            currP_mat(2, j) = currP[j][2];
        }

        // Move the curr model to the center (0, 0, 0)
        {
            Vector3d minPoint = currP_mat.rowwise().minCoeff();
            Vector3d maxPoint = currP_mat.rowwise().maxCoeff();
            Vector3d currCenter = (maxPoint + minPoint) / 2;
            Vector3d offset = Vector3d(0,0,0) - currCenter;

            for (int j = 0; j < currP_mat.cols(); ++j)
            {
                currP_mat(0, j) += offset(0);
                currP_mat(1, j) += offset(1);
                currP_mat(2, j) += offset(2);
            }
        }

        // Use shapeOp to find the best matching configuration
        ShapeOp::Solver s;
        s.setPoints(currP_mat);
        ShapeOp::Scalar weight = 1.0;

        // Add a closeness constraint to the vertex.
        {
            std::vector<int> id_vector;

            for(int j = 0; j < currP_mat.cols(); ++j)
            {
                id_vector.push_back(j);

                auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, currP_mat);

                c->setPosition(referenceP_mat.col(j));

                s.addConstraint(c);

                id_vector.clear();
            }
        }

        // Add a rigid constraint between 1st and 4th vertex.
        {
            std::vector<int> id_vector;

            for(int j = 0; j < currP_mat.cols(); ++j)
            {
                id_vector.push_back(j);
            }

            auto c = std::make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight, currP_mat, false, true, false);

            s.addConstraint(c);
        }

        s.initialize();
        s.solve(100);
        auto p = s.getPoints();

        double currSimi = 0;

        // Compute currSimi
        for (int j = 0; j < p.cols(); ++j)
        {
            currSimi += (referenceP_mat.col(j) - p.col(j)).norm();
        }

        // Compare currSimi and previous similarity
        if (currSimi < similarity)
        {
            similarity = currSimi;
        }

        // Rotate the polygon vertex
        currP.push_back(currP[0]);
        currP.erase(currP.begin());
    }
}




//**************************************************************************************//
//                                  Save Files
//**************************************************************************************//

void SurfaceOptimizer::SaveInput(string folderPath)
{
    folderPath += "/Input";

    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    OpenMesh::IO::write_mesh(baseSurface->inputSurface->inputSurfaceTriMesh, folderPath + "/InputSurface.obj");
    OpenMesh::IO::write_mesh(baseSurface->remeshedPolySurface, folderPath + "/InputRemeshedSurface.obj");
}

void SurfaceOptimizer::SaveSurface(string folderPath)
{
    folderPath += "/_surface";

    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    OpenMesh::IO::write_mesh(baseSurface->baseSurface, folderPath + "/OptimizedSurface.obj");
    OpenMesh::IO::write_mesh(baseSurface->replacedSurface, folderPath + "/ReplacedSurface.obj");

    SaveAllClusteringPolygonOBJs(folderPath + "/_clustered_polygons", folderPath + "/_polygon_templates");

    // SavePolygonClusteringAnalysisFiles(folderPath + "/_SurfaceAnalysis");
}

void SurfaceOptimizer::SavePolygonClusteringFiles()
{
    string folderPath = savingFullPath;

    // SaveEdgeDihedralAngleClusteringAnalysisFiles(folderPath);
    // SaveParameter2File(folderPath);
    SaveSurfaceOBJs(folderPath);
    SavePolygonClusteringAnalysisFiles(folderPath);
    // SaveAllClusteringPolygonOBJs(folderPath + "/ClusteredPolygons", folderPath + "/PolygonTemplates");
//    SaveBlockClusteringAnalysisFiles(folderPath);
}

void SurfaceOptimizer::SaveBlockClusteringFiles()
{
    string folderPath = savingFullPath;

    SaveBlockClusteringAnalysisFiles(folderPath);
    SaveShellStructureOBJs(folderPath);
}

void SurfaceOptimizer::CreateFolderName(string folderPath, string surfacePath, string tilePath)
{
    // Find the folderPath

    if (folderPath[folderPath.size() - 1] == '/')
    {
        folderPath.erase(folderPath.begin() + folderPath.size() - 1);
    }

    size_t foundFolder = folderPath.find_last_of('/'); // Note: this applies for Mac only
    string savingFolderPath = folderPath.substr(0, foundFolder);

    // Find the name of input surface
    string surfaceName = surfacePath;
    for (int i = surfaceName.size() - 1; i > 0; --i)
    {
        if (surfaceName[i] == '/')
        {
            surfaceName.erase(surfaceName.begin(), surfaceName.begin() + i + 1);
            break;
        }
    }

    size_t foundPoint = surfaceName.find_last_of('.'); // Note: this applies for Mac only
    surfaceName = surfaceName.substr(0, foundPoint);

    // Find the name of input tile pattern
    string tileName = tilePath;
    for (int i = tileName.size() - 1; i > 0; --i)
    {
        if (tileName[i] == '/')
        {
            tileName.erase(tileName.begin(), tileName.begin() + i + 1);
            break;
        }
    }

    foundPoint = tileName.find_last_of('.'); // Note: this applies for Mac only
    tileName = tileName.substr(0, foundPoint);

    // Get edgeClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface->baseSurface.get_property_handle(edgeClusterCentroidList, "edgeClusterCentroidList");

    // Get angleClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<double> > angleClusterCentroidList;
    baseSurface->baseSurface.get_property_handle(angleClusterCentroidList, "angleClusterCentroidList");

    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface->baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");

    // Full file path
    // savingFolderPath + "/" + surfaceName + "_" + tileName +
    savingFullPath = savingFolderPath + "/Surface_F" + to_string(baseSurface->baseSurface.n_faces())
            + "_EC" + to_string(baseSurface->baseSurface.property(edgeClusterCentroidList).size()) + "_DC" + to_string(baseSurface->baseSurface.property(angleClusterCentroidList).size())
            + "_PC" + to_string(baseSurface->baseSurface.property(polygonClusterList).size());

    // Get polygonSimiTable
    OpenMesh::MPropHandleT< vector<vector<double>> > polygonSimiTable;
    baseSurface->baseSurface.get_property_handle(polygonSimiTable, "polygonSimiTable");

    double maxPolygonError = 0;
    double totalError = 0;
    int count = 0;

    for (int i = 0; i < baseSurface->baseSurface.property(polygonSimiTable).size(); ++i)
    {
        for (int j = 0; j < baseSurface->baseSurface.property(polygonSimiTable)[i].size(); ++j)
        {
            if (baseSurface->baseSurface.property(polygonSimiTable)[i][j] > maxPolygonError)
                maxPolygonError = baseSurface->baseSurface.property(polygonSimiTable)[i][j];

            totalError += baseSurface->baseSurface.property(polygonSimiTable)[i][j];

            count ++;
        }
    }

    savingFullPath = savingFullPath + "_PEA" + to_string(totalError / (double)count) + "_PEW" + to_string(maxPolygonError);
}

void SurfaceOptimizer::SaveEdgeDihedralAngleClusteringAnalysisFiles(string folderPath)
{
    // cout << folderPath << endl;

    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    /// Extract data
    vector<double> remeshedSurfaceEdgeList;
    baseSurface->GetRemeshedSurfaceEdgeLengthList(remeshedSurfaceEdgeList);

    vector<double> remeshedSurfaceDihedralAngleList;
    baseSurface->GetRemeshedSurfaceDihedralAngleList(remeshedSurfaceDihedralAngleList);

    vector<double> optimizedSurfaceEdgeList;
    baseSurface->GetBaseSurfaceEdgeLengthList(optimizedSurfaceEdgeList);

    vector<double> optimizedSurfaceDihedralAngleList;
    baseSurface->GetBaseSurfaceDihedralAngleList(optimizedSurfaceDihedralAngleList);

    vector<vector<double>> optimizedSurfaceEdgeClusterList;
    baseSurface->GetEdgeClusterList(optimizedSurfaceEdgeClusterList, edgeClusteringNum_end);

    vector< std::tuple<double, double, double> > edgeClusterErrorList;
    baseSurface->GetEdgeClusterErrorList(optimizedSurfaceEdgeClusterList, edgeClusterErrorList, edgeErrorThreshold);

    vector<double> edgeClusterMeanErrorList;
    vector<double> edgeClusterMaxErrorList;
    for (int i = 0; i < edgeClusterErrorList.size(); ++i)
    {
        edgeClusterMeanErrorList.push_back(std::get<0>(edgeClusterErrorList[i]));
        edgeClusterMaxErrorList.push_back(std::get<1>(edgeClusterErrorList[i]));
    }

    vector<vector<double>> optimizedSurfaceDihedralAngleClusterList;
    baseSurface->GetDihedralAngleClusterList(optimizedSurfaceDihedralAngleClusterList, dihedralClusterNum_end);

    vector< std::tuple<double, double, double> > dihedralAngleClusterErrorList;
    baseSurface->GetDihedralAngleClusterErrorList(optimizedSurfaceDihedralAngleClusterList, dihedralAngleClusterErrorList, dihedralAngleErrorThreshold);

    vector<double> dihedralAngleClusterMeanErrorList;
    vector<double> dihedralAngleClusterMaxErrorList;
    for (int i = 0; i < dihedralAngleClusterErrorList.size(); ++i)
    {
        dihedralAngleClusterMeanErrorList.push_back(std::get<0>(dihedralAngleClusterErrorList[i]));
        dihedralAngleClusterMaxErrorList.push_back(std::get<1>(dihedralAngleClusterErrorList[i]));
    }

    map<string, string> kwrds;
    double nRow, nCol;

    plt::figure_size(2500, 1500);

    /// (1) Plot the Edge Length Bar Chart of Remeshed Surface
    nCol = 1;
    nRow = 2;

    plt::subplot2grid(long(nRow), long(nCol), 0, 0);
    plt::ylabel("Edge Length");
    plt::bar(remeshedSurfaceEdgeList);
    plt::title("Edge Length Bar Chart of Remeshed Surface");

    plt::subplot2grid(long(nRow), long(nCol), 1, 0);
    plt::ylabel("Dihedral Angle");
    plt::bar(remeshedSurfaceDihedralAngleList);
    plt::title("Dihedral Angle Bar Chart of Remeshed Surface");

    plt::save(folderPath + "/OrigEdgeLengthDihedralAngle.png");

    /// (2) Plot the Edge Length Bar Chart of Optimized Surface
    nCol = 1;
    nRow = 2;

    plt::subplot2grid(long(nRow), long(nCol), 0, 0);
    plt::ylabel("Edge Length");
    plt::bar(optimizedSurfaceEdgeList);
    plt::title("Edge Length Bar Chart of Optimized Surface");

    plt::subplot2grid(long(nRow), long(nCol), 1, 0);
    plt::ylabel("Dihedral Angle");
    plt::bar(optimizedSurfaceDihedralAngleList);
    plt::title("Dihedral Angle Bar Chart of Optimized Surface");

    plt::save(folderPath + "/OptimizedEdgeLengthDihedralAngle.png");

    /// (3) Plot the Edge Cluster Bar Chart of Optimized Surface
    nCol = 4;
    nRow = round(optimizedSurfaceEdgeClusterList.size() / double(nCol));

    for (int i = 0; i < optimizedSurfaceEdgeClusterList.size(); ++i)
    {
        long currRowID = i / 4;
        long currColID = i % 4;

        string currTitle = "C" + to_string(i) + ": " + to_string(optimizedSurfaceEdgeClusterList[i].size());

        plt::subplot2grid(long(nRow), long(nCol), currRowID, currColID);
        plt::bar(optimizedSurfaceEdgeClusterList[i]);
        plt::title(currTitle);
    }

    plt::save(folderPath + "/OptimizedEdgeClusterBarChart.png");

    /// (4) Plot the Edge Cluster Analysis Chart of Optimized Surface
    nCol = 1;
    nRow = 3;

    plt::subplot2grid(long(nRow), long(nCol), 0, 0);
    plt::ylabel("Edge Length");
    plt::boxplot(optimizedSurfaceEdgeClusterList);
    plt::title("Edge Cluster Box Chart of Optimized Surface");

    plt::subplot2grid(long(nRow), long(nCol), 1, 0);
    plt::ylabel("Standard Error");
    plt::bar(edgeClusterMeanErrorList);
    plt::title("Edge Cluster Mean Error Bar Chart");

    plt::subplot2grid(long(nRow), long(nCol), 2, 0);
    plt::ylabel("Standard Error");
    plt::bar(edgeClusterMaxErrorList);
    plt::title("Edge Cluster Max Error Bar Chart");

    plt::save(folderPath + "/OptimizedEdgeClusterAnalysisChart.png");

    /// (5) Plot the dihedral Angle Cluster Bar Chart of Optimized Surface
    nCol = 4;
    nRow = round(optimizedSurfaceDihedralAngleClusterList.size() / double(nCol));

    for (int i = 0; i < optimizedSurfaceDihedralAngleClusterList.size(); ++i)
    {
        long currRowID = i / 4;
        long currColID = i % 4;

        string currTitle = "C" + to_string(i) + ": " + to_string(optimizedSurfaceDihedralAngleClusterList[i].size());

        plt::subplot2grid(long(nRow), long(nCol), currRowID, currColID);
        plt::bar(optimizedSurfaceDihedralAngleClusterList[i]);
        plt::title(currTitle);
    }
    plt::save(folderPath + "/OptimizedDihedralAngleClusterBarChart.png");

    /// (6) Plot the dihedral Angle Analysis Chart of Optimized Surface
    nCol = 1;
    nRow = 3;

    plt::subplot2grid(long(nRow), long(nCol), 0, 0);
    plt::ylabel("Dihedral Angle");
    plt::boxplot(optimizedSurfaceDihedralAngleClusterList);
    plt::title("Dihedral Angle Cluster Box Chart of Optimized Surface");

    plt::subplot2grid(long(nRow), long(nCol), 1, 0);
    plt::ylabel("Standard Error");
    plt::bar(dihedralAngleClusterMeanErrorList);
    plt::title("Dihedral Angle Cluster Mean Error Bar Chart");

    plt::subplot2grid(long(nRow), long(nCol), 2, 0);
    plt::ylabel("Standard Error");
    plt::bar(dihedralAngleClusterMaxErrorList);
    plt::title("Dihedral Angle Cluster Max Error Bar Chart");

    plt::save(folderPath + "/OptimizedDihedralAngleClusterAnalysisChart.png");

    plt::close();

}

void SurfaceOptimizer::SavePolygonClusteringAnalysisFiles(string folderPath)
{
    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    /// Extract data
    vector<double> diagonalLengthList;
    baseSurface->GetDiagonalLengthList(diagonalLengthList);

    // Get polygonSimiTable
    OpenMesh::MPropHandleT< vector<vector<double>> > polygonSimiTable;
    baseSurface->baseSurface.get_property_handle(polygonSimiTable, "polygonSimiTable");

    vector<double> planarityList;
    baseSurface->GetPlanarityList(planarityList);

    /// (1) Plot the Edge Length Bar Chart of Optimized Surface
    plt::figure_size(2500, 1500);
    plt::ylabel("Diagonal Length");
    plt::bar(diagonalLengthList);
    plt::title("Diagonal Length Bar Chart of Optimized Surface");

    plt::save(folderPath + "/OptimizedDiagonalLengthBarChart.png");
    plt::close();

    /// (2) Plot the Polygon Similarity Table of Optimized Surface
    plt::figure_size(2500, 1500);
    plt::ylabel("Error");
    plt::boxplot(baseSurface->baseSurface.property(polygonSimiTable));
    plt::title("Polygon Similarity Box Plot of Each Polygon Cluster of Optimized Surface");

    plt::save(folderPath + "/OptimizedPolygonSimiBoxPlot.png");
    plt::close();

    /// (3) Plot the Planarity List
    plt::figure_size(2500, 1500);
    plt::ylabel("Planarity");
    plt::bar(planarityList);
    plt::title("Planarity List of Each Polygon of Optimized Surface");

    plt::save(folderPath + "/PlanarityListBarChartOptimizedSurface");
    plt::close();
}

void SurfaceOptimizer::SaveParameter2File(string folderPath)
{
    std::ofstream out(folderPath + "/Parameter.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    cout << "Surface: " << baseSurface->inputSurface->surfaceName << endl;
    cout << "Tile Pattern: " << baseSurface->inputTilePattern->patternName << endl;
    cout << "Pattern Scalar: " << baseSurface->inputTilePattern->scalar << endl;
    cout << "Rot Angle: " << baseSurface->inputTilePattern->rotationAngle << endl;

    cout << endl;

    cout << "Number of Vertex: " << baseSurface->baseSurface.n_vertices() << endl;
    cout << "Number of Edges: " << baseSurface->baseSurface.n_edges() << endl;
    cout << "Number of Faces: " << baseSurface->baseSurface.n_faces() << endl;
    cout << "Polygon Cluster: " << baseSurface->GetPolygonClusterNum() << endl;

    cout << endl;

    cout << "Edge k_start: " << edgeClusteringNum_start << endl;
    cout << "Edge k_end: " << edgeClusteringNum_end << endl;
    cout << "w1_planarity: " << w1_planarity << endl;
    cout << "w1_edge: " << w1_edge << endl;
    cout << "iter_1: " << iter_1 << endl;

    cout << endl;

    cout << "Dihedral Angle k_start: " << dihedralClusterNum_start << endl;
    cout << "Dihedral Angle k_end: " << dihedralClusterNum_end << endl;
    cout << "w2_planarity: " << w2_planarity << endl;
    cout << "w2_edge: " << w2_edge << endl;
    cout << "w2_dihedralAngle: " << w2_planarity << endl;
    cout << "iter_2: " << iter_2 << endl;

    cout << endl;

    cout << "w3_planarity: " << w3_planarity << endl;
    cout << "w3_edge: " << w3_edge << endl;
    cout << "w3_dihedralAngle: " << w3_planarity << endl;
    cout << "w3_diagonal: " << w3_diagonal << endl;
    cout << "iter3: " << iter_3 << endl;

    cout << endl;

    cout << "miniPolygonClusterNumThreshold: " << miniPolygonClusterNumThreshold << endl;
    cout << "mergingCandidateNum: " << mergingCandidateNum << endl;

    std::cout.rdbuf(coutbuf); //reset to standard output again
}

void SurfaceOptimizer::SaveFinalResult2File(string folderPath)
{
    std::ofstream out(folderPath + "/SurfaceResultAnalysis.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    cout << "Surface: " << baseSurface->inputSurface->surfaceName << endl;
    cout << "Tile Pattern: " << baseSurface->inputTilePattern->patternName << endl;
    cout << "Pattern Scalar: " << baseSurface->inputTilePattern->scalar << endl;
    cout << "Rot Angle: " << baseSurface->inputTilePattern->rotationAngle << endl;
}

void SurfaceOptimizer::SaveSurfaceOptimizationResult(string folderPath)
{
    std::ofstream out(folderPath + "/SurfaceOptimizationResult.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    cout << "Average Edge Length Error: " << baseSurface->avgEdgeLengthError << endl;
    cout << "Worst Edge Length Error: " << baseSurface->worstEdgeLengthError << endl;
    cout << endl;

    cout << "Average Dihedral Angle Error: " << baseSurface->avgDihedralAngleError << endl;
    cout << "Worst Dihedral Angle Error: " << baseSurface->worstDihedralAngleError << endl;
    cout << endl;

    cout << "Average Planarity Error: " << baseSurface->avgPlanarity << endl;
    cout << "Worst Planarity Error: " << baseSurface->worstPlanarity << endl;
    cout << endl;

    cout << "Average Polygon Error: " << baseSurface->avgPolygonSimilarity << endl;
    cout << "Worst Polygon Error: " << baseSurface->worstPolygonSimilarity << endl;
    cout << endl;

    cout << "Worst Penetration Approximation: " << baseSurface->worstFabApproximation << endl;
    cout << endl;

    cout << "Surface Optimization Time: " << surfaceOpTime << " mins" << endl;

    std::cout.rdbuf(coutbuf); //reset to standard output again
}

void SurfaceOptimizer::SaveSurfaceOBJs(string folderPath)
{
    OpenMesh::IO::write_mesh(baseSurface->remeshedPolySurface, folderPath + "/OriginalSurface.obj");
    OpenMesh::IO::write_mesh(baseSurface->baseSurface, folderPath + "/OptimizedSurface.obj");
    OpenMesh::IO::write_mesh(baseSurface->replacedSurface, folderPath + "/ReplacedSurface.obj");
}

void SurfaceOptimizer::SaveAllClusteringPolygonOBJs(string folderPath, string templatePath)
{
    // Get polygonClusterFullList to mesh
    OpenMesh::MPropHandleT< vector<vector<vector<int>>> > polygonClusterFullList;
    baseSurface->baseSurface.get_property_handle(polygonClusterFullList, "polygonClusterFullList");

    // Get polygonSimiTable
    OpenMesh::MPropHandleT< vector<vector<double>> > polygonSimiTable;
    baseSurface->baseSurface.get_property_handle(polygonSimiTable, "polygonSimiTable");

    /// Setup a folder to save template files
    {
        string command;
        command = "mkdir -p " + templatePath;
        system(command.c_str());
    }

    for (int i = 0; i < baseSurface->baseSurface.property(polygonClusterFullList).size(); ++i)
    {
        string currStr = folderPath + "/C" + to_string(i + 1) + "_" + to_string(baseSurface->baseSurface.property(polygonClusterFullList)[i].size()) + "_" + to_string(accumulate(baseSurface->baseSurface.property(polygonSimiTable)[i].begin(), baseSurface->baseSurface.property(polygonSimiTable)[i].end(), 0.0) / baseSurface->baseSurface.property(polygonSimiTable)[i].size());

        /// Setup a folder to save obj files
        string command;
        command = "mkdir -p " + currStr;
        system(command.c_str());

        SaveClusterPolygonOBJs(currStr, templatePath, i);

    }
}

void SurfaceOptimizer::SaveClusterPolygonOBJs(string folderPath, string templatePath, int clusterID)
{
    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    baseSurface->baseSurface.get_property_handle(clusteringLabel, "clusteringLabel");

    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface->baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");

    // Get edgeClusterCentroidList of mesh
    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface->baseSurface.get_property_handle(edgeClusterCentroidList, "edgeClusterCentroidList");

    // Get diagonalList to each face
    OpenMesh::FPropHandleT< vector<std::tuple<Vector3d, Vector3d, double>> > diagonalList;
    baseSurface->baseSurface.get_property_handle(diagonalList, "diagonalList");

    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface->baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // ShapeOp Matrix
    ShapeOp::Matrix3X referenceP, currP;

    // Find the first polygon for referenceP
    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        if (clusterID != baseSurface->baseSurface.property(clusteringLabel, *f_it))
            continue;

        PolyMesh::FaceVertexCWIter fv_it;

        vector<Vector3d> currList;
        vector<int> edgeLabelList;
        vector<PolyMesh::VertexHandle> verHandleList;

        for (fv_it = baseSurface->baseSurface.fv_cwiter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            auto curr = baseSurface->baseSurface.point(*fv_it);
            currList.push_back(Vector3d(curr[0], curr[1], curr[2]));
            verHandleList.push_back(*fv_it);
        }

        // Get related edgeLabel
        for (int i = 1; i < verHandleList.size(); ++i)
        {
            auto currEdgeHandle = FindEdge(baseSurface->baseSurface, verHandleList[i], verHandleList[i - 1]);
            edgeLabelList.push_back(baseSurface->baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }
        {
            auto currEdgeHandle = FindEdge(baseSurface->baseSurface, verHandleList[0], verHandleList[verHandleList.size() - 1]);
            edgeLabelList.push_back(baseSurface->baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }

        // Get related polygonClusterList
        vector<int> currRelatedList = baseSurface->baseSurface.property(polygonClusterList)[baseSurface->baseSurface.property(clusteringLabel, *f_it)];

        // Rotate the currList to match the related list
        int iteration = 0;
        vector<int> bestMatchingList = edgeLabelList;
        vector<Vector3d> bestMatchingVertexList = currList;
        int currMatchingNum = ComputeMatchingNum(bestMatchingList, currRelatedList);

        while (edgeLabelList != currRelatedList)
        {
            if (currMatchingNum < ComputeMatchingNum(edgeLabelList, currRelatedList))
            {
                bestMatchingList = edgeLabelList;
                bestMatchingVertexList = currList;
                currMatchingNum = ComputeMatchingNum(edgeLabelList, currRelatedList);
            }

            // Rotate edgeLabelList
            edgeLabelList.push_back(edgeLabelList[0]);
            edgeLabelList.erase(edgeLabelList.begin());

            // Rotate currList
            currList.push_back(currList[0]);
            currList.erase(currList.begin());

            if (iteration > currList.size())
            {
                cout << "Edge Clustering Error Warning." << endl;

                edgeLabelList = bestMatchingList;
                currList = bestMatchingVertexList;
                break;
            }

            iteration++;
        }

        referenceP.resize(3, currList.size());
        currP.resize(3, currList.size());

        // Move the model to the center (0, 0, 0)
        Vector3d minPoint = referenceP.rowwise().minCoeff();
        Vector3d maxPoint = referenceP.rowwise().maxCoeff();
        Vector3d currCenter = (maxPoint + minPoint) / 2;
        Vector3d offset = Vector3d(0,0,0) - currCenter;

        for (int i = 0; i < referenceP.cols(); ++i)
        {
            referenceP(0, i) += offset(0);
            referenceP(1, i) += offset(1);
            referenceP(2, i) += offset(2);
        }

        for (int i = 0; i < currList.size(); ++i)
        {
            referenceP(0, i) = currList[i][0];
            referenceP(1, i) = currList[i][1];
            referenceP(2, i) = currList[i][2];
        }

        break;
    }

    // Start saving OBJs
    vector<vector<Vector3d>> clusteredAlignedPolygons;
    int count = 0;
    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        if (clusterID != baseSurface->baseSurface.property(clusteringLabel, *f_it))
            continue;

        PolyMesh::FaceVertexCWIter fv_it;

        vector<Vector3d> currList;
        vector<int> edgeLabelList;
        vector<PolyMesh::VertexHandle> verHandleList;

        for (fv_it = baseSurface->baseSurface.fv_cwiter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            auto curr = baseSurface->baseSurface.point(*fv_it);
            currList.push_back(Vector3d(curr[0], curr[1], curr[2]));
            verHandleList.push_back(*fv_it);
        }

        // Get related edgeLabel
        for (int i = 1; i < verHandleList.size(); ++i)
        {
            auto currEdgeHandle = FindEdge(baseSurface->baseSurface, verHandleList[i], verHandleList[i - 1]);
            edgeLabelList.push_back(baseSurface->baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }
        {
            auto currEdgeHandle = FindEdge(baseSurface->baseSurface, verHandleList[0], verHandleList[verHandleList.size() - 1]);
            edgeLabelList.push_back(baseSurface->baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }

        // Get related polygonClusterList
        vector<int> currRelatedList = baseSurface->baseSurface.property(polygonClusterList)[baseSurface->baseSurface.property(clusteringLabel, *f_it)];

        // Rotate the currList to match the related list
        int iteration = 0;
        vector<int> bestMatchingList = edgeLabelList;
        vector<Vector3d> bestMatchingVertexList = currList;
        int currMatchingNum = ComputeMatchingNum(bestMatchingList, currRelatedList);

        while (edgeLabelList != currRelatedList)
        {
            if (currMatchingNum < ComputeMatchingNum(edgeLabelList, currRelatedList))
            {
                bestMatchingList = edgeLabelList;
                bestMatchingVertexList = currList;
                currMatchingNum = ComputeMatchingNum(edgeLabelList, currRelatedList);
            }

            // Rotate edgeLabelList
            edgeLabelList.push_back(edgeLabelList[0]);
            edgeLabelList.erase(edgeLabelList.begin());

            // Rotate currList
            currList.push_back(currList[0]);
            currList.erase(currList.begin());

            if (iteration > currList.size())
            {
                cout << "Edge Clustering Error Warning." << endl;

                edgeLabelList = bestMatchingList;
                currList = bestMatchingVertexList;
                break;
            }

            iteration++;
        }

        // Find the diagonal index
        vector< vector<int> > currDiagonalList;
        for (int i = 0; i < baseSurface->baseSurface.property(diagonalList, *f_it).size(); ++i)
        {
            auto p1 = std::get<0>(baseSurface->baseSurface.property(diagonalList, *f_it)[i]);
            auto p2 = std::get<1>(baseSurface->baseSurface.property(diagonalList, *f_it)[i]);

            int p1_index = find(currList.begin(), currList.end(), p1) - currList.begin() + 1;
            int p2_index = find(currList.begin(), currList.end(), p2) - currList.begin() + 1;

            vector<int> currIndexPair;
            currIndexPair.push_back(p1_index);
            currIndexPair.push_back(p2_index);
            currDiagonalList.push_back(currIndexPair);
        }

        // Prepare the shapeOp matrix
        for (int i = 0; i < currP.cols(); ++i)
        {
            currP(0, i) = currList[i][0];
            currP(1, i) = currList[i][1];
            currP(2, i) = currList[i][2];
        }

        // Save original polygon
        std::ofstream out(folderPath + "/OrigPos_Cluster_" + to_string(clusterID + 1) + "_" + to_string(count + 1) + ".obj");
        std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

        for (int i = 0; i < currP.cols(); ++i)
        {
            cout << "v " << currP(0, i) << " " << currP(1, i) << " " << currP(2, i) << endl;
        }

        cout << endl;

        cout << "f ";
        for (int i = 0; i < currP.cols(); ++i)
        {
            cout << to_string(i+1) << " ";
        }

        cout << endl;

        for (int i = 0; i < currDiagonalList.size(); ++i)
        {
            cout << "l " << to_string(currDiagonalList[i][0]) << " " << to_string(currDiagonalList[i][1]);
        }

        cout << endl;

        std::cout.rdbuf(coutbuf); //reset to standard output again

        // Move the model to the center (0, 0, 0)
        Vector3d minPoint = currP.rowwise().minCoeff();
        Vector3d maxPoint = currP.rowwise().maxCoeff();
        Vector3d currCenter = (maxPoint + minPoint) / 2;
        Vector3d offset = Vector3d(0,0,0) - currCenter;

        for (int i = 0; i < currP.cols(); ++i)
        {
            currP(0, i) += offset(0);
            currP(1, i) += offset(1);
            currP(2, i) += offset(2);
        }

        // Use shapeOp to find the best matching configuration
        ShapeOp::Solver s;
        s.setPoints(currP);
        ShapeOp::Scalar weight = 1.0;

        // Add a closeness constraint to the vertex.
        {
            std::vector<int> id_vector;

            for(int j = 0; j < currP.cols(); ++j)
            {
                id_vector.push_back(j);

                auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, currP);

                c->setPosition(referenceP.col(j));

                s.addConstraint(c);

                id_vector.clear();
            }
        }

        // Add a rigid constraint between 1st and 4th vertex.
        {
            std::vector<int> id_vector;

            for(int j = 0; j < currP.cols(); ++j)
            {
                id_vector.push_back(j);
            }

            auto c = std::make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight, currP, false, true, false);

            s.addConstraint(c);
        }

        s.initialize();
        s.solve(100);
        auto p = s.getPoints();

        // Push back the aligned vectex list
        vector<Vector3d> currAlignedList;
        for (int i = 0; i < p.cols(); ++i)
        {
            currAlignedList.push_back(Vector3d(p(0, i), p(1, i), p(2, i)));
        }
        clusteredAlignedPolygons.push_back(currAlignedList);

        // Save aligned polygon
        std::ofstream out_2(folderPath + "/Aligned_Cluster_" + to_string(clusterID + 1) + "_" + to_string(count + 1) + ".obj");
        std::streambuf *coutbuf_2 = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out_2.rdbuf()); //redirect std::cout to out.txt!

        for (int i = 0; i < p.cols(); ++i)
        {
            cout << "v " << p(0, i) << " " << p(1, i) << " " << p(2, i) << endl;
        }

        cout << endl;

        cout << "f ";
        for (int i = 0; i < p.cols(); ++i)
        {
            cout << to_string(i+1) << " ";
        }

        cout << endl;

        for (int i = 0; i < currDiagonalList.size(); ++i)
        {
            cout << "l " << to_string(currDiagonalList[i][0]) << " " << to_string(currDiagonalList[i][1]);
        }

        cout << endl;

        std::cout.rdbuf(coutbuf_2); //reset to standard output again

        count++;
    }

    // Compute and save cluster centroid
    vector<Vector3d> clusterCentroidVerList;
    for (int i = 0; i < clusteredAlignedPolygons[0].size(); ++i)
    {
        Vector3d currMeanVer(0,0,0);

        for (int j = 0; j < clusteredAlignedPolygons.size(); ++j)
        {
            currMeanVer = currMeanVer + clusteredAlignedPolygons[j][i];
        }

        currMeanVer = currMeanVer / double( clusteredAlignedPolygons.size());
        clusterCentroidVerList.push_back(currMeanVer);
    }

    // Save cluster centroid to cluster folder
    {
        std::ofstream out_3(folderPath + "/_Cluster_" + to_string(clusterID + 1) + "_Centroid" + ".obj");
        std::streambuf *coutbuf_3 = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out_3.rdbuf()); //redirect std::cout to out.txt!

        for (int i = 0; i < clusterCentroidVerList.size(); ++i)
        {
            cout << "v " << clusterCentroidVerList[i][0] << " " << clusterCentroidVerList[i][1] << " " << clusterCentroidVerList[i][2] << endl;
        }

        cout << endl;

        cout << "f ";
        for (int i = 0; i < clusterCentroidVerList.size(); ++i)
        {
            cout << to_string(i+1) << " ";
        }

        cout << endl;

        std::cout.rdbuf(coutbuf_3); //reset to standard output again
    }

    // Save cluster centroid to template folder
    {
        std::ofstream out_3(templatePath + "/Cluster_" + to_string(clusterID + 1) + "_Centroid" + ".obj");
        std::streambuf *coutbuf_3 = std::cout.rdbuf(); //save old buf
        std::cout.rdbuf(out_3.rdbuf()); //redirect std::cout to out.txt!

        for (int i = 0; i < clusterCentroidVerList.size(); ++i)
        {
            cout << "v " << clusterCentroidVerList[i][0] << " " << clusterCentroidVerList[i][1] << " " << clusterCentroidVerList[i][2] << endl;
        }

        cout << endl;

        cout << "f ";
        for (int i = 0; i < clusterCentroidVerList.size(); ++i)
        {
            cout << to_string(i+1) << " ";
        }

        cout << endl;

        std::cout.rdbuf(coutbuf_3); //reset to standard output again
    }
}

void SurfaceOptimizer::SaveBlockClusteringAnalysisFiles(string folderPath)
{
    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    // Get blockSimilarityTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > blockSimilarityTable;
    baseSurface->baseSurface.get_property_handle(blockSimilarityTable, "blockSimilarityTable");

    /// (1) Plot the Block Similarity Table of Optimized Surface
    plt::figure_size(2500, 1500);
    plt::ylabel("Error");
    plt::boxplot(baseSurface->baseSurface.property(blockSimilarityTable));
    plt::title("Block Similarity Box Plot of Each Polygon Cluster of Optimized Surface");

    plt::save(folderPath + "/OptimizedBlockSimiBoxPlot.png");
    plt::close();
}

void SurfaceOptimizer::SaveShellStructureOBJs(string folderPath)
{
    OpenMesh::IO::write_mesh(shellStructure->shellStructureTri, folderPath + "/OptimizedShellStructure.obj");
    OpenMesh::IO::write_mesh(shellStructure->replacedShellStructure_LSM, folderPath + "/ReplacedShellStructure_LSM.obj");
    OpenMesh::IO::write_mesh(shellStructure->replacedShellStructure_LSM_planar, folderPath + "/ReplacedShellStructure_LSM_planar.obj");
}