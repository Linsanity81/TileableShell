//
// Created by Linsanity on 18/4/22.
//

#include "BaseSurface.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>
#include "TileableStructure/CuttingPlane.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

BaseSurface::BaseSurface(Surface * _inputSurface, TilePattern * _inputTilePattern)
{
    inputSurface = _inputSurface;
    inputTilePattern = _inputTilePattern;

    tolerance = 0.00;
    fabToleranceScalar = 2;
    absoluteTolerance    = 0.0001;

    fabricationThreshold = 0.0002;
    planarityThreshold = 0.01;
    dihedralAngleClusterErrorThreshold = 0.01;

    worstFabApproximation = 0;

    avgPolygonSimilarity = 0;
    worstPolygonSimilarity = 0;

    avgPlanarity = 0;
    worstPlanarity = 0;

    avgDihedralAngleError = 0;
    worstDihedralAngleError = 0;

    avgEdgeLengthError = 0;
    worstEdgeLengthError = 0;

    closeness = 0;
    smoothness = 0;
}

BaseSurface::~BaseSurface()
{
    ClearSurface();
}

void BaseSurface::ClearSurface()
{
    inputSurface = nullptr;
    inputTilePattern = nullptr;

    baseSurface.clear();
    triBaseSurface.clear();

    tolerance = 0.007;
    fabToleranceScalar = 2;
    absoluteTolerance    = 0.0001;

    fabricationThreshold = 0.0002;
    planarityThreshold = 0.01;
    dihedralAngleClusterErrorThreshold = 0.01;

    worstFabApproximation = 0;

    avgPolygonSimilarity = 0;
    worstPolygonSimilarity = 0;

    avgPlanarity = 0;
    worstPlanarity = 0;

    avgDihedralAngleError = 0;
    worstDihedralAngleError = 0;

    avgEdgeLengthError = 0;
    worstEdgeLengthError = 0;

    closeness = 0;
    smoothness = 0;
}




//**************************************************************************************//
//                     Intialization for replacedSurface with gap
//**************************************************************************************//

void BaseSurface::InitReplacedSurfaceAndToleranceParameters()
{
    InitReplacedSurface();
    InitToleranceParameters();
}

void BaseSurface::InitReplacedSurface()
{
    PolyMesh polyMesh;

    /// Add properties for replacedSurface
    // Add centroidPolygonList to mesh -- replacedSurface
    OpenMesh::MPropHandleT< vector<vector<Vector3d>> > centroidPolygonList;
    polyMesh.add_property(centroidPolygonList, "centroidPolygonList");

    // Add alignedVerList to each face -- replacedSurface
    OpenMesh::FPropHandleT< vector<Vector3d> > alignedVerList;
    polyMesh.add_property(alignedVerList, "alignedVerList");

    // Add clusterLabel to each face -- replacedSurface
    OpenMesh::FPropHandleT< int > clusteringLabel;
    polyMesh.add_property(clusteringLabel, "clusteringLabel");

    /// Get properties of baseSurface
    // Get centroidPolygonList to mesh -- baseSurface
    OpenMesh::MPropHandleT< vector<vector<Vector3d>> > origCentroidPolygonList;
    baseSurface.get_property_handle(origCentroidPolygonList, "centroidPolygonList");

    // Get alignedVerList to each face -- baseSurface
    OpenMesh::FPropHandleT< vector<Vector3d> > origAlignedVerList;
    baseSurface.get_property_handle(origAlignedVerList, "alignedVerList");

    // Get clusterLabel to each face -- baseSurface
    OpenMesh::FPropHandleT< int > origClusteringLabel;
    baseSurface.get_property_handle(origClusteringLabel, "clusteringLabel");

    // Get replacedFaceHandle to each face -- baseSurface
    OpenMesh::FPropHandleT< PolyMesh::FaceHandle > replacedFaceHandle;
    baseSurface.get_property_handle(replacedFaceHandle, "replacedFaceHandle");

    // Mesh property transfer
    polyMesh.property(centroidPolygonList) = baseSurface.property(origCentroidPolygonList);

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        // Add face to replacedSurface
        PolyMesh::FaceVertexIter fv_it;
        std::vector<PolyMesh::VertexHandle>  face_vhandles;
        for (fv_it = baseSurface.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            face_vhandles.push_back(polyMesh.add_vertex(PolyMesh::Point(baseSurface.point(*fv_it)[0], baseSurface.point(*fv_it)[1],  baseSurface.point(*fv_it)[2])));
        }

        auto currFace = polyMesh.add_face(face_vhandles);

        polyMesh.property(alignedVerList, currFace) = baseSurface.property(origAlignedVerList, *f_it);
        polyMesh.property(clusteringLabel, currFace) = baseSurface.property(origClusteringLabel, *f_it);

        baseSurface.property(replacedFaceHandle, *f_it) = currFace;
    }

    replacedSurface = polyMesh;

    for (int i = 0; i < replacedSurface.property(centroidPolygonList).size(); ++i)
    {
        for (PolyMesh::FaceIter f_it=replacedSurface.faces_begin(); f_it!=replacedSurface.faces_end(); ++f_it)
        {
            if (i != replacedSurface.property(clusteringLabel, *f_it))
                continue;

            MatrixXd alignedMat;
            AlignPolygon_LSM_FixedConfig( replacedSurface.property(alignedVerList , *f_it), replacedSurface.property(centroidPolygonList)[i],alignedMat);

            PolyMesh::FaceVertexIter fv_it;
            int k = replacedSurface.property(alignedVerList , *f_it).size() - 1;

            for (fv_it = replacedSurface.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
            {
                replacedSurface.set_point(*fv_it, PolyMesh::Point(alignedMat(0, k), alignedMat(1, k), alignedMat(2, k)));
                --k;
            }
        }
    }

    replacedTriSurface = replacedSurface;
    replacedTriSurface.triangulate();
}

void BaseSurface::InitToleranceParameters()
{
    double totalLength = 0;

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

        totalLength += (to - from).norm();
    }

    double averageLength = totalLength / double(baseSurface.n_edges());

    absoluteTolerance = averageLength * tolerance;

    fabricationThreshold = fabToleranceScalar * absoluteTolerance;
}




//**************************************************************************************//
//                              Read and Write Operation
//**************************************************************************************//

void BaseSurface::ReadBaseSurface(char *fileName)
{

}

void BaseSurface::WriteBaseSurface(char *fileName)
{
    string fileNameString = string(fileName);
    fileNameString.append(".obj");

    OpenMesh::IO::write_mesh(baseSurface, fileNameString);
}

void BaseSurface::WriteRemeshedBaseSurface(string filePath)
{
    filePath += ".obj";

    OpenMesh::IO::write_mesh(remeshedPolySurface, filePath);

}




//**************************************************************************************//
//                                Update Operation
//**************************************************************************************//

void BaseSurface::UpdateEdgeLength()
{
    // Get edgeLength for each edge
    OpenMesh::EPropHandleT< double > edgeLength;
    baseSurface.get_property_handle(edgeLength, "edgeLength");

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

        baseSurface.property(edgeLength, *e_it) = (to - from).length();
    }
}

void BaseSurface::UpdateEdgeClusterCentroidList()
{
    UpdateEdgeLength();

    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface.get_property_handle(edgeClusterCentroidList, "edgeClusterCentroidList");

    // Get edgeLength for each edge
    OpenMesh::EPropHandleT< double > edgeLength;
    baseSurface.get_property_handle(edgeLength, "edgeLength");

    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get edgeClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > edgeClusterCentroid;
    baseSurface.get_property_handle(edgeClusterCentroid, "edgeClusterCentroid");

    // Update edgeClusterCentroidList of mesh
    for (int i = 0; i < baseSurface.property(edgeClusterCentroidList).size(); ++i)
    {
        double currTotalLength = 0;
        int currEdgeNum = 0;

        for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface.property(edgeClusteringLabel, *e_it) == i)
            {
                currTotalLength += baseSurface.property(edgeLength, *e_it);
                currEdgeNum++;
            }
        }

//        cout << currTotalLength / double(currEdgeNum) << " ";
        baseSurface.property(edgeClusterCentroidList)[i] = currTotalLength / double(currEdgeNum);
    }

    // Update edgeClusterCentroid of each edge
//    cout << "edge length list: " << endl;
//    double totalError = 0;
    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
//        cout << baseSurface.property(edgeLength, *e_it) << ", " ;
        baseSurface.property(edgeClusterCentroid, *e_it) = baseSurface.property(edgeClusterCentroidList)[baseSurface.property(edgeClusteringLabel, *e_it)];
//        totalError += fabs(baseSurface.property(edgeClusterCentroid, *e_it) - baseSurface.property(edgeLength, *e_it));
    }
//    cout << endl << "edge error: " << totalError << endl;
}

void BaseSurface::UpdateDihedralAngle()
{
    // Get dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface.property(dihedralAngle, *e_it) == -1)
            continue;

        double angle;

        angle = double(baseSurface.calc_dihedral_angle(*e_it));

        angle = 180 - angle / 3.1415926 * 180;

        baseSurface.property(dihedralAngle, *e_it) = angle;
    }
}

void BaseSurface::UpdateDihedralAngleClusterCentroidList()
{
    UpdateDihedralAngle();

    // Get dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    // Get dihedralAngleLabel for each edge
    OpenMesh::EPropHandleT< int > dihedralAngleLabel;
    baseSurface.get_property_handle(dihedralAngleLabel, "dihedralAngleLabel");

    // Get angleClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > angleClusterCentroid;
    baseSurface.get_property_handle(angleClusterCentroid, "angleClusterCentroid");

    // Get angleClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<double> > angleClusterCentroidList;
    baseSurface.get_property_handle(angleClusterCentroidList, "angleClusterCentroidList");

//    cout << "dihedral angle centroid list: " << endl;
    for (int i = 0; i < baseSurface.property(angleClusterCentroidList).size(); ++i)
    {
        double currSum = 0;
        int currNum = 0;

        for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface.property(dihedralAngleLabel, *e_it) == i)
            {
                currSum += baseSurface.property(dihedralAngle, *e_it);
                currNum++;
            }
        }
        baseSurface.property(angleClusterCentroidList)[i] = currSum / double(currNum);
//        cout << baseSurface.property(angleClusterCentroidList)[i] << ", ";
    }
//    cout << endl;

//    cout << "dihedral angle list: " << endl;
    double totalError = 0;
    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface.property(dihedralAngle, *e_it) <= 0)
            continue;
//        cout << baseSurface.property(dihedralAngle, *e_it) << ", ";
        baseSurface.property(angleClusterCentroid, *e_it) = baseSurface.property(angleClusterCentroidList)[baseSurface.property(dihedralAngleLabel, *e_it)];
        totalError += fabs(baseSurface.property(angleClusterCentroid, *e_it) - baseSurface.property(dihedralAngle, *e_it));
    }
//    cout << endl << "dihedral angle error: " << totalError << endl;
}

void BaseSurface::UpdateDiagnonalList()
{
    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    baseSurface.get_property_handle(clusteringLabel, "clusteringLabel");

    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");

    // Get diagonalList of each face
    OpenMesh::FPropHandleT< vector<std::tuple<Vector3d, Vector3d, double>> > diagonalList;
    baseSurface.get_property_handle(diagonalList, "diagonalList");

    // Get edgeClusterCentroidList of mesh
    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface.get_property_handle(edgeClusterCentroidList, "edgeClusterCentroidList");

    // Get diagonalClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > diagonalClusterCentroidList;
    baseSurface.get_property_handle(diagonalClusterCentroidList, "diagonalClusterCentroidList");

    // Save reference verList for each cluster
    vector< vector<Vector3d> > referPolygonList;
    vector< vector<int> > referPolygonEdgeLabelList;
    for (int i = 0; i < baseSurface.property(polygonClusterList).size(); ++i)
    {
        vector<Vector3d> currList;
        vector<int> currLabelList;
        referPolygonList.push_back(currList);
        referPolygonEdgeLabelList.push_back(currLabelList);
    }

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        int currLabel = baseSurface.property(clusteringLabel, *f_it);

        if (referPolygonList[currLabel].size() != 0)
            continue;
        else
        {
            PolyMesh::FaceVertexCWIter fv_it;
            vector<PolyMesh::VertexHandle> verHandleList;

            // Get the curr Vector3d point list clockwise
            for (fv_it = baseSurface.fv_cwiter(*f_it); fv_it.is_valid(); ++fv_it)
            {
                auto curr = baseSurface.point(*fv_it);
                referPolygonList[currLabel].push_back(Vector3d(curr[0], curr[1], curr[2]));
                verHandleList.push_back(*fv_it);
            }

            // Get related edgeLabel
            for (int i = 1; i < verHandleList.size(); ++i)
            {
                auto currEdgeHandle = FindEdge(baseSurface, verHandleList[i], verHandleList[i - 1]);
                referPolygonEdgeLabelList[currLabel].push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
            }
            {
                auto currEdgeHandle = FindEdge(baseSurface, verHandleList[0], verHandleList[verHandleList.size() - 1]);
                referPolygonEdgeLabelList[currLabel].push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
            }
        }
    }

//    dbg(referPolygonList);
//    dbg(referPolygonEdgeLabelList);

    // Update diagonalList of each face
    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexCWIter fv_it;

        vector<Vector3d> currList;
        vector<int> edgeLabelList;
        vector<PolyMesh::VertexHandle> verHandleList;

        // Get the curr Vector3d point list clockwise
        for (fv_it = baseSurface.fv_cwiter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            auto curr = baseSurface.point(*fv_it);
            currList.push_back(Vector3d(curr[0], curr[1], curr[2]));
            verHandleList.push_back(*fv_it);
        }

        // No need to compute diagonal for n-edge polygon (n < 4)
        if (currList.size() < 4)
            continue;

        // Get related edgeLabel
        for (int i = 1; i < verHandleList.size(); ++i)
        {
            auto currEdgeHandle = FindEdge(baseSurface, verHandleList[i], verHandleList[i - 1]);
            edgeLabelList.push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }
        {
            auto currEdgeHandle = FindEdge(baseSurface, verHandleList[0], verHandleList[verHandleList.size() - 1]);
            edgeLabelList.push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }

        // Get related polygonClusterList
        const vector<int> currRelatedList = referPolygonEdgeLabelList[baseSurface.property(clusteringLabel, *f_it)];
        const vector<Vector3d> currReferVerList = referPolygonList[baseSurface.property(clusteringLabel, *f_it)];

        // Rotate the currList to match the related list
        int iteration = 0;
        vector<int> bestMatchingList = edgeLabelList;
        vector<Vector3d> bestMatchingVertexList = currList;

        double simi;
//        ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, simi);
        ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, simi);

        // Before flipping
        while (iteration <= currList.size())
        {
            double currSimi;
//            ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, currSimi);
            ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, currSimi);

            if (currSimi < simi)
            {
                simi = currSimi;
                bestMatchingVertexList = currList;
            }

            // Rotate edgeLabelList
            edgeLabelList.push_back(edgeLabelList[0]);
            edgeLabelList.erase(edgeLabelList.begin());

            // Rotate currList
            currList.push_back(currList[0]);
            currList.erase(currList.begin());

            iteration++;
        }

        // After flipping
        reverse(currList.begin(), currList.end());
        iteration = 0;
        while (iteration <= currList.size())
        {
            double currSimi;
//            ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, currSimi);
            ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, currSimi);

            if (currSimi < simi)
            {
                simi = currSimi;
                bestMatchingVertexList = currList;
            }

            // Rotate edgeLabelList
            edgeLabelList.push_back(edgeLabelList[0]);
            edgeLabelList.erase(edgeLabelList.begin());

            // Rotate currList
            currList.push_back(currList[0]);
            currList.erase(currList.begin());

            iteration++;
        }

        currList = bestMatchingVertexList;

        // Update the diagonalList of each face
        baseSurface.property(diagonalList, *f_it).clear();

        for (int i = 2; i < currList.size() - 1; ++i)
        {
            std::tuple<Vector3d, Vector3d, double> currDiagonal(currList[0], currList[i], (currList[0] - currList[i]).norm());
            baseSurface.property(diagonalList, *f_it).push_back(currDiagonal);
        }
    }

    // Update diagonalClusterCentroidList of mesh
    baseSurface.property(diagonalClusterCentroidList).clear();
    for (int i = 0; i < baseSurface.property(polygonClusterList).size(); ++i)
    {
        if (baseSurface.property(polygonClusterList)[i].size() <= 3)
        {
            vector<double> currDiagonalClusterCentroid;
            baseSurface.property(diagonalClusterCentroidList).push_back(currDiagonalClusterCentroid);
            continue;
        }

        vector<double> currDiagonalClusterCentroid(baseSurface.property(polygonClusterList)[i].size() - 3, 0);
        int polygonNum = 0;

        for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
        {
            if (baseSurface.property(clusteringLabel, *f_it) != i)
                continue;

            for (int j = 0; j < currDiagonalClusterCentroid.size(); ++j)
            {
                double currDiagonalLength = std::get<2>(baseSurface.property(diagonalList, *f_it)[j]);
                currDiagonalClusterCentroid[j] += currDiagonalLength;
            }
            polygonNum++;
        }

        for (int j = 0; j < currDiagonalClusterCentroid.size(); ++j)
        {
            currDiagonalClusterCentroid[j] = currDiagonalClusterCentroid[j] / double(polygonNum);
        }

        baseSurface.property(diagonalClusterCentroidList).push_back(currDiagonalClusterCentroid);

    }
}

void BaseSurface::UpdatePolygonSimilarityTable_LSM()
{
    // Get polygonSimiTable
    OpenMesh::MPropHandleT< vector<vector<double>> > polygonSimiTable;
    baseSurface.get_property_handle(polygonSimiTable, "polygonSimiTable");
    baseSurface.property(polygonSimiTable).clear();

    // Get centroidPolygonList to mesh
    OpenMesh::MPropHandleT< vector<vector<Vector3d>> > centroidPolygonList;
    baseSurface.get_property_handle(centroidPolygonList, "centroidPolygonList");
    baseSurface.property(centroidPolygonList).clear();

    for (int c = 0; c < GetPolygonClusterNum(); ++c)
    {
        vector<double> currSimiList;
        vector<Vector3d> centroidPolygonVerList;

        ComputeClusterSimilarityList(c, currSimiList, centroidPolygonVerList);

        baseSurface.property(polygonSimiTable).push_back(currSimiList);
        baseSurface.property(centroidPolygonList).push_back(centroidPolygonVerList);
    }

//    cout << "polygon simi table: " << endl;
//    double totalError = 0;
//    for (int i = 0; i < baseSurface.property(polygonSimiTable).size(); ++i)
//    {
//        for (int j = 0; j < baseSurface.property(polygonSimiTable)[i].size(); ++j)
//        {
//            cout << baseSurface.property(polygonSimiTable)[i][j] << ", ";
//            totalError += baseSurface.property(polygonSimiTable)[i][j];
//        }
//        cout << endl;
//    }
//    cout << "polygon error: " << totalError << endl;

}

void BaseSurface::ComputeClusterSimilarityList(int clusterID, vector<double> & similarityList, vector<Vector3d> & centroidPolygonVerList)
{
    vector<vector<Vector3d>> clusteredAlignedPolygons;

    vector<PolyMesh::FaceHandle> currClusterFaceHandleList;

    ComputeClusterCentroidPolygon_LSM(clusterID, centroidPolygonVerList, clusteredAlignedPolygons, currClusterFaceHandleList);

    double averageLength = ComputeAverageLength(centroidPolygonVerList);

    // Get polygonCentroidDistance to each face
    OpenMesh::FPropHandleT< double > polygonCentroidDistance;
    baseSurface.get_property_handle(polygonCentroidDistance, "polygonCentroidDistance");

    for (int i = 0; i < clusteredAlignedPolygons.size(); ++i)
    {
        double currDiff = 0;
        for (int j = 0; j < clusteredAlignedPolygons[i].size(); ++j)
        {
            if ((clusteredAlignedPolygons[i][j] - centroidPolygonVerList[j]).norm() > currDiff)
                currDiff = (clusteredAlignedPolygons[i][j] - centroidPolygonVerList[j]).norm() * (clusteredAlignedPolygons[i][j] - centroidPolygonVerList[j]).norm();
        }
        similarityList.push_back(currDiff);

        baseSurface.property(polygonCentroidDistance, currClusterFaceHandleList[i]) = currDiff;
    }
}

void BaseSurface::ComputeClusterCentroidPolygon_LSM(int clusterID, vector<Vector3d> & centroidPolygonVerList,
                                                    vector<vector<Vector3d>> & clusteredAlignedPolygons, vector<PolyMesh::FaceHandle> & currClusterFaceHandleList)
{
    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    baseSurface.get_property_handle(clusteringLabel, "clusteringLabel");

    // Get alignedVerList to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > alignedVerList;
    baseSurface.get_property_handle(alignedVerList, "alignedVerList");

    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");

    // Get edgeClusterCentroidList of mesh
    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface.get_property_handle(edgeClusterCentroidList, "edgeClusterCentroidList");

    // Save reference verList for each cluster
    vector< vector<Vector3d> > referPolygonList;
    vector< vector<int> > referPolygonEdgeLabelList;
    for (int i = 0; i < baseSurface.property(polygonClusterList).size(); ++i)
    {
        vector<Vector3d> currList;
        vector<int> currLabelList;
        referPolygonList.push_back(currList);
        referPolygonEdgeLabelList.push_back(currLabelList);
    }

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        int currLabel = baseSurface.property(clusteringLabel, *f_it);

        if (referPolygonList[currLabel].size() != 0)
            continue;
        else
        {
            PolyMesh::FaceVertexCWIter fv_it;
            vector<PolyMesh::VertexHandle> verHandleList;

            // Get the curr Vector3d point list clockwise
            for (fv_it = baseSurface.fv_cwiter(*f_it); fv_it.is_valid(); ++fv_it)
            {
                auto curr = baseSurface.point(*fv_it);
                referPolygonList[currLabel].push_back(Vector3d(curr[0], curr[1], curr[2]));
                verHandleList.push_back(*fv_it);
            }

            // Get related edgeLabel
            for (int i = 1; i < verHandleList.size(); ++i)
            {
                auto currEdgeHandle = FindEdge(baseSurface, verHandleList[i], verHandleList[i - 1]);
                referPolygonEdgeLabelList[currLabel].push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
            }
            {
                auto currEdgeHandle = FindEdge(baseSurface, verHandleList[0], verHandleList[verHandleList.size() - 1]);
                referPolygonEdgeLabelList[currLabel].push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
            }
        }
    }

    // Start alignment
    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        if (clusterID != baseSurface.property(clusteringLabel, *f_it))
            continue;

        currClusterFaceHandleList.push_back(*f_it);

        PolyMesh::FaceVertexCWIter fv_it;

        vector<Vector3d> currList;
        vector<int> edgeLabelList;
        vector<PolyMesh::VertexHandle> verHandleList;

        for (fv_it = baseSurface.fv_cwiter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            auto curr = baseSurface.point(*fv_it);
            currList.push_back(Vector3d(curr[0], curr[1], curr[2]));
            verHandleList.push_back(*fv_it);
        }

        // Get related edgeLabel
        for (int i = 1; i < verHandleList.size(); ++i)
        {
            auto currEdgeHandle = FindEdge(baseSurface, verHandleList[i], verHandleList[i - 1]);
            edgeLabelList.push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }
        {
            auto currEdgeHandle = FindEdge(baseSurface, verHandleList[0], verHandleList[verHandleList.size() - 1]);
            edgeLabelList.push_back(baseSurface.property(edgeClusteringLabel, currEdgeHandle));
        }

        // Get related polygonClusterList
        const vector<int> currRelatedList = referPolygonEdgeLabelList[baseSurface.property(clusteringLabel, *f_it)];
        const vector<Vector3d> currReferVerList = referPolygonList[baseSurface.property(clusteringLabel, *f_it)];

        vector<Vector3d> bestMatchingVertexList;
        FindBestMatchingVerListWithFlipping(currReferVerList, currList, bestMatchingVertexList);

        baseSurface.property(alignedVerList, *f_it).clear();
        baseSurface.property(alignedVerList, *f_it) = bestMatchingVertexList;

        MatrixXd p;
//        AlignPolygon_ShapeOp_FixedConfig(currReferVerList, bestMatchingVertexList, p);
        AlignPolygon_LSM_FixedConfig(currReferVerList, bestMatchingVertexList, p);

        // Push back the aligned vertex list
        vector<Vector3d> currAlignedList;
        for (int i = 0; i < p.cols(); ++i)
        {
            currAlignedList.push_back(Vector3d(p(0, i), p(1, i), p(2, i)));
        }

        clusteredAlignedPolygons.push_back(currAlignedList);
    }

    for (int i = 0; i < clusteredAlignedPolygons[0].size(); ++i)
    {
        Vector3d currMeanVer(0,0,0);

        for (int j = 0; j < clusteredAlignedPolygons.size(); ++j)
        {
            currMeanVer = currMeanVer + clusteredAlignedPolygons[j][i];
        }

        currMeanVer = currMeanVer / double( clusteredAlignedPolygons.size());
        centroidPolygonVerList.push_back(currMeanVer);
    }
}

void BaseSurface::UpdatePolygonPlanarity_LSM(PolyMesh * myMesh, PolyMesh * myTriMesh)
{
    // Request face normal property
    if (!myMesh->has_vertex_normals())
    {
        myMesh->request_face_normals();
        myMesh->update_normals();
    }

    // Get planarity to each face
    OpenMesh::FPropHandleT< double > facePlanarity;
    myMesh->get_property_handle( facePlanarity ,"facePlanarity");

    for (PolyMesh::FaceIter f_it=myMesh->faces_begin(); f_it!=myMesh->faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector<Vector3d> verList;

        double averageLength = 0;
        double currPlanarity = 0;

        // Extract all vertices of a given face
        for (fv_it=myMesh->fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer(myMesh->point(*fv_it)[0], myMesh->point(*fv_it)[1], myMesh->point(*fv_it)[2]);
            verList.push_back(currVer);
        }

        // Compute average length
        for (int i = 1; i < verList.size(); ++i)
        {
            averageLength += (verList[i] - verList[i - 1]).norm();
        }
        averageLength += (verList[0] - verList[verList.size() - 1]).norm();
        averageLength = averageLength / double(verList.size());

        if (verList.size() != 3)
        {
            // Put the vertices into a matrix
            MatrixXd points;
            points.resize(verList.size(), 3);

            for (int j = 0; j < verList.size(); ++j)
            {
                points(j, 0) = verList[j][0];
                points(j, 1) = verList[j][1];
                points(j, 2) = verList[j][2];
            }

            // Find the fitted plane using LSM
            Vector3d currFittedPlaneNormal;
            double d;

            FindFittedPlane_LSM(points, currFittedPlaneNormal, d);

            // Compute peak to valley flatness deviation
            VectorXd distList(points.rows());
            for (int j = 0; j < points.rows(); ++j)
            {
                Vector3d currVer(points(j, 0), points(j, 1), points(j, 2));
                distList(j) = currFittedPlaneNormal.dot(currVer) + d;
            }

            currPlanarity = fabs(distList.maxCoeff()) + fabs(distList.minCoeff());
            // currPlanarity = currPlanarity * (1.0f / averageLength);
        }

        myMesh->property( facePlanarity , *f_it ) = currPlanarity;
    }

    PlanarityPropertyTransfer(myMesh, myTriMesh);
}

void BaseSurface::UpdateEdgeNormal(PolyMesh * myMesh, PolyMesh * myTriMesh)
{
    // Request Face Normal
    baseSurface.request_face_normals();
    baseSurface.update_normals();

    // Get normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface.get_property_handle( normal ,"normal");

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface.is_boundary(*e_it))
        {
            continue;
        }

        const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

        vector<Vector3d> currList;

        for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
        {
            PolyMesh::FaceEdgeIter fe_it;

            for (fe_it = baseSurface.fe_iter(*f_it); fe_it.is_valid(); ++fe_it)
            {
                const PolyMesh::Point currTo   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*fe_it,0)));
                const PolyMesh::Point currFrom = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*fe_it,0)));

                if ((currTo == to and currFrom == from) or (currTo == from and currFrom == to))
                {
                    auto faceNorm = baseSurface.normal(*f_it);
                    Vector3d currNorm = Vector3d(faceNorm[0], faceNorm[1], faceNorm[2]);

                    currNorm.normalize();

                    currList.push_back(currNorm);
                }
            }

            if (currList.size() == 2)
            {
                Vector3d edgeNormal = (currList[0] + currList[1]) / 2;
                edgeNormal.normalize();
                baseSurface.property(normal, *e_it) = edgeNormal;
            }

            if (currList.size() == 1)
            {
                Vector3d edgeNormal = currList[0];
                edgeNormal.normalize();
                baseSurface.property(normal, *e_it) = edgeNormal;
            }
        }
    }

    EdgeNormalPropertyTransfer(myMesh, myTriMesh);
}

void BaseSurface::UpdateBlockSimilarityTable_LSM()
{
    // Get blockSimilarityTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > blockSimilarityTable;
    baseSurface.get_property_handle(blockSimilarityTable, "blockSimilarityTable");
    baseSurface.property(blockSimilarityTable).clear();

    // Get blockCentroidList to mesh
    OpenMesh::MPropHandleT< vector< tuple<vector<Vector3d>, vector<Vector3d>> > > blockCentroidList;
    baseSurface.get_property_handle(blockCentroidList, "blockCentroidList");
    baseSurface.property(blockCentroidList).clear();

    // Get blockCentroidList_combined to mesh
    OpenMesh::MPropHandleT< vector< tuple<vector<Vector3d>, vector<Vector3d>> > > blockCentroidList_combined;
    baseSurface.get_property_handle(blockCentroidList_combined, "blockCentroidList_combined");
    baseSurface.property(blockCentroidList_combined).clear();

    for (int c = 0; c < GetBlockClusterNum(); ++c)
    {
        vector<double> currSimiList;
        tuple<vector<Vector3d>, vector<Vector3d>> currCentroidBlock;
        tuple<vector<Vector3d>, vector<Vector3d>> currCentroidBlock_combined;

        ComputeClusterSimilarityList(c, currSimiList, currCentroidBlock, currCentroidBlock_combined);

        baseSurface.property(blockSimilarityTable).push_back(currSimiList);
        baseSurface.property(blockCentroidList).push_back(currCentroidBlock);
        baseSurface.property(blockCentroidList_combined).push_back(currCentroidBlock_combined);
    }
}

void BaseSurface::ComputeClusterSimilarityList(int clusterID, vector<double> & similarityList,
                                               tuple<vector<Vector3d>, vector<Vector3d>> & currCentroidBlock,
                                               tuple<vector<Vector3d>, vector<Vector3d>> & currCentroidBlock_combined)
{
    vector< tuple<vector<Vector3d>, vector<Vector3d>> > clusteredAlignedBlocks;
    vector< tuple<vector<Vector3d>, vector<Vector3d>> > clusteredAlignedBlocks_combined;

    ComputeClusterCentroidBlock_LSM(clusterID, currCentroidBlock, clusteredAlignedBlocks);

    vector<Vector3d> centroidUpperList = get<0>(currCentroidBlock);
    vector<Vector3d> centroidLowerList = get<1>(currCentroidBlock);

    for (int i = 0; i < clusteredAlignedBlocks.size(); ++i)
    {
        double currDiff = 0;

        vector<Vector3d> upperList = get<0>(clusteredAlignedBlocks[i]);
        vector<Vector3d> lowerList = get<1>(clusteredAlignedBlocks[i]);

        for (int j = 0; j < upperList.size(); ++j)
        {
            if ( (upperList[j] - centroidUpperList[j]).norm() > currDiff )
                currDiff = (upperList[j] - centroidUpperList[j]).norm() * (upperList[j] - centroidUpperList[j]).norm();
        }

        for (int j = 0; j < lowerList.size(); ++j)
        {
            if ( (lowerList[j] - centroidLowerList[j]).norm() > currDiff )
                currDiff = (lowerList[j] - centroidLowerList[j]).norm() * (lowerList[j] - centroidLowerList[j]).norm();
        }

        similarityList.push_back(currDiff);
    }
}

void BaseSurface::ComputeClusterCentroidBlock_LSM(int clusterID, tuple<vector<Vector3d>, vector<Vector3d>> & currCentroidBlock, vector< tuple<vector<Vector3d>, vector<Vector3d>> > & clusteredAlignedBlocks)
{
    // Get upperPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > upperPointList;
    baseSurface.get_property_handle(upperPointList, "upperPointList");

    // Get lowerPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > lowerPointList;
    baseSurface.get_property_handle(lowerPointList, "lowerPointList");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    baseSurface.get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get blockAlignedVerList to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > blockAlignedVerList;
    baseSurface.get_property_handle(blockAlignedVerList, "blockAlignedVerList");

    // Get blockCentroid to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > blockCentroid;
    baseSurface.get_property_handle(blockCentroid, "blockCentroid");

    // Save reference block
    tuple<vector<Vector3d>, vector<Vector3d>> referBlockTuple;

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        int currLabel = baseSurface.property(blockClusteringLabel, *f_it);

        if (currLabel != clusterID)
            continue;

        else
        {
            vector<Vector3d> currUpperList;
            vector<Vector3d> currLowerList;

            currUpperList = PolyMeshPointList2EigenVector3dList(baseSurface.property(upperPointList, *f_it));
            currLowerList = PolyMeshPointList2EigenVector3dList(baseSurface.property(lowerPointList, *f_it));

            get<0>(referBlockTuple) = currUpperList;
            get<1>(referBlockTuple) = currLowerList;

            break;
        }
    }

//    vector<Vector3d> referCombinedVerList = CombineTwoVector(get<0>(referBlockTuple), get<1>(referBlockTuple));
    vector<Vector3d> referCombinedVerList = CombineTwoBlockVerLists(get<0>(referBlockTuple), get<1>(referBlockTuple));
//    dbg(referCombinedVerList);

    // Start alignment
    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        if (clusterID != baseSurface.property(blockClusteringLabel, *f_it))
            continue;

        int currLabel = baseSurface.property(blockClusteringLabel, *f_it);
        //dbg(referCombinedVerList);

        vector<Vector3d> currUpperList;
        vector<Vector3d> currLowerList;

        currUpperList = PolyMeshPointList2EigenVector3dList(baseSurface.property(upperPointList, *f_it));
        currLowerList = PolyMeshPointList2EigenVector3dList(baseSurface.property(lowerPointList, *f_it));

        double simi, simi_1, simi_2, simi_3, simi_4;

        MatrixXd p, p_reversed, p_flip_reversed, p_flip, p_orig;

        // 1) first config
//        cout << "Start aligning. " << endl;
        AlignBlock_LSM(referCombinedVerList, currUpperList, currLowerList, p_orig);

        simi_1 = 0;
        for (int i = 0; i < p_orig.cols(); ++i)
        {
            simi_1 += (p_orig.col(i) - referCombinedVerList[i]).norm();
        }
        simi = simi_1;
//        p = p_orig;
//
//        // 2) second config : reversed
//        reverse(currUpperList.begin(), currUpperList.end());
//        reverse(currLowerList.begin(), currLowerList.end());
//        AlignBlock_LSM(referCombinedVerList, currUpperList, currLowerList, p_reversed);
//
//        simi_2 = 0;
//        for (int i = 0; i < p_reversed.cols(); ++i)
//        {
//            simi_2 += (p_reversed.col(i) - referCombinedVerList[i]).norm();
//        }
//
//        if (simi_2 < simi)
//        {
////            cout << "Reversed block to align. " << endl;
////            p = p_flip;
//            simi = simi_2;
//        }
//
//        // 3) third config : reversed-flip
//        AlignBlock_LSM(referCombinedVerList, currLowerList, currUpperList, p_flip_reversed);
//
//        simi_3 = 0;
//        for (int i = 0; i < p_flip_reversed.cols(); ++i)
//        {
//            simi_3 += (p_flip_reversed.col(i) - referCombinedVerList[i]).norm();
//        }
//
//        if (simi_3 < simi)
//        {
////            cout << "Reversed-flip block to align. " << endl;
////            p = p_flip_reversed;
//            simi = simi_3;
//        }
//
//        // 4) fourth config : flip
//        reverse(currUpperList.begin(), currUpperList.end());
//        reverse(currLowerList.begin(), currLowerList.end());
//        AlignBlock_LSM(referCombinedVerList, currLowerList, currUpperList, p_flip);
//
//        simi_4 = 0;
//        for (int i = 0; i < p_flip.cols(); ++i)
//        {
//            simi_4 += (p_flip.col(i) - referCombinedVerList[i]).norm();
//        }
//
//        if (simi_4 < simi)
//        {
////            cout << "Reversed-flip block to align. " << endl;
////            p = p_flip;
//            simi = simi_4;
//        }
//
//        if (simi == simi_1)
//            p = p_orig;
//        else if (simi == simi_2)
//            p = p_reversed;
//        else if (simi == simi_3)
//            p = p_flip_reversed;
//        else
//            p = p_flip;

        p = p_orig;
//
//        if (simi != simi_1)
//            cout << "Flip or reverse block to align. " << endl << endl;

        // Push back the aligned vertex list
        vector<Vector3d> currAlignedList;
        for (int i = 0; i < p.cols(); ++i)
        {
            currAlignedList.push_back(Vector3d(p(0, i), p(1, i), p(2, i)));
        }

        tuple< vector<Vector3d>, vector<Vector3d> > currBlockTuple;
        SplitTwoVector(currAlignedList, get<0>(currBlockTuple), get<1>(currBlockTuple));

//        if (simi == simi_1 or simi == simi_2)
//            SplitTwoVector(currAlignedList, get<0>(currBlockTuple), get<1>(currBlockTuple));
//        else
//        {
//            SplitTwoVector(currAlignedList, get<1>(currBlockTuple), get<0>(currBlockTuple));
//            reverse(get<1>(currBlockTuple).begin(), get<1>(currBlockTuple).end());
//            reverse(get<0>(currBlockTuple).begin(), get<0>(currBlockTuple).end());
//        }

        clusteredAlignedBlocks.push_back(currBlockTuple);
    }

    // Compute upperCentroidList and lowerCentroidList
    vector<Vector3d> upperCentroidList;
    vector<Vector3d> lowerCentroidList;
    for (int i = 0; i < get<0>(clusteredAlignedBlocks[0]).size(); ++i)
    {
        Vector3d currMeanVer_upper(get<0>(clusteredAlignedBlocks[0])[i][0],get<0>(clusteredAlignedBlocks[0])[i][1],get<0>(clusteredAlignedBlocks[0])[i][2]);
        Vector3d currMeanVer_lower(get<1>(clusteredAlignedBlocks[0])[i][0],get<1>(clusteredAlignedBlocks[0])[i][1],get<1>(clusteredAlignedBlocks[0])[i][2]);

        for (int j = 1; j < clusteredAlignedBlocks.size(); ++j)
        {
            currMeanVer_upper = currMeanVer_upper + get<0>(clusteredAlignedBlocks[j])[i];
            currMeanVer_lower = currMeanVer_lower + get<1>(clusteredAlignedBlocks[j])[i];
        }

        double count = clusteredAlignedBlocks.size();

        Vector3d currMeanVer_up = currMeanVer_upper / double( count );
        Vector3d currMeanVer_low = currMeanVer_lower / double( count );

        upperCentroidList.push_back(currMeanVer_up);
        lowerCentroidList.push_back(currMeanVer_low);
    }

//    get<0>(currCentroidBlock) = get<0>(referBlockTuple);
//    get<1>(currCentroidBlock) = get<1>(referBlockTuple);

    get<0>(currCentroidBlock) = upperCentroidList;
    get<1>(currCentroidBlock) = lowerCentroidList;
}

void BaseSurface::UpdateBlockSimilarityTable_LSM_planar()
{

}

void BaseSurface::ComputeClusterSimilarityList_planar(int clusterID, vector<double> &similarityList, 
                                            tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock_planar)
{

}

void BaseSurface::ComputeClusterCentroidBlock_LSM_planar(int clusterID, tuple<vector<Vector3d>, vector<Vector3d>> &currCentroidBlock_planar,
                                         vector<tuple<vector<Vector3d>, vector<Vector3d>>> &clusteredAlignedBlocks_planar)
{

}




//**************************************************************************************//
//                                  Worst-case Analysis
//**************************************************************************************//

void BaseSurface::WorstCaseAnalysis()
{
    ComputeWorstCaseSurfaceMeetFabRequirement();

    ComputeWorstPolygonSimilarityCase();

    ComputeWorstPlanarityCase();

    ComputeWorstDihedralAngleError();

    ComputeWorstEdgeLengthError();

    cout << endl << "----------------------------------" << endl;
    cout << "worstFabApproximation: " <<  worstFabApproximation << endl;
    cout << "worstPolygonSimilarity: " <<  worstPolygonSimilarity << endl;
    cout << "worstPlanarity: " <<  worstPlanarity << endl;
    cout << "worstDihedralAngleError: " <<  worstDihedralAngleError << endl;
    cout << "worstEdgeLengthError: " << worstEdgeLengthError << endl;
    cout << "----------------------------------" << endl << endl;
}




//**************************************************************************************//
// Check worst case: 1) optimized surface meets fabrication requirement (penetration / gap)
//**************************************************************************************//

void BaseSurface::ComputeWorstCaseSurfaceMeetFabRequirement()
{
    // Get problemVerPair to mesh
    OpenMesh::MPropHandleT< vector<std::tuple<Vector3d, Vector3d, int>> > problemVerPair;
    baseSurface.get_property_handle(problemVerPair, "problemVerPair");
    baseSurface.property(problemVerPair).clear();

    // Get isSatisfyFabRequirement to each face
    OpenMesh::FPropHandleT< bool > isSatisfyFabRequirement;
    baseSurface.get_property_handle(isSatisfyFabRequirement, "isSatisfyFabRequirement");

    // Get replacedFaceHandle to each face
    OpenMesh::FPropHandleT< PolyMesh::FaceHandle > replacedFaceHandle;
    baseSurface.get_property_handle(replacedFaceHandle, "replacedFaceHandle");

    // Get worstFabVerHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::VertexHandle > worstFabVerHandle;
    baseSurface.get_property_handle(worstFabVerHandle, "worstFabVerHandle");

    worstFabApproximation = 0;

    for (PolyMesh::VertexIter v_it=baseSurface.vertices_begin(); v_it!=baseSurface.vertices_end(); ++v_it)
    {
        PolyMesh::VertexFaceIter vf_it;

        int i = 0;
        Vector3d refFaceCenter;
        Vector3d refMovedVer;
        Vector3d currOrigVer = Vector3d(baseSurface.point(*v_it)[0], baseSurface.point(*v_it)[1],baseSurface.point(*v_it)[2]);

        for (vf_it = baseSurface.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
        {
            if (i == 0)
            {
                // Compute refFaceCenter
                refFaceCenter = Vector3d(baseSurface.calc_face_centroid(*vf_it)[0], baseSurface.calc_face_centroid(*vf_it)[1], baseSurface.calc_face_centroid(*vf_it)[2]);

                // Compute refMovedVer
                PolyMesh::FaceVertexIter fv_it;
                double maxDistance = 1000000;

                for (fv_it = replacedSurface.fv_iter(baseSurface.property(replacedFaceHandle, *vf_it)); fv_it.is_valid(); ++fv_it)
                {
                    Vector3d currVer(replacedSurface.point(*fv_it)[0], replacedSurface.point(*fv_it)[1], replacedSurface.point(*fv_it)[2]);

                    if ((currVer - currOrigVer).norm() < maxDistance)
                    {
                        maxDistance = (currVer - currOrigVer).norm();
                        refMovedVer = currVer;
                    }
                }

                i++;
            }
            else
            {
                // Compute origVec
                Vector3d currFaceCenter;
                currFaceCenter = Vector3d(baseSurface.calc_face_centroid(*vf_it)[0], baseSurface.calc_face_centroid(*vf_it)[1], baseSurface.calc_face_centroid(*vf_it)[2]);

                Vector3d origVec = currFaceCenter - refFaceCenter;
                origVec.normalize();

                // Compute currMovedVer
                Vector3d currMovedVer;

                PolyMesh::FaceVertexIter fv_it;
                double maxDistance = 1000000;

                for (fv_it = replacedSurface.fv_iter(baseSurface.property(replacedFaceHandle, *vf_it)); fv_it.is_valid(); ++fv_it)
                {
                    Vector3d currVer(replacedSurface.point(*fv_it)[0], replacedSurface.point(*fv_it)[1], replacedSurface.point(*fv_it)[2]);

                    if ((currVer - currOrigVer).norm() < maxDistance)
                    {
                        maxDistance = (currVer - currOrigVer).norm();
                        currMovedVer = currVer;
                    }
                }

                // Compute movedDirection
                Vector3d movedDirection = currMovedVer - refMovedVer;

                double currResult;

                int fabStatus = IsVertexMeetFabRequirement(origVec, movedDirection, currResult);

                if (currResult < worstFabApproximation)
                {
                    worstFabApproximation = currResult;
                    baseSurface.property(worstFabVerHandle) = *v_it;
                }

                if (fabStatus != CASE_FAB_READY)
                {
                    baseSurface.property(isSatisfyFabRequirement, *vf_it) = false;

                    Vector3d currLineCenter = (refFaceCenter + currFaceCenter) / 2;
                    Vector3d diff = (currOrigVer - currLineCenter);

                    std::tuple<Vector3d, Vector3d, int> currPair(refFaceCenter + diff, currFaceCenter + diff, fabStatus);

                    if (find(baseSurface.property(problemVerPair).begin(), baseSurface.property(problemVerPair).end(), currPair) == baseSurface.property(problemVerPair).end()
                        and find(baseSurface.property(problemVerPair).begin(), baseSurface.property(problemVerPair).end(), std::tuple<Vector3d, Vector3d, int>(currFaceCenter + diff, refFaceCenter + diff, fabStatus)) == baseSurface.property(problemVerPair).end())
                    {
                        baseSurface.property(problemVerPair).push_back(currPair);
                    }
                }
            }
        }
    }

    worstFabApproximation = fabs(worstFabApproximation);

//    dbg(baseSurface.property(problemVerPair));
}

int BaseSurface::IsVertexMeetFabRequirement(Vector3d & origVec, Vector3d & movedVec, double & result)
{
    origVec.normalize();

    result = origVec.dot(movedVec);

    if (result >= -fabricationThreshold and result <= fabricationThreshold)
        return CASE_FAB_READY;

    else if (result < -fabricationThreshold)
        return CASE_PENETRATION;

    else if (result > fabricationThreshold)
        return CASE_GAP;

    else
        return CASE_FAB_READY;
}




//**************************************************************************************//
//                      Check worst case: 2) polygon similarity
//**************************************************************************************//

void BaseSurface::ComputeWorstPolygonSimilarityCase()
{
    // Get polygonCentroidDistance to each face
    OpenMesh::FPropHandleT< double > polygonCentroidDistance;
    baseSurface.get_property_handle(polygonCentroidDistance, "polygonCentroidDistance");

    // Get worstPolygonSimiFaceHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::FaceHandle > worstPolygonSimiFaceHandle;
    baseSurface.get_property_handle(worstPolygonSimiFaceHandle, "worstPolygonSimiFaceHandle");

    worstPolygonSimilarity = 0;
    E_polygon = 0;
    double totalDiff = 0;

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        if (baseSurface.property(polygonCentroidDistance, *f_it) > worstPolygonSimilarity)
        {
            worstPolygonSimilarity = baseSurface.property(polygonCentroidDistance, *f_it);
            baseSurface.property(worstPolygonSimiFaceHandle) = *f_it;

//            cout << "currPolygonCentroidDistance: " << baseSurface.property(polygonCentroidDistance, *f_it) << endl;
        }

        totalDiff += baseSurface.property(polygonCentroidDistance, *f_it);
    }

    avgPolygonSimilarity = totalDiff / double(baseSurface.n_faces());
    E_polygon = totalDiff;
}




//**************************************************************************************//
//                          Check worst case: 3) planarity
//**************************************************************************************//

void BaseSurface::ComputeWorstPlanarityCase()
{
    // Get worstPlanarFaceHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::FaceHandle > worstPlanarFaceHandle;
    baseSurface.get_property_handle(worstPlanarFaceHandle, "worstPlanarFaceHandle");

    // Get facePlanarity to each face
    OpenMesh::FPropHandleT< double > facePlanarity;
    baseSurface.get_property_handle( facePlanarity ,"facePlanarity");

    worstPlanarity = 0;
    double totalPlanarity = 0;
    E_planar = 0;

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        if (baseSurface.property(facePlanarity, *f_it) > worstPlanarity)
        {
            worstPlanarity = baseSurface.property(facePlanarity, *f_it);
            baseSurface.property(worstPlanarFaceHandle) = *f_it;
        }

        totalPlanarity += baseSurface.property(facePlanarity, *f_it);
    }

    avgPlanarity = totalPlanarity / double(baseSurface.n_faces());
    E_planar = totalPlanarity;
}




//**************************************************************************************//
//                      Check worst case: 4) dihedral angle
//**************************************************************************************//

void BaseSurface::ComputeWorstDihedralAngleError()
{
    // Get dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    // Get angleClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > angleClusterCentroid;
    baseSurface.get_property_handle(angleClusterCentroid, "angleClusterCentroid");

    // Get worstDihedralAngleEdgeHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::EdgeHandle > worstDihedralAngleEdgeHandle;
    baseSurface.get_property_handle(worstDihedralAngleEdgeHandle, "worstDihedralAngleEdgeHandle");

    worstDihedralAngleError = 0;
    double totalError = 0;
    int count = 0;
    E_dihed = 0;

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        double currAngle = baseSurface.property(dihedralAngle, *e_it);
        double currCentroid = baseSurface.property(angleClusterCentroid, *e_it);

        if (fabs(currAngle - currCentroid) > worstDihedralAngleError)
        {
            worstDihedralAngleError = fabs(currAngle - currCentroid);
            baseSurface.property(worstDihedralAngleEdgeHandle) = *e_it;
        }

        if( !baseSurface.is_boundary(*e_it))
        {
            count ++;
            totalError += fabs(currAngle - currCentroid);

            double currErrorInRadius = fabs(currAngle - currCentroid) / 180.0f * M_PI;
            E_dihed += currErrorInRadius * currErrorInRadius;
        }
    }

    avgDihedralAngleError = totalError / double(count);
}




//**************************************************************************************//
//                      Check worst case: 5) edge length
//**************************************************************************************//

void BaseSurface::ComputeWorstEdgeLengthError()
{
    // Get edgeLength for each edge
    OpenMesh::EPropHandleT< double > edgeLength;
    baseSurface.get_property_handle(edgeLength, "edgeLength");

    // Get edgeClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > edgeClusterCentroid;
    baseSurface.get_property_handle(edgeClusterCentroid, "edgeClusterCentroid");

    worstEdgeLengthError = 0;
    double totalError = 0;
    int count = 0;
    E_edge = 0;

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        double currEdgeLength = baseSurface.property(edgeLength, *e_it);
        double currCentroid = baseSurface.property(edgeClusterCentroid, *e_it);

        if (fabs(currEdgeLength - currCentroid) > worstEdgeLengthError)
        {
            worstEdgeLengthError = fabs(currEdgeLength - currCentroid);
        }

        if( !baseSurface.is_boundary(*e_it))
        {
            count ++;
            totalError += fabs(currEdgeLength - currCentroid);
            E_edge += fabs(currEdgeLength - currCentroid) * fabs(currEdgeLength - currCentroid);
        }
    }

    avgEdgeLengthError = totalError / double(count);
}




//**************************************************************************************//
//                                 Surface analysis
//**************************************************************************************//

void BaseSurface::SurfaceAnalysis()
{
    ComputeSurfaceCloseness();
    ComputeSurfaceSmoothness();
}

void BaseSurface::ComputeSurfaceCloseness()
{
    // Get origPoint to each vertex
    OpenMesh::VPropHandleT< PolyMesh::Point > origPoint;
    baseSurface.get_property_handle( origPoint ,"origPoint");

    closeness = 0;

    for (PolyMesh::VertexIter v_it=baseSurface.vertices_begin(); v_it!=baseSurface.vertices_end(); ++v_it)
    {
        closeness += (baseSurface.property(origPoint, *v_it) - baseSurface.point(*v_it)).norm() * (baseSurface.property(origPoint, *v_it) - baseSurface.point(*v_it)).norm();
    }
}

void BaseSurface::ComputeSurfaceSmoothness()
{

}


//**************************************************************************************//
//                                 Get Operation
//**************************************************************************************//

void BaseSurface::GetBaseSurfaceEdgeLengthList(vector<double> & edgeLengthList)
{
    vector<vector<PolyMesh::Point>> edgePointList;

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

        double currLength = (to - from).norm();

        bool flag = true;
        for (int i = 0; i < edgePointList.size(); ++i)
        {
            if ((edgePointList[i][0] == to and edgePointList[i][1] == from) or (edgePointList[i][1] == to and edgePointList[i][0] == from))
            {
                flag = false;
                break;
            }
        }

        if (flag)
        {
            vector<PolyMesh::Point> currPointList;
            currPointList.push_back(to);
            currPointList.push_back(from);
            edgePointList.push_back(currPointList);

            edgeLengthList.push_back(currLength);
        }
    }

    // Sort this list
    sort(edgeLengthList.begin(), edgeLengthList.end());
}

void BaseSurface::GetRemeshedSurfaceEdgeLengthList(vector<double> & edgeLengthList)
{
    vector<vector<PolyMesh::Point>> edgePointList;

    for (PolyMesh::EdgeIter e_it=remeshedPolySurface.edges_begin(); e_it!=remeshedPolySurface.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = remeshedPolySurface.point(remeshedPolySurface.to_vertex_handle(remeshedPolySurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = remeshedPolySurface.point(remeshedPolySurface.from_vertex_handle(remeshedPolySurface.halfedge_handle(*e_it,0)));

        double currLength = (to - from).norm();

        bool flag = true;
        for (int i = 0; i < edgePointList.size(); ++i)
        {
            if ((edgePointList[i][0] == to and edgePointList[i][1] == from) or (edgePointList[i][1] == to and edgePointList[i][0] == from))
            {
                flag = false;
                break;
            }
        }

        if (flag)
        {
            vector<PolyMesh::Point> currPointList;
            currPointList.push_back(to);
            currPointList.push_back(from);
            edgePointList.push_back(currPointList);

            edgeLengthList.push_back(currLength);
        }
    }

    // Sort this list
    sort(edgeLengthList.begin(), edgeLengthList.end());
}

void BaseSurface::GetEdgeClusterList(vector<vector<double>> &edgeClusterList, int clusterNum)
{
    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    for (int i = 0; i < clusterNum; ++i)
    {
        vector<double> currEdgeList;

        vector<vector<PolyMesh::Point>> edgePointList;

        for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
        {
            const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
            const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

            double currLength = (to - from).norm();

            if (baseSurface.property(edgeClusteringLabel, *e_it) == i) {
                bool flag = true;
                for (int i = 0; i < edgePointList.size(); ++i) {
                    if ((edgePointList[i][0] == to and edgePointList[i][1] == from) or
                        (edgePointList[i][1] == to and edgePointList[i][0] == from)) {
                        flag = false;
                        break;
                    }
                }

                if (flag) {
                    vector<PolyMesh::Point> currPointList;
                    currPointList.push_back(to);
                    currPointList.push_back(from);
                    edgePointList.push_back(currPointList);

                    currEdgeList.push_back(currLength);
                }
            }
        }

        // Sort this list
        sort(currEdgeList.begin(), currEdgeList.end());

        edgeClusterList.push_back(currEdgeList);
    }
}

void BaseSurface::GetEdgeTupleList(vector< std::tuple<PolyMesh::Point, PolyMesh::Point, double> > & edgeTupleList)
{
    vector<vector<PolyMesh::Point>> edgePointList;

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

        double currLength = (to - from).norm();

        bool flag = true;
        for (int i = 0; i < edgePointList.size(); ++i)
        {
            if ((edgePointList[i][0] == to and edgePointList[i][1] == from) or (edgePointList[i][1] == to and edgePointList[i][0] == from))
            {
                flag = false;
                break;
            }
        }

        if (flag)
        {
            vector<PolyMesh::Point> currPointList;
            currPointList.push_back(to);
            currPointList.push_back(from);
            edgePointList.push_back(currPointList);
        }
    }
}

void BaseSurface::GetEdgeClusterErrorList(vector<vector<double>> & edgeClusterList, vector< std::tuple<double, double, double> > & edgeClusterErrorList, float threshold)
{
    edgeClusterErrorList.clear();

    for (int i = 0; i < edgeClusterList.size(); ++i)
    {
        double currMean, currMeanError, currMaxError = 0, currErrorNum = 0;

        currMean = accumulate(edgeClusterList[i].begin(), edgeClusterList[i].end(), double(0.0)) / double(edgeClusterList[i].size());

        double totalError = 0;
        for (int j = 0; j < edgeClusterList[i].size(); ++j)
        {
            double curr = fabs(currMean - edgeClusterList[i][j]);
            totalError += curr;

            if (curr > currMean * threshold)
                currErrorNum++;

            if (curr > currMaxError)
                currMaxError = curr;
        }

        currMeanError = totalError / edgeClusterList[i].size();

        double standardScalar = 1.0 / currMean;

        currMeanError *= standardScalar;
        currMaxError *= standardScalar;

        auto t = make_tuple(currMeanError, currMaxError, currErrorNum);

        edgeClusterErrorList.push_back(t);
    }
}




//**************************************************************************************//
//                           Get dihedral angle operation
//**************************************************************************************//

void BaseSurface::GetBaseSurfaceDihedralAngleList(vector<double> &angleList)
{
    angleList.clear();

    // Get dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface.property(dihedralAngle, *e_it) > 0)
            angleList.push_back(baseSurface.property(dihedralAngle, *e_it));

//        dbg(baseSurface.property(dihedralAngle, *e_it));
    }

    sort(angleList.begin(), angleList.end());
}

void BaseSurface::GetRemeshedSurfaceDihedralAngleList(vector<double> &angleList)
{
    angleList.clear();
    
    // Get dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    remeshedPolySurface.get_property_handle(dihedralAngle, "dihedralAngle");

    for (PolyMesh::EdgeIter e_it=remeshedPolySurface.edges_begin(); e_it!=remeshedPolySurface.edges_end(); ++e_it)
    {
        if (remeshedPolySurface.property(dihedralAngle, *e_it) > 0)
            angleList.push_back(remeshedPolySurface.property(dihedralAngle, *e_it));

//        dbg(baseSurface.property(dihedralAngle, *e_it));
    }

    sort(angleList.begin(), angleList.end());
}

void BaseSurface::GetDihedralAngleClusterList(vector<vector<double>> &dihedralAngleClusterList, int clusterNum)
{
    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > dihedralAngleLabel;
    baseSurface.get_property_handle(dihedralAngleLabel, "dihedralAngleLabel");

    // Get dihedralAngle
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    for (int i = 0; i < clusterNum; ++i)
    {
        vector<double> currAngleList;

        for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
        {
            if (baseSurface.property(dihedralAngleLabel, *e_it) == i) {
                currAngleList.push_back(baseSurface.property(dihedralAngle, *e_it));
            }
        }

        // Sort this list
        sort(currAngleList.begin(), currAngleList.end());

        dihedralAngleClusterList.push_back(currAngleList);
    }
}

void BaseSurface::GetDihedralAngleClusterErrorList(vector<vector<double>> & dihedralAngleClusterList, vector< std::tuple<double, double, double> > & dihedralAngleClusterErrorList, float threshold)
{
    dihedralAngleClusterErrorList.clear();

    for (int i = 0; i < dihedralAngleClusterList.size(); ++i)
    {
        double currMean, currMeanError, currMaxError = 0, currErrorNum = 0;

        currMean = accumulate(dihedralAngleClusterList[i].begin(), dihedralAngleClusterList[i].end(), double(0.0)) / double(dihedralAngleClusterList[i].size());

        double totalError = 0;
        for (int j = 0; j < dihedralAngleClusterList[i].size(); ++j)
        {
            double curr = fabs(currMean - dihedralAngleClusterList[i][j]);
            totalError += curr;

            if (curr > currMean * threshold)
                currErrorNum++;

            if (curr > currMaxError)
                currMaxError = curr;
        }

        currMeanError = totalError / dihedralAngleClusterList[i].size();

//        double standardScalar = 1.0 / currMean;
//
//        currMeanError *= standardScalar;
//        currMaxError *= standardScalar;

        auto t = make_tuple(currMeanError, currMaxError, currErrorNum);

        dihedralAngleClusterErrorList.push_back(t);
    }
}




//**************************************************************************************//
//                         Get polygon cluster operation
//**************************************************************************************//

int BaseSurface::GetPolygonClusterNum()
{
    // Get polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface.get_property_handle(polygonClusterList, "polygonClusterList");

    return int (baseSurface.property(polygonClusterList).size());
}

void BaseSurface::GetDiagonalLengthList(vector<double> &diagonalLengthList)
{
    // Get diagonalList to each face
    OpenMesh::FPropHandleT< vector<std::tuple<Vector3d, Vector3d, double>> > diagonalList;
    baseSurface.get_property_handle(diagonalList, "diagonalList");

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        for (int i = 0; i < baseSurface.property(diagonalList, *f_it).size(); ++i)
        {
            diagonalLengthList.push_back(std::get<2>(baseSurface.property(diagonalList, *f_it)[i]));
        }
    }

    sort(diagonalLengthList.begin(), diagonalLengthList.end());
}

void BaseSurface::GetPlanarityList(vector<double> &planarityList)
{
    // Get planarity to each face
    OpenMesh::FPropHandleT< double > facePlanarity;
    baseSurface.get_property_handle( facePlanarity ,"facePlanarity");

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        if (baseSurface.property(facePlanarity, *f_it) > 0)
        {
            planarityList.push_back(baseSurface.property(facePlanarity, *f_it));
        }
    }

    sort(planarityList.begin(), planarityList.end());
}

int BaseSurface::ComputePolygonClusterNumByEdgeLabel()
{
    vector< vector<int> > polygonClusterList;

    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        PolyMesh::FaceEdgeIter fe_it;

        vector<int> currLabelList;
        for (fe_it = baseSurface.fe_iter(*f_it); fe_it.is_valid(); ++fe_it)
        {
            currLabelList.push_back(baseSurface.property(edgeClusteringLabel, *fe_it));
        }

        if (IsCurrListInExistingList(polygonClusterList, currLabelList) == -1)
            polygonClusterList.push_back(currLabelList);
    }

    return polygonClusterList.size();
}




//**************************************************************************************//
//                             Get block cluster operation
//**************************************************************************************//

int BaseSurface::GetBlockClusterNum()
{
    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    baseSurface.get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    int count = 0;

    for (int i = 0; i < baseSurface.property(blockCuttingAngleLabelList).size(); ++i)
    {
        count += (baseSurface.property(blockCuttingAngleLabelList)[i].size());
    }

    return count;
}

vector<int> BaseSurface::GetBlockClusterBasedOnPolygonCluster(int polygonClusterID)
{
    // Get blockCuttingAngleLabelList
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    baseSurface.get_property_handle(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    // Get blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    baseSurface.get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    vector<int> result;

//    dbg(baseSurface.property(blockCuttingAngleLabelList)[polygonClusterID]);

    for (int i = 0; i < baseSurface.property(blockCuttingAngleLabelList)[polygonClusterID].size(); ++i)
    {
        for (int j = 0; j < baseSurface.property(blockCuttingAngleLabelList_full).size(); ++j)
        {
            if (find(baseSurface.property(blockCuttingAngleLabelList_full)[j].begin(), baseSurface.property(blockCuttingAngleLabelList_full)[j].end(),
                     baseSurface.property(blockCuttingAngleLabelList)[polygonClusterID][i]) != baseSurface.property(blockCuttingAngleLabelList_full)[j].end())
            {
                result.push_back(j);
                break;
            }
        }
    }

    return result;
}




//**************************************************************************************//
//                             Disassembly Planning
//**************************************************************************************//

void BaseSurface::ComputeDisassemblyPlan()
{
    // Get isHold to each edge
    OpenMesh::EPropHandleT< bool > isHold;
    baseSurface.get_property_handle( isHold ,"isHold");

    // Get seqID to each face
    OpenMesh::FPropHandleT< int > seqID;
    baseSurface.get_property_handle(seqID, "seqID");

    // Get index to each face
    OpenMesh::FPropHandleT< int > index;
    baseSurface.get_property_handle(index, "index");

    // Start assembly by disassembly
    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        // Init isHold for each edge
        for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
        {
            baseSurface.property(isHold, *e_it) = !baseSurface.is_boundary(*e_it);
        }

//        cout << "Start 1" << endl;

        vector<PolyMesh::FaceHandle> removedFaceHandleList;
        vector<PolyMesh::FaceHandle> nextFaceHandleList;
        int count = 0;

        nextFaceHandleList.push_back(*f_it);

        while (nextFaceHandleList.size() > 0 and count <= baseSurface.n_faces())
        {
            PolyMesh::FaceHandle currFace = nextFaceHandleList[0];
            nextFaceHandleList.erase(nextFaceHandleList.begin());

            if (IsRemovableFace(currFace) and find(removedFaceHandleList.begin(), removedFaceHandleList.end(), currFace) == removedFaceHandleList.end())
            {
                removedFaceHandleList.push_back(currFace);

                baseSurface.property(seqID, currFace) = baseSurface.n_faces() - count;
                count ++;

//                cout << "face " << baseSurface.property(index, currFace) << " is taken out in " << baseSurface.property(seqID, currFace) << endl;

                for (PolyMesh::FaceFaceIter ff_it = baseSurface.ff_iter(currFace); ff_it.is_valid(); ++ff_it)
                {
                    if (find(removedFaceHandleList.begin(), removedFaceHandleList.end(), *ff_it) == removedFaceHandleList.end())
                    {
                        nextFaceHandleList.push_back(*ff_it);
                    }
                }

                // Update the edge isHold status
                for (PolyMesh::FaceEdgeIter fe_it = baseSurface.fe_iter(currFace); fe_it.is_valid(); ++fe_it)
                {
                    baseSurface.property(isHold, *fe_it) = false;
                }

            }
        }

        if (count == baseSurface.n_faces())
        {
            cout << "Find a feasible assembly plan. " << endl;
            break;
        }
    }
}

bool BaseSurface::IsRemovableFace(const PolyMesh::FaceHandle & currFH)
{
    // Get isHold to each edge
    OpenMesh::EPropHandleT< bool > isHold;
    baseSurface.get_property_handle( isHold ,"isHold");

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface.get_property_handle( cuttingPlane ,"cuttingPlane");

    PolyMesh::FaceEdgeIter fe_it;

//    cout << "Starting judging. " << endl

    for (fe_it = baseSurface.fe_iter(currFH); fe_it.is_valid(); ++fe_it)
    {
        if (baseSurface.property(isHold, *fe_it) == false)
            continue;

        CuttingPlane currCuttingPlane = baseSurface.property(cuttingPlane, *fe_it);

        if ((currFH == currCuttingPlane.positiveFaceHandle and currCuttingPlane.offsetAngle > 0)
        or (currFH == currCuttingPlane.negativeFaceHandle and currCuttingPlane.offsetAngle < 0))
        {
            return true;
        }
    }

    return true;
}




//**************************************************************************************//
//                                Helper Function
//**************************************************************************************//

void BaseSurface::AlignBlock_LSM(const vector<Vector3d> & referBlockVerList, vector<Vector3d> & currUpperList, vector<Vector3d> & currLowerList, MatrixXd & p)
{
    double simi = 0;
    vector<Vector3d> currBlockVerList = CombineTwoBlockVerLists(currUpperList, currLowerList);

    vector<Vector3d> bestMatchingVerList;
    MatrixXd bestTransMat;
    FindBlockBestMatchingTransMatWithoutFlipping(referBlockVerList, currBlockVerList, bestMatchingVerList, bestTransMat);

    while (bestMatchingVerList != currBlockVerList)
    {
        currUpperList.push_back(currUpperList[0]);
        currUpperList.erase(currUpperList.begin());

        currLowerList.push_back(currLowerList[0]);
        currLowerList.erase(currLowerList.begin());

        currBlockVerList = CombineTwoBlockVerLists(currUpperList, currLowerList);
    }

    vector<Vector3d> combinedCurrCentroid = CombineTwoVector(currUpperList, currLowerList);

    p.resize(3, combinedCurrCentroid.size());

    for (int i = 0; i < combinedCurrCentroid.size(); ++i)
    {
        p(0, i) = combinedCurrCentroid[i][0];
        p(1, i) = combinedCurrCentroid[i][1];
        p(2, i) = combinedCurrCentroid[i][2];
    }

    // Move p to (0, 0, 0)
    {
        Vector3d currCentroid = ComputePolygonCentroid(p);

        Vector3d currOffset = Vector3d(0, 0, 0) - currCentroid;

        for (int i = 0; i < combinedCurrCentroid.size(); ++i)
        {
            p(0, i) += currOffset[0];
            p(1, i) += currOffset[1];
            p(2, i) += currOffset[2];
        }
    }

    p = bestTransMat * p;

    // Move p to referCentroid
    {
        Vector3d referCentroid = ComputePolygonCentroid(referBlockVerList);

        Vector3d currCentroid = ComputePolygonCentroid(p);

        Vector3d currOffset = referCentroid - currCentroid;

        for (int i = 0; i < combinedCurrCentroid.size(); ++i)
        {
            p(0, i) += currOffset[0];
            p(1, i) += currOffset[1];
            p(2, i) += currOffset[2];
        }
    }

//    MatrixXd result;
//    int
//    cout << "simi: " << simi << endl;
}

void BaseSurface::FindBestMatchingVerListWithFlipping(const vector<Vector3d> & currReferVerList, vector<Vector3d> & currList, vector<Vector3d> & bestMatchingVertexList)
{
    int iteration = 0;
    bestMatchingVertexList = currList;

    double simi;
//        ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, simi);
    ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, simi);

    // Before flipping
    while (iteration <= currList.size())
    {
        double currSimi;
//            ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, currSimi);
        ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, currSimi);

        if (currSimi < simi)
        {
            simi = currSimi;
            bestMatchingVertexList = currList;
        }

        // Rotate edgeLabelList
//        edgeLabelList.push_back(edgeLabelList[0]);
//        edgeLabelList.erase(edgeLabelList.begin());

        // Rotate currList
        currList.push_back(currList[0]);
        currList.erase(currList.begin());

        iteration++;
    }

    // After flipping
//    reverse(currList.begin(), currList.end());
//    iteration = 0;
//    while (iteration <= currList.size())
//    {
//        double currSimi;
////            ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, currSimi);
//        ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, currSimi);
//
//        if (currSimi < simi)
//        {
//            simi = currSimi;
//            bestMatchingVertexList = currList;
//        }
//
//        // Rotate edgeLabelList
////        edgeLabelList.push_back(edgeLabelList[0]);
////        edgeLabelList.erase(edgeLabelList.begin());
//
//        // Rotate currList
//        currList.push_back(currList[0]);
//        currList.erase(currList.begin());
//
//        iteration++;
//    }
}

void BaseSurface::FindBestMatchingVerListWithFlipping(const vector<Vector3d> &currReferVerList, vector<Vector3d> &currList, vector<Vector3d> &bestMatchingVertexList, double & simi)
{
    int iteration = 0;
    bestMatchingVertexList = currList;

//        ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, simi);
    ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, simi);

    // Before flipping
    while (iteration <= currList.size())
    {
        double currSimi;
//            ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, currSimi);
        ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, currSimi);

        if (currSimi < simi)
        {
            simi = currSimi;
            bestMatchingVertexList = currList;
        }

        // Rotate edgeLabelList
//        edgeLabelList.push_back(edgeLabelList[0]);
//        edgeLabelList.erase(edgeLabelList.begin());

        // Rotate currList
        currList.push_back(currList[0]);
        currList.erase(currList.begin());

        iteration++;
    }

    // After flipping
    reverse(currList.begin(), currList.end());
    iteration = 0;
    while (iteration <= currList.size())
    {
        double currSimi;
//            ComputePolygonSimilarity_ShapeOp_FixedConfig(currReferVerList, currList, currSimi);
        ComputePolygonSimilarity_LSM_FixedConfig(currReferVerList, currList, currSimi);

        if (currSimi < simi)
        {
            simi = currSimi;
            bestMatchingVertexList = currList;
        }

        // Rotate edgeLabelList
//        edgeLabelList.push_back(edgeLabelList[0]);
//        edgeLabelList.erase(edgeLabelList.begin());

        // Rotate currList
        currList.push_back(currList[0]);
        currList.erase(currList.begin());

        iteration++;
    }
}

void BaseSurface::FindBlockBestMatchingTransMatWithoutFlipping(const vector<Vector3d> & currReferVerList, vector<Vector3d> & currList, vector<Vector3d> & bestMatchingVerList, MatrixXd & transMat)
{
    int iteration = 0;

    double simi;
    ComputePolygonBestTransMat_LSM_FixedConfig(currReferVerList, currList, simi, transMat);
    bestMatchingVerList = currList;

    // Before flipping
    while (iteration < currList.size())
    {
        double currSimi;
        MatrixXd currTransMat;
        ComputePolygonBestTransMat_LSM_FixedConfig(currReferVerList, currList, currSimi, currTransMat);

        if (currSimi < simi)
        {
            simi = currSimi;
            bestMatchingVerList = currList;
            transMat = currTransMat;
        }

        // Rotate currList
        currList.push_back(currList[0]);
        currList.push_back(currList[1]);

        currList.erase(currList.begin());
        currList.erase(currList.begin());

        iteration++;
    }

//    cout << "simi: " << simi << endl;

//    dbg(currReferVerList);
//    dbg(bestMatchingVerList);
}

void BaseSurface::FindBestMatchingBlockVerListWithoutFlipping(const vector<Vector3d> & referCombinedVerList, vector<Vector3d> & currUpperList, vector<Vector3d> & currLowerList, vector<Vector3d> & bestMatchingVertexList)
{
    double simi;
    vector<Vector3d> combinedVerList = CombineTwoVector(currUpperList, currLowerList);
    bestMatchingVertexList = combinedVerList;

    int iteration = 0;
    while (iteration <= combinedVerList.size() / 2.0f)
    {
        double currSimi;

        //ComputePolygonSimilarity_ShapeOp_FixedConfig(referCombinedVerList, combinedVerList, currSimi);
        ComputePolygonSimilarity_LSM_FixedConfig(referCombinedVerList, combinedVerList, currSimi);

        if (currSimi < simi)
        {
            simi = currSimi;
            bestMatchingVertexList = combinedVerList;
        }

        // Rotate
        currUpperList.push_back(currUpperList[0]);
        currUpperList.erase(currUpperList.begin());

        currLowerList.push_back(currLowerList[0]);
        currLowerList.erase(currLowerList.begin());

        combinedVerList = CombineTwoVector(currUpperList, currLowerList);

        iteration ++;
    }
}

void BaseSurface::ComputePolygonSimilarity_ShapeOp_FixedConfig(const vector<Vector3d> & referP, const vector<Vector3d> & currP, double & similarity)
{
    // Init similarity
    similarity = 0;

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

    // Prepare currP_mat
    currP_mat.resize(3, currP.size());
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

        auto c = std::make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight * 2, currP_mat, false, true, false);

        s.addConstraint(c);
    }

    s.initialize();
    s.solve(100);
    auto p = s.getPoints();

    for (int j = 0; j < p.cols(); ++j)
    {
        similarity += (referenceP_mat.col(j) - p.col(j)).norm();
    }

}

void BaseSurface::ComputePolygonSimilarity_LSM_FixedConfig(const vector<Vector3d> & referP, const vector<Vector3d> & currP, double & similarity)
{
    // Init similarity
    similarity = 0;

    // Matrix
    MatrixXd referenceP_mat, referenceP_moved, currP_mat, currP_moved;

    // Prepare referenceP_mat
    referenceP_mat.resize(3, currP.size());
    for (int i = 0; i < currP.size(); ++i)
    {
        referenceP_mat(0, i) = referP[i][0];
        referenceP_mat(1, i) = referP[i][1];
        referenceP_mat(2, i) = referP[i][2];
    }
    referenceP_moved = referenceP_mat;

    // Compute referCenter
    Vector3d referCentroid = ComputePolygonCentroid(referenceP_mat);

    // Move the refer model to the (0, 0, 0)
    {
        Vector3d offset = Vector3d(0, 0, 0) - referCentroid;

        for (int j = 0; j < referenceP_moved.cols(); ++j)
        {
            referenceP_moved(0, j) += offset(0);
            referenceP_moved(1, j) += offset(1);
            referenceP_moved(2, j) += offset(2);
        }
    }

    // Prepare currP_mat
    currP_mat.resize(3, currP.size());
    for (int j = 0; j < currP.size(); ++j)
    {
        currP_mat(0, j) = currP[j][0];
        currP_mat(1, j) = currP[j][1];
        currP_mat(2, j) = currP[j][2];
    }
    currP_moved = currP_mat;

    // Compute currCenter
    Vector3d currCentroid = ComputePolygonCentroid(currP_mat);

    // Move the curr model to the (0, 0, 0)
    {
        Vector3d offset = Vector3d(0, 0, 0) - currCentroid;

        for (int j = 0; j < currP_moved.cols(); ++j)
        {
            currP_moved(0, j) += offset(0);
            currP_moved(1, j) += offset(1);
            currP_moved(2, j) += offset(2);
        }
    }

    MatrixXd H;
    H.resize(3,3);
    H.setZero();

    for (int i = 0; i < currP.size(); ++i)
    {
        H = H + currP_moved.col(i) * referenceP_moved.col(i).transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U = svd.matrixU();

    MatrixXd R = V * U.transpose();

    auto det = R.determinant();
    if (det == -1)
    {
        V(0, 2) = -V(0, 2);
        V(1, 2) = -V(1, 2);
        V(2, 2) = -V(2, 2);

        R = V * U.transpose();
    }

    MatrixXd alignedMat = R * currP_mat;

    // Compute alignedCentroid
    Vector3d alignedCentroid = ComputePolygonCentroid(alignedMat);

    // Move the curr model to the referCenter
    {
        Vector3d offset = referCentroid - alignedCentroid;

        for (int j = 0; j < alignedMat.cols(); ++j)
        {
            alignedMat(0, j) += offset(0);
            alignedMat(1, j) += offset(1);
            alignedMat(2, j) += offset(2);
        }
    }

    for (int i = 0; i < currP.size(); ++i)
    {
        similarity += (referenceP_mat.col(i) - alignedMat.col(i)).norm() * (referenceP_mat.col(i) - alignedMat.col(i)).norm();
    }
}

void BaseSurface::ComputePolygonBestTransMat_LSM_FixedConfig(const vector<Vector3d> & referP, const vector<Vector3d> & currP, double & similarity, MatrixXd & transMat)
{
    // Init similarity
    similarity = 0;

    // Matrix
    MatrixXd referenceP_mat, referenceP_moved, currP_mat, currP_moved;

    // Prepare referenceP_mat
    referenceP_mat.resize(3, currP.size());
    for (int i = 0; i < currP.size(); ++i)
    {
        referenceP_mat(0, i) = referP[i][0];
        referenceP_mat(1, i) = referP[i][1];
        referenceP_mat(2, i) = referP[i][2];
    }
    referenceP_moved = referenceP_mat;

    // Compute referCentroid
    Vector3d referCentroid = ComputePolygonCentroid(referenceP_mat);

    // Move the refer model to the (0, 0, 0)
    {
        Vector3d offset = Vector3d(0, 0, 0) - referCentroid;

        for (int j = 0; j < referenceP_moved.cols(); ++j)
        {
            referenceP_moved(0, j) += offset(0);
            referenceP_moved(1, j) += offset(1);
            referenceP_moved(2, j) += offset(2);
        }
    }

    // Prepare currP_mat
    currP_mat.resize(3, currP.size());
    for (int j = 0; j < currP.size(); ++j)
    {
        currP_mat(0, j) = currP[j][0];
        currP_mat(1, j) = currP[j][1];
        currP_mat(2, j) = currP[j][2];
    }
    currP_moved = currP_mat;

    // Compute currCenter
    Vector3d currCentroid = ComputePolygonCentroid(currP_mat);

    // Move the curr model to the (0, 0, 0)
    {
        Vector3d offset = Vector3d(0, 0, 0) - currCentroid;

        for (int j = 0; j < currP_moved.cols(); ++j)
        {
            currP_moved(0, j) += offset(0);
            currP_moved(1, j) += offset(1);
            currP_moved(2, j) += offset(2);
        }
    }

    MatrixXd H;
    H.resize(3,3);
    H.setZero();

    for (int i = 0; i < currP.size(); ++i)
    {
        H = H + currP_moved.col(i) * referenceP_moved.col(i).transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U = svd.matrixU();

    MatrixXd R = V * U.transpose();

    auto det = R.determinant();
    if (det == -1)
    {
//        cout << "det == -1" << endl;

        V(0, 2) = -V(0, 2);
        V(1, 2) = -V(1, 2);
        V(2, 2) = -V(2, 2);

        R = V.transpose() * U.transpose();
    }

    MatrixXd alignedMat = R * currP_moved;

    // Compute alignedCentroid
    Vector3d alignedCentroid = ComputePolygonCentroid(alignedMat);

    // Move the curr model to the referCenter
    {
        Vector3d offset = referCentroid;

        for (int j = 0; j < alignedMat.cols(); ++j)
        {
            alignedMat(0, j) += offset(0);
            alignedMat(1, j) += offset(1);
            alignedMat(2, j) += offset(2);
        }

        for (int i = 0; i < currP.size(); ++i)
        {
            similarity += (referenceP_mat.col(i) - alignedMat.col(i)).norm() * (referenceP_mat.col(i) - alignedMat.col(i)).norm();
        }

//        cout << "simi: " << similarity << endl;
//        cout << "referCentroid: " << endl << referCentroid << endl;
//        cout << "alignedCentroid: " << endl << alignedCentroid << endl;
//        cout << "offset: " << endl << offset << endl << endl;
    }

    transMat = R;
}

void BaseSurface::AlignPolygon_ShapeOp_FixedConfig(const vector<Vector3d> & referP, const vector<Vector3d> & currP, MatrixXd & alignedP_mat)
{
    // ShapeOp Matrix
    ShapeOp::Matrix3X referenceP_mat, currP_mat;

    referenceP_mat.resize(3, currP.size());
    currP_mat.resize(3, currP.size());

    // Prepare referenceP
    for (int i = 0; i < referP.size(); ++i)
    {
        referenceP_mat(0, i) = referP[i][0];
        referenceP_mat(1, i) = referP[i][1];
        referenceP_mat(2, i) = referP[i][2];
    }

    // Prepare currP
    for (int i = 0; i < currP.size(); ++i)
    {
        currP_mat(0, i) = currP[i][0];
        currP_mat(1, i) = currP[i][1];
        currP_mat(2, i) = currP[i][2];
    }

    MoveToCenter_ShapeOp(referenceP_mat);
    MoveToCenter_ShapeOp(currP_mat);

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
    alignedP_mat = s.getPoints();
}

void BaseSurface::AlignPolygon_LSM_FixedConfig(const vector<Vector3d> & referP, const vector<Vector3d> & currP, MatrixXd & alignedP_mat)
{
    // Matrix
    MatrixXd referenceP_mat, referenceP_moved, currP_mat, currP_moved;

    // Prepare referenceP_mat
    referenceP_mat.resize(3, currP.size());
    for (int i = 0; i < currP.size(); ++i)
    {
        referenceP_mat(0, i) = referP[i][0];
        referenceP_mat(1, i) = referP[i][1];
        referenceP_mat(2, i) = referP[i][2];
    }
    referenceP_moved = referenceP_mat;

    // Compute referCentroid
    Vector3d referCentroid = ComputePolygonCentroid(referP);

    // Move the refer model to the (0, 0, 0)
    {
        Vector3d offset = Vector3d(0, 0, 0) - referCentroid;

        for (int j = 0; j < referenceP_moved.cols(); ++j)
        {
            referenceP_moved(0, j) += offset(0);
            referenceP_moved(1, j) += offset(1);
            referenceP_moved(2, j) += offset(2);
        }
    }

    // Prepare currP_mat
    currP_mat.resize(3, currP.size());
    for (int j = 0; j < currP.size(); ++j)
    {
        currP_mat(0, j) = currP[j][0];
        currP_mat(1, j) = currP[j][1];
        currP_mat(2, j) = currP[j][2];
    }
    currP_moved = currP_mat;

    // Compute currCentroid
    Vector3d currCentroid = ComputePolygonCentroid(currP_mat);

    // Move the curr model to the (0, 0, 0)
    {
        Vector3d offset = Vector3d(0, 0, 0) - currCentroid;

        for (int j = 0; j < currP_moved.cols(); ++j)
        {
            currP_moved(0, j) += offset(0);
            currP_moved(1, j) += offset(1);
            currP_moved(2, j) += offset(2);
        }
    }

    MatrixXd H;
    H.resize(3,3);
    H.setZero();

    for (int i = 0; i < currP.size(); ++i)
    {
        H = H + currP_moved.col(i) * referenceP_moved.col(i).transpose();
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U = svd.matrixU();

    MatrixXd R = V * U.transpose();

    double det = R.determinant();

//    cout << "currDet: " << det << endl;
    if (det == -1)
    {
//        cout << "currDet: " << det << endl;
        V(0, 2) = -V(0, 2);
        V(1, 2) = -V(1, 2);
        V(2, 2) = -V(2, 2);

        R = V * U.transpose();
    }

    alignedP_mat = R * currP_mat ;

    // Compute alignedCentroid
    Vector3d alignedCentroid = ComputePolygonCentroid(alignedP_mat);

    // Move the curr model to the referCenter
    {
        Vector3d offset = referCentroid - alignedCentroid;

        for (int j = 0; j < alignedP_mat.cols(); ++j)
        {
            alignedP_mat(0, j) += offset(0);
            alignedP_mat(1, j) += offset(1);
            alignedP_mat(2, j) += offset(2);
        }
    }
}



