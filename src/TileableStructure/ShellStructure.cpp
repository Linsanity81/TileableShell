//
// Created by Linsanity on 27/6/22.
//

#include "ShellStructure.h"
#include <CGAL/intersections.h>
#include "pugixml.hpp"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include "Mesh/MeshBoolean.h"


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

ShellStructure::ShellStructure(BaseSurface * _baseSurface)
{
    baseSurface = _baseSurface;

    cutLower = 0.04;
    cutUpper = 0.04;

    absoluteTolerance = baseSurface->absoluteTolerance;

    penetrationVolumeSum = 0;
    worstOverlappedVolumeBlockRatio = 0;
    avgOverlappedVolumeBlockRatio = 0;

    avgBlockError = 0;
    worstBlockError = 0;

    avgContactAngleError = 0;
    worstContactAngleError = 0;

    avgGapVolumeRatio = 0;
    worstGapVolumeRatio = 0;

    shellOptimizationTime = 0;
}

ShellStructure::~ShellStructure()
{
    baseSurface = nullptr;

    cutLower = 0.02;
    cutUpper = 0.02;

    penetrationVolumeSum = 0;
    worstOverlappedVolumeBlockRatio = 0;
    avgOverlappedVolumeBlockRatio = 0;

    avgBlockError = 0;
    worstBlockError = 0;

    avgContactAngleError = 0;
    worstContactAngleError = 0;

    shellOptimizationTime = 0;

    shellBlockList.clear();
}

void ShellStructure::InitOrigAndReplacedShellStructures()
{
    InitShellStructure();
    ComputeBlockSimilarityTable_LSM();

    InitReplacedShellStructure_LSM();
    InitReplacedShellStructure_LSM_planar();

    // Worst case analysis
    WorstCaseAnalysis();
}

void ShellStructure::InitShellStructure()
{
    UpdateTolerance();

    // Request Face Normal of baseSurface
    baseSurface->baseSurface.request_face_normals();
    baseSurface->baseSurface.update_face_normals();

    // Create new mesh
    PolyMesh myMesh;

    shellBlockList.clear();

    /// Add properties
    // Add clusterLabel to each face
    OpenMesh::FPropHandleT< int > shellFaceLabel;
    myMesh.add_property(shellFaceLabel, "clusteringLabel");

    // Add clusterLabel to each face
    OpenMesh::FPropHandleT< PolyMesh::FaceHandle > origFaceHandle;
    myMesh.add_property(origFaceHandle, "origFaceHandle");

    /// Get properties
    // Get normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface->baseSurface.get_property_handle( normal ,"normal");

    // Get shellBlockPointer to each face
    OpenMesh::FPropHandleT< int > shellBlockPointer;
    baseSurface->baseSurface.get_property_handle(shellBlockPointer, "shellBlockPointer");

    // Get isBoundaryFace to each face
    OpenMesh::FPropHandleT< bool > isBoundaryFace;
    baseSurface->baseSurface.get_property_handle(isBoundaryFace, "isBoundaryFace");

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > origClusteringLabel;
    baseSurface->baseSurface.get_property_handle(origClusteringLabel, "clusteringLabel");

    // Get blockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > blockFaceHandleList;
    baseSurface->baseSurface.get_property_handle(blockFaceHandleList, "blockFaceHandleList");

    // Get upperPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > origUpperPointList;
    baseSurface->baseSurface.get_property_handle(origUpperPointList, "upperPointList");

    // Get lowerPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > origLowerPointList;
    baseSurface->baseSurface.get_property_handle(origLowerPointList, "lowerPointList");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    baseSurface->baseSurface.get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get index to each face
    OpenMesh::FPropHandleT< int > index;
    baseSurface->baseSurface.get_property_handle(index, "index");

    // Get seqID to each face
    OpenMesh::FPropHandleT< int > seqID;
    baseSurface->baseSurface.get_property_handle(seqID, "seqID");

    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        bool isCurrFaceBoundary = baseSurface->baseSurface.property(isBoundaryFace, *f_it);

        PolyMesh currTempBlock;
        PolyMesh currTempBlock_overlap;
        PolyMesh currTempBlock_overlap_tolerance;

        int currIndex = baseSurface->baseSurface.property(index, *f_it);

        int currSeqID = baseSurface->baseSurface.property(seqID, *f_it);

        int currFaceClusteringLabel = baseSurface->baseSurface.property(blockClusteringLabel, *f_it);

        baseSurface->baseSurface.property(blockFaceHandleList, *f_it).clear();
        baseSurface->baseSurface.property(origUpperPointList, *f_it).clear();
        baseSurface->baseSurface.property(origUpperPointList, *f_it).clear();

        auto faceCenter = baseSurface->baseSurface.calc_face_centroid(*f_it);
        auto faceNormal = baseSurface->baseSurface.normal(*f_it);
        faceNormal.normalize();

        Point_3 middleFaceCenter_3(faceCenter[0], faceCenter[1], faceCenter[2]);
        Vector_3 middleFaceNormal_3(faceNormal[0], faceNormal[1], faceNormal[2]);

        Point_3 upperFaceCenter_3 = middleFaceCenter_3 + middleFaceNormal_3 * cutUpper;
        Vector_3 upperFaceNormal_3 = middleFaceNormal_3;

        Point_3 upperFaceCenter_3_overlap;
        Point_3 upperFaceCenter_3_overlap_tolerance = middleFaceCenter_3 + middleFaceNormal_3 * cutUpper;
        Vector_3 upperFaceNormal_3_overlap = middleFaceNormal_3;
        Vector_3 upperFaceNormal_3_overlap_tolerance = middleFaceNormal_3;

        Point_3 lowerFaceCenter_3 = middleFaceCenter_3 - middleFaceNormal_3 * cutLower;
        Vector_3 lowerFaceNormal_3 = -middleFaceNormal_3;
        Vector3d lowerFaceNormal(-faceNormal[0], -faceNormal[1], -faceNormal[2]);
        Vector3d upperFaceNormal(faceNormal[0], faceNormal[1], faceNormal[2]);

        Point_3 lowerFaceCenter_3_overlap;
        Point_3 lowerFaceCenter_3_overlap_tolerance = middleFaceCenter_3 - middleFaceNormal_3 * (cutLower + 4 * absoluteTolerance);
        Vector_3 lowerFaceNormal_3_overlap = -middleFaceNormal_3;
        Vector_3 lowerFaceNormal_3_overlap_tolerance = -middleFaceNormal_3;

        if (!isCurrFaceBoundary)
        {
            upperFaceCenter_3_overlap = middleFaceCenter_3 + middleFaceNormal_3 * cutUpper * 3;
            lowerFaceCenter_3_overlap = middleFaceCenter_3 - middleFaceNormal_3 * cutLower * 3;
        }
        else
        {
            upperFaceCenter_3_overlap = middleFaceCenter_3 + middleFaceNormal_3 * cutUpper;
            lowerFaceCenter_3_overlap = middleFaceCenter_3 - middleFaceNormal_3 * cutLower;
        }

        Plane_3 currMiddlePlane(middleFaceCenter_3, middleFaceNormal_3);
        Plane_3 currUpperPlane(upperFaceCenter_3, upperFaceNormal_3);
        Plane_3 currLowerPlane(lowerFaceCenter_3, lowerFaceNormal_3);

        Plane_3 currUpperPlane_overlap(upperFaceCenter_3_overlap, upperFaceNormal_3_overlap);
        Plane_3 currLowerPlane_overlap(lowerFaceCenter_3_overlap, lowerFaceNormal_3_overlap);

        Plane_3 currUpperPlane_overlap_tolerance(upperFaceCenter_3_overlap_tolerance, upperFaceNormal_3_overlap_tolerance);
        Plane_3 currLowerPlane_overlap_tolerance(lowerFaceCenter_3_overlap_tolerance, lowerFaceNormal_3_overlap_tolerance);

        vector<Vector3d> currVerList;
        vector<PolyMesh::VertexHandle> currVerHandleList;

        vector<Vector3d> currEdgeCenterList;
        vector<Vector3d> currEdgeCenterList_overlap;
        vector<Vector3d> currEdgeCenterList_overlap_tolerance;

        vector<Vector3d> currEdgeNormalList;
        vector<Vector3d> currEdgeList;

        PolyMesh::FaceVertexCCWIter fv_it;
        for (fv_it = baseSurface->baseSurface.fv_ccwiter(*f_it); fv_it.is_valid(); ++fv_it) {
            Vector3d currVer(baseSurface->baseSurface.point(*fv_it)[0], baseSurface->baseSurface.point(*fv_it)[1],
                             baseSurface->baseSurface.point(*fv_it)[2]);

            currVerList.push_back(currVer);
            currVerHandleList.push_back(*fv_it);
        }

//        cout << "verList and verHandleList done. " << endl;

        for (int i = 1; i < currVerList.size(); ++i)
        {
            // Moved Vector
            Vector3d currMovedVec = Vector3d(faceCenter[0], faceCenter[1], faceCenter[2]) - ((currVerList[i] + currVerList[i - 1]) / 2);
            currMovedVec.normalize();

            // Edge center
            currEdgeCenterList.push_back((currVerList[i] + currVerList[i - 1]) / 2 + currMovedVec * absoluteTolerance);

            // Overlap Edge center
            PolyMesh::EdgeHandle currEdgeHandle = FindEdge(baseSurface->baseSurface, currVerHandleList[i], currVerHandleList[i - 1]);

            if (baseSurface->baseSurface.is_boundary(currEdgeHandle))
            {
                currEdgeCenterList_overlap.push_back((currVerList[i] + currVerList[i - 1]) / 2 + currMovedVec * absoluteTolerance);
            }
            else
            {
                currEdgeCenterList_overlap.push_back((currVerList[i] + currVerList[i - 1]) / 2 - currMovedVec * absoluteTolerance);
            }

            // Overlap_tolerance Edge Center
            currEdgeCenterList_overlap_tolerance.push_back((currVerList[i] + currVerList[i - 1]) / 2 - 3 * currMovedVec * absoluteTolerance);

            // Edge
            Vector3d currEdge = currVerList[i] - currVerList[i - 1];
            currEdge.normalize();
            currEdgeList.push_back(currEdge);

            // Edge normal
            auto currEH = FindEdge(baseSurface->baseSurface, currVerHandleList[i], currVerHandleList[i - 1]);
            currEdgeNormalList.push_back(baseSurface->baseSurface.property(normal, currEH));

//            if (baseSurface->baseSurface.is_boundary(currEH))
//            {
//                cout << "shellEdgeNormal: " << baseSurface->baseSurface.property(normal, currEH)[0] << baseSurface->baseSurface.property(normal, currEH)[1] << baseSurface->baseSurface.property(normal, currEH)[2] << endl;
//            }
        }
        {
            // Moved Vector
            Vector3d currMovedVec = Vector3d(faceCenter[0], faceCenter[1], faceCenter[2]) - ((currVerList[0] + currVerList[currVerList.size() - 1]) / 2);
            currMovedVec.normalize();

            // Edge center
            currEdgeCenterList.push_back((currVerList[currVerList.size() - 1] + currVerList[0]) / 2 + currMovedVec * absoluteTolerance);

            // Overlap Edge center
            PolyMesh::EdgeHandle currEdgeHandle = FindEdge(baseSurface->baseSurface, currVerHandleList[currVerList.size() - 1], currVerHandleList[0]);

            if (baseSurface->baseSurface.is_boundary(currEdgeHandle))
            {
                currEdgeCenterList_overlap.push_back((currVerList[currVerList.size() - 1] + currVerList[0]) / 2 + currMovedVec * absoluteTolerance);
            }
            else
            {
                currEdgeCenterList_overlap.push_back((currVerList[currVerList.size() - 1] + currVerList[0]) / 2 - currMovedVec * absoluteTolerance);
            }

            // Overlap_tolerance Edge Center
            currEdgeCenterList_overlap_tolerance.push_back((currVerList[currVerList.size() - 1] + currVerList[0]) / 2 - 3 * currMovedVec * absoluteTolerance);

            // Edge
            Vector3d currEdge = currVerList[currVerList.size() - 1] - currVerList[0];
            currEdge.normalize();
            currEdgeList.push_back(currEdge);

            // Edge normal
            auto currEH = FindEdge(baseSurface->baseSurface, currVerHandleList[currVerList.size() - 1],
                                   currVerHandleList[0]);
            currEdgeNormalList.push_back(baseSurface->baseSurface.property(normal, currEH));

//            if (baseSurface->baseSurface.is_boundary(currEH))
//            {
//                cout << "shellEdgeNormal: " << baseSurface->baseSurface.property(normal, currEH)[0] << baseSurface->baseSurface.property(normal, currEH)[1] << baseSurface->baseSurface.property(normal, currEH)[2] << endl;
//            }
        }

        vector<Plane_3> edgeCuttingPlaneList;
        vector<Plane_3> edgeCuttingPlaneList_overlap;
        vector<Plane_3> edgeCuttingPlaneList_overlap_tolerance;

        for (int i = 0; i < currEdgeNormalList.size(); ++i) {
            // without overlap
            {
                Point_3 currEdgeCenter_3(currEdgeCenterList[i][0], currEdgeCenterList[i][1], currEdgeCenterList[i][2]);

                Vector_3 currEdge_3(currEdgeList[i][0], currEdgeList[i][1], currEdgeList[i][2]);
                Vector_3 currEdgeNormal_3(currEdgeNormalList[i][0], currEdgeNormalList[i][1], currEdgeNormalList[i][2]);
                Vector_3 currPlaneVector_3 = CGAL::cross_product(currEdge_3, currEdgeNormal_3);

                Plane_3 currCuttingPlane(currEdgeCenter_3, currPlaneVector_3);
                edgeCuttingPlaneList.push_back(currCuttingPlane);
            }

            // overlap
            {
                Point_3 currEdgeCenter_3(currEdgeCenterList_overlap[i][0], currEdgeCenterList_overlap[i][1], currEdgeCenterList_overlap[i][2]);

                Vector_3 currEdge_3(currEdgeList[i][0], currEdgeList[i][1], currEdgeList[i][2]);
                Vector_3 currEdgeNormal_3(currEdgeNormalList[i][0], currEdgeNormalList[i][1], currEdgeNormalList[i][2]);
                Vector_3 currPlaneVector_3 = CGAL::cross_product(currEdge_3, currEdgeNormal_3);

                Plane_3 currCuttingPlane(currEdgeCenter_3, currPlaneVector_3);
                edgeCuttingPlaneList_overlap.push_back(currCuttingPlane);
            }

            // overlap_tolerance
            {
                Point_3 currEdgeCenter_3(currEdgeCenterList_overlap_tolerance[i][0], currEdgeCenterList_overlap_tolerance[i][1], currEdgeCenterList_overlap_tolerance[i][2]);

                Vector_3 currEdge_3(currEdgeList[i][0], currEdgeList[i][1], currEdgeList[i][2]);
                Vector_3 currEdgeNormal_3(currEdgeNormalList[i][0], currEdgeNormalList[i][1], currEdgeNormalList[i][2]);
                Vector_3 currPlaneVector_3 = CGAL::cross_product(currEdge_3, currEdgeNormal_3);

                Plane_3 currCuttingPlane(currEdgeCenter_3, currPlaneVector_3);
                edgeCuttingPlaneList_overlap_tolerance.push_back(currCuttingPlane);
            }
        }

        vector<PolyMesh::Point> upperPointList;
        vector<PolyMesh::Point> lowerPointList;

        vector<PolyMesh::Point> upperPointList_overlap;
        vector<PolyMesh::Point> lowerPointList_overlap;

        vector<PolyMesh::Point> upperPointList_overlap_tolerance;
        vector<PolyMesh::Point> lowerPointList_overlap_tolerance;

        for (int i = 1; i < edgeCuttingPlaneList.size(); ++i) {
            {
                auto currUpperResult = CGAL::intersection(currUpperPlane, edgeCuttingPlaneList[i],
                                                          edgeCuttingPlaneList[i - 1]);

                if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                    upperPointList.push_back(
                            PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                            CGAL::to_double(currUpperPoint[0][2])));
                }

                auto currLowerResult = CGAL::intersection(currLowerPlane, edgeCuttingPlaneList[i],
                                                          edgeCuttingPlaneList[i - 1]);

                if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                    lowerPointList.push_back(
                            PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                            CGAL::to_double(currLowerPoint[0][2])));
                }
            }

            {
                auto currUpperResult = CGAL::intersection(currUpperPlane_overlap, edgeCuttingPlaneList_overlap[i],
                                                          edgeCuttingPlaneList_overlap[i - 1]);

                if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                    upperPointList_overlap.push_back(
                            PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                            CGAL::to_double(currUpperPoint[0][2])));
                }

                auto currLowerResult = CGAL::intersection(currLowerPlane_overlap, edgeCuttingPlaneList_overlap[i],
                                                          edgeCuttingPlaneList_overlap[i - 1]);

                if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                    lowerPointList_overlap.push_back(
                            PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                            CGAL::to_double(currLowerPoint[0][2])));
                }
            }

            {
                auto currUpperResult = CGAL::intersection(currUpperPlane_overlap_tolerance, edgeCuttingPlaneList_overlap_tolerance[i],
                                                          edgeCuttingPlaneList_overlap_tolerance[i - 1]);

                if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                    upperPointList_overlap_tolerance.push_back(
                            PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                            CGAL::to_double(currUpperPoint[0][2])));
                }

                auto currLowerResult = CGAL::intersection(currLowerPlane_overlap_tolerance, edgeCuttingPlaneList_overlap_tolerance[i],
                                                          edgeCuttingPlaneList_overlap_tolerance[i - 1]);

                if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                    lowerPointList_overlap_tolerance.push_back(
                            PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                            CGAL::to_double(currLowerPoint[0][2])));
                }
            }
        }
        {
            {
                auto currUpperResult = CGAL::intersection(currUpperPlane, edgeCuttingPlaneList[0],
                                                          edgeCuttingPlaneList[edgeCuttingPlaneList.size() - 1]);

                if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
//                std::cout << currUpperPoint[0][0] << currUpperPoint[0][1] << currUpperPoint[0][2] << std::endl;
                    upperPointList.push_back(
                            PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                            CGAL::to_double(currUpperPoint[0][2])));
                }

                auto currLowerResult = CGAL::intersection(currLowerPlane, edgeCuttingPlaneList[0],
                                                          edgeCuttingPlaneList[edgeCuttingPlaneList.size() - 1]);

                if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                    lowerPointList.push_back(
                            PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                            CGAL::to_double(currLowerPoint[0][2])));
                }
            }

            {
                auto currUpperResult = CGAL::intersection(currUpperPlane_overlap, edgeCuttingPlaneList_overlap[0],
                                                          edgeCuttingPlaneList_overlap[edgeCuttingPlaneList.size() - 1]);

                if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                    upperPointList_overlap.push_back(
                            PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                            CGAL::to_double(currUpperPoint[0][2])));
                }

                auto currLowerResult = CGAL::intersection(currLowerPlane_overlap, edgeCuttingPlaneList_overlap[0],
                                                          edgeCuttingPlaneList_overlap[edgeCuttingPlaneList.size() - 1]);

                if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                    lowerPointList_overlap.push_back(
                            PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                            CGAL::to_double(currLowerPoint[0][2])));
                }
            }

            {
                auto currUpperResult = CGAL::intersection(currUpperPlane_overlap_tolerance, edgeCuttingPlaneList_overlap_tolerance[0],
                                                          edgeCuttingPlaneList_overlap_tolerance[edgeCuttingPlaneList.size() - 1]);

                if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                    upperPointList_overlap_tolerance.push_back(
                            PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                            CGAL::to_double(currUpperPoint[0][2])));
                }

                auto currLowerResult = CGAL::intersection(currLowerPlane_overlap_tolerance, edgeCuttingPlaneList_overlap_tolerance[0],
                                                          edgeCuttingPlaneList_overlap_tolerance[edgeCuttingPlaneList.size() - 1]);

                if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                    lowerPointList_overlap_tolerance.push_back(
                            PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                            CGAL::to_double(currLowerPoint[0][2])));
                }
            }
        }

        baseSurface->baseSurface.property(origUpperPointList, *f_it) = upperPointList;
        baseSurface->baseSurface.property(origLowerPointList, *f_it) = lowerPointList;

        vector<PolyMesh::VertexHandle> upperPointHandleList;
        vector<PolyMesh::VertexHandle> lowerPointHandleList;

        vector<PolyMesh::VertexHandle> upperPointHandleList_overlap;
        vector<PolyMesh::VertexHandle> lowerPointHandleList_overlap;

        vector<PolyMesh::VertexHandle> upperPointHandleList_overlap_tolerance;
        vector<PolyMesh::VertexHandle> lowerPointHandleList_overlap_tolerance;

        vector<PolyMesh::VertexHandle> blockUpperPointHandleList;
        vector<PolyMesh::VertexHandle> blockLowerPointHandleList;

        baseSurface->baseSurface.property(blockFaceHandleList, *f_it).clear();

        for (int i = 0; i < upperPointList.size(); ++i) {
            upperPointHandleList.push_back(myMesh.add_vertex(upperPointList[i]));
            lowerPointHandleList.push_back(myMesh.add_vertex(lowerPointList[i]));

            upperPointHandleList_overlap.push_back(currTempBlock_overlap.add_vertex(upperPointList_overlap[i]));
            lowerPointHandleList_overlap.push_back(currTempBlock_overlap.add_vertex(lowerPointList_overlap[i]));

            upperPointHandleList_overlap_tolerance.push_back(currTempBlock_overlap_tolerance.add_vertex(upperPointList_overlap_tolerance[i]));
            lowerPointHandleList_overlap_tolerance.push_back(currTempBlock_overlap_tolerance.add_vertex(lowerPointList_overlap_tolerance[i]));

            blockUpperPointHandleList.push_back(currTempBlock.add_vertex(upperPointList[i]));
            blockLowerPointHandleList.push_back(currTempBlock.add_vertex(lowerPointList[i]));
        }

        int pointListSize = lowerPointList.size();

        Vector3d currUpperStart;
        Vector3d currUpperEnd;

        if (pointListSize % 2 != 0)
        {
            currUpperStart = Vector3d(upperPointList[0][0], upperPointList[0][1], upperPointList[0][2]);

            int currEndIndex = pointListSize / 2;
            PolyMesh::Point currEndPoint = (upperPointList[currEndIndex] + upperPointList[currEndIndex + 1]) / 2;
            currUpperEnd = Vector3d(currEndPoint[0], currEndPoint[1], currEndPoint[2]);
        }
        else
        {
            currUpperStart = Vector3d((upperPointList[0][0] + upperPointList[1][0]) / 2.0f,
                                 (upperPointList[0][1] + upperPointList[1][1]) / 2.0f,
                                 (upperPointList[0][2] + upperPointList[1][2]) / 2.0f);

            int currEndIndex = pointListSize / 2;
            PolyMesh::Point currEndPoint = (upperPointList[currEndIndex] + upperPointList[currEndIndex + 1]) / 2;
            currUpperEnd = Vector3d(currEndPoint[0], currEndPoint[1], currEndPoint[2]);
        }

        Vector3d currLowerStart;
        Vector3d currLowerEnd;

        if (pointListSize % 2 != 0)
        {
            currLowerStart = Vector3d(lowerPointList[0][0], lowerPointList[0][1], lowerPointList[0][2]);

            int currEndIndex = pointListSize / 2;
            PolyMesh::Point currEndPoint = (lowerPointList[currEndIndex] + lowerPointList[currEndIndex + 1]) / 2;
            currLowerEnd = Vector3d(currEndPoint[0], currEndPoint[1], currEndPoint[2]);
        }
        else
        {
            currLowerStart = Vector3d((lowerPointList[0][0] + lowerPointList[1][0]) / 2.0f,
                                      (lowerPointList[0][1] + lowerPointList[1][1]) / 2.0f,
                                      (lowerPointList[0][2] + lowerPointList[1][2]) / 2.0f);

            int currEndIndex = pointListSize / 2;
            PolyMesh::Point currEndPoint = (lowerPointList[currEndIndex] + lowerPointList[currEndIndex + 1]) / 2;
            currLowerEnd = Vector3d(currEndPoint[0], currEndPoint[1], currEndPoint[2]);
        }

        for (int i = 1; i < upperPointList.size(); ++i)
        {
            vector<PolyMesh::VertexHandle> currFaceHandleList;

            currFaceHandleList.push_back(upperPointHandleList[i]);
            currFaceHandleList.push_back(upperPointHandleList[i - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[i]);

            vector<PolyMesh::VertexHandle> currFaceHandleList_overlap;

            currFaceHandleList_overlap.push_back(upperPointHandleList_overlap[i]);
            currFaceHandleList_overlap.push_back(upperPointHandleList_overlap[i - 1]);
            currFaceHandleList_overlap.push_back(lowerPointHandleList_overlap[i - 1]);
            currFaceHandleList_overlap.push_back(lowerPointHandleList_overlap[i]);

            vector<PolyMesh::VertexHandle> currFaceHandleList_overlap_tolerance;

            currFaceHandleList_overlap_tolerance.push_back(upperPointHandleList_overlap_tolerance[i]);
            currFaceHandleList_overlap_tolerance.push_back(upperPointHandleList_overlap_tolerance[i - 1]);
            currFaceHandleList_overlap_tolerance.push_back(lowerPointHandleList_overlap_tolerance[i - 1]);
            currFaceHandleList_overlap_tolerance.push_back(lowerPointHandleList_overlap_tolerance[i]);

            vector<PolyMesh::VertexHandle> currBlockFaceHandleList;

            currBlockFaceHandleList.push_back(blockUpperPointHandleList[i]);
            currBlockFaceHandleList.push_back(blockUpperPointHandleList[i - 1]);
            currBlockFaceHandleList.push_back(blockLowerPointHandleList[i - 1]);
            currBlockFaceHandleList.push_back(blockLowerPointHandleList[i]);

            auto currP = myMesh.add_face(currFaceHandleList);
            myMesh.property(shellFaceLabel, currP) = currFaceClusteringLabel;
            myMesh.property(origFaceHandle, currP) = *f_it;
            baseSurface->baseSurface.property(blockFaceHandleList, *f_it).push_back(currP);

            currTempBlock_overlap.add_face(currFaceHandleList_overlap);
            currTempBlock_overlap_tolerance.add_face(currFaceHandleList_overlap_tolerance);

            currTempBlock.add_face(currBlockFaceHandleList);
        }
        {
            vector<PolyMesh::VertexHandle> currFaceHandleList;

            currFaceHandleList.push_back(upperPointHandleList[0]);
            currFaceHandleList.push_back(upperPointHandleList[upperPointList.size() - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[upperPointList.size() - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[0]);

            vector<PolyMesh::VertexHandle> currFaceHandleList_overlap;

            currFaceHandleList_overlap.push_back(upperPointHandleList_overlap[0]);
            currFaceHandleList_overlap.push_back(upperPointHandleList_overlap[upperPointList.size() - 1]);
            currFaceHandleList_overlap.push_back(lowerPointHandleList_overlap[upperPointList.size() - 1]);
            currFaceHandleList_overlap.push_back(lowerPointHandleList_overlap[0]);

            vector<PolyMesh::VertexHandle> currFaceHandleList_overlap_tolerance;

            currFaceHandleList_overlap_tolerance.push_back(upperPointHandleList_overlap_tolerance[0]);
            currFaceHandleList_overlap_tolerance.push_back(upperPointHandleList_overlap_tolerance[upperPointList.size() - 1]);
            currFaceHandleList_overlap_tolerance.push_back(lowerPointHandleList_overlap_tolerance[upperPointList.size() - 1]);
            currFaceHandleList_overlap_tolerance.push_back(lowerPointHandleList_overlap_tolerance[0]);

            vector<PolyMesh::VertexHandle> currBlockFaceHandleList;

            currBlockFaceHandleList.push_back(blockUpperPointHandleList[0]);
            currBlockFaceHandleList.push_back(blockUpperPointHandleList[upperPointList.size() - 1]);
            currBlockFaceHandleList.push_back(blockLowerPointHandleList[upperPointList.size() - 1]);
            currBlockFaceHandleList.push_back(blockLowerPointHandleList[0]);

            auto currP = myMesh.add_face(currFaceHandleList);
            myMesh.property(shellFaceLabel, currP) = currFaceClusteringLabel;
            myMesh.property(origFaceHandle, currP) = *f_it;
            baseSurface->baseSurface.property(blockFaceHandleList, *f_it).push_back(currP);

            currTempBlock_overlap.add_face(currFaceHandleList_overlap);
            currTempBlock_overlap_tolerance.add_face(currFaceHandleList_overlap_tolerance);

            currTempBlock.add_face(currBlockFaceHandleList);
        }

        // Add upper face
        auto upperP = myMesh.add_face(upperPointHandleList);
        myMesh.property(shellFaceLabel, upperP) = currFaceClusteringLabel;
        myMesh.property(origFaceHandle, upperP) = *f_it;
        baseSurface->baseSurface.property(blockFaceHandleList, *f_it).push_back(upperP);

        currTempBlock_overlap.add_face(upperPointHandleList_overlap);
        currTempBlock_overlap_tolerance.add_face(upperPointHandleList_overlap_tolerance);

        currTempBlock.add_face(blockUpperPointHandleList);

        // Add lower face
        reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
        auto lowerP = myMesh.add_face(lowerPointHandleList);
        myMesh.property(shellFaceLabel, lowerP) = currFaceClusteringLabel;
        myMesh.property(origFaceHandle, lowerP) = *f_it;
        baseSurface->baseSurface.property(blockFaceHandleList, *f_it).push_back(lowerP);

        reverse(lowerPointHandleList_overlap.begin(), lowerPointHandleList_overlap.end());
        currTempBlock_overlap.add_face(lowerPointHandleList_overlap);

        reverse(lowerPointHandleList_overlap_tolerance.begin(), lowerPointHandleList_overlap_tolerance.end());
        currTempBlock_overlap_tolerance.add_face(lowerPointHandleList_overlap_tolerance);

        reverse(blockLowerPointHandleList.begin(),blockLowerPointHandleList.end());
        currTempBlock.add_face(blockLowerPointHandleList);

        // Push_back new shell block
        ShellBlock currShellBlock(*f_it, currSeqID, currFaceClusteringLabel, cutUpper, cutLower,
                                  lowerFaceNormal, upperFaceNormal,
                                  currLowerStart, currLowerEnd,
                                  currUpperStart, currUpperEnd);

        currShellBlock.origShellBlock = currTempBlock;
        currShellBlock.origShellBlockTri = currTempBlock;
        currShellBlock.origShellBlockTri.triangulate();

        currShellBlock.origShellBlock_overlap = currTempBlock_overlap;
        currShellBlock.origShellBlockTri_overlap = currTempBlock_overlap;
        currShellBlock.origShellBlockTri_overlap.triangulate();

        currShellBlock.origShellBlock_overlap_tolerance = currTempBlock_overlap_tolerance;
        currShellBlock.origShellBlockTri_overlap_tolerance = currTempBlock_overlap_tolerance;
        currShellBlock.origShellBlockTri_overlap_tolerance.triangulate();

        shellBlockList.push_back(currShellBlock);

        baseSurface->baseSurface.property(shellBlockPointer, *f_it) = shellBlockList.size() - 1;
    }

    shellStructure = myMesh;
    shellStructureTri = myMesh;
    shellStructureTri.triangulate();

    ClusteringLabelPropertyTransfer(&shellStructure, &shellStructureTri);

    cout << "Init Shell Structure Done. " << endl;
}

void ShellStructure::InitShellSupportSurface()
{
    UpdateTolerance();

    // Request Face Normal of baseSurface
    baseSurface->baseSurface.request_face_normals();
    baseSurface->baseSurface.update_normals();

    // Update edge normal
    baseSurface->UpdateEdgeNormal(&baseSurface->baseSurface, &baseSurface->triBaseSurface);

    // Create new mesh
    PolyMesh currFullSupport;
    PolyMesh currSupport;
    PolyMesh currSupport_tolerance;

    vector<PolyMesh::Point> addedVerList;

    // Get isBoundaryFace to each face
    OpenMesh::FPropHandleT< bool > isBoundaryFace;
    baseSurface->baseSurface.get_property_handle(isBoundaryFace, "isBoundaryFace");

    // Get normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface->baseSurface.get_property_handle( normal ,"normal");

    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        auto faceCenter = baseSurface->baseSurface.calc_face_centroid(*f_it);
        auto faceNormal = baseSurface->baseSurface.normal(*f_it);
        faceNormal.normalize();

        Point_3 middleFaceCenter_3(faceCenter[0], faceCenter[1], faceCenter[2]);
        Vector_3 middleFaceNormal_3(faceNormal[0], faceNormal[1], faceNormal[2]);

        Point_3 upperFaceCenter_3 = middleFaceCenter_3 + middleFaceNormal_3 * cutUpper;
        Vector_3 upperFaceNormal_3 = middleFaceNormal_3;

        Point_3 lowerFaceCenter_3 = middleFaceCenter_3 - middleFaceNormal_3 * cutLower;
        Vector_3 lowerFaceNormal_3 = -middleFaceNormal_3;
        Vector3d lowerFaceNormal(-faceNormal[0], -faceNormal[1], -faceNormal[2]);

        Point_3 lowerFaceCenter_3_tolerance = middleFaceCenter_3 - middleFaceNormal_3 * (cutLower + 4 * absoluteTolerance);
        Vector_3 lowerFaceNormal_3_tolerance = -middleFaceNormal_3;

        Plane_3 currMiddlePlane(middleFaceCenter_3, middleFaceNormal_3);
        Plane_3 currUpperPlane(upperFaceCenter_3, upperFaceNormal_3);
        Plane_3 currLowerPlane(lowerFaceCenter_3, lowerFaceNormal_3);

        Plane_3 currLowerPlane_tolerance(lowerFaceCenter_3_tolerance, lowerFaceNormal_3_tolerance);

        vector<PolyMesh::Point> currVerList;
        vector<PolyMesh::VertexHandle> currOrigVerHandle;

        PolyMesh::FaceVertexCCWIter fv_it;
        for (fv_it = baseSurface->baseSurface.fv_ccwiter(*f_it); fv_it.is_valid(); ++fv_it) {
            currVerList.push_back(baseSurface->baseSurface.point(*fv_it));
            currOrigVerHandle.push_back(*fv_it);
        }

        vector<PolyMesh::VertexHandle> currVerHandleList;

        for (int i = 0; i < currVerList.size(); ++i)
        {
            currVerHandleList.push_back(currFullSupport.add_vertex(currVerList[i]));
        }

        reverse(currVerHandleList.begin(), currVerHandleList.end());
        currFullSupport.add_face(currVerHandleList);

        if (baseSurface->baseSurface.property(isBoundaryFace, *f_it))
        {
            PolyMesh::FaceEdgeIter fe_it;

            vector< vector<PolyMesh::Point> > currAddedFaces;
            vector< vector<PolyMesh::Point> > currAddedFaces_tolerance;
            vector< vector<PolyMesh::Point> > currAddedFaces_full;

            vector<PolyMesh::EdgeHandle> currBoundaryEdgeHandleList;
            vector<vector<PolyMesh::VertexHandle>> vhLists;
            vector<PolyMesh::Point> currEdges;
            vector<PolyMesh::Point> currEdgeCenters;
            vector<Vector3d> currEdgeNormals;

            for (int i = 1; i < currOrigVerHandle.size(); ++i)
            {
                PolyMesh::EdgeHandle currEdge = FindEdge(baseSurface->baseSurface, currOrigVerHandle[i], currOrigVerHandle[i - 1]);

                if (baseSurface->baseSurface.is_boundary(currEdge))
                {
                    currBoundaryEdgeHandleList.push_back(currEdge);
                    currEdges.push_back((currVerList[i] - currVerList[i - 1]).normalize());
                    currEdgeCenters.push_back((currVerList[i] + currVerList[i - 1]) / 2);
                    currEdgeNormals.push_back(baseSurface->baseSurface.property(normal, currEdge));

                    vector<PolyMesh::VertexHandle> currVhList;

                    currVhList.push_back(currOrigVerHandle[i]);
                    currVhList.push_back(currOrigVerHandle[i - 1]);

                    vhLists.push_back(currVhList);
                }
            }
            {
                PolyMesh::EdgeHandle currEdge = FindEdge(baseSurface->baseSurface, currOrigVerHandle[0], currOrigVerHandle[currOrigVerHandle.size() - 1]);

                if (baseSurface->baseSurface.is_boundary(currEdge))
                {
                    currBoundaryEdgeHandleList.push_back(currEdge);
                    currEdges.push_back((currVerList[currVerList.size() - 1] - currVerList[0]).normalize());
                    currEdgeCenters.push_back((currVerList[currVerList.size() - 1] + currVerList[0]) / 2);
                    currEdgeNormals.push_back(baseSurface->baseSurface.property(normal, currEdge));

                    vector<PolyMesh::VertexHandle> currVhList;

                    currVhList.push_back(currOrigVerHandle[0]);
                    currVhList.push_back(currOrigVerHandle[currOrigVerHandle.size() - 1]);

                    vhLists.push_back(currVhList);
                }
            }

            for (int i = 0; i < currBoundaryEdgeHandleList.size(); ++i)
            {
                vector<PolyMesh::Point> currFace;
                vector<PolyMesh::Point> currFace_tolerance;
                vector<PolyMesh::Point> currFace_full;

                // Init edge cutting plane
                Point_3 currEdgeCenter_3(currEdgeCenters[i][0], currEdgeCenters[i][1], currEdgeCenters[i][2]);

                Vector_3 currEdge_3(currEdges[i][0], currEdges[i][1], currEdges[i][2]);
                Vector_3 currEdgeNormal_3(currEdgeNormals[i][0], currEdgeNormals[i][1], currEdgeNormals[i][2]);
                Vector_3 currPlaneVector_3 = CGAL::cross_product(currEdge_3, currEdgeNormal_3);

                Plane_3 currCuttingPlane(currEdgeCenter_3, currPlaneVector_3);

                // Find the first intersection line
                for (PolyMesh::VertexVertexIter vv_it = baseSurface->baseSurface.vv_iter(vhLists[i][0]); vv_it.is_valid(); ++vv_it) {
                    PolyMesh::EdgeHandle currEdge = FindEdge(baseSurface->baseSurface, vhLists[i][0], *vv_it);

                    if (baseSurface->baseSurface.is_boundary(currEdge) and currEdge != currBoundaryEdgeHandleList[i])
                    {
                        PolyMesh::Point currPoint_0 = baseSurface->baseSurface.point(*vv_it);
                        PolyMesh::Point currPoint_1 = baseSurface->baseSurface.point(vhLists[i][0]);

                        PolyMesh::Point currNeighbourEdge = (currPoint_0 - currPoint_1).normalize();

                        Vector3d firstEdgeNormal = baseSurface->baseSurface.property(normal, currEdge);

                        auto firstEdgeCenter = baseSurface->baseSurface.calc_edge_midpoint(currEdge);

                        // Init firstNeighbourCuttingPlane
                        Point_3 firstNeighbourEdgeCenter_3(firstEdgeCenter[0], firstEdgeCenter[1], firstEdgeCenter[2]);

                        Vector_3 firstEdge_3(currNeighbourEdge[0], currNeighbourEdge[1], currNeighbourEdge[2]);
                        Vector_3 firstEdgeNormal_3(firstEdgeNormal[0], firstEdgeNormal[1], firstEdgeNormal[2]);
                        Vector_3 firstPlaneVector_3 = CGAL::cross_product(firstEdge_3, firstEdgeNormal_3);

                        Plane_3 firstNeighbourCuttingPlane(firstNeighbourEdgeCenter_3, firstPlaneVector_3);

                        currFace_full.push_back(baseSurface->baseSurface.point(vhLists[i][0]));

                        // Find intersection point
                        {
                            auto currLowerResult = CGAL::intersection(currLowerPlane, currCuttingPlane,
                                                                      firstNeighbourCuttingPlane);

                            if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                                currFace.push_back(
                                        PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]),
                                                        CGAL::to_double(currLowerPoint[0][1]),
                                                        CGAL::to_double(currLowerPoint[0][2])));
                            }

                            auto currUpperResult = CGAL::intersection(currUpperPlane, currCuttingPlane,
                                                                      firstNeighbourCuttingPlane);

                            if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                                currFace.push_back(
                                        PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]),
                                                        CGAL::to_double(currUpperPoint[0][1]),
                                                        CGAL::to_double(currUpperPoint[0][2])));
//                            cout << "intersection. " << endl;
                                currFace_full.push_back(PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]),
                                                                        CGAL::to_double(currUpperPoint[0][1]),
                                                                        CGAL::to_double(currUpperPoint[0][2])));
                            }
                        }

                        // Find intersection point with tolerance
                        {
                            auto currLowerResult = CGAL::intersection(currLowerPlane_tolerance, currCuttingPlane,
                                                                      firstNeighbourCuttingPlane);

                            if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                                currFace_tolerance.push_back(
                                        PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]),
                                                        CGAL::to_double(currLowerPoint[0][1]),
                                                        CGAL::to_double(currLowerPoint[0][2])));
                            }

                            auto currUpperResult = CGAL::intersection(currUpperPlane, currCuttingPlane,
                                                                      firstNeighbourCuttingPlane);

                            if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                                currFace_tolerance.push_back(
                                        PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]),
                                                        CGAL::to_double(currUpperPoint[0][1]),
                                                        CGAL::to_double(currUpperPoint[0][2])));
                            }
                        }

                        break;
                    }
                }

                // Find the second intersection line
                for (PolyMesh::VertexVertexIter vv_it = baseSurface->baseSurface.vv_iter(vhLists[i][1]); vv_it.is_valid(); ++vv_it) {
                    PolyMesh::EdgeHandle currEdge = FindEdge(baseSurface->baseSurface, vhLists[i][1], *vv_it);

                    if (baseSurface->baseSurface.is_boundary(currEdge) and currEdge != currBoundaryEdgeHandleList[i])
                    {
                        PolyMesh::Point currPoint_0 = baseSurface->baseSurface.point(*vv_it);
                        PolyMesh::Point currPoint_1 = baseSurface->baseSurface.point(vhLists[i][1]);

                        PolyMesh::Point currNeighbourEdge = (currPoint_0 - currPoint_1).normalize();

                        Vector3d secondEdgeNormal = baseSurface->baseSurface.property(normal, currEdge);

                        auto secondEdgeCenter = baseSurface->baseSurface.calc_edge_midpoint(currEdge);

                        // Init secondNeighbourCuttingPlane
                        Point_3 secondNeighbourEdgeCenter_3(secondEdgeCenter[0], secondEdgeCenter[1], secondEdgeCenter[2]);

                        Vector_3 secondEdge_3(currNeighbourEdge[0], currNeighbourEdge[1], currNeighbourEdge[2]);
                        Vector_3 secondEdgeNormal_3(secondEdgeNormal[0], secondEdgeNormal[1], secondEdgeNormal[2]);
                        Vector_3 secondPlaneVector_3 = CGAL::cross_product(secondEdge_3, secondEdgeNormal_3);

                        Plane_3 secondNeighbourCuttingPlane(secondNeighbourEdgeCenter_3, secondPlaneVector_3);

                        // Find intersection point
                        {
                            auto currUpperResult = CGAL::intersection(currUpperPlane, currCuttingPlane,
                                                                      secondNeighbourCuttingPlane);

                            if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                                currFace.push_back(
                                        PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                                        CGAL::to_double(currUpperPoint[0][2])));

                                currFace_full.push_back(PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                                                        CGAL::to_double(currUpperPoint[0][2])));
//                            cout << "intersection. " << endl;
                            }

                            auto currLowerResult = CGAL::intersection(currLowerPlane, currCuttingPlane,
                                                                      secondNeighbourCuttingPlane);

                            if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                                currFace.push_back(
                                        PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]),
                                                        CGAL::to_double(currLowerPoint[0][1]),
                                                        CGAL::to_double(currLowerPoint[0][2])));
                            }

                            currFace_full.push_back(baseSurface->baseSurface.point(vhLists[i][1]));
                        }

                        // Find intersection point with tolerance
                        {
                            auto currUpperResult = CGAL::intersection(currUpperPlane, currCuttingPlane,
                                                                      secondNeighbourCuttingPlane);

                            if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
                                currFace_tolerance.push_back(
                                        PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                                        CGAL::to_double(currUpperPoint[0][2])));
                            }

                            auto currLowerResult = CGAL::intersection(currLowerPlane_tolerance, currCuttingPlane,
                                                                      secondNeighbourCuttingPlane);

                            if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                                currFace_tolerance.push_back(
                                        PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]),
                                                        CGAL::to_double(currLowerPoint[0][1]),
                                                        CGAL::to_double(currLowerPoint[0][2])));
                            }
                        }

                        break;
                    }
                }

                currAddedFaces.push_back(currFace);
                currAddedFaces_tolerance.push_back(currFace_tolerance);
                currAddedFaces_full.push_back(currFace_full);

//                dbg(currFace.size());
            }

            for (int i = 0; i < currAddedFaces.size(); ++i)
            {
                vector<PolyMesh::VertexHandle> currAddedVerHandleList;
                vector<PolyMesh::VertexHandle> currAddedVerHandleList_tolerance;
                vector<PolyMesh::VertexHandle> currSupportVerHandleList;

                for (int j = 0; j < currAddedFaces[i].size(); ++j)
                {
                    currAddedVerHandleList.push_back(currFullSupport.add_vertex(currAddedFaces_full[i][j]));
                    currSupportVerHandleList.push_back(currSupport.add_vertex(currAddedFaces[i][j]));
                    currAddedVerHandleList_tolerance.push_back(currSupport_tolerance.add_vertex(currAddedFaces_tolerance[i][j]));
                }

                currFullSupport.add_face(currAddedVerHandleList);
                currSupport.add_face(currSupportVerHandleList);
                currSupport_tolerance.add_face(currAddedVerHandleList_tolerance);
            }
        }
    }

    fullSupportSurface = currFullSupport;
    fullSupportSurfaceTri = currFullSupport;
    fullSupportSurfaceTri.triangulate();

    supportSurface = currSupport;
    supportSurfaceTri = currSupport;
    supportSurfaceTri.triangulate();

    supportSurface_tolerance = currSupport_tolerance;
    supportSurfaceTri_tolerance = currSupport_tolerance;
    supportSurfaceTri_tolerance.triangulate();
}

void ShellStructure::InitReplacedShellStructure_LSM()
{
    /// Get properties
    // Get centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList;
    baseSurface->baseSurface.get_property_handle(centroidBlockFaceHandleList, "centroidBlockFaceHandleList");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    baseSurface->baseSurface.get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get upperPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > upperPointList;
    baseSurface->baseSurface.get_property_handle(upperPointList, "upperPointList");

    // Get lowerPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > lowerPointList;
    baseSurface->baseSurface.get_property_handle(lowerPointList, "lowerPointList");

    // Get blockCentroidList to mesh
    OpenMesh::MPropHandleT< vector<tuple<vector<Vector3d>, vector<Vector3d>>> > blockCentroidList;
    baseSurface->baseSurface.get_property_handle(blockCentroidList, "blockCentroidList");

    // Get blockAlignedVerList to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > blockAlignedVerList;
    baseSurface->baseSurface.get_property_handle(blockAlignedVerList, "blockAlignedVerList");

    // Get replacedBlock to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > replacedBlock;
    baseSurface->baseSurface.get_property_handle(replacedBlock, "replacedBlock");

    // Get shellBlockPointer to each face
    OpenMesh::FPropHandleT< int > shellBlockPointer;
    baseSurface->baseSurface.get_property_handle(shellBlockPointer, "shellBlockPointer");

    // Create new mesh
    PolyMesh myMesh;

    /// Add properties
    // Add clusterLabel to each face
    OpenMesh::FPropHandleT< int > shellFaceLabel;
    myMesh.add_property(shellFaceLabel, "clusteringLabel");

    // Add clusterLabel to each face
    OpenMesh::FPropHandleT< PolyMesh::FaceHandle > origFaceHandle;
    myMesh.add_property(origFaceHandle, "origFaceHandle");

    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        int currFaceClusteringLabel = baseSurface->baseSurface.property(blockClusteringLabel, *f_it);

        // Get currRefer
        vector<Vector3d> referUpperList = PolyMeshPointList2EigenVector3dList(baseSurface->baseSurface.property(upperPointList, *f_it));
        vector<Vector3d> referLowerList = PolyMeshPointList2EigenVector3dList(baseSurface->baseSurface.property(lowerPointList, *f_it));
        vector<Vector3d> referBlockVerList = CombineTwoBlockVerLists(referUpperList, referLowerList);

        // Get curr
        vector<Vector3d> currUpperList = get<0>(baseSurface->baseSurface.property(blockCentroidList)[currFaceClusteringLabel]);
        vector<Vector3d> currLowerList = get<1>(baseSurface->baseSurface.property(blockCentroidList)[currFaceClusteringLabel]);

        double simi, simi_1, simi_2, simi_3, simi_4;

        MatrixXd p, p_reversed, p_flip_reversed, p_flip, p_orig;

        // 1) first config
//        cout << "Start aligning. " << endl;
        baseSurface->AlignBlock_LSM(referBlockVerList, currUpperList, currLowerList, p_orig);

        simi_1 = 0;
        for (int i = 0; i < p_orig.cols(); ++i)
        {
            simi_1 += (p_orig.col(i) - referBlockVerList[i]).norm();
        }
        simi = simi_1;
//        p = p_orig;

        // 2) second config : reversed
//        reverse(currUpperList.begin(), currUpperList.end());
//        reverse(currLowerList.begin(), currLowerList.end());
//        baseSurface->AlignBlock_LSM(referBlockVerList, currUpperList, currLowerList, p_reversed);
//
//        simi_2 = 10;
//        for (int i = 0; i < p_reversed.cols(); ++i)
//        {
//            simi_2 += (p_reversed.col(i) - referBlockVerList[i]).norm();
//        }
//
//        if (simi_2 < simi)
//        {
//            cout << "Reversed block to align. " << endl;
////            p = p_flip;
//            simi = simi_2;
//        }
//
//        // 3) third config : reversed-flip
//        baseSurface->AlignBlock_LSM(referBlockVerList, currLowerList, currUpperList, p_flip_reversed);
//
//        simi_3 = 10;
//        for (int i = 0; i < p_flip_reversed.cols(); ++i)
//        {
//            simi_3 += (p_flip_reversed.col(i) - referBlockVerList[i]).norm();
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
//        baseSurface->AlignBlock_LSM(referBlockVerList, currLowerList, currUpperList, p_flip);
//
//        simi_4 = 10;
//        for (int i = 0; i < p_flip.cols(); ++i)
//        {
//            simi_4 += (p_flip.col(i) - referBlockVerList[i]).norm();
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

//        if (simi != simi_1)
//            cout << "Flip or reverse block to align. " << endl << endl;

        tuple< vector<Vector3d>, vector<Vector3d> > currBlockTuple;

        // Push back the aligned vertex list
        vector<Vector3d> currAlignedList;
        for (int i = 0; i < p.cols(); ++i)
        {
            currAlignedList.push_back(Vector3d(p(0, i), p(1, i), p(2, i)));
        }

        SplitTwoVector(currAlignedList, get<0>(currBlockTuple), get<1>(currBlockTuple));

//        if (simi == simi_1 or simi == simi_2)
//            SplitTwoVector(currAlignedList, get<0>(currBlockTuple), get<1>(currBlockTuple));
//        else
//            SplitTwoVector(currAlignedList, get<1>(currBlockTuple), get<0>(currBlockTuple));

        vector<PolyMesh::VertexHandle> upperPointHandleList;
        vector<PolyMesh::VertexHandle> lowerPointHandleList;

        baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).clear();

        for (int i = 0; i < get<0>(currBlockTuple).size(); ++i) {
            upperPointHandleList.push_back(myMesh.add_vertex(EigenVector3d2PolyMeshPoint(get<0>(currBlockTuple)[i])));
            lowerPointHandleList.push_back(myMesh.add_vertex(EigenVector3d2PolyMeshPoint(get<1>(currBlockTuple)[i])));
        }

        for (int i = 1; i < get<0>(currBlockTuple).size(); ++i)
        {
            vector<PolyMesh::VertexHandle> currFaceHandleList;

            currFaceHandleList.push_back(upperPointHandleList[i]);
            currFaceHandleList.push_back(upperPointHandleList[i - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[i]);

            auto currH = myMesh.add_face(currFaceHandleList);
            myMesh.property(origFaceHandle, currH) = *f_it;
            myMesh.property(shellFaceLabel, currH) = currFaceClusteringLabel;
            baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).push_back(currH);
        }
        {
            vector<PolyMesh::VertexHandle> currFaceHandleList;

            currFaceHandleList.push_back(upperPointHandleList[0]);
            currFaceHandleList.push_back(upperPointHandleList[get<0>(currBlockTuple).size() - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[get<0>(currBlockTuple).size() - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[0]);

            auto currH = myMesh.add_face(currFaceHandleList);
            myMesh.property(origFaceHandle, currH) = *f_it;
            myMesh.property(shellFaceLabel, currH) = currFaceClusteringLabel;
            baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).push_back(currH);
        }

        // Add upper face
        auto upperP = myMesh.add_face(upperPointHandleList);
        myMesh.property(shellFaceLabel, upperP) = currFaceClusteringLabel;
        myMesh.property(origFaceHandle, upperP) = *f_it;
        baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).push_back(upperP);

        // Add lower face
        reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
        auto lowerP = myMesh.add_face(lowerPointHandleList);
        myMesh.property(shellFaceLabel, lowerP) = currFaceClusteringLabel;
        myMesh.property(origFaceHandle, lowerP) = *f_it;
        baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).push_back(lowerP);
    }

    replacedShellStructure_LSM = myMesh;
    replacedShellStructureTri_LSM = myMesh;
    replacedShellStructureTri_LSM.triangulate();

    ClusteringLabelPropertyTransfer(&replacedShellStructure_LSM, &replacedShellStructureTri_LSM);

    cout << "Compute Math Block Centroid Done. " << endl;
}

void ShellStructure::InitReplacedShellStructure_LSM_planar() {

    UpdateTolerance();

    /// Get properties
    // Get upperPointList_planar to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > upperPointList_planar;
    baseSurface->baseSurface.get_property_handle(upperPointList_planar, "upperPointList_planar");

    // Get lowerPointList_planar to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > lowerPointList_planar;
    baseSurface->baseSurface.get_property_handle(lowerPointList_planar, "lowerPointList_planar");

    // Get blockCentroidList_planar to mesh
    OpenMesh::MPropHandleT< vector<tuple<vector<Vector3d>, vector<Vector3d>>> > blockCentroidList_planar;
    baseSurface->baseSurface.get_property_handle(blockCentroidList_planar, "blockCentroidList_planar");

    // Get centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList_planar;
    baseSurface->baseSurface.get_property_handle(centroidBlockFaceHandleList_planar, "centroidBlockFaceHandleList_planar");

    // Get centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList;
    baseSurface->baseSurface.get_property_handle(centroidBlockFaceHandleList, "centroidBlockFaceHandleList");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT<int> blockClusteringLabel;
    baseSurface->baseSurface.get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get upperPointList to each face
    OpenMesh::FPropHandleT<vector<PolyMesh::Point> > upperPointList;
    baseSurface->baseSurface.get_property_handle(upperPointList, "upperPointList");

    // Get lowerPointList to each face
    OpenMesh::FPropHandleT<vector<PolyMesh::Point> > lowerPointList;
    baseSurface->baseSurface.get_property_handle(lowerPointList, "lowerPointList");

    // Get blockCentroidList to mesh
    OpenMesh::MPropHandleT<vector<tuple<vector<Vector3d>, vector<Vector3d>>> > blockCentroidList;
    baseSurface->baseSurface.get_property_handle(blockCentroidList, "blockCentroidList");

    // Get blockAlignedVerList to each face
    OpenMesh::FPropHandleT<tuple<vector<Vector3d>, vector<Vector3d>>> blockAlignedVerList;
    baseSurface->baseSurface.get_property_handle(blockAlignedVerList, "blockAlignedVerList");

    // Get replacedBlock to each face
    OpenMesh::FPropHandleT<tuple<vector<Vector3d>, vector<Vector3d>>> replacedBlock;
    baseSurface->baseSurface.get_property_handle(replacedBlock, "replacedBlock");

    // Get shellBlockPointer to each face
    OpenMesh::FPropHandleT< int > shellBlockPointer;
    baseSurface->baseSurface.get_property_handle(shellBlockPointer, "shellBlockPointer");

    // Create new mesh
    PolyMesh myMesh;

    /// Add properties

    // Add clusterLabel to each face
    OpenMesh::FPropHandleT<int> shellFaceLabel;
    myMesh.add_property(shellFaceLabel, "clusteringLabel");

    // Add clusterLabel to each face
    OpenMesh::FPropHandleT<PolyMesh::FaceHandle> origFaceHandle;
    myMesh.add_property(origFaceHandle, "origFaceHandle");

    PolyMesh tempMesh;

    /// Request Face Normal
    replacedShellStructure_LSM.request_face_normals();
    replacedShellStructure_LSM.update_normals();

    for (PolyMesh::FaceIter f_it = baseSurface->baseSurface.faces_begin();
        f_it != baseSurface->baseSurface.faces_end(); ++f_it) {

        baseSurface->baseSurface.property(upperPointList_planar, *f_it).clear();
        baseSurface->baseSurface.property(lowerPointList_planar, *f_it).clear();

        int currFaceClusteringLabel = baseSurface->baseSurface.property(blockClusteringLabel, *f_it);

        PolyMesh currTempBlock_replaced;

        // Compute block centroid
        vector<Vector3d> currFaceCentroidList;
        vector<Vector3d> currFaceNormalList;
        Vector3d currCentroid(0, 0, 0);
        for (int i = 0; i < baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).size(); ++i)
        {
            auto currCenter = replacedShellStructure_LSM.calc_face_centroid(baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it)[i]);
            auto currNormal = replacedShellStructure_LSM.normal(baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it)[i]);

            currCentroid = currCentroid + Vector3d(currCenter[0], currCenter[1], currCenter[2]);

            currFaceCentroidList.push_back(Vector3d(currCenter[0], currCenter[1], currCenter[2]));
            currFaceNormalList.push_back(Vector3d(currNormal[0], currNormal[1], currNormal[2]));
        }
        currCentroid = currCentroid / double(baseSurface->baseSurface.property(centroidBlockFaceHandleList, *f_it).size());

        Point_3 upperFaceCenter_3(currFaceCentroidList[currFaceCentroidList.size() - 2][0],
                                  currFaceCentroidList[currFaceCentroidList.size() - 2][1],
                                  currFaceCentroidList[currFaceCentroidList.size() - 2][2]);
        Vector_3 upperFaceNormal_3(currFaceNormalList[currFaceNormalList.size() - 2][0],
                                   currFaceNormalList[currFaceNormalList.size() - 2][1],
                                   currFaceNormalList[currFaceNormalList.size() - 2][2]);
        Plane_3 currUpperPlane_3(upperFaceCenter_3, upperFaceNormal_3);

        Point_3 lowerFaceCenter_3(currFaceCentroidList[currFaceCentroidList.size() - 1][0],
                                  currFaceCentroidList[currFaceCentroidList.size() - 1][1],
                                  currFaceCentroidList[currFaceCentroidList.size() - 1][2]);
        Vector_3 lowerFaceNormal_3(currFaceNormalList[currFaceNormalList.size() - 1][0],
                                   currFaceNormalList[currFaceNormalList.size() - 1][1],
                                   currFaceNormalList[currFaceNormalList.size() - 1][2]);
        Plane_3 currLowerPlane_3(lowerFaceCenter_3, lowerFaceNormal_3);

        vector<Plane_3> currCuttingPlaneList;

        for (int i = 0; i < currFaceCentroidList.size() - 2; ++i)
        {
            Vector3d currMoveVec(currCentroid[0] - currFaceCentroidList[i][0],
                                   currCentroid[1] - currFaceCentroidList[i][1],
                                   currCentroid[2] - currFaceCentroidList[i][2]);
            currMoveVec.normalize();

            Vector_3 currMoveVec_3(currMoveVec[0], currMoveVec[1], currMoveVec[2]);

            Point_3 currFaceCenter_3(currFaceCentroidList[i][0], currFaceCentroidList[i][1], currFaceCentroidList[i][2]);

            currFaceCenter_3 = currFaceCenter_3 + currMoveVec_3 * absoluteTolerance * 0.4;

            Vector_3 currFaceNormal_3(currFaceNormalList[i][0], currFaceNormalList[i][1], currFaceNormalList[i][2]);

            Plane_3 currCuttingPlane_3(currFaceCenter_3, currFaceNormal_3);

            currCuttingPlaneList.push_back(currCuttingPlane_3);
        }

        vector<PolyMesh::Point> currUpperPointList;
        vector<PolyMesh::Point> currLowerPointList;

        for (int i = 1; i < currCuttingPlaneList.size(); ++i) {
            auto currUpperResult = CGAL::intersection(currUpperPlane_3, currCuttingPlaneList[i],
                                                      currCuttingPlaneList[i - 1]);

            if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
//                std::cout << currUpperPoint[0][0] << currUpperPoint[0][1] << currUpperPoint[0][2] << std::endl;
                currUpperPointList.push_back(
                        PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                        CGAL::to_double(currUpperPoint[0][2])));
            }

            auto currLowerResult = CGAL::intersection(currLowerPlane_3, currCuttingPlaneList[i],
                                                      currCuttingPlaneList[i - 1]);

            if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                currLowerPointList.push_back(
                        PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                        CGAL::to_double(currLowerPoint[0][2])));
            }
        }
        {
            auto currUpperResult = CGAL::intersection(currUpperPlane_3, currCuttingPlaneList[0],
                                                      currCuttingPlaneList[currCuttingPlaneList.size() - 1]);

            if (const auto currUpperPoint = boost::get<Point_3>(&*currUpperResult)) {
//                std::cout << currUpperPoint[0][0] << currUpperPoint[0][1] << currUpperPoint[0][2] << std::endl;
                currUpperPointList.push_back(
                        PolyMesh::Point(CGAL::to_double(currUpperPoint[0][0]), CGAL::to_double(currUpperPoint[0][1]),
                                        CGAL::to_double(currUpperPoint[0][2])));
            }

            auto currLowerResult = CGAL::intersection(currLowerPlane_3, currCuttingPlaneList[0],
                                                      currCuttingPlaneList[currCuttingPlaneList.size() - 1]);

            if (const auto currLowerPoint = boost::get<Point_3>(&*currLowerResult)) {
                currLowerPointList.push_back(
                        PolyMesh::Point(CGAL::to_double(currLowerPoint[0][0]), CGAL::to_double(currLowerPoint[0][1]),
                                        CGAL::to_double(currLowerPoint[0][2])));
            }
        }

        for (int i = 0; i < currUpperPointList.size(); ++i)
        {
            baseSurface->baseSurface.property(upperPointList_planar, *f_it).push_back(Vector3d(currUpperPointList[i][0], currUpperPointList[i][1], currUpperPointList[i][2]));
            baseSurface->baseSurface.property(lowerPointList_planar, *f_it).push_back(Vector3d(currLowerPointList[i][0], currLowerPointList[i][1], currLowerPointList[i][2]));
        }

        vector<PolyMesh::VertexHandle> upperPointHandleList;
        vector<PolyMesh::VertexHandle> lowerPointHandleList;

        vector<PolyMesh::VertexHandle> upperPointHandleList_replaced;
        vector<PolyMesh::VertexHandle> lowerPointHandleList_replaced;

        baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, *f_it).clear();

        for (int i = 0; i < currUpperPointList.size(); ++i) {
            upperPointHandleList.push_back(myMesh.add_vertex(currUpperPointList[i]));
            lowerPointHandleList.push_back(myMesh.add_vertex(currLowerPointList[i]));

            upperPointHandleList_replaced.push_back(currTempBlock_replaced.add_vertex(currUpperPointList[i]));
            lowerPointHandleList_replaced.push_back(currTempBlock_replaced.add_vertex(currLowerPointList[i]));
        }

        for (int i = 1; i < currUpperPointList.size(); ++i)
        {
            vector<PolyMesh::VertexHandle> currFaceHandleList;

            currFaceHandleList.push_back(upperPointHandleList[i]);
            currFaceHandleList.push_back(upperPointHandleList[i - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[i]);

            vector<PolyMesh::VertexHandle> currFaceHandleList_replaced;

            currFaceHandleList_replaced.push_back(upperPointHandleList_replaced[i]);
            currFaceHandleList_replaced.push_back(upperPointHandleList_replaced[i - 1]);
            currFaceHandleList_replaced.push_back(lowerPointHandleList_replaced[i - 1]);
            currFaceHandleList_replaced.push_back(lowerPointHandleList_replaced[i]);

            auto currH = myMesh.add_face(currFaceHandleList);
            myMesh.property(shellFaceLabel, currH) = currFaceClusteringLabel;
            myMesh.property(origFaceHandle, currH) = *f_it;
            baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, *f_it).push_back(currH);

            currTempBlock_replaced.add_face(currFaceHandleList_replaced);
        }
        {
            vector<PolyMesh::VertexHandle> currFaceHandleList;

            currFaceHandleList.push_back(upperPointHandleList[0]);
            currFaceHandleList.push_back(upperPointHandleList[currUpperPointList.size() - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[currUpperPointList.size() - 1]);
            currFaceHandleList.push_back(lowerPointHandleList[0]);

            vector<PolyMesh::VertexHandle> currFaceHandleList_replaced;

            currFaceHandleList_replaced.push_back(upperPointHandleList_replaced[0]);
            currFaceHandleList_replaced.push_back(upperPointHandleList_replaced[currUpperPointList.size() - 1]);
            currFaceHandleList_replaced.push_back(lowerPointHandleList_replaced[currUpperPointList.size() - 1]);
            currFaceHandleList_replaced.push_back(lowerPointHandleList_replaced[0]);

            auto currH = myMesh.add_face(currFaceHandleList);
            myMesh.property(shellFaceLabel, currH) = currFaceClusteringLabel;
            myMesh.property(origFaceHandle, currH) = *f_it;
            baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, *f_it).push_back(currH);

            currTempBlock_replaced.add_face(currFaceHandleList_replaced);
        }

        // Add upper face
        auto upperP = myMesh.add_face(upperPointHandleList);
        myMesh.property(shellFaceLabel, upperP) = currFaceClusteringLabel;
        myMesh.property(origFaceHandle, upperP) = *f_it;
        baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, *f_it).push_back(upperP);

        currTempBlock_replaced.add_face(upperPointHandleList_replaced);

        // Add lower face
        reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
        auto lowerP = myMesh.add_face(lowerPointHandleList);
        myMesh.property(shellFaceLabel, lowerP) = currFaceClusteringLabel;
        myMesh.property(origFaceHandle, lowerP) = *f_it;
        baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, *f_it).push_back(lowerP);

        reverse(lowerPointHandleList_replaced.begin(), lowerPointHandleList_replaced.end());
        currTempBlock_replaced.add_face(lowerPointHandleList_replaced);

        int currShellBlockPointer = baseSurface->baseSurface.property(shellBlockPointer, *f_it);

        shellBlockList[currShellBlockPointer].replacedShellBlock = currTempBlock_replaced;
        shellBlockList[currShellBlockPointer].replacedShellBlockTri = currTempBlock_replaced;
        shellBlockList[currShellBlockPointer].replacedShellBlockTri.triangulate();
    }

    replacedShellStructure_LSM_planar = myMesh;
    replacedShellStructureTri_LSM_planar = myMesh;
    replacedShellStructureTri_LSM_planar.triangulate();

    ClusteringLabelPropertyTransfer(&replacedShellStructure_LSM_planar, &replacedShellStructureTri_LSM_planar);

    cout << "Compute Planar Block Centroid Done. " << endl;
}

void ShellStructure::UpdateTolerance()
{
    baseSurface->InitToleranceParameters();
    absoluteTolerance = baseSurface->absoluteTolerance;
}




//**************************************************************************************//
//                                  Support Generation
//**************************************************************************************//

void ShellStructure::GenerateSupportForOrigShellBlocks(string folderPath)
{
    // Find the folderPath
    if (folderPath[folderPath.size() - 1] == '/')
    {
        folderPath.erase(folderPath.begin() + folderPath.size() - 1);
    }

    size_t foundFolder = folderPath.find_last_of('/'); // Note: this applies for Mac only
    string savingFolderPath = folderPath.substr(0, foundFolder);

    PolyMesh arcSupport, tempSupport, optimizedShell_overlap, optimizedShell_overlap_tolerance, arcSupport_tolerance, arcSupport_expand;

    OpenMesh::IO::read_mesh(arcSupport, savingFolderPath + "/Support/ArcSupport.obj" );
    OpenMesh::IO::read_mesh(tempSupport, savingFolderPath + "/Support/TempSupport.obj" );
    OpenMesh::IO::read_mesh(optimizedShell_overlap, savingFolderPath + "/OptimizedShellStructure_overlap.obj" );
    OpenMesh::IO::read_mesh(optimizedShell_overlap_tolerance, savingFolderPath + "/OptimizedShellStructure_overlap_tolerance.obj" );
    OpenMesh::IO::read_mesh(arcSupport_tolerance, savingFolderPath + "/Support/ArcSupport_tolerance.obj" );
    OpenMesh::IO::read_mesh(arcSupport_expand, savingFolderPath + "/Support/ArcSupport_expand.obj" );

    MatrixXd V_arcSupport, V_tempSupport, V_overlap, V_overlap_tolerance, V_arcSupport_tolerance, V_arcSupport_expand;
    MatrixXi F_arcSupport, F_tempSupport, F_overlap, F_overlap_tolerance, F_arcSupport_tolerance, F_arcSupport_expand;

    PolyMesh2Matrix(arcSupport, V_arcSupport, F_arcSupport);
    PolyMesh2Matrix(tempSupport, V_tempSupport, F_tempSupport);
    PolyMesh2Matrix(optimizedShell_overlap, V_overlap, F_overlap);
    PolyMesh2Matrix(optimizedShell_overlap_tolerance, V_overlap_tolerance, F_overlap_tolerance);
    PolyMesh2Matrix(arcSupport_tolerance, V_arcSupport_tolerance, F_arcSupport_tolerance);
    PolyMesh2Matrix(arcSupport_expand, V_arcSupport_expand, F_arcSupport_expand);

    MatrixXd V_arcSupport_csg, V_arcSupport_csg_1, V_tempSupport_csg_1, V_tempSupport_csg_2;
    MatrixXi F_arcSupport_csg, F_arcSupport_csg_1, F_tempSupport_csg_1, F_tempSupport_csg_2;

    MeshBoolean meshBoolean;

    // ArcSupport
    meshBoolean.MeshUnion(V_arcSupport, F_arcSupport, V_arcSupport_expand, F_arcSupport_expand, V_arcSupport_csg_1, F_arcSupport_csg_1);
    meshBoolean.MeshMinus(V_arcSupport_csg_1, F_arcSupport_csg_1, V_overlap, F_overlap, V_arcSupport_csg, F_arcSupport_csg);

    // TempSupport
    meshBoolean.MeshMinus(V_tempSupport, F_tempSupport, V_overlap_tolerance, F_overlap_tolerance, V_tempSupport_csg_1, F_tempSupport_csg_1);
    meshBoolean.MeshMinus(V_tempSupport_csg_1, F_tempSupport_csg_1, V_arcSupport_tolerance, F_arcSupport_tolerance, V_tempSupport_csg_2, F_tempSupport_csg_2);

    PolyMesh arcSupport_csg;
    PolyMesh tempSupport_csg;

    Matrix2PolyMesh(arcSupport_csg, V_arcSupport_csg, F_arcSupport_csg);
    Matrix2PolyMesh(tempSupport_csg, V_tempSupport_csg_2, F_tempSupport_csg_2);

    // Save files
    string fullPath = savingFolderPath + "/Support_csg";

    string command;
    command = "mkdir -p " + fullPath;
    system(command.c_str());

    OpenMesh::IO::write_mesh(arcSupport_csg, fullPath + "/ArcSupport_csg.obj");
    OpenMesh::IO::write_mesh(tempSupport_csg, fullPath + "/TempSupport_csg.obj");
}




//**************************************************************************************//
//                                  Result Analysis
//**************************************************************************************//

void ShellStructure::WorstCaseAnalysis()
{
    ComputeReplacedPenetrationVolumeTri();

    ComputeWorstBlockSimilarityError();

    ComputeWorstContactAngleError();
}

void ShellStructure::ComputeReplacedPenetrationVolumeTri()
{
    // Get centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList_planar;
    baseSurface->baseSurface.get_property_handle(centroidBlockFaceHandleList_planar, "centroidBlockFaceHandleList_planar");

    PolyMesh myMesh;

    double avgBlockVolume = 0;

    penetrationVolumeSum = 0;

    worstOverlappedVolumeBlockRatio = 0;

    gapVolumeSum = 0;

    worstGapVolumeRatio = 0;

    double totalRatio = 0;

    double totalRatio_gap = 0;

    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        // Get refer V and F
        MatrixXd V_refer;
        MatrixXi F_refer;

        GetReplacedBlockMatrixTri(*f_it, V_refer, F_refer);

        avgBlockVolume += ComputeVolumeOfMesh(V_refer, F_refer);
    }

    avgBlockVolume = avgBlockVolume / double(baseSurface->baseSurface.n_faces());

//    cout << "avgBlockVolume: " << avgBlockVolume << endl;

    for (PolyMesh::EdgeIter e_it = baseSurface->baseSurface.edges_begin(); e_it!= baseSurface->baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface->baseSurface.is_boundary(*e_it))
            continue;

        // Get face pair
        PolyMesh::FaceHandle fh_1 = baseSurface->baseSurface.face_handle(baseSurface->baseSurface.halfedge_handle(*e_it, 0));
        PolyMesh::FaceHandle fh_2 = baseSurface->baseSurface.face_handle(baseSurface->baseSurface.opposite_halfedge_handle(baseSurface->baseSurface.halfedge_handle(*e_it, 0)));

        // Get refer V and F
        MatrixXd V_refer;
        MatrixXi F_refer;

        GetReplacedBlockMatrixTri(fh_1, V_refer, F_refer);

        // Get curr V and F
        MatrixXd V_curr, V_intersection;
        MatrixXi F_curr, F_intersection;

        GetReplacedBlockMatrixTri(fh_2, V_curr, F_curr);

        // CSG operation
        MeshBoolean myMeshBoolean;

        myMeshBoolean.MeshIntersect(V_refer, F_refer, V_curr, F_curr, V_intersection, F_intersection);

        if (V_intersection.rows() != 0
            and !IsTwoBlockMatrixEqual(V_intersection, V_refer)
            and !IsTwoBlockMatrixEqual(V_intersection, V_curr)) {
            vector<PolyMesh::VertexHandle> currVerHandleList;

            for (int j = 0; j < V_intersection.rows(); ++j) {
                currVerHandleList.push_back(myMesh.add_vertex(
                        PolyMesh::Point(V_intersection(j, 0), V_intersection(j, 1), V_intersection(j, 2))));
            }

            double currVolume = ComputeVolumeOfMesh(V_intersection, F_intersection);

            penetrationVolumeSum += currVolume;

            float currRatio = currVolume / avgBlockVolume;

            totalRatio += currRatio;

            if (currRatio > worstOverlappedVolumeBlockRatio) {
                worstOverlappedVolumeBlockRatio = currRatio;
//                    cout << "worstOverlappedVolumeBlockRatio: " << currRatio << endl << endl;
            }

            for (int j = 0; j < F_intersection.rows(); ++j) {
                vector<PolyMesh::VertexHandle> currFaceVHList;

                currFaceVHList.push_back(currVerHandleList[F_intersection(j, 0)]);
                currFaceVHList.push_back(currVerHandleList[F_intersection(j, 1)]);
                currFaceVHList.push_back(currVerHandleList[F_intersection(j, 2)]);

                myMesh.add_face(currFaceVHList);
            }
        }
        else
        {
            // Compute gap volume
            const PolyMesh::Point t   = baseSurface->baseSurface.point(baseSurface->baseSurface.to_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));
            const PolyMesh::Point f = baseSurface->baseSurface.point(baseSurface->baseSurface.from_vertex_handle(baseSurface->baseSurface.halfedge_handle(*e_it,0)));

            Vector3d to(t[0], t[1], t[2]);
            Vector3d from(f[0], f[1], f[2]);

            PolyMesh::FaceHandle upperFH_1, upperFH_2, lowerFH_1, lowerFH_2;

            upperFH_1 = baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_1)[baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_1).size() - 2];
            lowerFH_1 = baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_1)[baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_1).size() - 1];

            upperFH_2 = baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_2)[baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_2).size() - 2];
            lowerFH_2 = baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_2)[baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, fh_2).size() - 1];

            // Get upper points
            vector<PolyMesh::Point> upperPoints;

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(upperFH_1); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                upperPoints.push_back(currPoint);
            }

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(upperFH_1); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                upperPoints.push_back(currPoint);
            }

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(upperFH_2); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                upperPoints.push_back(currPoint);
            }

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(upperFH_2); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                upperPoints.push_back(currPoint);
            }

            // Get lower points
            vector<PolyMesh::Point> lowerPoints;

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(lowerFH_1); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                lowerPoints.push_back(currPoint);
            }

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(lowerFH_1); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                lowerPoints.push_back(currPoint);
            }

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(lowerFH_2); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - t ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                lowerPoints.push_back(currPoint);
            }

            {
                double minDist = 10000;
                PolyMesh::Point currPoint;
                for (PolyMesh::FaceVertexIter fv_it=replacedShellStructure_LSM_planar.fv_begin(lowerFH_2); fv_it.is_valid() ; ++fv_it)
                {
                    if ( (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm() < minDist )
                    {
                        minDist = (replacedShellStructure_LSM_planar.point(*fv_it) - f ).norm();
                        currPoint = replacedShellStructure_LSM_planar.point(*fv_it);
                    }
                }

                lowerPoints.push_back(currPoint);
            }

//            dbg(upperPoints);
//            dbg(lowerPoints);

            PolyMesh gapMesh;

            vector<PolyMesh::VertexHandle> upperPointHandleList;
            vector<PolyMesh::VertexHandle> lowerPointHandleList;

            for (int i = 0; i < upperPoints.size(); ++i)
            {
                upperPointHandleList.push_back(gapMesh.add_vertex(upperPoints[i]));
                lowerPointHandleList.push_back(gapMesh.add_vertex(lowerPoints[i]));
            }

            for (int i = 1; i < upperPoints.size(); ++i)
            {
                vector<PolyMesh::VertexHandle> currFaceHandleList;

                currFaceHandleList.push_back(upperPointHandleList[i]);
                currFaceHandleList.push_back(upperPointHandleList[i - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[i]);

                gapMesh.add_face(currFaceHandleList);
            }
            {
                vector<PolyMesh::VertexHandle> currFaceHandleList;

                currFaceHandleList.push_back(upperPointHandleList[0]);
                currFaceHandleList.push_back(upperPointHandleList[upperPoints.size() - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[upperPoints.size() - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[0]);

                gapMesh.add_face(currFaceHandleList);
            }

            // Add upper face
            gapMesh.add_face(upperPointHandleList);

            // Add lower face
            reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
            gapMesh.add_face(lowerPointHandleList);

            gapMesh.triangulate();

            MatrixXd V_gap;
            MatrixXi F_gap;

            PolyMesh2Matrix(gapMesh, V_gap, F_gap);

            double currVolume = ComputeVolumeOfMesh(V_gap, F_gap);

            gapVolumeSum += currVolume;

            float currRatio = currVolume / avgBlockVolume;

            totalRatio_gap += currRatio;

            if (currRatio > worstGapVolumeRatio) {
                worstGapVolumeRatio = currRatio;
//                    cout << "worstOverlappedVolumeBlockRatio: " << currRatio << endl << endl;
            }

//            cout << "currGapVolumeRatio: " << currRatio << endl;
        }
    }

//    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
//    {
//        // Get refer V and F
//        MatrixXd V_refer;
//        MatrixXi F_refer;
//
//        GetReplacedBlockMatrixTri(*f_it, V_refer, F_refer);
//
//        double currReferBlockVolume = ComputeVolumeOfMesh(V_refer, F_refer);
//
//        // Get surrounding face handles
//        vector<PolyMesh::FaceHandle> currFaceHandleList;
//
//        for (PolyMesh::FaceFaceIter ff_it = baseSurface->baseSurface.ff_iter(*f_it); ff_it.is_valid(); ++ff_it)
//        {
//            currFaceHandleList.push_back(*ff_it);
//        }
//
////        for (PolyMesh::FaceVertexIter fv_it = baseSurface->baseSurface.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
////        {
////            PolyMesh::VertexFaceIter vf_it;
////
////            for (vf_it = baseSurface->baseSurface.vf_iter(*fv_it); vf_it.is_valid(); ++vf_it)
////            {
////                if (find(currFaceHandleList.begin(), currFaceHandleList.end(), *vf_it) == currFaceHandleList.end())
////                    currFaceHandleList.push_back(*vf_it);
////            }
////        }
//
//        // Do intersection CSG for each pair
//        for (int i = 0; i < currFaceHandleList.size(); ++i)
//        {
//            MatrixXd V_curr, V_intersection;
//            MatrixXi F_curr, F_intersection;
//
//            GetReplacedBlockMatrixTri(currFaceHandleList[i], V_curr, F_curr);
//
//            MeshBoolean myMeshBoolean;
//
//            myMeshBoolean.MeshIntersect(V_refer, F_refer, V_curr, F_curr, V_intersection, F_intersection);
//
//            if (V_intersection.rows() != 0
//                and !IsTwoBlockMatrixEqual(V_intersection, V_refer)
//                and !IsTwoBlockMatrixEqual(V_intersection, V_curr))
//            {
//                vector<PolyMesh::VertexHandle> currVerHandleList;
//
//                for (int j = 0; j < V_intersection.rows(); ++j)
//                {
//                    currVerHandleList.push_back(myMesh.add_vertex(PolyMesh::Point(V_intersection(j, 0), V_intersection(j, 1), V_intersection(j, 2))));
//                }
//
//                double currVolume = ComputeVolumeOfMesh(V_intersection, F_intersection);
//
////                cout << "currVolume: " << currVolume << endl << endl;
//
//                penetrationVolumeSum += currVolume;
//
//                float currRatio = currVolume / currReferBlockVolume;
//
//                totalRatio += currRatio;
//
//                if (currRatio > worstOverlappedVolumeBlockRatio)
//                {
//                    worstOverlappedVolumeBlockRatio = currRatio;
////                    cout << "worstOverlappedVolumeBlockRatio: " << currRatio << endl << endl;
//                }
//
//                for (int j = 0; j < F_intersection.rows(); ++j)
//                {
//                    vector<PolyMesh::VertexHandle> currFaceVHList;
//
//                    currFaceVHList.push_back(currVerHandleList[F_intersection(j, 0)]);
//                    currFaceVHList.push_back(currVerHandleList[F_intersection(j, 1)]);
//                    currFaceVHList.push_back(currVerHandleList[F_intersection(j, 2)]);
//
//                    myMesh.add_face(currFaceVHList);
//                }
//            }
//        }
//    }
//
//    penetrationVolumeSum = penetrationVolumeSum / 2.0f;

    avgOverlappedVolumeBlockRatio = totalRatio / double(baseSurface->baseSurface.n_edges());

    avgGapVolumeRatio = totalRatio_gap / double(baseSurface->baseSurface.n_edges());

//    replacedPenetrationVolumeTri = myMesh;

    cout << "Compute Gap/Penetration Volume Done." << endl;
}

void ShellStructure::GetReplacedBlockMatrixTri(const PolyMesh::FaceHandle & currFaceHandle, MatrixXd & V, MatrixXi & F)
{
    // Get centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList_planar;
    baseSurface->baseSurface.get_property_handle(centroidBlockFaceHandleList_planar, "centroidBlockFaceHandleList_planar");

    PolyMesh myMesh;
    vector<PolyMesh::Point> currVerList;
    vector<PolyMesh::VertexHandle> currVerHandleList;
    vector< vector<int> > currFaceVerIndexLists;

    for (int i = 0; i < baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, currFaceHandle).size(); ++i)
    {
        PolyMesh::FaceHandle currBlockFaceHandle = baseSurface->baseSurface.property(centroidBlockFaceHandleList_planar, currFaceHandle)[i];
        PolyMesh::FaceVertexCCWIter fv_it;

        for (fv_it = replacedShellStructure_LSM_planar.fv_ccwiter(currBlockFaceHandle); fv_it.is_valid(); ++fv_it)
        {
            if (find(currVerList.begin(), currVerList.end(), replacedShellStructure_LSM_planar.point(*fv_it)) == currVerList.end())
            {
                currVerList.push_back(replacedShellStructure_LSM_planar.point(*fv_it));
            }
        }

        vector<int> currFaceVerIndexList;
        for (fv_it = replacedShellStructure_LSM_planar.fv_ccwiter(currBlockFaceHandle); fv_it.is_valid(); ++fv_it)
        {
            int pos = find(currVerList.begin(), currVerList.end(), replacedShellStructure_LSM_planar.point(*fv_it)) - currVerList.begin();
            currFaceVerIndexList.push_back(pos);
        }

        currFaceVerIndexLists.push_back(currFaceVerIndexList);
    }

    for (int i = 0; i < currVerList.size(); ++i)
    {
        currVerHandleList.push_back(myMesh.add_vertex(currVerList[i]));
    }

    for (int i = 0; i < currFaceVerIndexLists.size(); ++i)
    {
        vector<PolyMesh::VertexHandle> currFaceVerHandleList;

        for (int j = 0; j < currFaceVerIndexLists[i].size(); ++j)
        {
            currFaceVerHandleList.push_back(currVerHandleList[currFaceVerIndexLists[i][j]]);
        }

        myMesh.add_face(currFaceVerHandleList);
    }

    myMesh.triangulate();

    PolyMesh2Matrix(myMesh, V, F);
}

void ShellStructure::ComputeWorstBlockSimilarityError()
{
    // ComputeBlockSimilarityTable_LSM();

    // Get blockSimilarityTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > blockSimilarityTable;
    baseSurface->baseSurface.get_property_handle(blockSimilarityTable, "blockSimilarityTable");

    worstBlockError = 0;
    double totalError = 0;

    for (int i = 0; i < baseSurface->baseSurface.property(blockSimilarityTable).size(); ++i)
    {
        for (int j = 0; j < baseSurface->baseSurface.property(blockSimilarityTable)[i].size(); ++j)
        {
            if (baseSurface->baseSurface.property(blockSimilarityTable)[i][j] > worstBlockError)
            {
                worstBlockError = baseSurface->baseSurface.property(blockSimilarityTable)[i][j];
            }

            totalError += baseSurface->baseSurface.property(blockSimilarityTable)[i][j];
        }
    }

    avgBlockError = totalError / double(baseSurface->baseSurface.n_faces());
}

void ShellStructure::ComputeBlockSimilarityTable_LSM()
{
    baseSurface->UpdateBlockSimilarityTable_LSM();
    BlockClusteringLabelPropertyTransfer(&baseSurface->baseSurface, &baseSurface->triBaseSurface);
}

void ShellStructure::ComputeWorstContactAngleError()
{
    // Get blockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > blockFaceHandleList;
    baseSurface->baseSurface.get_property_handle(blockFaceHandleList, "blockFaceHandleList");

    // Get centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList;
    baseSurface->baseSurface.get_property_handle(centroidBlockFaceHandleList, "centroidBlockFaceHandleList");

    // Get cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface->baseSurface.get_property_handle( cuttingPlane ,"cuttingPlane");

    // Get upperPointList_planar to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > upperPointList_planar;
    baseSurface->baseSurface.get_property_handle(upperPointList_planar, "upperPointList_planar");

    // Get lowerPointList_planar to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > lowerPointList_planar;
    baseSurface->baseSurface.get_property_handle(lowerPointList_planar, "lowerPointList_planar");

    // Get shellBlockPointer to each face
    OpenMesh::FPropHandleT< int > shellBlockPointer;
    baseSurface->baseSurface.get_property_handle(shellBlockPointer, "shellBlockPointer");

    worstContactAngleError = 0;
    double totalError = 0;
    int count = 0;

    for (PolyMesh::EdgeIter e_it=baseSurface->baseSurface.edges_begin(); e_it!=baseSurface->baseSurface.edges_end(); ++e_it)
    {
        if (baseSurface->baseSurface.is_boundary(*e_it))
            continue;

        CuttingPlane currPlane = baseSurface->baseSurface.property(cuttingPlane, *e_it);
        // dbg(currPlane.computationalDihedralAngle);
        Vector3d currEdgeCenter = currPlane.edgeCenter;

        PolyMesh::FaceHandle positiveFH = currPlane.positiveFaceHandle;
        PolyMesh::FaceHandle negetiveFH = currPlane.negativeFaceHandle;

        vector<PolyMesh::FaceHandle> fh_1 = baseSurface->baseSurface.property(centroidBlockFaceHandleList, positiveFH);
        vector<PolyMesh::FaceHandle> fh_2 = baseSurface->baseSurface.property(centroidBlockFaceHandleList, negetiveFH);

        double currAngle = 180;
        
        for (int i = 0; i < fh_1.size() - 2; ++i)
        {
            auto normal_1 = replacedShellStructure_LSM.normal(fh_1[i]);

            Vector3d n1(normal_1[0], normal_1[1], normal_1[2]);
            // dbg(n1);

            for (int j = 0; j < fh_2.size() - 2; ++j)
            {
                auto normal_2 = replacedShellStructure_LSM.normal(fh_2[j]);

                Vector3d n2(normal_2[0], normal_2[1], normal_2[2]);
                // dbg(n2);

                double currResult = AngleBetweenVectors(n1, n2);

                currResult = (3.1415927 - currResult) / 3.1415927 * 180.0f;

                if (currResult < currAngle)
                {
                    currAngle = currResult;
                }

            }
        }

        if (currAngle > worstContactAngleError)
        {
            worstContactAngleError = currAngle;
        }

        totalError += currAngle;

        // // Compute faceNormal_positive
        // vector<Vector3d> upperPointList_planar_positive, lowerPointList_planar_positive;

        // upperPointList_planar_positive = baseSurface->baseSurface.property(upperPointList_planar, positiveFH);
        // lowerPointList_planar_positive = baseSurface->baseSurface.property(lowerPointList_planar, positiveFH);

        // vector<Vector3d> cuttingPlane_positive;

        // cuttingPlane_positive.push_back(upperPointList_planar_positive[upperPointList_planar_positive.size() - 1]);
        // cuttingPlane_positive.push_back(upperPointList_planar_positive[0]);
        // cuttingPlane_positive.push_back(lowerPointList_planar_positive[0]);
        // cuttingPlane_positive.push_back(lowerPointList_planar_positive[lowerPointList_planar_positive.size() - 1]);

        // double distance_pos = (currEdgeCenter - ComputePolygonCentroid(cuttingPlane_positive)).norm();

        // Vector3d faceNormal_positive = (upperPointList_planar_positive[upperPointList_planar_positive.size() - 1] - upperPointList_planar_positive[0]).cross(lowerPointList_planar_positive[0] - upperPointList_planar_positive[0]);

        // for (int i = 1; i < upperPointList_planar_positive.size(); ++i)
        // {
        //     cuttingPlane_positive.clear();

        //     cuttingPlane_positive.push_back(upperPointList_planar_positive[i - 1]);
        //     cuttingPlane_positive.push_back(upperPointList_planar_positive[i]);
        //     cuttingPlane_positive.push_back(lowerPointList_planar_positive[i]);
        //     cuttingPlane_positive.push_back(lowerPointList_planar_positive[i - 1]);

        //     double currDist = (currEdgeCenter - ComputePolygonCentroid(cuttingPlane_positive)).norm();

        //     if (currDist < distance_pos)
        //     {
        //         distance_pos = currDist;
        //         faceNormal_positive = (upperPointList_planar_positive[i - 1] - upperPointList_planar_positive[i]).cross(lowerPointList_planar_positive[i] - upperPointList_planar_positive[i]);
        //     }
        // }

        // // Compute faceNormal_negative
        // vector<Vector3d> upperPointList_planar_negative, lowerPointList_planar_negative;

        // upperPointList_planar_negative = baseSurface->baseSurface.property(upperPointList_planar, negetiveFH);
        // lowerPointList_planar_negative = baseSurface->baseSurface.property(lowerPointList_planar, negetiveFH);

        // vector<Vector3d> cuttingPlane_negative;

        // cuttingPlane_negative.push_back(upperPointList_planar_negative[upperPointList_planar_negative.size() - 1]);
        // cuttingPlane_negative.push_back(upperPointList_planar_negative[0]);
        // cuttingPlane_negative.push_back(lowerPointList_planar_negative[0]);
        // cuttingPlane_negative.push_back(lowerPointList_planar_negative[lowerPointList_planar_negative.size() - 1]);

        // double distance_neg = (currEdgeCenter - ComputePolygonCentroid(cuttingPlane_negative)).norm();

        // Vector3d faceNormal_negative = (upperPointList_planar_negative[upperPointList_planar_negative.size() - 1] - upperPointList_planar_negative[0]).cross(lowerPointList_planar_negative[0] - upperPointList_planar_negative[0]);

        // for (int i = 1; i < upperPointList_planar_negative.size(); ++i)
        // {
        //     cuttingPlane_negative.clear();

        //     cuttingPlane_negative.push_back(upperPointList_planar_negative[i - 1]);
        //     cuttingPlane_negative.push_back(upperPointList_planar_negative[i]);
        //     cuttingPlane_negative.push_back(lowerPointList_planar_negative[i]);
        //     cuttingPlane_negative.push_back(lowerPointList_planar_negative[i - 1]);

        //     double currDist = (currEdgeCenter - ComputePolygonCentroid(cuttingPlane_negative)).norm();

        //     if (currDist < distance_neg)
        //     {
        //         distance_neg = currDist;
        //         faceNormal_negative = (upperPointList_planar_negative[i - 1] - upperPointList_planar_negative[i]).cross(lowerPointList_planar_negative[i] - upperPointList_planar_negative[i]);
        //     }
        // }

        // double currContactAngleError = (3.1415927f - AngleBetweenVectors(faceNormal_positive, faceNormal_negative)) / 3.1415927f * 180.0f ;

        // totalError += currContactAngleError;

        // if (currContactAngleError > worstContactAngleError)
        // {
        //     worstContactAngleError = currContactAngleError;
        // }

        // count++;
    }

    avgContactAngleError = double(totalError) / double( baseSurface->baseSurface.n_edges() );
}

double ShellStructure::ComputeAngleBetweenTwoVectors(const PolyMesh::Point & v1, const PolyMesh::Point & v2)
{
    Vector3d p1(v1[0], v1[1], v1[2]);
    Vector3d p2(v2[0], v2[1], v2[2]);

    double result = AngleBetweenVectors(p1, p2);

    return (3.1415927f - result) / 3.1415927f * 180.0f ;
}




//**************************************************************************************//
//                                    Save Files
//**************************************************************************************//

string ShellStructure::Create3DPrintingFolderName(string folderPath, string surfacePath, string tilePath)
{
    string result;

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

    // Get thickness
    string thicknessFull = to_string(cutLower + cutUpper);
    string thickness = thicknessFull.substr(0, 4);

    // Get tolerance
    string toleranceFull = to_string(baseSurface->tolerance);
    string tolerance = toleranceFull.substr(0, 5);

    // Full file path
    result = savingFolderPath + "/" + surfaceName + "_" + tileName + "_F" + to_string(baseSurface->baseSurface.n_faces()) + "_TH"
    + thickness + "_TO" + tolerance ;

    string command;
    command = "mkdir -p " + result;
    system(command.c_str());

    return result;
}

string ShellStructure::CreateSavingAllFolderName(string folderPath, string surfacePath, string tilePath)
{
    string result;

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

    // Get thickness
    string thicknessFull = to_string(cutLower + cutUpper);
    string thickness = thicknessFull.substr(0, 4);

    // Get tolerance
    string toleranceFull = to_string(baseSurface->tolerance);
    string tolerance = toleranceFull.substr(0, 5);

    // Get blockSimiTable
    OpenMesh::MPropHandleT< vector<vector<double>> > blockSimilarityTable;
    baseSurface->baseSurface.get_property_handle(blockSimilarityTable, "blockSimilarityTable");

    double maxError = 0;

    for (int i = 0; i < baseSurface->baseSurface.property(blockSimilarityTable).size(); ++i)
    {
        for (int j = 0; j < baseSurface->baseSurface.property(blockSimilarityTable)[i].size(); ++j)
        {
            if (baseSurface->baseSurface.property(blockSimilarityTable)[i][j] > maxError)
                maxError = baseSurface->baseSurface.property(blockSimilarityTable)[i][j];
        }
    }

    // Get maxBlockError
    string maxBlockErrorFull = to_string(maxError);
    string maxBlockError = maxBlockErrorFull.substr(0, 8);

    // Full file path
    result = savingFolderPath + "/" + surfaceName + "_" + tileName + "_F" + to_string(baseSurface->baseSurface.n_faces()) + "_BC"
             + to_string(baseSurface->GetBlockClusterNum()) + "_TH" + thickness + "_TO" + tolerance + "_BE" + maxBlockError ;

    string command;
    command = "mkdir -p " + result;
    system(command.c_str());

    return result;
}

void ShellStructure::SaveShell(string folderPath, bool isSavePrintingFiles)
{
    /// Save orig shell files
    string shellPath = folderPath + "/_shell";
    {
        string command;
        command = "mkdir -p " + shellPath;
        system(command.c_str());
    }

    SaveOrigShellBlocksMotionXMLFile(shellPath);
    SaveOrigShellBlocks(shellPath);
    SaveReplacedShellBlocks(shellPath);

    OpenMesh::IO::write_mesh(shellStructure, shellPath + "/OptimizedShell.obj");
    OpenMesh::IO::write_mesh(replacedShellStructure_LSM_planar, shellPath + "/ReplacedShell.obj");

    SaveAllClusteringBlockOBJs(shellPath + "/_clustered_blocks", shellPath + "/_block_templates");

    /// Save support_virtual files
    string supportPath = shellPath + "/_support_virtual";
    {
        string command;
        command = "mkdir -p " + supportPath;
        system(command.c_str());
    }
    SaveShellSupportSurface(supportPath);

    /// Save printing shell files
    if (isSavePrintingFiles)
    {
        string shellPrintingPath = folderPath + "/PrintingOBJs";
        {
            string command;
            command = "mkdir -p " + shellPrintingPath;
            system(command.c_str());
        }

        string shellSupportGenerationPath = shellPrintingPath + "/SupportGeneration";
        {
            string command;
            command = "mkdir -p " + shellSupportGenerationPath;
            system(command.c_str());
        }

//        SaveOrigShellBlocksWithDigits(shellPrintingPath);
        SaveReplacedShellBlocksWithDigits(shellPrintingPath);

        SaveBaseSurface(shellSupportGenerationPath);
        SaveShellSupportSurface(shellSupportGenerationPath);
//        SaveOverlappedShellForSupportGeneration(shellSupportGenerationPath);
//        SaveOverlappedToleranceShellForSupportGeneration(shellSupportGenerationPath);
    }
}

void ShellStructure::SaveOrigOBJsFor3dPrinting(string folderPath)
{
    SaveBaseSurface(folderPath);
    SaveOrigShellBlocks(folderPath);
    SaveOrigShellBlocksWithDigits(folderPath);
    SaveOverlappedShellBlocks(folderPath);
    SaveShellSupportSurface(folderPath);
    SaveOverlappedShellForSupportGeneration(folderPath);
    SaveOverlappedToleranceShellForSupportGeneration(folderPath);
    SaveOrigShellBlocksMotionXMLFile(folderPath);
}

void ShellStructure::SaveBaseSurface(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/OptimizedSurface.obj";

    OpenMesh::IO::write_mesh(baseSurface->baseSurface, fullPath);
}

void ShellStructure::SaveOrigShellBlocks(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/_shell_blocks";

    // Setup a folder to save files
    string command;
    command = "mkdir -p " + fullPath;
    system(command.c_str());

    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        shellBlockList[i].SaveOrigShellBlock(fullPath);
    }
}

void ShellStructure::SaveReplacedShellBlocks(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/_replaced_blocks";

    // Setup a folder to save files
    string command;
    command = "mkdir -p " + fullPath;
    system(command.c_str());

    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        shellBlockList[i].SaveReplacedShellBlock(fullPath);
    }
}

void ShellStructure::SaveOrigShellBlocksMotionXMLFile(string folderPath)
{
    // Get seqID to each face
    OpenMesh::FPropHandleT< int > seqID;
    baseSurface->baseSurface.get_property_handle(seqID, "seqID");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    baseSurface->baseSurface.get_property_handle(blockClusteringLabel, "blockClusteringLabel");

    // Get the number of block cluster
    int blockClusterNum = baseSurface->GetBlockClusterNum();

    // Get normal of each face
    baseSurface->baseSurface.request_face_normals();
    baseSurface->baseSurface.update_normals();

    // Create xml
    pugi::xml_document doc;
    string folder_name(folderPath);
    pugi::xml_node root_node=doc.append_child("root");
    root_node.append_attribute("start_id")="1";

    int num=0; int Cluster_num=0; int max=0;
    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        num++;
    }
    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        int currClusterID = baseSurface->baseSurface.property(blockClusteringLabel, *f_it);
        if(currClusterID >= max)
        {
            max = currClusterID;
        }
    }
    
    root_node.append_attribute("Objects_num") = num; 
    root_node.append_attribute("Cluster_num") = max + 1;
    
    int a_flat=0;

    pugi::xml_node block_node=root_node.append_child("block_name");
    for (int i = 0; i < baseSurface->baseSurface.n_faces(); ++i)
    {
        a_flat++;
        pugi::xml_node b_node=block_node.append_child("block");
        b_node.append_attribute("id")=(a_flat);
    }

    vector<Vector3d> removeVectorList;
    vector<int> ClusterIDList;
    vector<int> IdList;
    for (int i = 0; i < baseSurface->baseSurface.n_faces(); ++i)
    {
        removeVectorList.push_back(Vector3d(0, 0, 0));
        IdList.push_back(0);
        ClusterIDList.push_back(0);
    }

    a_flat = 0;
    Vector3d temp(0,0,0);

    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        PolyMesh::Point currFaceNormal = baseSurface->baseSurface.normal(*f_it);
        temp(0)=currFaceNormal[0]; temp(1)=currFaceNormal[1]; temp(2)=currFaceNormal[2];
        removeVectorList[a_flat] = temp;

        int currClusterID = baseSurface->baseSurface.property(blockClusteringLabel, *f_it) + 1;

        int currSeqID = baseSurface->baseSurface.property(seqID, *f_it);
        ClusterIDList[a_flat] = currClusterID; 
        IdList[a_flat] = currSeqID;
        a_flat++;
    }

    // Sorting
    for (int i = 0; i < num; ++i)
    {
        for(int j = 0; j < num - 1 - i; ++j)
        {
            if(IdList[j] > IdList[j+1])
            {
                int a = IdList[j];
                IdList[j] = IdList[j+1];
                IdList[j+1] = a;
                int c = ClusterIDList[j];
                ClusterIDList[j] = ClusterIDList[j+1];
                ClusterIDList[j+1] = c;
                Vector3d b=removeVectorList[j];
                removeVectorList[j]=removeVectorList[j+1];
                removeVectorList[j+1] = b;
            }
        }
    }

    // Action
    a_flat = 0;
    pugi::xml_node action_all_node=root_node.append_child("action_all");

    for (int i = 0; i < num; i++)
    {
        //int currSeqID = baseSurface->baseSurface.property(seqID, *f_it);
        //PolyMesh::Point currFaceNormal = baseSurface->baseSurface.normal(*f_it);
        int frameNum = 20;
        pugi::xml_node action_node=action_all_node.append_child("action");
        action_node.append_attribute("frameNum") = (frameNum);
        temp = removeVectorList[a_flat];
        action_node.append_attribute("x") = temp(0);
        action_node.append_attribute("y") = temp(1);
        action_node.append_attribute("z") = temp(2);
        ///for(int k=0;k < baseSurface->baseSurface.faces_end();k++)
        //{
        pugi::xml_node assemblyID_node = action_node.append_child("block");
        assemblyID_node.append_attribute("assemblyID") = (IdList[a_flat]);
        pugi::xml_node clusterID_node = action_node.append_child("block");
        clusterID_node.append_attribute("clusterID") = (ClusterIDList[a_flat]);
        //}
        a_flat++;
    }
   
/*     for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        int currSeqID = baseSurface->baseSurface.property(seqID, *f_it);
        PolyMesh::Point currFaceNormal = baseSurface->baseSurface.normal(*f_it);
        pugi::xml_node action2_node=action_all_node.append_child("action2");
        int frameNum2=20;
        action2_node.append_attribute("frameNum2")=(frameNum2);
        action2_node.append_attribute("x")=(currFaceNormal[0]);
        action2_node.append_attribute("y")=(currFaceNormal[1]);
        action2_node.append_attribute("z")=(currFaceNormal[2]);
        pugi::xml_node blockId2_node=action2_node.append_child("block");
        blockId2_node.append_attribute("id")=(currSeqID+1);
    } */
    doc.save_file((folder_name+"/animation.motion.xml").data());
}


void ShellStructure::SaveOrigShellBlocksWithDigits(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/ShellBlocks_digits";

    // Setup a folder to save files
    string command;
    command = "mkdir -p " + fullPath;
    system(command.c_str());

    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        shellBlockList[i].AddDigit2OrigShellBlock();
        shellBlockList[i].SaveOrigShellBlockWithDigits(fullPath);
    }
}

void ShellStructure::SaveReplacedShellBlocksWithDigits(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/ReplacedBlocks_digits";

    // Setup a folder to save files
    string command;
    command = "mkdir -p " + fullPath;
    system(command.c_str());

    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        shellBlockList[i].AddDigit2ReplacedShellBlock();
        shellBlockList[i].SaveReplacedShellBlockWithDigits(fullPath);
    }
}

void ShellStructure::SaveOverlappedShellBlocks(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/_ShellBlocks_overlapped";

    // Setup a folder to save files
    string command;
    command = "mkdir -p " + fullPath;
    system(command.c_str());

    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        shellBlockList[i].SaveOrigShellBlock_overlapped(fullPath);
    }
}

void ShellStructure::SaveOverlappedShellForSupportGeneration(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/OptimizedShellStructure_overlap.obj";

    Eigen::MatrixXd V_block;
    Eigen::MatrixXi F_block;

    PolyMesh2Matrix(shellBlockList[0].origShellBlockTri_overlap, V_block, F_block);

    for (int i = 1; i < shellBlockList.size(); ++i)
    {
        Eigen::MatrixXd V_curr;
        Eigen::MatrixXi F_curr;

        PolyMesh2Matrix(shellBlockList[i].origShellBlockTri_overlap, V_curr, F_curr);

        Eigen::MatrixXd V_csg;
        Eigen::MatrixXi F_csg;

        MeshBoolean meshBoolean;
        meshBoolean.MeshUnion(V_block, F_block, V_curr, F_curr, V_csg, F_csg);

        V_block = V_csg;
        F_block = F_csg;

//        cout << i << "done. " << endl;
    }

    PolyMesh blockAfterCSG;
    Matrix2PolyMesh(blockAfterCSG, V_block, F_block);

    OpenMesh::IO::write_mesh(blockAfterCSG, fullPath);
}

void ShellStructure::SaveOverlappedToleranceShellForSupportGeneration(string folderPath)
{
    // Full file path
    string fullPath = folderPath + "/OptimizedShellStructure_overlap_tolerance.obj";

    Eigen::MatrixXd V_block;
    Eigen::MatrixXi F_block;

    PolyMesh2Matrix(shellBlockList[0].origShellBlockTri_overlap_tolerance, V_block, F_block);

    for (int i = 1; i < shellBlockList.size(); ++i)
    {
        Eigen::MatrixXd V_curr;
        Eigen::MatrixXi F_curr;

        PolyMesh2Matrix(shellBlockList[i].origShellBlockTri_overlap_tolerance, V_curr, F_curr);

        Eigen::MatrixXd V_csg;
        Eigen::MatrixXi F_csg;

        MeshBoolean meshBoolean;
        meshBoolean.MeshUnion(V_block, F_block, V_curr, F_curr, V_csg, F_csg);

        V_block = V_csg;
        F_block = F_csg;

//        cout << i << "done. " << endl;
    }

    PolyMesh blockAfterCSG;
    Matrix2PolyMesh(blockAfterCSG, V_block, F_block);

    OpenMesh::IO::write_mesh(blockAfterCSG, fullPath);
}

void ShellStructure::SaveShellSupportSurface(string folderPath)
{
    InitShellSupportSurface();

    // Full file path
    string fullSupportFullPath = folderPath + "/FullSupportSurface.obj";
    string supportFullPath = folderPath + "/SupportSurface.obj";
    // string supportToleranceFullPath = folderPath + "/SupportSurface_tolerance.obj";

    OpenMesh::IO::write_mesh(fullSupportSurface, fullSupportFullPath);
    OpenMesh::IO::write_mesh(supportSurface, supportFullPath);
    // OpenMesh::IO::write_mesh(supportSurfaceTri_tolerance, supportToleranceFullPath);
}

void ShellStructure::SaveShellOptimizationResult(string folderPath)
{
    std::ofstream out(folderPath + "/ShellOptimizationResult.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    cout << "Thinkness: " << cutLower + cutUpper << endl;
    cout << endl;

    cout << "Average Block Error: " << avgBlockError << endl;
    cout << "Worst Block Error: " << worstBlockError << endl;
    cout << endl;

    cout << "Average Contact Angle Error: " << avgContactAngleError << endl;
    cout << "Worst Contact Angle Error: " << worstContactAngleError << endl;
    cout << endl;

    cout << "Average Penetration Volume Ratio: " << avgOverlappedVolumeBlockRatio << endl;
    cout << "Total Penetration Volume: " << penetrationVolumeSum << endl;
    cout << "Worst Penetration Volume Ratio: " << worstOverlappedVolumeBlockRatio << endl;
    cout << endl;

    cout << "Shell Optimization Time: " << shellOptimizationTime << " mins" << endl;

    std::cout.rdbuf(coutbuf); //reset to standard output again
}

void ShellStructure::SaveAllClusteringBlockOBJs(string folderPath, string templatePath)
{
    // Get blockCuttingAngleList_full to mesh
    OpenMesh::MPropHandleT< vector<vector<vector<int>>> > blockCuttingAngleLabelList_full;
    baseSurface->baseSurface.get_property_handle(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Get blockSimilarityTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > blockSimilarityTable;
    baseSurface->baseSurface.get_property_handle(blockSimilarityTable, "blockSimilarityTable");

    /// Setup a folder to save template files
    {
        string command;
        command = "mkdir -p " + folderPath;
        system(command.c_str());
    }

    /// Setup a folder to save template files
    {
        string command;
        command = "mkdir -p " + templatePath;
        system(command.c_str());
    }

    for (int i = 0; i < baseSurface->baseSurface.property(blockCuttingAngleLabelList_full).size(); ++i)
    {
        string currStr = folderPath + "/C" + to_string(i + 1) + "_" + to_string(baseSurface->baseSurface.property(blockCuttingAngleLabelList_full)[i].size()) + "_" + to_string(accumulate(baseSurface->baseSurface.property(blockSimilarityTable)[i].begin(), baseSurface->baseSurface.property(blockSimilarityTable)[i].end(), 0.0) / baseSurface->baseSurface.property(blockSimilarityTable)[i].size());

        /// Setup a folder to save obj files
        string command;
        command = "mkdir -p " + currStr;
        system(command.c_str());

        SaveClusterBlockOBJs(currStr, templatePath, i);
    }
}

void ShellStructure::SaveClusterBlockOBJs(string folderPath, string templatePath, int clusterID)
{
    // Add upperPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > upperPointList;
    baseSurface->baseSurface.get_property_handle(upperPointList, "upperPointList");

    // Add lowerPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > lowerPointList;
    baseSurface->baseSurface.get_property_handle(lowerPointList, "lowerPointList");

    /// Setup a folder to save puzzle files
    string command;
    command = "mkdir -p " + folderPath;
    system(command.c_str());

    vector<Vector3d> referUpperList, referLowerList, referBlockVerList;

    /// Save template first
    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        if (shellBlockList[i].clusterID - 1 == clusterID)
        {
            OpenMesh::IO::write_mesh(shellBlockList[i].replacedShellBlock, folderPath + "/_Cluster_" + to_string(clusterID + 1) + "_Centroid.obj");
            OpenMesh::IO::write_mesh(shellBlockList[i].replacedShellBlock, templatePath + "/_Cluster_" + to_string(clusterID + 1) + "_Centroid.obj");

            PolyMesh::FaceHandle currFH = shellBlockList[i].origFaceHandle;

            // Get currRefer
            referUpperList = PolyMeshPointList2EigenVector3dList(baseSurface->baseSurface.property(upperPointList, currFH));
            referLowerList = PolyMeshPointList2EigenVector3dList(baseSurface->baseSurface.property(lowerPointList, currFH));

            referBlockVerList = CombineTwoBlockVerLists(referUpperList, referLowerList);

            break;
        }
    }

    /// Save origPos block and aligned block
    int count = 0;

    for (int i = 0; i < shellBlockList.size(); ++i)
    {
        if (shellBlockList[i].clusterID - 1 == clusterID)
        {
            OpenMesh::IO::write_mesh(shellBlockList[i].origShellBlock, folderPath + "/OrigPos_Cluster_" + to_string(clusterID + 1) + "_" + to_string(count + 1) + ".obj");

            PolyMesh::FaceHandle currFH = shellBlockList[i].origFaceHandle;

            vector<Vector3d> currUpperList, currLowerList;

            currUpperList = PolyMeshPointList2EigenVector3dList(baseSurface->baseSurface.property(upperPointList, currFH));
            currLowerList = PolyMeshPointList2EigenVector3dList(baseSurface->baseSurface.property(lowerPointList, currFH));

            MatrixXd p;
            baseSurface->AlignBlock_LSM(referBlockVerList, currUpperList, currLowerList, p);

            // Push back the aligned vertex list
            vector<Vector3d> currAlignedList;
            for (int i = 0; i < p.cols(); ++i)
            {
                currAlignedList.push_back(Vector3d(p(0, i), p(1, i), p(2, i)));
            }

            tuple< vector<Vector3d>, vector<Vector3d> > currBlockTuple;
            SplitTwoVector(currAlignedList, get<0>(currBlockTuple), get<1>(currBlockTuple));

            PolyMesh myMesh;

            vector<PolyMesh::VertexHandle> upperPointHandleList;
            vector<PolyMesh::VertexHandle> lowerPointHandleList;

            for (int i = 0; i < get<0>(currBlockTuple).size(); ++i) {
                upperPointHandleList.push_back(myMesh.add_vertex(EigenVector3d2PolyMeshPoint(get<0>(currBlockTuple)[i])));
                lowerPointHandleList.push_back(myMesh.add_vertex(EigenVector3d2PolyMeshPoint(get<1>(currBlockTuple)[i])));
            }

            for (int i = 1; i < get<0>(currBlockTuple).size(); ++i)
            {
                vector<PolyMesh::VertexHandle> currFaceHandleList;

                currFaceHandleList.push_back(upperPointHandleList[i]);
                currFaceHandleList.push_back(upperPointHandleList[i - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[i]);

                auto currH = myMesh.add_face(currFaceHandleList);
            }
            {
                vector<PolyMesh::VertexHandle> currFaceHandleList;

                currFaceHandleList.push_back(upperPointHandleList[0]);
                currFaceHandleList.push_back(upperPointHandleList[get<0>(currBlockTuple).size() - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[get<0>(currBlockTuple).size() - 1]);
                currFaceHandleList.push_back(lowerPointHandleList[0]);

                auto currH = myMesh.add_face(currFaceHandleList);
            }

            // Add upper face
            auto upperP = myMesh.add_face(upperPointHandleList);

            // Add lower face
            reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
            auto lowerP = myMesh.add_face(lowerPointHandleList);

            OpenMesh::IO::write_mesh(myMesh, folderPath + "/Aligned_Cluster_" + to_string(clusterID + 1) + "_" + to_string(count + 1) + ".obj");

            count++;
        }
    }
}




//**************************************************************************************//
//                                   Helper Function
//**************************************************************************************//

void ShellStructure::UpperAndLowerPointList2PolyMesh(const vector<PolyMesh::Point> & currUpperPointList, const vector<PolyMesh::Point> & currLowerPointList, PolyMesh & myMesh)
{
    vector<PolyMesh::VertexHandle> upperPointHandleList;
    vector<PolyMesh::VertexHandle> lowerPointHandleList;

    for (int i = 0; i < currUpperPointList.size(); ++i) 
    {
        upperPointHandleList.push_back(myMesh.add_vertex(currUpperPointList[i]));
        lowerPointHandleList.push_back(myMesh.add_vertex(currLowerPointList[i]));
    }

    for (int i = 1; i < currUpperPointList.size(); ++i)
    {
        vector<PolyMesh::VertexHandle> currFaceHandleList;

        currFaceHandleList.push_back(upperPointHandleList[i]);
        currFaceHandleList.push_back(upperPointHandleList[i - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[i]);

        auto currH = myMesh.add_face(currFaceHandleList);
    }
    {
        vector<PolyMesh::VertexHandle> currFaceHandleList;

        currFaceHandleList.push_back(upperPointHandleList[0]);
        currFaceHandleList.push_back(upperPointHandleList[currUpperPointList.size() - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[currUpperPointList.size() - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[0]);

        auto currH = myMesh.add_face(currFaceHandleList);
    }

    // Add upper face
    auto upperP = myMesh.add_face(upperPointHandleList);

    // Add lower face
    reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
    auto lowerP = myMesh.add_face(lowerPointHandleList);
}

void ShellStructure::UpperAndLowerPointList2PolyMesh(const vector<Vector3d> & upperPointList, const vector<Vector3d> & lowerPointList, PolyMesh & myMesh)
{
    vector<PolyMesh::Point> currUpperPointList, currLowerPointList;

    for (int i = 0; i < upperPointList.size(); ++i)
    {
        currUpperPointList.push_back(PolyMesh::Point(upperPointList[i][0], upperPointList[i][1], upperPointList[i][2]));
        currLowerPointList.push_back(PolyMesh::Point(lowerPointList[i][0], lowerPointList[i][1], lowerPointList[i][2]));
    }

    vector<PolyMesh::VertexHandle> upperPointHandleList;
    vector<PolyMesh::VertexHandle> lowerPointHandleList;

    for (int i = 0; i < currUpperPointList.size(); ++i) 
    {
        upperPointHandleList.push_back(myMesh.add_vertex(currUpperPointList[i]));
        lowerPointHandleList.push_back(myMesh.add_vertex(currLowerPointList[i]));
    }

    for (int i = 1; i < currUpperPointList.size(); ++i)
    {
        vector<PolyMesh::VertexHandle> currFaceHandleList;

        currFaceHandleList.push_back(upperPointHandleList[i]);
        currFaceHandleList.push_back(upperPointHandleList[i - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[i - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[i]);

        auto currH = myMesh.add_face(currFaceHandleList);
    }
    {
        vector<PolyMesh::VertexHandle> currFaceHandleList;

        currFaceHandleList.push_back(upperPointHandleList[0]);
        currFaceHandleList.push_back(upperPointHandleList[currUpperPointList.size() - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[currUpperPointList.size() - 1]);
        currFaceHandleList.push_back(lowerPointHandleList[0]);

        auto currH = myMesh.add_face(currFaceHandleList);
    }

    // Add upper face
    auto upperP = myMesh.add_face(upperPointHandleList);

    // Add lower face
    reverse(lowerPointHandleList.begin(),lowerPointHandleList.end());
    auto lowerP = myMesh.add_face(lowerPointHandleList);
}
