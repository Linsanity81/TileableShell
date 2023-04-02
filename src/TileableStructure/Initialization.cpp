//
// Created by Linsanity on 18/4/22.
//

#include "Initialization.h"
#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>
#include "TileableStructure/ShellBlock.h"
#include <OpenMesh/Core/IO/MeshIO.hh>

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

Initialization::Initialization()
{
    isPlanarization = true;
}

Initialization::~Initialization()
{

}




//**************************************************************************************//
//                                   Init BaseSurface
//**************************************************************************************//

void Initialization::InitBaseSurface(string inputFilePath, BaseSurface * baseSurface)
{
    PolyMesh myMesh;

    OpenMesh::IO::read_mesh(myMesh, inputFilePath);

    baseSurface->baseSurface = myMesh;
    baseSurface->triBaseSurface = myMesh;
    baseSurface->triBaseSurface.triangulate();

    // ShapeOp planarization for a better initial solution
    if (isPlanarization)
        ShapeOp_Planarization(baseSurface);

    AddProperty2BaseSurface(baseSurface->baseSurface, baseSurface->triBaseSurface);

//    baseSurface->remeshedPolySurface_without_shapeop = myMesh;
//    baseSurface->remeshedTriSurface_without_shapeop = myMesh;
//    baseSurface->remeshedTriSurface_without_shapeop.triangulate();
//
//    AddProperty2BaseSurface(baseSurface->remeshedPolySurface_without_shapeop, baseSurface->remeshedTriSurface_without_shapeop);

    baseSurface->remeshedPolySurface = baseSurface->baseSurface;
    baseSurface->remeshedTriSurface = baseSurface->triBaseSurface;

    baseSurface->UpdatePolygonPlanarity_LSM(&baseSurface->remeshedPolySurface, &baseSurface->remeshedTriSurface);
}




//**************************************************************************************//
//                                   Remeshing
//**************************************************************************************//

void Initialization::Remeshing(Surface * surface, TilePattern * tilePattern, BaseSurface * baseSurface)
{
    // Scale
    tilePattern->inputTilePattern = tilePattern->origTilePattern;

    PolyMesh::VertexIter          v_it, v_end(tilePattern->inputTilePattern.vertices_end());
    for (v_it=tilePattern->inputTilePattern.vertices_begin(); v_it!=v_end; ++v_it)
    {
        tilePattern->inputTilePattern.point(*v_it) *= tilePattern->scalar;
    }

    // Rotation
    if (tilePattern->rotationAngle >= 360)
        tilePattern->rotationAngle -= 360;

    Eigen::AngleAxisd rollAngle(tilePattern->rotationAngle, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    for (v_it=tilePattern->inputTilePattern.vertices_begin(); v_it!=v_end; ++v_it)
    {
        RowVector3d currVer(tilePattern->inputTilePattern.point(*v_it)[0], tilePattern->inputTilePattern.point(*v_it)[1], tilePattern->inputTilePattern.point(*v_it)[2]);
        auto result = currVer * rotationMatrix;
        tilePattern->inputTilePattern.point(*v_it)[0] = result[0];
        tilePattern->inputTilePattern.point(*v_it)[1] = result[1];
        tilePattern->inputTilePattern.point(*v_it)[2] = result[2];
    }

    PolyMesh myMesh;

    for (PolyMesh::FaceIter f_it=tilePattern->inputTilePattern.faces_begin(); f_it!=tilePattern->inputTilePattern.faces_end(); ++f_it) {
        PolyMesh::FaceVertexIter fv_it;

        bool isValid = true;
        vector<Vector3d> currFaceVerList;

        for (fv_it = tilePattern->inputTilePattern.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
            Vector3d currVer(tilePattern->inputTilePattern.point(*fv_it)[0],
                             tilePattern->inputTilePattern.point(*fv_it)[1],
                             tilePattern->inputTilePattern.point(*fv_it)[2]);
            Vector3d currSurfCoord;

            isValid = ComputeSurfaceCoord(surface, tilePattern, baseSurface, currVer, currSurfCoord);
            if (!isValid)
                break;

            currFaceVerList.push_back(currSurfCoord);
        }

        if (isValid) {
            std::vector<PolyMesh::VertexHandle> face_vhandles;
            for (int i = 0; i < currFaceVerList.size(); ++i) {
                face_vhandles.push_back(myMesh.add_vertex(
                        PolyMesh::Point(currFaceVerList[i](0), currFaceVerList[i](1), currFaceVerList[i](2))));
            }
            myMesh.add_face(face_vhandles);
        }
    }

    baseSurface->baseSurface = myMesh;
    baseSurface->triBaseSurface = myMesh;
    baseSurface->triBaseSurface.triangulate();

//    baseSurface->remeshedPolySurface_without_shapeop = myMesh;
//    baseSurface->remeshedTriSurface_without_shapeop = myMesh;
//    baseSurface->remeshedTriSurface_without_shapeop.triangulate();
//
//    AddProperty2BaseSurface(baseSurface->remeshedPolySurface_without_shapeop, baseSurface->remeshedTriSurface_without_shapeop);

    // ShapeOp planarization for a better initial solution
    if (isPlanarization)
        ShapeOp_Planarization(baseSurface);

    AddProperty2BaseSurface(baseSurface->baseSurface, baseSurface->triBaseSurface);

    baseSurface->remeshedPolySurface = baseSurface->baseSurface;
    baseSurface->remeshedTriSurface = baseSurface->triBaseSurface;

    baseSurface->UpdatePolygonPlanarity_LSM(&baseSurface->remeshedPolySurface, &baseSurface->remeshedTriSurface);
}

bool Initialization::ComputeSurfaceCoord(Surface * surface, TilePattern * tilePattern, BaseSurface * baseSurface, Vector3d ptTexCoord, Vector3d &ptSurfCoord)
{
    if ( ptTexCoord(0) < surface->uvBBox.minPt(0) || ptTexCoord(0) > surface->uvBBox.maxPt(0) ||
         ptTexCoord(1) < surface->uvBBox.minPt(1) || ptTexCoord(1) > surface->uvBBox.maxPt(1) )
    {
        return false;
    }

    ////////////////////////////////////////////////////////
    // Barycentric interpolation for point's geometrical coordinate
    // Note: only consider the triangles inside the bucket

    for (int i=0; i<surface->triPairlist.size(); i++)
    {
        Triangle texTri = surface->triPairlist[i].texTriangle;
        Triangle geoTri = surface->triPairlist[i].geoTriangle;

        // Barycentric interpolation if point is inside the triangle
        if( IsPointInsideTriangle(ptTexCoord, texTri) )
        {
            // Compute total and partial triangle area
            float totalArea = GetTriangleArea(texTri.v[0], texTri.v[1], texTri.v[2]);
            float subArea0  = GetTriangleArea(ptTexCoord, texTri.v[1], texTri.v[2]);
            float subArea1  = GetTriangleArea(texTri.v[0], ptTexCoord, texTri.v[2]);
            float subArea2  = GetTriangleArea(texTri.v[0], texTri.v[1], ptTexCoord);

            ptSurfCoord = (subArea0/totalArea)*geoTri.v[0] +
                          (subArea1/totalArea)*geoTri.v[1] +
                          (subArea2/totalArea)*geoTri.v[2];
            return true;
        }
    }

    return false;
}

void Initialization::ShapeOp_Planarization(BaseSurface * baseSurface)
{
    ShapeOp::Matrix3X verMat; //column major

    vector <vector<int>> faceVerticeList;

    PolyMesh2ShapeOpMatrix(baseSurface->baseSurface, verMat, faceVerticeList);

    ShapeOp::Solver s;
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
                auto c = std::make_shared<ShapeOp::PlaneConstraint>(id_vector, weight * 0.001, s.getPoints());

                s.addConstraint(c);
            }
        }
    }

    // Add a angle constraint to all the faces
    // Have no effect to the final result, only putting here for avoiding optimization errors
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

    baseSurface->baseSurface.clear();
    baseSurface->triBaseSurface.clear();

    ShapeOpMatrix2PolyMesh(baseSurface->baseSurface, verMat, faceVerticeList);

    baseSurface->triBaseSurface = baseSurface->baseSurface;
    baseSurface->triBaseSurface.triangulate();
}




//**************************************************************************************//
//                               Properties Operation
//**************************************************************************************//

void Initialization::AddProperty2BaseSurface(PolyMesh & baseSurface, PolyMesh & triBaseSurface)
{
    /// Request Face Normal
    baseSurface.request_face_normals();
    baseSurface.update_normals();

    // Add origPoint to each vertex
    OpenMesh::VPropHandleT< PolyMesh::Point > origPoint;
    baseSurface.add_property( origPoint ,"origPoint");

    /// First we add 2 basic properties for each edge
    /// (1) normal (2) cuttingPlane

    // Add normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface.add_property( normal ,"normal");

    // Add cuttingPlane to each edge
    OpenMesh::EPropHandleT< CuttingPlane > cuttingPlane;
    baseSurface.add_property( cuttingPlane ,"cuttingPlane");

    // Add isHold to each edge
    OpenMesh::EPropHandleT< bool > isHold;
    baseSurface.add_property( isHold ,"isHold");

    /// Then we add 4 basic properties for each face
    /// (1) faceCenter (2) index (3) clusteringLabel (4) diagonalList (5) alignedVerList

    // Add shellBlockPointer to each face
    OpenMesh::FPropHandleT< int > shellBlockPointer;
    baseSurface.add_property(shellBlockPointer, "shellBlockPointer");

    // Add isBoundaryFace to each face
    OpenMesh::FPropHandleT< bool > isBoundaryFace;
    baseSurface.add_property(isBoundaryFace, "isBoundaryFace");

    // Add faceCenter to each face
    OpenMesh::FPropHandleT< Vector3d > faceCenter;
    baseSurface.add_property(faceCenter, "faceCenter");

    // Add index to each face
    OpenMesh::FPropHandleT< int > index;
    baseSurface.add_property(index, "index");

    // Add seqID to each face
    OpenMesh::FPropHandleT< int > seqID;
    baseSurface.add_property(seqID, "seqID");

    // Add clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    baseSurface.add_property(clusteringLabel, "clusteringLabel");

    // Add diagonalList to each face
    OpenMesh::FPropHandleT< vector<std::tuple<Vector3d, Vector3d, double>> > diagonalList;
    baseSurface.add_property(diagonalList, "diagonalList");

    // Add alignedVerList to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > alignedVerList;
    baseSurface.add_property(alignedVerList, "alignedVerList");

    // Add polygonCentroidDistance to each face
    OpenMesh::FPropHandleT< double > polygonCentroidDistance;
    baseSurface.add_property(polygonCentroidDistance, "polygonCentroidDistance");

    // Add replacedFaceHandle to each face
    OpenMesh::FPropHandleT< PolyMesh::FaceHandle > replacedFaceHandle;
    baseSurface.add_property(replacedFaceHandle, "replacedFaceHandle");

    // Add isSatisfyFabRequirement to each face
    OpenMesh::FPropHandleT< bool > isSatisfyFabRequirement;
    baseSurface.add_property(isSatisfyFabRequirement, "isSatisfyFabRequirement");

    // Add facePlanarity to each face
    OpenMesh::FPropHandleT< double > facePlanarity;
    baseSurface.add_property( facePlanarity ,"facePlanarity");

    // Add blockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > blockFaceHandleList;
    baseSurface.add_property(blockFaceHandleList, "blockFaceHandleList");

    // Add upperPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > upperPointList;
    baseSurface.add_property(upperPointList, "upperPointList");

    // Add lowerPointList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::Point> > lowerPointList;
    baseSurface.add_property(lowerPointList, "lowerPointList");

    // Add upperPointList_planar to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > upperPointList_planar;
    baseSurface.add_property(upperPointList_planar, "upperPointList_planar");

    // Add lowerPointList_planar to each face
    OpenMesh::FPropHandleT< vector<Vector3d> > lowerPointList_planar;
    baseSurface.add_property(lowerPointList_planar, "lowerPointList_planar");

    // Add edgeCuttingAngleLabelList;
    OpenMesh::FPropHandleT< vector<tuple<int, int>> > edgeCuttingAngleLabelList;
    baseSurface.add_property(edgeCuttingAngleLabelList, "edgeCuttingAngleLabelList");

    // Add cuttingAngleLabelList;
    OpenMesh::FPropHandleT< vector<int> > cuttingAngleLabelList;
    baseSurface.add_property(cuttingAngleLabelList, "cuttingAngleLabelList");

    // Add cuttingAngleList;
    OpenMesh::FPropHandleT< vector<double> > cuttingAngleList;
    baseSurface.add_property(cuttingAngleList, "cuttingAngleList");

    // Add blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > blockClusteringLabel;
    baseSurface.add_property(blockClusteringLabel, "blockClusteringLabel");

    // Add blockAlignedVerList to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > blockAlignedVerList;
    baseSurface.add_property(blockAlignedVerList, "blockAlignedVerList");

    // Add blockCentroid to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > blockCentroid;
    baseSurface.add_property(blockCentroid, "blockCentroid");

    // Add replacedBlock to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > replacedBlock;
    baseSurface.add_property(replacedBlock, "replacedBlock");

    // Add replacedBlock_combined to each face
    OpenMesh::FPropHandleT< tuple<vector<Vector3d>, vector<Vector3d>> > replacedBlock_combined;
    baseSurface.add_property(replacedBlock_combined, "replacedBlock_combined");

    // Add centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList;
    baseSurface.add_property(centroidBlockFaceHandleList, "centroidBlockFaceHandleList");

    // Add centroidBlockFaceHandleList to each face
    OpenMesh::FPropHandleT< vector<PolyMesh::FaceHandle> > centroidBlockFaceHandleList_planar;
    baseSurface.add_property(centroidBlockFaceHandleList_planar, "centroidBlockFaceHandleList_planar");

    /// Then we add 4 properties for edge clustering of each edge
    /// (1) edgeLength  (2) similarityList  (3) edgeClusteringLabel  (4) edgeClusteringCentroid

    // Add edgeLength for each edge
    OpenMesh::EPropHandleT< double > edgeLength;
    baseSurface.add_property(edgeLength, "edgeLength");

    // Add similarityList for each edge
    OpenMesh::EPropHandleT< vector<double> > similarityList;
    baseSurface.add_property(similarityList, "similarityList");

    // Add edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface.add_property(edgeClusteringLabel, "edgeClusteringLabel");

    // Add edgeClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > edgeClusterCentroid;
    baseSurface.add_property(edgeClusterCentroid, "edgeClusterCentroid");

    // Add edgeClusterLabelList for each face
    OpenMesh::FPropHandleT< vector<int> > edgeClusterLabelList;
    baseSurface.add_property(edgeClusterLabelList, "edgeClusterLabelList");

    /// Then we add 4 properties for dihedral angle clustering of each edge
    /// (1) dihedralAngle  (2) angleSimiList  (3) dihedralAngleLabel (4) angleClusterCentroid

    // Add dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface.add_property(dihedralAngle, "dihedralAngle");

    // Add dihedral angle similarity list for each edge
    OpenMesh::EPropHandleT< vector<double> > angleSimiList;
    baseSurface.add_property(angleSimiList, "angleSimiList");

    // Add dihedralAngleLabel for each edge
    OpenMesh::EPropHandleT< int > dihedralAngleLabel;
    baseSurface.add_property(dihedralAngleLabel, "dihedralAngleLabel");

    // Add angleClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > angleClusterCentroid;
    baseSurface.add_property(angleClusterCentroid, "angleClusterCentroid");

    // Add isFixed for each edge
    OpenMesh::EPropHandleT< bool > isFixed;
    baseSurface.add_property(isFixed, "isFixed");

    /// Then we add 5 properties to the whole mesh
    /// (1) similarityTable  (2) angleSimiTable  (3) averageAngle (4) averageLength (5) polygonClusterList

    // Add similarityTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > similarityTable;
    baseSurface.add_property(similarityTable, "similarityTable");

    // Add angleSimiTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > angleSimiTable;
    baseSurface.add_property(angleSimiTable, "angleSimiTable");

    // Add polygonClusterList to mesh
    OpenMesh::MPropHandleT< vector<vector<int>> > polygonClusterList;
    baseSurface.add_property(polygonClusterList, "polygonClusterList");

    // Add polygonClusterFullList to mesh
    OpenMesh::MPropHandleT< vector<vector<vector<int>>> > polygonClusterFullList;
    baseSurface.add_property(polygonClusterFullList, "polygonClusterFullList");

    // Add polygonSimiTable
    OpenMesh::MPropHandleT< vector<vector<double>> > polygonSimiTable;
    baseSurface.add_property(polygonSimiTable, "polygonSimiTable");

    // Add edgeClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<double> > edgeClusterCentroidList;
    baseSurface.add_property(edgeClusterCentroidList, "edgeClusterCentroidList");

    // Add angleClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<double> > angleClusterCentroidList;
    baseSurface.add_property(angleClusterCentroidList, "angleClusterCentroidList");

    // Add diagonalClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > diagonalClusterCentroidList;
    baseSurface.add_property(diagonalClusterCentroidList, "diagonalClusterCentroidList");

    // Add centroidPolygonList to mesh
    OpenMesh::MPropHandleT< vector<vector<Vector3d>> > centroidPolygonList;
    baseSurface.add_property(centroidPolygonList, "centroidPolygonList");

    // Add problemVerPair to mesh
    OpenMesh::MPropHandleT< vector<std::tuple<Vector3d, Vector3d, int>> > problemVerPair;
    baseSurface.add_property(problemVerPair, "problemVerPair");

    // Add cuttingAngleCentroidList to mesh
    OpenMesh::MPropHandleT< vector<double> > cuttingAngleCentroidList;
    baseSurface.add_property(cuttingAngleCentroidList, "cuttingAngleCentroidList");

    // Add blockCuttingAngleLabelList to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList;
    baseSurface.add_property(blockCuttingAngleLabelList, "blockCuttingAngleLabelList");

    // Add blockCuttingAngleLabelList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleLabelList_full;
    baseSurface.add_property(blockCuttingAngleLabelList_full, "blockCuttingAngleLabelList_full");

    // Add blockCuttingAngleList_full to mesh
    OpenMesh::MPropHandleT< vector< vector< vector<int> > > > blockCuttingAngleList_full;
    baseSurface.add_property(blockCuttingAngleList_full, "blockCuttingAngleList_full");

    // Add blockSimilarityTable to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > blockSimilarityTable;
    baseSurface.add_property(blockSimilarityTable, "blockSimilarityTable");

    // Add blockCentroidList to mesh
    OpenMesh::MPropHandleT< vector<tuple<vector<Vector3d>, vector<Vector3d>>> > blockCentroidList;
    baseSurface.add_property(blockCentroidList, "blockCentroidList");

    // Add blockCentroidList_combined to mesh
    OpenMesh::MPropHandleT< vector< tuple<vector<Vector3d>, vector<Vector3d>> > > blockCentroidList_combined;
    baseSurface.add_property(blockCentroidList_combined, "blockCentroidList_combined");

    // Add blockCentroidList_planar to mesh
    OpenMesh::MPropHandleT< vector<tuple<vector<Vector3d>, vector<Vector3d>>> > blockCentroidList_planar;
    baseSurface.add_property(blockCentroidList_planar, "blockCentroidList_planar");

    // Add worstFabVerHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::VertexHandle > worstFabVerHandle;
    baseSurface.add_property(worstFabVerHandle, "worstFabVerHandle");

    // Add worstPolygonSimiFaceHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::FaceHandle > worstPolygonSimiFaceHandle;
    baseSurface.add_property(worstPolygonSimiFaceHandle, "worstPolygonSimiFaceHandle");

    // Add worstPlanarFaceHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::FaceHandle > worstPlanarFaceHandle;
    baseSurface.add_property(worstPlanarFaceHandle, "worstPlanarFaceHandle");

    // Add worstDihedralAngleEdgeHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::EdgeHandle > worstDihedralAngleEdgeHandle;
    baseSurface.add_property(worstDihedralAngleEdgeHandle, "worstDihedralAngleEdgeHandle");

    // Add cuttingAngleAndEdgeLabelPairs to mesh
    OpenMesh::MPropHandleT< vector< tuple<float, int> > > cuttingAngleAndEdgeLabelPairs;
    baseSurface.add_property(cuttingAngleAndEdgeLabelPairs, "cuttingAngleAndEdgeLabelPairs");

    // Add blockClusterFaceHandleList to mesh
    OpenMesh::MPropHandleT< vector< vector<PolyMesh::FaceHandle> > > blockClusterFaceHandleList;
    baseSurface.add_property(blockClusterFaceHandleList, "blockClusterFaceHandleList");

    // Add blockClusterConnectionGraph
    OpenMesh::MPropHandleT< vector< vector<int> > > blockClusterConnectionGraph;
    baseSurface.add_property(blockClusterConnectionGraph, "blockClusterConnectionGraph");

    // Add blockSingleNeighbourNumList
    OpenMesh::MPropHandleT< vector<int> > blockSingleNeighbourNumList;
    baseSurface.add_property(blockSingleNeighbourNumList, "blockSingleNeighbourNumList");

    // Add isBoundaryBlockList
    OpenMesh::MPropHandleT< vector<int> > isBoundaryBlockList;
    baseSurface.add_property(isBoundaryBlockList, "isBoundaryBlockList");

    int dihedralAngleNum = 0;
    double totalAngle = 0;
    double totalLength = 0;
    int faceIndex = 0;

    for (PolyMesh::VertexIter v_it=baseSurface.vertices_begin(); v_it!=baseSurface.vertices_end(); ++v_it)
    {
        baseSurface.property(origPoint, *v_it) = baseSurface.point(*v_it);
    }

    for (PolyMesh::FaceIter f_it=baseSurface.faces_begin(); f_it!=baseSurface.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;
        vector<Vector3d> currList;

        baseSurface.property(isBoundaryFace, *f_it) = false;

        for (fv_it = baseSurface.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
        {
            auto curr = baseSurface.point(*fv_it);
            currList.push_back(Vector3d(curr[0], curr[1], curr[2]));
        }

        PolyMesh::FaceEdgeIter fe_it;

        for (fe_it = baseSurface.fe_iter(*f_it); fe_it.is_valid(); ++fe_it)
        {
            if (baseSurface.is_boundary(*fe_it))
            {
                baseSurface.property(isBoundaryFace, *f_it) = true;
                break;
            }
        }

        baseSurface.property(faceCenter, *f_it) = ComputePolygonCenter(currList);
        baseSurface.property(index, *f_it)  = faceIndex;
        baseSurface.property(clusteringLabel, *f_it) = 0;
        baseSurface.property(blockClusteringLabel, *f_it) = 0;
        baseSurface.property(isSatisfyFabRequirement, *f_it) = true;
        baseSurface.property(polygonCentroidDistance, *f_it) = 0;
        baseSurface.property(blockClusteringLabel, *f_it) = 0;
        baseSurface.property(seqID, *f_it) = 0;

        faceIndex++;
    }

    baseSurface.property(similarityTable).clear();
    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
        baseSurface.property(isHold, *e_it) = !baseSurface.is_boundary(*e_it);
        baseSurface.property(isFixed, *e_it) = false;

        const PolyMesh::Point to   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));
        const PolyMesh::Point from = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it,0)));

        double l1 = (to - from).norm();

        vector<Vector3d> currList;
        vector<Vector3d> currCenterList;

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
                    currCenterList.push_back(baseSurface.property(faceCenter, *f_it));
                }
            }
        }

        if (currList.size() == 2)
        {
            Vector3d edgeNormal = (currList[0] + currList[1]) / 2;
            edgeNormal.normalize();
            baseSurface.property(normal, *e_it) = edgeNormal;

            currList[0].normalize();
            currList[1].normalize();

            double angle;

            angle = double(baseSurface.calc_dihedral_angle(*e_it));
            angle = 180 - angle / 3.1415926 * 180;

            baseSurface.property(dihedralAngle, *e_it) = angle;
//            cout << angle  << ", ";

            dihedralAngleNum++;
            totalAngle += angle;
        }
        else if (currList.size() == 1)
        {
            Vector3d edgeNormal = (currList[0]) / 2;
            edgeNormal.normalize();
            baseSurface.property(normal, *e_it) = edgeNormal;
            baseSurface.property(dihedralAngle, *e_it) = -1;
        }
        else
        {
            cout << "Loading mesh error! Alone edge here. " << endl;
        }

        baseSurface.property(edgeLength, *e_it) = (to - from).norm();

        totalLength +=  (to - from).norm();

        baseSurface.property(similarityList, *e_it).clear();
        for (PolyMesh::EdgeIter e_it_2=baseSurface.edges_begin(); e_it_2!=baseSurface.edges_end(); ++e_it_2)
        {
            const PolyMesh::Point currTo   = baseSurface.point(baseSurface.to_vertex_handle(baseSurface.halfedge_handle(*e_it_2,0)));
            const PolyMesh::Point currFrom = baseSurface.point(baseSurface.from_vertex_handle(baseSurface.halfedge_handle(*e_it_2,0)));

            double l2 = (currTo - currFrom).norm();

            double diff = fabs(l1 - l2);

            baseSurface.property(similarityList, *e_it).push_back(diff);
        }

        baseSurface.property(similarityTable).push_back(baseSurface.property(similarityList, *e_it));

        baseSurface.property(edgeClusteringLabel, *e_it) = 0;

        if (baseSurface.property(dihedralAngle, *e_it) == -1)
            baseSurface.property(dihedralAngleLabel, *e_it) = -1;
        else
        {
//            cout << baseSurface.property(dihedralAngle, *e_it) << " ";
            baseSurface.property(dihedralAngleLabel, *e_it) = 0;
        }
    }

//    cout << endl;

    baseSurface.property(angleSimiTable).clear();
    for (PolyMesh::EdgeIter e_it=baseSurface.edges_begin(); e_it!=baseSurface.edges_end(); ++e_it)
    {
//        cout << baseSurface.calc_edge_length(*e_it) << ", ";

        if (baseSurface.property(dihedralAngle, *e_it) == -1)
            continue;

        baseSurface.property(angleSimiList, *e_it).clear();
        for (PolyMesh::EdgeIter e_it_2=baseSurface.edges_begin(); e_it_2!=baseSurface.edges_end(); ++e_it_2)
        {
            if (baseSurface.property(dihedralAngle, *e_it_2) == -1)
                continue;

            baseSurface.property(angleSimiList, *e_it).push_back(fabs(baseSurface.property(dihedralAngle, *e_it) - baseSurface.property(dihedralAngle, *e_it_2)));
        }

        baseSurface.property(angleSimiTable).push_back(baseSurface.property(angleSimiList, *e_it));
    }

//    cout << endl;
}