///////////////////////////////////////////////////////////////
//
// MeshObject.cpp
//
//   An object represented as a group of meshes, mainly for rendering
//
// by Peng Song
//
// 29/Nov/2020
//
//
///////////////////////////////////////////////////////////////


#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include "Utility/HelpFunc.h"
#include "Mesh.h"
#include "MeshCreator.h"
#include "MeshObject.h"
#include "MeshBoolean.h"
#include <igl/gaussian_curvature.h>
#include <igl/massmatrix.h>
#include <igl/invert_diag.h>
#include <igl/jet.h>
#include <Utility/tinycolormap.hpp>

RowVector3d ColorTable[16] = {
        RowVector3d(0.9, 0.5, 0.5),   //  1: Red
        RowVector3d(0.4, 0.9, 0.4),   //  2: Green
        RowVector3d(0.4, 0.4, 0.9),   //  3: Blue
        RowVector3d(0.9, 0.9, 0.5),   //  4: Yellow
        RowVector3d(0.7, 0.3, 0.9),   //  5: Purple
        RowVector3d(0.4, 0.9, 0.9),   //  6: Cyan

        RowVector3d(0.9, 0.6, 0.4),   //  7: Orange
        RowVector3d(0.6, 0.6, 0.9),   //  8: Light Blue
        RowVector3d(0.8, 0.8, 0.8),   //  9: Light Gray
        RowVector3d(0.9, 0.3, 0.6),   // 10: Pink
        RowVector3d(0.6, 0.4, 0.3),   // 11: Brown
        RowVector3d(0.9, 0.6, 0.5),   // 12: LightSalmon

        RowVector3d(0.5, 0.2, 0.5),   // 13: Dark Purple
        RowVector3d(0.4, 0.8, 0.7),   // 14: Dark Cyan
        RowVector3d(0.3, 0.4, 0.7),   // 15: Dark Blue
        RowVector3d(0.6, 0.6, 0.3),   // 16: Dark Yellow
};


///=========================================================================================///
///                                        Ground
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject::CreateGround(Vector3d origin, double size, int gridNum, int sampNum)
{
    double halfSize = 0.5f * size;
    double cyliRadius = 0.002 * size;


    /////////////////////////////////////////////////////////////////////////
    /// 1. Create 7 meshes for the axes (3 cylinders, 3 cones, and 1 sphere)

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    vector<Mesh*> meshList;

    for (int i = 0; i <= gridNum; i++)
    {
        double z = -halfSize + (i/(double)gridNum) * size;

        Vector3d point1 = origin + Vector3d( -halfSize, 0, z);
        Vector3d point2 = origin + Vector3d(  halfSize, 0 ,z);

        Mesh *cylinder = meshCreator.CreateCylinder(point1, point2, cyliRadius, sampNum);

        meshList.push_back( cylinder );

    }

    for (int i = 0; i <= gridNum; i++)
    {
        double x = -halfSize + (i/(double)gridNum) * size;

        Vector3d point1 = origin + Vector3d( x, 0,  -halfSize);
        Vector3d point2 = origin + Vector3d( x, 0,   halfSize);

        Mesh *cylinder = meshCreator.CreateCylinder(point1, point2, cyliRadius, sampNum);

        meshList.push_back( cylinder );
    }

    /////////////////////////////////////////////////////////////////////////
    /// 1&1/2. Merge all cylinders as a ground
    /*
    Mesh* MGround = meshList[0];
    for(auto mcy : meshList)
        MGround = meshBoolean.MeshUnion(mcy, MGround);
    meshList.clear();
    meshList.push_back(MGround);
    */
    /////////////////////////////////////////////////////////////////////////
    /// 2. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;

    for (auto mesh : meshList)
    {
        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(mesh->verM, mesh->triM);

        viewerData.set_colors(Eigen::RowVector3d(0.8, 0.8, 0.8));

        viewerData.show_lines = unsigned (0);
        viewerData.face_based = true;

        viewerDataList.push_back( viewerData );
    }


    /////////////////////////////////////////////////////////////////////////
    /// 3. Release memory for the meshList

    for (auto & i : meshList)
    {
        delete i;
    }
    meshList.clear();

    return viewerDataList;
}




///=========================================================================================///
///                                         Axes
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject::CreateAxes(Vector3d origin, double size, int sampNum)
{
    /////////////////////////////////////////////////////////////////////////
    /// 1. Compute and specify parameters for the axes

    Vector3d xPoint1 = origin + Vector3d( 0.9*size, 0,         0);
    Vector3d xPoint2 = origin + Vector3d( 1.0*size, 0,         0);
    Vector3d yPoint1 = origin + Vector3d( 0,         0.9*size, 0);
    Vector3d yPoint2 = origin + Vector3d( 0,         1.0*size, 0);
    Vector3d zPoint1 = origin + Vector3d( 0,         0,         0.9*size);
    Vector3d zPoint2 = origin + Vector3d( 0,         0,         1.0*size);

    double cyliRadius = 0.02 * size;
    double coneRadius = 0.04 * size;
    double spheRadius = 0.04 * size;


    /////////////////////////////////////////////////////////////////////////
    /// 2. Create 7 meshes for the axes (3 cylinders, 3 cones, and 1 sphere)

    MeshCreator meshCreator;

    Mesh *cylinderX = meshCreator.CreateCylinder(origin, xPoint1, cyliRadius, sampNum);
    Mesh *cylinderY = meshCreator.CreateCylinder(origin, yPoint1, cyliRadius, sampNum);
    Mesh *cylinderZ = meshCreator.CreateCylinder(origin, zPoint1, cyliRadius, sampNum);

    Mesh *coneX = meshCreator.CreateCone(xPoint1, xPoint2, coneRadius, sampNum);
    Mesh *coneY = meshCreator.CreateCone(yPoint1, yPoint2, coneRadius, sampNum);
    Mesh *coneZ = meshCreator.CreateCone(zPoint1, zPoint2, coneRadius, sampNum);

    Mesh *sphere = meshCreator.CreateSphere(spheRadius, origin, sampNum, 2*sampNum);

    vector<Mesh*> meshList;

    meshList.push_back( cylinderX );
    meshList.push_back( coneX );

    meshList.push_back( cylinderY );
    meshList.push_back( coneY );

    meshList.push_back( cylinderZ );
    meshList.push_back( coneZ );

    meshList.push_back( sphere );


    /////////////////////////////////////////////////////////////////////////
    /// 3. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;

    for (int i = 0; i < meshList.size(); i++)
    {
        Mesh *mesh = meshList[i];

        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(mesh->verM, mesh->triM);

        if     ( i == 0 || i == 1 )    viewerData.set_colors(Eigen::RowVector3d(0.9, 0.4, 0.4));
        else if( i == 2 || i == 3 )    viewerData.set_colors(Eigen::RowVector3d(0.4, 0.9, 0.4));
        else if( i == 4 || i == 5 )    viewerData.set_colors(Eigen::RowVector3d(0.4, 0.4, 0.9));
        else                           viewerData.set_colors(Eigen::RowVector3d(0.6, 0.6, 0.6));

        viewerData.show_lines = unsigned (0);
        viewerData.face_based = true;

        viewerDataList.push_back( viewerData );
    }


    /////////////////////////////////////////////////////////////////////////
    /// 4. Release memory for the meshList

    for (auto & i : meshList)
    {
        delete i;
    }
    meshList.clear();

    return viewerDataList;
}


///=========================================================================================///
///                                  Obj Model MeshCreator
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject::CreateObjModel(MatrixX3d verM, MatrixX3i triM, RowVector3d color)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    viewerData.set_mesh(verM, triM);

    MatrixXd C(triM.rows(), 3);
    for (int i = 0; i < triM.rows(); ++i)
    {
        C(i, 0) = color[0];
        C(i, 1) = color[1];
        C(i, 2) = color[2];
    }
    viewerData.set_colors(C);

    viewerData.show_lines = unsigned (0);
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateObjModel(MatrixX3d verM, MatrixX3i triM)
{
    VectorXd K;
    // Compute integral of Gaussian curvature
    igl::gaussian_curvature(verM,triM,K);
    // Compute mass matrix
    SparseMatrix<double> M,Minv;
    igl::massmatrix(verM,triM,igl::MASSMATRIX_TYPE_DEFAULT,M);
    igl::invert_diag(M,Minv);
    // Divide by area to get integral average
    K = (Minv*K).eval();

    double minValue = 10000;

    for (int i = 0; i < K.size(); ++i)
    {
//        K(i) = -atan(K(i) * 0.4) * 0;
        if (fabs(K(i)) > 100)
            K(i) = 0;

//        if (K(i) > 0)
//            K(i) = -K(i);

//        K(i) = atan(K(i));
//        else
//            K(i) -= 0.727;
        else
        {
            if (fabs(K(i)) < minValue)
            {
                minValue = fabs(K(i));
            }
        }
//        cout << K(i) << endl;
    }
//    cout << "min: " << minValue << endl;

    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    viewerData.set_mesh(verM, triM);
//    viewerData.set_data(K, igl::COLOR_MAP_TYPE_VIRIDIS, 255);
    viewerData.set_data(K, -3, 3, igl::COLOR_MAP_TYPE_VIRIDIS, 255);

//    dbg(K);

    // Set color
//    MatrixXd Ct;
//    RedColorInterpolation(planarityList, Ct);
//    viewerData.set_colors(Ct);

    MatrixXd C(triM.rows(), 3);
    for (int i = 0; i < triM.rows(); ++i)
    {
//        double currK = (K(i) - K.minCoeff()) / (K.maxCoeff() - K.minCoeff());
//        tinycolormap::Color color = tinycolormap::GetQuantizedColor(currK, 255, tinycolormap::ColormapType::Parula);

        C(i, 0) = 0.85;
        C(i, 1) = 0.85;
        C(i, 2) = 0.85;
    }
//    viewerData.set_colors(C);

    viewerData.show_lines = unsigned (0);
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateObjModelWithEdgeCluster(MatrixX3d verM, MatrixX3i triM,
                                                                MatrixX3d e1_mat, MatrixX3d e2_mat,
                                                                VectorXd edgeLabelList)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    // Set mesh
    viewerData.set_mesh(verM, triM);

    MatrixXd C(triM.rows(), 3);
    for (int i = 0; i < triM.rows(); ++i)
    {
        C(i, 0) = 1;
        C(i, 1) = 1;
        C(i, 2) = 1;
    }
    viewerData.set_colors(C);

    // Set edge
    for (int i = 0; i < e1_mat.rows(); ++i)
    {
        RowVector3d currColor = ColorTable[int(edgeLabelList(i)) % 16];
        viewerData.add_edges(e1_mat.row(i), e2_mat.row(i), currColor);
    }

    viewerData.show_lines = false;
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateRemeshedSurface(MatrixX3d verM, MatrixX3i triM, MatrixX3d p1_mat, MatrixX3d p2_mat, MatrixX3d e1_mat, MatrixX3d e2_mat, VectorXd planarityList, bool isShowEdgeNormal)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    // Set mesh
    viewerData.set_mesh(verM, triM);

    // Set color
    MatrixXd C(triM.rows(), 3);
    for (int i = 0; i < triM.rows(); ++i)
    {
        C(i, 0) = 1;
        C(i, 1) = 1;
        C(i, 2) = 1;
    }
    viewerData.set_colors(C);

    // Set edge
    for (int i = 0; i < p1_mat.rows(); ++i)
    {
        viewerData.add_edges(p1_mat.row(i), p2_mat.row(i), Eigen::RowVector3d(0.0, 0.0, 0.0));
    }

    if (isShowEdgeNormal)
    {
        // Set edge normal
        for (int i = 0; i < e1_mat.rows(); ++i)
        {
            viewerData.add_edges(e1_mat.row(i), e2_mat.row(i), Eigen::RowVector3d(0.9,0.5,0.5));
        }
    }

    viewerData.show_lines = false;
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateTexture(MatrixX3d verM, MatrixX3i triM, MatrixX3d p1_mat, MatrixX3d p2_mat, bool isShowLine)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    // Set mesh
    viewerData.set_mesh(verM, triM);

    MatrixXd C(triM.rows(), 3);
    for (int i = 0; i < triM.rows(); ++i)
    {
        C(i, 0) = 1;
        C(i, 1) = 1;
        C(i, 2) = 1;
    }
    viewerData.set_colors(C);

    // Set edge
    for (int i = 0; i < p1_mat.rows(); ++i)
    {
        viewerData.add_edges(p1_mat.row(i), p2_mat.row(i), Eigen::RowVector3d(0.4, 0.4, 0.9));
    }

    viewerData.show_lines = false;
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

// Optimized surface
vector<igl::opengl::ViewerData> MeshObject::CreateOptimizedSurface(MatrixX3d verM, MatrixX3i triM,
                                                                   MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                                   MatrixX3d e1_mat, MatrixX3d e2_mat,
                                                                   VectorXd edgeLabelList, VectorXd polygonLabelList,
                                                                   bool isShowEdgeClusterColor, bool isShowPolygonColor, bool isShowEdgeNormal)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    // Set mesh
    viewerData.set_mesh(verM, triM);

    // Set color
    if (isShowPolygonColor)
    {
        MatrixXd C(triM.rows(), 3);
        for (int i = 0; i < triM.rows(); ++i)
        {
            RowVector3d currColor = ColorTable[int(polygonLabelList(i)) % 16];
            C(i, 0) = currColor(0);
            C(i, 1) = currColor(1);
            C(i, 2) = currColor(2);
        }
        viewerData.set_colors(C);
    }
    else
    {
        MatrixXd C(triM.rows(), 3);
        for (int i = 0; i < triM.rows(); ++i)
        {
            RowVector3d currColor = ColorTable[int(polygonLabelList(i)) % 16];
            C(i, 0) = 1;
            C(i, 1) = 1;
            C(i, 2) = 1;
        }
        viewerData.set_colors(C);
    }

    // Set edge
    for (int i = 0; i < p1_mat.rows(); ++i)
    {
        RowVector3d currColor;
        if (isShowEdgeClusterColor)
            currColor = ColorTable[int(edgeLabelList(i)) % 16];
        else
            currColor = RowVector3d(0, 0, 0);

        viewerData.add_edges(p1_mat.row(i), p2_mat.row(i), currColor);
//        viewerData.line_width = 50.0f;
    }

    if (isShowEdgeNormal)
    {
        // Set edge normal
        for (int i = 0; i < e1_mat.rows(); ++i)
        {
            viewerData.add_edges(e1_mat.row(i), e2_mat.row(i), Eigen::RowVector3d(0.9,0.5,0.5));
        }
    }

    viewerData.show_lines = false;
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateReplacedSurfaceModel(MatrixX3d verM, MatrixX3i triM,
                                                           MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                           MatrixX3d fp1_mat, MatrixX3d fp2_mat,
                                                           VectorXd fabLabelList)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

//    // Set mesh
//    viewerData.set_mesh(verM, triM);
//
//    // Set color
//    MatrixXd C(triM.rows(), 3);
//    for (int i = 0; i < triM.rows(); ++i)
//    {
//        C(i, 0) = 1;
//        C(i, 1) = 1;
//        C(i, 2) = 1;
//    }
//    viewerData.set_colors(C);

    // Set edge
    for (int i = 0; i < p1_mat.rows(); ++i)
    {
        viewerData.add_edges(p1_mat.row(i), p2_mat.row(i), RowVector3d(0.0, 0.0, 0.0));
    }

    // Set fab test edge
    for (int i = 0; i < fp1_mat.rows(); ++i)
    {
        if (fabLabelList(i) == CASE_PENETRATION)
            viewerData.add_edges(fp1_mat.row(i), fp2_mat.row(i), RowVector3d(0.9, 0.5, 0.5));

        else
        {
            viewerData.add_edges(fp1_mat.row(i), fp2_mat.row(i), RowVector3d(0.4, 0.9, 0.4));
        }
    }

    viewerData.show_lines = false;
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateShellStructure(MatrixX3d verM, MatrixX3i triM,
                                                     MatrixX3d p1_mat, MatrixX3d p2_mat,
                                                     VectorXd polygonLabelList)
{
    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    // Set mesh
    viewerData.set_mesh(verM, triM);

    // Set color
    MatrixXd C(triM.rows(), 3);
    for (int i = 0; i < triM.rows(); ++i)
    {
        RowVector3d currColor = ColorTable[int(polygonLabelList(i)) % 16];
        C(i, 0) = currColor(0);
        C(i, 1) = currColor(1);
        C(i, 2) = currColor(2);
    }
    viewerData.set_colors(C);

    // Set edge
    for (int i = 0; i < p1_mat.rows(); ++i)
    {
        RowVector3d currColor(0,0,0);

        viewerData.add_edges(p1_mat.row(i), p2_mat.row(i), currColor);
    }

    viewerData.show_lines = false;
    viewerData.face_based = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );

    return viewerDataList;
}

vector<igl::opengl::ViewerData> MeshObject::CreateVolume(MatrixXd voxelMinPts,
                                                         MatrixXd voxelMaxPts,
                                                         MatrixXd voxelCenPts)
{
    /////////////////////////////////////////////////////////////////////////
    /// 1. Create meshes for puzzle voxels

    MeshCreator meshCreator;

    Vector3d minPt;
    Vector3d maxPt;

    // Check the number of vertices and tris in each cube mesh
    minPt(0) = voxelMinPts(0,0);
    minPt(1) = voxelMinPts(0,1);
    minPt(2) = voxelMinPts(0,2);

    maxPt(0) = voxelMaxPts(0,0);
    maxPt(1) = voxelMaxPts(0,1);
    maxPt(2) = voxelMaxPts(0,2);

    Mesh *testCube = meshCreator.CreateCuboid(minPt, maxPt);
    int verNum = testCube->verM.rows();
    int triNum = testCube->triM.rows();

    Mesh * volumeCubeMesh = new Mesh();

    volumeCubeMesh->verM.resize(verNum * voxelMinPts.rows(), 3);
    volumeCubeMesh->triM.resize(triNum * voxelMinPts.rows(), 3);

    for(int j = 0; j < voxelMinPts.rows(); j++)
    {
        minPt(0) = voxelMinPts(j,0);
        minPt(1) = voxelMinPts(j,1);
        minPt(2) = voxelMinPts(j,2);

        maxPt(0) = voxelMaxPts(j,0);
        maxPt(1) = voxelMaxPts(j,1);
        maxPt(2) = voxelMaxPts(j,2);

        Mesh *cuboid = meshCreator.CreateCuboid(minPt, maxPt);

        for (int z = 0; z < cuboid->verM.rows(); ++z)
        {
            volumeCubeMesh->verM(j * verNum + z, 0) = cuboid->verM(z, 0);
            volumeCubeMesh->verM(j * verNum + z, 1) = cuboid->verM(z, 1);
            volumeCubeMesh->verM(j * verNum + z, 2) = cuboid->verM(z, 2);
        }

        for (int z = 0; z < cuboid->triM.rows(); ++z)
        {
            volumeCubeMesh->triM(j * triNum + z, 0) = cuboid->triM(z, 0) + j * verNum;
            volumeCubeMesh->triM(j * triNum + z, 1) = cuboid->triM(z, 1) + j * verNum;
            volumeCubeMesh->triM(j * triNum + z, 2) = cuboid->triM(z, 2) + j * verNum;
        }
    }

    /////////////////////////////////////////////////////////////////////////
    /// 2. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;

    igl::opengl::ViewerData viewerData;

    viewerData.set_mesh(volumeCubeMesh->verM, volumeCubeMesh->triM);

    viewerData.set_colors(ColorTable[0]);

    viewerData.show_lines = unsigned (0);
    viewerData.face_based = true;
    viewerData.is_visible = true;
    viewerData.double_sided = true;

    viewerDataList.push_back( viewerData );


    return viewerDataList;
}



///=========================================================================================///
///                               Fuction for testing MeshCreator
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject:: Function_Test()
{
    /////////////////////////////////////////////////////////////////////////
    /// 1. Create meshes for debug

    MeshCreator meshCreator;
    //Mesh *cuboid = meshCreator.CreateCuboid();
    //Mesh *sphere = meshCreator.CreateSphere(1.0, 10, 20);
    Mesh *sphere = meshCreator.CreateSphere(1.0, Vector3d(0, 2, 0), 100, 200);

    //Mesh *cylinder = meshCreator.CreateCylinder(2.0, 0.2, 100);
    Mesh *cylinder = meshCreator.CreateCylinder(Vector3d(0,0,0), Vector3d(1,1,1), 0.1, 100);

    Mesh *cone = meshCreator.CreateCone(Vector3d(1,1,1), Vector3d(1.2,1.2,1.2), 0.2, 100);
    //Mesh *cone = meshCreator.CreateCone(2.0, 2.0, 100);

    Mesh *cuboid = meshCreator.CreateCuboid(Vector3d(-1, 0.1, -1), Vector3d(1, 0.5, 1));
    // Mesh *cuboid = meshCreator.CreateCuboid(Vector3d(2, 0.2, 4));

    Mesh *fan = meshCreator.CreateFan(2.0, 0.6, 0.5, 0.4);

    vector<Mesh*> meshList;

    meshList.push_back( sphere );
    meshList.push_back( cylinder );

    meshList.push_back( cone );
    meshList.push_back( cuboid );

    meshList.push_back( fan );


    /////////////////////////////////////////////////////////////////////////
    /// 2. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;
    for (auto mesh : meshList)
    {
        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(mesh->verM, mesh->triM);

        viewerData.show_lines = unsigned (0);
        viewerData.face_based = true;

        viewerDataList.push_back( viewerData );
    }


    /////////////////////////////////////////////////////////////////////////
    /// 3. Release memory for the meshList

    for (auto & i : meshList)
    {
        delete i;
    }
    meshList.clear();

    return viewerDataList;
}