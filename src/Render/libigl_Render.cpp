///////////////////////////////////////////////////////////////
//
// libigl_Render.cpp
//
//   Rendering various 3D contents with libigl
//
// by Peng Song
//
// 01/Dec/2020
//
//
///////////////////////////////////////////////////////////////


#include <utility>
#include <vector>
#include <Eigen/Eigen>
#include "libigl_Render.h"
#include "Mesh/MeshObject.h"
#include "Utility/HelpFunc.h"

///=========================================================================================///
///                                        Initialization
///=========================================================================================///

libigl_Render::libigl_Render()
{
    groundMeshNum = 0;
    axesMeshNum = 0;

    objMeshNum = 0;
    objMeshTextureNum = 0;
    objRemeshingMeshNum = 0;

    objOptimizedSurfaceNum = 0;
    objReplacedSurfaceNum = 0;

    objShellStrucNum = 0;
    objReplacedShellNum = 0;
    objReplacedShellPlanarNum = 0;
    objReplacedPenetrationVolumeNum = 0;

}

libigl_Render::~libigl_Render()
{

}




///=========================================================================================///
///                                       Render Objects
///=========================================================================================///

void libigl_Render::RenderScene(iglViewer &viewer)
{
//    DrawMechanism(viewer, std::move(myViewerData)); // We have to draw mechanism first (for animating it properly)
    DrawGround(viewer, Vector3d(0,0,0), 4.0, 10, 10);
    DrawWorldAxes(viewer, Vector3d(0,0,0), 2.0, 20);
    //DrawMeshForDebug(viewer);
}

void libigl_Render::DrawGround(iglViewer &viewer, Vector3d origin, double size, int gridNum, int sampNum)
{
    MeshObject meshObject;
    vector<igl::opengl::ViewerData> viewerDataList =  meshObject.CreateGround(std::move(origin), size, gridNum, sampNum);

    AppendDataToViewer( viewer, viewerDataList );

    groundMeshNum = viewerDataList.size();
}

void libigl_Render::DrawWorldAxes(iglViewer &viewer, Vector3d origin, double size, int sampNum)
{
    MeshObject meshObject;
    vector<igl::opengl::ViewerData> viewerDataList =  meshObject.CreateAxes(std::move(origin), size, sampNum);

    AppendDataToViewer( viewer, viewerDataList );

    axesMeshNum = viewerDataList.size();
}

void libigl_Render::DrawObjModelWithEdgeCluster(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh, const PolyMesh optimizedPolyMesh)
{
    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(triPolyMesh, verM, triM);

    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the edge label data
    VectorXd edgeLabelList;

    PolyMesh2EdgeClusteringLabel(optimizedPolyMesh, edgeLabelList);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateObjModelWithEdgeCluster(verM, triM, p1_mat, p2_mat, edgeLabelList);

    AppendDataToViewer(viewer, viewerDataList);

    objRemeshingMeshNum = viewerDataList.size();
}

void libigl_Render::DrawObjModel(iglViewer &viewer, const PolyMesh polyMesh)
{
    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(polyMesh, verM, triM);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateObjModel(verM, triM);

    AppendDataToViewer(viewer, viewerDataList);

    objMeshNum = viewerDataList.size();
}

void libigl_Render::DrawObjModelTexture(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(triPolyMesh, verM, triM);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateTexture(verM, triM, p1_mat, p2_mat, false);

    AppendDataToViewer(viewer, viewerDataList);

    objMeshTextureNum = viewerDataList.size();
}

void libigl_Render::DrawObjRemeshingModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh, bool isShowEdgeNormal)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(triPolyMesh, verM, triM);

    /// Prepare the planarity data
    VectorXd planarityList;

    PolyMesh2Planarity(triPolyMesh, planarityList);

    /// Prepare the edge normal data
    MatrixXd e1_mat;
    MatrixXd e2_mat;

    PolyMesh2EdgeNormal(polyMesh, e1_mat, e2_mat);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateRemeshedSurface(verM, triM, p1_mat, p2_mat, e1_mat, e2_mat, planarityList, isShowEdgeNormal);

    AppendDataToViewer(viewer, viewerDataList);

    objRemeshingMeshNum = viewerDataList.size();
}

void libigl_Render::DrawObjOptimizedModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh,
                                          bool isShowEdgeClusterColor, int polygonColorState, bool isShowEdgeNormal)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(triPolyMesh, verM, triM);

    /// Prepare the edge label data
    VectorXd edgeLabelList;

    PolyMesh2EdgeClusteringLabel(polyMesh, edgeLabelList);

    /// Prepare the polygon label data
    VectorXd polygonLabelList;

    if (polygonColorState != SHOW_WHITE_FACE)
    {
        if (polygonColorState == SHOW_POLYGON_CLUSTER_COLOR)
            PolyMesh2ClusteringLabel(triPolyMesh, polygonLabelList);
        else
            PolyMesh2BlockClusterLabel(triPolyMesh, polygonLabelList);
    }

    /// Prepare the edge normal data
    MatrixXd e1_mat;
    MatrixXd e2_mat;

    PolyMesh2EdgeNormal(polyMesh, e1_mat, e2_mat);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateOptimizedSurface(verM, triM, p1_mat, p2_mat, e1_mat, e2_mat,
                                                                                       edgeLabelList, polygonLabelList, isShowEdgeClusterColor, polygonColorState, isShowEdgeNormal);

    AppendDataToViewer(viewer, viewerDataList);

    objOptimizedSurfaceNum = viewerDataList.size();
}

void libigl_Render::DrawObjShellStructModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(polyMesh, verM, triM);

    /// Prepare the label data
    VectorXd polygonLabelList;

    PolyMesh2ClusteringLabel(triPolyMesh, polygonLabelList);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateShellStructure(verM, triM, p1_mat, p2_mat, polygonLabelList);

    AppendDataToViewer(viewer, viewerDataList);

    objShellStrucNum = viewerDataList.size();
}

void libigl_Render::DrawObjReplacedSurface(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh, const PolyMesh origPolyMesh)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(triPolyMesh, verM, triM);

    /// Prepare the fab test data
    MatrixXd fp1_mat;
    MatrixXd fp2_mat;
    VectorXd fabLabelList;

    // PolyMesh2FabTestLabel(origPolyMesh, fp1_mat, fp2_mat, fabLabelList);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateReplacedSurfaceModel(verM, triM, p1_mat, p2_mat, fp1_mat, fp2_mat, fabLabelList);

    AppendDataToViewer(viewer, viewerDataList);

    objReplacedSurfaceNum = viewerDataList.size();
}

void libigl_Render::DrawObjReplacedShellStructModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(polyMesh, verM, triM);

    /// Prepare the block label data
    VectorXd blockLabelList;

    PolyMesh2ClusteringLabel(triPolyMesh, blockLabelList);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateShellStructure(verM, triM, p1_mat, p2_mat, blockLabelList);

    AppendDataToViewer(viewer, viewerDataList);

    objReplacedShellNum = viewerDataList.size();
}

void libigl_Render::DrawObjReplacedShellPlanarModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh)
{
    /// Prepare the edges data
    MatrixXd p1_mat;
    MatrixXd p2_mat;

    PolyMesh2Edge(polyMesh, p1_mat, p2_mat);

    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(polyMesh, verM, triM);

    /// Prepare the block label data
    VectorXd blockLabelList;

    PolyMesh2ClusteringLabel(triPolyMesh, blockLabelList);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateShellStructure(verM, triM, p1_mat, p2_mat, blockLabelList);

    AppendDataToViewer(viewer, viewerDataList);

    objReplacedShellPlanarNum = viewerDataList.size();
}

void libigl_Render::DrawObjReplacedPenetrationVolumeModel(iglViewer &viewer, const PolyMesh polyMesh)
{
    /// Prepare the triangle surface data
    MatrixXd verM;
    MatrixXi triM;

    PolyMesh2Matrix(polyMesh, verM, triM);

    /// Draw object
    MeshObject meshObject;

    vector<igl::opengl::ViewerData> viewerDataList = meshObject.CreateObjModel(verM, triM, RowVector3d(0.9, 0.5, 0.5));

    AppendDataToViewer(viewer, viewerDataList);

    objReplacedPenetrationVolumeNum = viewerDataList.size();
}



///=========================================================================================///
///                                     Show/hide Objects
///=========================================================================================///


void libigl_Render::ShowGround(iglViewer &viewer, bool isVisible) const
{
    for(int i = 0; i <= groundMeshNum; i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowAxes(iglViewer &viewer, bool isVisible) const
{
    for(int i = groundMeshNum + 1; i <= (groundMeshNum + axesMeshNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowObjModel(iglViewer &viewer, bool isVisible)
{
    if (objMeshNum == 0)
        return;

    for(int i = groundMeshNum + axesMeshNum + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowTexture(iglViewer &viewer, bool isVisible)
{
    if (objMeshTextureNum == 0)
        return;

    for(int i = groundMeshNum + axesMeshNum + objMeshNum + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowRemeshingModel(iglViewer &viewer, bool isVisible)
{
    if (objRemeshingMeshNum == 0)
        return;

    for(int i = groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowOptimizedSurface(iglViewer &viewer, bool isVisible)
{
    if (objOptimizedSurfaceNum == 0)
        return;

    for(int i = groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowReplacedSurface(iglViewer &viewer, bool isVisible)
{
    if (objReplacedSurfaceNum == 0)
        return;

    for(int i = groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowShellStructModel(iglViewer &viewer, bool isVisible)
{
    if (objShellStrucNum == 0)
        return;

    for(int i = (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum) + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum) ; i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowReplaceShellModel(iglViewer &viewer, bool isVisible)
{
    if (objReplacedShellNum == 0)
        return;

    for(int i = (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum) + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum) ; i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowReplaceShellPlanarModel(iglViewer &viewer, bool isVisible)
{
    if (objReplacedShellPlanarNum == 0)
        return;

    for(int i = (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum) + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum + objReplacedShellPlanarNum) ; i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowPenetrationVolume(iglViewer &viewer, bool isVisible)
{
    if (objReplacedPenetrationVolumeNum == 0)
        return;

    for(int i = (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum + objReplacedShellPlanarNum) + 1; i <= (groundMeshNum + axesMeshNum + objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum + objReplacedShellPlanarNum + objReplacedPenetrationVolumeNum) ; i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}



///=========================================================================================///
///                                        Set Camera
///=========================================================================================///

void libigl_Render::SetCamera(iglViewer &viewer, float zoom, Vector3f eyePos)
{
    // change the camera parameters here
    // the rest of the parameters can be found at line 127(ViewerCore.h)

    viewer.core().camera_zoom  =  zoom;
    viewer.core().camera_eye   =  eyePos;
}




///=========================================================================================///
///                                        Ground
///=========================================================================================///

void libigl_Render::AppendDataToViewer(iglViewer &viewer, vector<igl::opengl::ViewerData> viewerDataList)
{
    for (const auto & i : viewerDataList)
    {
        viewer.data_list.push_back( i );
    }
}

void libigl_Render::ClearViewer(iglViewer &viewer)
{
    if ( (objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum + objReplacedShellPlanarNum + objReplacedPenetrationVolumeNum) != 0)
    {
        ClearMesh(viewer, (objMeshNum + objMeshTextureNum + objRemeshingMeshNum + objOptimizedSurfaceNum + objReplacedSurfaceNum + objShellStrucNum + objReplacedShellNum + objReplacedShellPlanarNum + objReplacedPenetrationVolumeNum));

        objMeshNum = 0;
        objMeshTextureNum = 0;
        objRemeshingMeshNum = 0;

        objOptimizedSurfaceNum = 0;
        objReplacedSurfaceNum = 0;

        objShellStrucNum = 0;
        objReplacedShellNum = 0;
        objReplacedShellPlanarNum = 0;
        objReplacedPenetrationVolumeNum = 0;
    }
}

// Clear last N meshes added to viewerDataList recently
void libigl_Render::ClearMesh(iglViewer &viewer,int meshNum)
{
    for (int i = 0; i < meshNum; i++)
    {
        viewer.selected_data_index = viewer.data_list.size()-1;
        viewer.erase_mesh(viewer.selected_data_index);
        viewer.data(viewer.selected_data_index).clear();
    }
}

