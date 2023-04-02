///////////////////////////////////////////////////////////////
//
// libigl_Render.h
//
//   Rendering various 3D contents with libigl
//
// by Peng Song
//
// 01/Dec/2020
//
//
///////////////////////////////////////////////////////////////


#ifndef _LIBIGL_RENDER_H
#define _LIBIGL_RENDER_H

#include "Utility/HelpTypedef.h"
#include "Utility/HelpFunc.h"

using namespace std;
using namespace Eigen;

class libigl_Render
{
public:

    int groundMeshNum;
    int axesMeshNum;

    int objMeshNum;
    int objMeshTextureNum;
    int objRemeshingMeshNum;

    int objOptimizedSurfaceNum;
    int objReplacedSurfaceNum;

    int objShellStrucNum;
    int objReplacedShellNum;
    int objReplacedShellPlanarNum;
    int objReplacedPenetrationVolumeNum;

public:
    libigl_Render();
    ~libigl_Render();

    /// Render Objects
    void RenderScene(iglViewer &viewer);
    void DrawGround(iglViewer &viewer, Vector3d origin, double size, int gridNum, int sampNum);
    void DrawWorldAxes(iglViewer &viewer, Vector3d origin, double size, int sampNum);

    void DrawObjModelWithEdgeCluster(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh, const PolyMesh optimizedPolyMesh);
    void DrawObjModel(iglViewer &viewer, const PolyMesh polyMesh);
    void DrawObjModelTexture(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh);
    void DrawObjRemeshingModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh, bool isShowEdgeNormal = false);

    void DrawObjOptimizedModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh,
                               bool isShowEdgeClusterColor = false, int polygonColorState = SHOW_WHITE_FACE, bool isShowEdgeNormal = false);
    void DrawObjReplacedSurface(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh, const PolyMesh origPolyMesh);

    void DrawObjShellStructModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh);
    void DrawObjReplacedShellStructModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh);
    void DrawObjReplacedShellPlanarModel(iglViewer &viewer, const PolyMesh polyMesh, const PolyMesh triPolyMesh);
    void DrawObjReplacedPenetrationVolumeModel(iglViewer &viewer, const PolyMesh polyMesh);

    /// Show/hide Objects
    void ShowGround(iglViewer &viewer,bool isVisible) const;
    void ShowAxes(iglViewer &viewer, bool isVisible) const;

    void ShowObjModel(iglViewer &viewer, bool isVisible) ;
    void ShowTexture(iglViewer &viewer, bool isVisible) ;
    void ShowRemeshingModel(iglViewer &viewer, bool isVisible) ;

    void ShowOptimizedSurface(iglViewer &viewer, bool isVisible) ;
    void ShowReplacedSurface(iglViewer &viewer, bool isVisible) ;
    void ShowShellStructModel(iglViewer &viewer, bool isVisible) ;
    void ShowReplaceShellModel(iglViewer &viewer, bool isVisible) ;
    void ShowReplaceShellPlanarModel(iglViewer &viewer, bool isVisible) ;
    void ShowPenetrationVolume(iglViewer &viewer, bool isVisible) ;

    /// Set Camera
    void SetCamera(iglViewer &viewer, float zoom, Vector3f eyePos);

    /// Utility
    void AppendDataToViewer(iglViewer &viewer, vector<iglViewerData> viewerDataList);
    void ClearViewer(iglViewer &viewer);
    void ClearMesh(iglViewer &viewer, int meshNum);
};


#endif //_LIBIGL_RENDER_H
