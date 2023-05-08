///////////////////////////////////////////////////////////////
//
// main.cpp
//
//   Volume Creator Program
//
// by Rulin Chen, Yingjie Chen and Peng Song
//
// 30/Aug/2021
//
//
///////////////////////////////////////////////////////////////


#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/unproject_onto_mesh.h>
#include "Utility/HelpTypedef.h"
#include "Render/libigl_Render.h"
#include "Render/libigl_UI.h"
#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"
#include "TileableStructure/BaseSurface.h"
#include "TileableStructure/Initialization.h"
#include "TileableStructure/SurfaceOptimizer.h"
#include "TileableStructure/ShellStructure.h"
#include "TileableStructure/CuttingPlaneOptimizer.h"

///////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////

// the viewer
iglViewer viewer;

// a menu plugin
igl::opengl::glfw::imgui::ImGuiMenu menu;

int windowWidth;
int windowHeight;
static bool isFirstCall;

bool visibleAxes;
bool visibleGround;

bool visibleObjModel;
bool visibleObjTexture;
bool visibleObjRemeshingModel;

bool visibleObjOptimizedModel;
bool visibleObjReplacedSurface;

bool visibleObjShellStructure;
bool visibleObjReplacedShell;
bool visibleObjReplacedPlanarShell;
bool visibleObjPenetrationVolume;

bool isPlanarization;
bool isSetSimilarity;

bool is_restart;

float currPickedPolygonPlanarity;
int currPickedPolygonLabel;
int unPlanarNum_before;
int unPlanarNum_after;
int realClusteringNum;

libigl_Render myRender;

int frame;

bool isSavePrintingFiles;

// Input model name (.obj)
string inputFileName;
string tileFileName;

Surface mySurface;
TilePattern myTilePattern;
BaseSurface myBaseSurface(&mySurface, &myTilePattern);
ShellStructure myShellStructure(&myBaseSurface);
SurfaceOptimizer mySurfaceOptimizer(&myBaseSurface, &myShellStructure);
CuttingPlaneOptimizer myCuttingPlaneOptimizer(&myBaseSurface);


///////////////////////////////////////////////////////////////
// Function Declarations
///////////////////////////////////////////////////////////////

void InitSetting();
void InitViewer();


///=========================================================================================///
///                                       Initialization
///=========================================================================================///

void InitSetting()
{
    windowWidth       =  1600;
    windowHeight      =  1000;

    isFirstCall       =  true;

    visibleAxes       =  false;
    visibleGround     =  false;
    visibleObjModel   =  true;

    visibleObjTexture =  false;
    visibleObjRemeshingModel = false;

    visibleObjOptimizedModel = false;
    visibleObjReplacedSurface = false;

    visibleObjShellStructure = false;
    visibleObjReplacedShell = false;
    visibleObjReplacedPlanarShell = false;
    visibleObjPenetrationVolume = false;

    isPlanarization = true;
    isSetSimilarity = true;

    isSavePrintingFiles = false;

    is_restart = false;
    frame = 0;
}

void InitViewer()
{
    ///set animation
    viewer.core().animation_max_fps = 45.;
    viewer.core().is_animating = true;
    viewer.core().background_color = RowVector4f (1,1,1,0);
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL); // For Trackball rotation setting
}




///=========================================================================================///
///                                       Interaction
///=========================================================================================///

bool key_down(iglViewer &Viewer, unsigned char key, int modifier)
{
    if (key == 'M' and mySurface.V.size() != 0 and myTilePattern.inputTilePattern.n_edges() != 0)
    {
        myTilePattern.scalar -= 0.1;

        Initialization myInit;
        myInit.isPlanarization = isPlanarization;
        myInit.Remeshing(&mySurface, &myTilePattern, &myBaseSurface);

        myRender.ClearViewer(viewer);

        myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
        myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
        myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface);
    }

    else if (key == 'N' and mySurface.V.size() != 0 and myTilePattern.inputTilePattern.n_edges() != 0)
    {
        myTilePattern.scalar += 0.1;

        Initialization myInit;
        myInit.isPlanarization = isPlanarization;
        myInit.Remeshing(&mySurface, &myTilePattern, &myBaseSurface);

        myRender.ClearViewer(viewer);

        myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
        myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
        myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface);
    }

    else if (key == 'U' and mySurface.V.size() != 0 and myTilePattern.inputTilePattern.n_edges() != 0)
    {
        myTilePattern.rotationAngle += 10;

        Initialization myInit;
        myInit.isPlanarization = isPlanarization;
        myInit.Remeshing(&mySurface, &myTilePattern, &myBaseSurface);

        myRender.ClearViewer(viewer);

        myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
        myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
        myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface);
    }

    else if (key == 'I' and mySurface.V.size() != 0 and myTilePattern.inputTilePattern.n_edges() != 0)
    {
        myTilePattern.rotationAngle -= 10;

        Initialization myInit;
        myInit.isPlanarization = isPlanarization;
        myInit.Remeshing(&mySurface, &myTilePattern, &myBaseSurface);

        myRender.ClearViewer(viewer);

        myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
        myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
        myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface);
    }

    return false;
}




///=========================================================================================///
///                                       Main Function
///=========================================================================================///

int main(int argc, char *argv[])
{
    setViewerUI(viewer);

    InitViewer();

    InitSetting();

    myRender.RenderScene(viewer);

    viewer.callback_key_down = &key_down;

    viewer.callback_mouse_down =
            [](igl::opengl::glfw::Viewer& viewer, int, int)->bool
            {
                if(!OpenMesh::hasProperty<OpenMesh::FaceHandle, double>(myBaseSurface.triBaseSurface, "facePlanarity")
                        or !OpenMesh::hasProperty<OpenMesh::FaceHandle, double>(myBaseSurface.triBaseSurface, "clusteringLabel"))
                {
                    return false;
                }

                int fid;
                Eigen::Vector3f bc;
                // Cast a ray in the view direction starting from the mouse position
                double x = viewer.current_mouse_x;
                double y = viewer.core().viewport(3) - viewer.current_mouse_y;

                MatrixXd verM;
                MatrixXi triM;

                PolyMesh2Matrix(myBaseSurface.triBaseSurface, verM, triM);

                /// Prepare the label data
                VectorXd labelList;

                PolyMesh2ClusteringLabel(myBaseSurface.triBaseSurface, labelList);

                /// Prepare the planarity data
                VectorXd planarityList;

                PolyMesh2Planarity(myBaseSurface.triBaseSurface, planarityList);

                if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core().view,
                                            viewer.core().proj, viewer.core().viewport, verM, triM, fid, bc))
                {
                    currPickedPolygonPlanarity = planarityList(fid);
                    currPickedPolygonLabel = labelList(fid);

                    return true;
                }
                return false;
            };

    viewer.callback_pre_draw =
            [&](igl::opengl::glfw::Viewer &)
            {
                myRender.ShowGround( viewer, visibleGround );
                myRender.ShowAxes(viewer, visibleAxes );

                myRender.ShowObjModel(viewer, visibleObjModel);
                myRender.ShowTexture(viewer, visibleObjTexture);
                myRender.ShowRemeshingModel(viewer, visibleObjRemeshingModel);

                myRender.ShowOptimizedSurface(viewer, visibleObjOptimizedModel);
                myRender.ShowReplacedSurface(viewer, visibleObjReplacedSurface);

                myRender.ShowShellStructModel(viewer, visibleObjShellStructure);
                myRender.ShowReplaceShellModel(viewer, visibleObjReplacedShell);
                myRender.ShowReplaceShellPlanarModel(viewer, visibleObjReplacedPlanarShell);

                myRender.ShowPenetrationVolume(viewer, visibleObjPenetrationVolume);

                if(isFirstCall)
                {
                    myRender.SetCamera( viewer, 0.1, Vector3f (-10, 10, 10) );
                    isFirstCall = false;
                }

                return 0;
            };

    viewer.launch(true,false,"Tileable Shell", windowWidth, windowHeight);

    return 1;
}

