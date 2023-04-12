///////////////////////////////////////////////////////////////
//
// libigl_UI.h
//
//   User interface with libigl
//
// by Rulin Chen
//
// 19/Apr/2022
//
//
///////////////////////////////////////////////////////////////

#include "libigl_UI.h"
#include <time.h>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include "Utility/HelpTypedef.h"
#include "Render/libigl_Render.h"

#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"
#include "TileableStructure/BaseSurface.h"
#include "TileableStructure/Initialization.h"
#include "TileableStructure/SurfaceOptimizer.h"
#include "TileableStructure/ShellStructure.h"
#include "TileableStructure/CuttingPlaneOptimizer.h"

using std::vector;
using std::cout;
using std::endl;

///////////////////////////////////////////////////////////////
// Extern Variables
///////////////////////////////////////////////////////////////

extern iglViewer viewer;
extern igl::opengl::glfw::imgui::ImGuiMenu menu;

extern bool visibleAxes;
extern bool visibleGround;

extern bool visibleObjModel;
extern bool visibleObjTexture;
extern bool visibleObjRemeshingModel;

extern bool visibleObjOptimizedModel;
extern bool visibleObjReplacedSurface;

extern bool visibleObjShellStructure;
extern bool visibleObjReplacedShell;
extern bool visibleObjReplacedPlanarShell;
extern bool visibleObjPenetrationVolume;

extern bool isPlanarization;
extern bool isSetSimilarity;

extern float currPickedPolygonPlanarity;
extern int currPickedPolygonLabel;

extern bool isSavePrintingFiles;

extern bool is_restart;

extern libigl_Render myRender;

// to show tips hovering on the UI items
static void HelpMarker(const char* desc);

// Input model name (.obj)
extern string inputFileName;
extern string tileFileName;
extern Surface mySurface;
extern TilePattern myTilePattern;
extern BaseSurface myBaseSurface;
extern SurfaceOptimizer mySurfaceOptimizer;
extern ShellStructure myShellStructure;
extern CuttingPlaneOptimizer myCuttingPlaneOptimizer;
//extern Initialization myInit;

int clusterNum = 10;
extern int unPlanarNum_before;
extern int unPlanarNum_after;
extern int realClusteringNum;

// clustering index for saving objs
int clusterID = 0;

// Rendering setting for remeshed surface
bool isShowRemeshedSurfaceEdgeNormal = false;

// Rendering setting for optimized surface
int polygonColorState = SHOW_POLYGON_CLUSTER_COLOR;
bool isShowEdgeClusterColor = true;
bool isShowOptimizedSurfaceEdgeNormal = false;

float thickness = 0.04;

///=========================================================================================///
///                                      Helper Functions
///=========================================================================================///

void StatusBar()
{

    string fileName = inputFileName;

    for (int i = fileName.size() - 1; i > 0; --i)
    {
        if (fileName[i] == '/')
        {
            fileName.erase(fileName.begin(), fileName.begin() + i + 1);
            break;
        }
    }

    ImGui::Text("Model : %s", fileName.c_str());

    fileName = tileFileName;

    for (int i = fileName.size() - 1; i > 0; --i)
    {
        if (fileName[i] == '/')
        {
            fileName.erase(fileName.begin(), fileName.begin() + i + 1);
            break;
        }
    }

    ImGui::Text("Tile : %s", fileName.c_str());

    ImGui::Text("Tile Number : %zu", myBaseSurface.baseSurface.n_faces());

    ImGui::Text("Unique Polygon : %d", mySurfaceOptimizer.polygonClusterNum);

    ImGui::Text("Unique Block : %d", myCuttingPlaneOptimizer.blockClusterNum);

    ImGui::Text("Avg Contact Angle Error : %f", myShellStructure.avgContactAngleError);

    ImGui::Text("Avg Overlap Ratio : %f", myShellStructure.avgOverlappedVolumeBlockRatio);

    ImGui::Text("Avg Gap Ratio : %f", myShellStructure.avgGapVolumeRatio);
}

static void HelpMarker(const char* desc)
{
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void OpCuttingPlane()
{
    myCuttingPlaneOptimizer.CuttingPlaneOptimization_mini();
}

void InitShellStructure()
{
    myBaseSurface.ComputeDisassemblyPlan();
    myShellStructure.InitOrigAndReplacedShellStructures();
}


///=========================================================================================///
///                                      Set Viewer UI
///=========================================================================================///

//// UI main function
void setViewerUI(igl::opengl::glfw::Viewer &viewer)
{
    menu.callback_draw_viewer_window = [&]()
    {
        //// color preset:
        ImGui::GetStyle().Colors[ImGuiCol_WindowBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_TitleBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_TitleBgActive] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);

        ImGui::GetStyle().Colors[ImGuiCol_FrameBg] = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_FrameBgHovered] = ImVec4(1.0f, 1.0f, 1.0f, 0.50f);
        ImGui::GetStyle().Colors[ImGuiCol_FrameBgActive] = ImVec4(1.0f, 1.0f, 1.0f, 0.50f);

        ImGui::GetStyle().Colors[ImGuiCol_MenuBarBg] = ImVec4(0.9f, 0.9f, 0.9f, 1.00f);
        ImGui::GetStyle().Colors[ImGuiCol_Text] = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

        ImGui::GetStyle().Colors[ImGuiCol_PopupBg] = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);

        //ImGui::TextWrapped();
        //// Define window position + size
        float menu_width = 240.f * menu.menu_scaling();

        //// Warning: do get the true windows width to relocate the menu, the viewer using highdpi (see Viewer.cpp to support highdpi display）
        int width_window, height_window;
        glfwGetWindowSize(viewer.window,&width_window, &height_window);

        ImGui::SetNextWindowPos(ImVec2(width_window - menu_width, 0.0f),ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
                "Control Panel", &_viewer_menu_visible,
                ImGuiWindowFlags_NoSavedSettings
        );
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
        if (menu.callback_draw_viewer_menu) { menu.callback_draw_viewer_menu(); }
        ImGui::PopItemWidth();
        ImGui::End();
    };


    // Add content to the default menu window
    menu.callback_draw_viewer_menu = [&]()
    {

        //// global styles of UI
        float w = ImGui::GetContentRegionAvailWidth();
        float p = ImGui::GetStyle().FramePadding.x;
        float gap_between_controlGroups = 4.0f;

        float button_verticalGap = 2.0f;
        float button_horizontalGap = 4*p;
        float button_width = (w-button_horizontalGap)/2.f;

        ///////////////////////////////////////////////////////////////////////////
        //// Status Bar
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Status Bar", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Dummy(ImVec2(0.0f, 2.0f));
            float head_scale = 1.3f;
            float gap_between_renderGroups = 4.0f;
            float half_width = (w - p) / 2.0f;
            //float transparency;

            StatusBar();

            ImGui::Dummy(ImVec2(0.0f, gap_between_renderGroups));

        }
        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

        ///////////////////////////////////////////////////////////////////////////
        //// Operation Control - 1
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Import & Remesh", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float head_scale = 1.3f;
            float gap_between_paraGroups = 8.0f;
            float half_width = (w - p) / 2.f;

            //// gap between the button group and head
            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ImGui::Text("Scalar");
            ImGui::SameLine(half_width, p);
            ImGui::DragFloat("##Scalar", &myTilePattern.scalar);

            ImGui::Text("RotAngle");
            ImGui::SameLine(half_width, p);
            ImGui::DragFloat("##RotAngle", &myTilePattern.rotationAngle);

            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ////////////////////////////////////////////////////////////////////
            //// model&shell&parts related
            //// buttons for model/shell IO/generate

            if (ImGui::Button("Read", ImVec2(button_width * 0.65, 0)))
            {
                inputFileName = igl::file_dialog_open();
                if( inputFileName.empty() )
                    return;

                mySurface.ClearSurface();
                mySurface.LoadModel((char*)inputFileName.data());

                myRender.ClearViewer(viewer);
                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
            }

            ImGui::SameLine(0, button_horizontalGap * 0.5);

            if (ImGui::Button("Texture", ImVec2(button_width * 0.65, 0)))
            {
                tileFileName = igl::file_dialog_open();
                if( tileFileName.empty() )
                    return;

                myTilePattern.LoadTexture((char*)tileFileName.data());

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
            }

            ImGui::SameLine(0, button_horizontalGap * 0.5);

            if (ImGui::Button("Remesh", ImVec2(button_width*0.65, 0)))
            {
                Initialization myInit;
                myInit.isPlanarization = isPlanarization;
                myInit.Remeshing(&mySurface, &myTilePattern, &myBaseSurface);

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            ImGui::Text("---------------------------------------------------------");

            if (ImGui::Button("Load Remeshed Model", ImVec2(button_width * 2.2, 0)))
            {
                inputFileName = igl::file_dialog_open();
                if( inputFileName.empty() )
                    return;

                mySurface.ClearSurface();
                mySurface.LoadModel((char*)inputFileName.data());

                Initialization myInit;
                myInit.isPlanarization = isPlanarization;
                myInit.InitBaseSurface(inputFileName, &myBaseSurface);

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            }
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

        ///////////////////////////////////////////////////////////////////////////
        //// Operation Control - 5
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Mesh Optimization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float half_width = (w - p) / 2.f;

            //// gap between the button group and head
            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ImGui::Text("Edge K");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragInt("##Edge K", &mySurfaceOptimizer.edgeClusteringNum_end);

            mySurfaceOptimizer.edgeClusteringNum_start = mySurfaceOptimizer.edgeClusteringNum_end;

            ImGui::Text("Dihedral K");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragInt("##Dihedral K", &mySurfaceOptimizer.dihedralClusterNum_end);

            mySurfaceOptimizer.dihedralClusterNum_start = mySurfaceOptimizer.dihedralClusterNum_end;

            ImGui::Text("Mini Block Cluster");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragInt("##Mini Block Cluster Num Threshold", &myCuttingPlaneOptimizer.miniBlockClusterThreshold);

            ImGui::Text("Target Block Cluster");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragInt("##Target Block Cluster Num", &myCuttingPlaneOptimizer.targetBlockClusterNum);

            ImGui::Text("Thickness");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragFloat("##Thickness", &thickness);

            myShellStructure.cutUpper = thickness / 2.0f;
            myShellStructure.cutLower = thickness / 2.0f;

            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ////////////////////////////////////////////////////////////////////
            //// model&shell&parts related
            //// buttons for model/shell IO/generate

            if (ImGui::Button("Opt Surface", ImVec2(button_width, 0)))
            {
                clock_t start,end;
                start = clock(); 

                mySurfaceOptimizer.PolygonOptimization();

                end = clock();
                mySurfaceOptimizer.surfaceOpTime = (double(end-start)/CLOCKS_PER_SEC) / 60.0f;

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
//                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
                myRender.DrawObjModelWithEdgeCluster(viewer, myBaseSurface.remeshedPolySurface , myBaseSurface.remeshedTriSurface, myBaseSurface.baseSurface);
                myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface, isShowEdgeClusterColor, polygonColorState, isShowOptimizedSurfaceEdgeNormal);
                myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
            }

            ImGui::SameLine(0, button_horizontalGap);

            if (ImGui::Button("Opt CuttingPlane", ImVec2(button_width, 0)))
            {
                clock_t start,end;
                start = clock();  

                OpCuttingPlane();

                end = clock(); 
                myShellStructure.shellOptimizationTime = (double(end-start)/CLOCKS_PER_SEC) / 60.0f;

                // InitShellStructure();

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
                myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface, isShowEdgeClusterColor, polygonColorState, isShowOptimizedSurfaceEdgeNormal);
                myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
                myRender.DrawObjShellStructModel(viewer, myShellStructure.shellStructure, myShellStructure.shellStructureTri);
                myRender.DrawObjReplacedShellStructModel(viewer, myShellStructure.replacedShellStructure_LSM, myShellStructure.replacedShellStructureTri_LSM);
                myRender.DrawObjReplacedShellPlanarModel(viewer, myShellStructure.replacedShellStructure_LSM_planar, myShellStructure.replacedShellStructureTri_LSM_planar);
                myRender.DrawObjReplacedPenetrationVolumeModel(viewer, myShellStructure.replacedPenetrationVolumeTri);
            }

            ImGui::Dummy(ImVec2(0.0f, 3.0f));

            if (ImGui::Button("Create Shell", ImVec2(button_width, 0)))
            {
                InitShellStructure();

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
                myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface, isShowEdgeClusterColor, polygonColorState, isShowOptimizedSurfaceEdgeNormal);
                myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
                myRender.DrawObjShellStructModel(viewer, myShellStructure.shellStructure, myShellStructure.shellStructureTri);
                myRender.DrawObjReplacedShellStructModel(viewer, myShellStructure.replacedShellStructure_LSM, myShellStructure.replacedShellStructureTri_LSM);
                myRender.DrawObjReplacedShellPlanarModel(viewer, myShellStructure.replacedShellStructure_LSM_planar, myShellStructure.replacedShellStructureTri_LSM_planar);
                myRender.DrawObjReplacedPenetrationVolumeModel(viewer, myShellStructure.replacedPenetrationVolumeTri);
            }

            ImGui::SameLine(0, button_horizontalGap);

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            if (ImGui::Button("Save Shell", ImVec2(button_width, 0)))
            {
                string outputFolderPath = igl::file_dialog_save();
                if( outputFolderPath.empty())
                    return;

                string currPath = myShellStructure.CreateSavingAllFolderName(outputFolderPath, inputFileName, tileFileName);
                myShellStructure.SaveShell(currPath, false);
                mySurfaceOptimizer.SaveSurface(currPath);

                // mySurfaceOptimizer.SaveParameter2File(currPath);
                mySurfaceOptimizer.SaveSurfaceOptimizationResult(currPath);
                myShellStructure.SaveShellOptimizationResult(currPath);
                // SaveExcelTxt(currPath);
            }

            ImGui::Dummy(ImVec2(0.0f, 3.0f));
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

        ///////////////////////////////////////////////////////////////////////////
        //// Render Control
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Render Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Dummy(ImVec2(0.0f, 2.0f));
            float head_scale = 1.3f;
            float gap_between_renderGroups = 4.0f;
            float half_width = (w - p) / 1.2f;
            float transparency;

            ImGui::Dummy(ImVec2(0.0f, gap_between_renderGroups));
            ImGui::SetWindowFontScale(1);

            ImGui::Text("Show Input Surface");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Input Surface", &visibleObjModel);

            ImGui::Text("Show Texture");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Texture", &visibleObjTexture);

            ImGui::Text("Show Remeshed Model");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Remeshed Model", &visibleObjRemeshingModel);

            ImGui::Text("Show Optimized Model");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Optimized Model", &visibleObjOptimizedModel);

            ImGui::Text("Show Shell Structure");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Shell Structure", &visibleObjShellStructure);

            ImGui::Text("Show Replaced Shell");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Replaced Shell", &visibleObjReplacedPlanarShell);

            ImGui::Dummy(ImVec2(0.0f, gap_between_renderGroups));

        }
        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));
    };

    viewer.plugins.push_back(&menu);
    viewer.data().face_based = true;
    viewer.core().background_color.setConstant(1.0f);
}
