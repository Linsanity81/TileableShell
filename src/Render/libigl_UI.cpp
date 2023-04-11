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

void SaveExcelTxt(string folderPath)
{
    std::ofstream out(folderPath + "/ExcelData.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    double shapeDiagnal = (mySurface.bbox.maxPt - mySurface.bbox.minPt).norm();

    cout << "Excel_paper: " << endl;

    cout << myShellStructure.cutLower + myShellStructure.cutUpper << " ";

    cout << myBaseSurface.baseSurface.n_faces() << " ";

    cout << mySurfaceOptimizer.edgeClusteringNum_end << " " << mySurfaceOptimizer.dihedralClusterNum_end << " ";

    cout << mySurfaceOptimizer.polygonClusterNum << " " << myBaseSurface.GetBlockClusterNum() << " " << myBaseSurface.baseSurface.n_faces() / double(myBaseSurface.GetBlockClusterNum() ) << " ";

    cout << myShellStructure.avgOverlappedVolumeBlockRatio << " " << myShellStructure.worstOverlappedVolumeBlockRatio << " ";

    cout << myShellStructure.avgGapVolumeRatio << " " << myShellStructure.worstGapVolumeRatio << " ";

    cout << myShellStructure.avgContactAngleError << " " << myShellStructure.worstContactAngleError << " ";

    cout << mySurfaceOptimizer.surfaceOpTime << " " << myShellStructure.shellOptimizationTime << endl << endl;

    cout << "Excel_full: " << endl;

    cout << myShellStructure.cutLower + myShellStructure.cutUpper << " ";

    cout << myBaseSurface.baseSurface.n_faces() << " ";

    cout << mySurfaceOptimizer.edgeClusteringNum_end << " " << mySurfaceOptimizer.dihedralClusterNum_end << " ";

    cout << mySurfaceOptimizer.polygonClusterNum << " " << myBaseSurface.GetBlockClusterNum() << " " << myBaseSurface.baseSurface.n_faces() / double(myBaseSurface.GetBlockClusterNum() ) << " ";

    cout << myShellStructure.avgOverlappedVolumeBlockRatio << " " << myShellStructure.worstOverlappedVolumeBlockRatio << " ";

    cout << myShellStructure.avgGapVolumeRatio << " " << myShellStructure.worstGapVolumeRatio << " ";

    cout << myShellStructure.avgContactAngleError << " " << myShellStructure.worstContactAngleError << " ";

    cout << mySurfaceOptimizer.surfaceOpTime << " " << myShellStructure.shellOptimizationTime << endl << endl;

    // For supp material

    cout << myBaseSurface.E_edge << " ";

    cout << myBaseSurface.E_dihed << " ";

    cout << myBaseSurface.E_planar << " " ;

    cout << myBaseSurface.E_polygon << " ";

    cout << myBaseSurface.closeness << " ";

    cout << myBaseSurface.smoothness << " ";

    cout << myShellStructure.avgBlockError *  myBaseSurface.baseSurface.n_faces() << " ";

    std::cout.rdbuf(coutbuf); //reset to standard output again
}

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

    // ImGui::Text("Vertex Number : %zu", myBaseSurface.baseSurface.n_vertices());

    // ImGui::Text("Edge Number : %zu", myBaseSurface.baseSurface.n_edges());

    ImGui::Text("Tile Number : %zu", myBaseSurface.baseSurface.n_faces());

    ImGui::Text("Unique Polygon : %d", mySurfaceOptimizer.polygonClusterNum);

    ImGui::Text("Unique Block : %d", myCuttingPlaneOptimizer.blockClusterNum);

    // ImGui::Text("       ");

    // ImGui::Text("Worst Penetration Appro : %f", myBaseSurface.worstFabApproximation);

    // ImGui::Text("Worst Polygon Simi : %f", myBaseSurface.worstPolygonSimilarity);

    // ImGui::Text("Worst Planarity : %f", myBaseSurface.worstPlanarity);

    // ImGui::Text("Worst Dihedral Angle Error : %f", myBaseSurface.worstDihedralAngleError);

    // ImGui::Text("Worst Edge Length Error: %f", myBaseSurface.worstEdgeLengthError);

    // ImGui::Text("       ");

    // ImGui::Text("Worst Block Error : %f", myShellStructure.worstBlockError);

    // ImGui::Text("Worst Contact Angle Error : %f", myShellStructure.worstContactAngleError);

    // ImGui::Text("Worst Overlapped Volume Ratio : %f", myShellStructure.worstOverlappedVolumeBlockRatio);

    // ImGui::Text("Worst Gap Volume Ratio : %f", myShellStructure.worstGapVolumeRatio);
}

void PolygonInformation()
{
    ImGui::Text("   Planarity: %f", currPickedPolygonPlanarity);
    ImGui::Text("   Clustering Label: %d", currPickedPolygonLabel);
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

            // ImGui::Text("Planarization for Init");
            // ImGui::SameLine(half_width, p);
            // ImGui::Checkbox("##Planarization for Init", &isPlanarization);

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

            if (ImGui::Button("Read", ImVec2(button_width, 0)))
            {
                inputFileName = igl::file_dialog_open();
                if( inputFileName.empty() )
                    return;

                mySurface.ClearSurface();
                mySurface.LoadModel((char*)inputFileName.data());

                myRender.ClearViewer(viewer);
                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
            }

            ImGui::SameLine(0, button_horizontalGap);

            if (ImGui::Button("Texture", ImVec2(button_width, 0)))
            {
                tileFileName = igl::file_dialog_open();
                if( tileFileName.empty() )
                    return;

                myTilePattern.LoadTexture((char*)tileFileName.data());

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
            }

            ImGui::Dummy(ImVec2(0.0f, 3.0f));

            if (ImGui::Button("Remesh", ImVec2(button_width*2.2, 0)))
            {
                Initialization myInit;
                myInit.isPlanarization = isPlanarization;
                myInit.Remeshing(&mySurface, &myTilePattern, &myBaseSurface);

                myRender.ClearViewer(viewer);

                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            }

            // ImGui::SameLine(0, button_horizontalGap);

            // if (ImGui::Button("Save Remeshed Surface", ImVec2(button_width, 0)))
            // {
            //     string outputFolderPath = igl::file_dialog_save();
            //     if( outputFolderPath.empty())
            //         return;

            //     myBaseSurface.WriteRemeshedBaseSurface(outputFolderPath);
            // }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            // if (ImGui::Button("Load Remeshed Mesh", ImVec2(button_width, 0)))
            // {
            //     inputFileName = igl::file_dialog_open();
            //     if( inputFileName.empty() )
            //         return;

            //     mySurface.ClearSurface();
            //     mySurface.LoadModel((char*)inputFileName.data());

            //     Initialization myInit;
            //     myInit.isPlanarization = isPlanarization;
            //     myInit.InitBaseSurface(inputFileName, &myBaseSurface);

            //     myRender.ClearViewer(viewer);

            //     myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
            //     myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
            //     myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            // }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

        ///////////////////////////////////////////////////////////////////////////
        //// Operation Control - 1
        ///////////////////////////////////////////////////////////////////////////

        if (ImGui::CollapsingHeader("Import Remeshed Model", ImGuiTreeNodeFlags_DefaultOpen))
        {
            float head_scale = 1.3f;
            float gap_between_paraGroups = 8.0f;
            float half_width = (w - p) / 2.f;

            //// gap between the button group and head
            ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ////////////////////////////////////////////////////////////////////
            //// model&shell&parts related
            //// buttons for model/shell IO/generate

            if (ImGui::Button("Load Remeshed Mesh", ImVec2(button_width*2.2, 0)))
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

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));
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

            // ImGui::Dummy(ImVec2(0.0f, 2.0f));

            // ImGui::Dummy(ImVec2(0.0f, 2.0f));

            // ImGui::Text("Mini Polygon Cluster Num Threshold");
            // ImGui::SameLine(half_width, p);
            // ImGui::SetNextItemWidth(half_width);
            // ImGui::DragInt("##Mini Polygon Cluster Num Threshold", &mySurfaceOptimizer.miniPolygonClusterNumThreshold);

            // ImGui::Text("Merging Polygon Candi Num");
            // ImGui::SameLine(half_width, p);
            // ImGui::SetNextItemWidth(half_width);
            // ImGui::DragInt("##Merging Polygon Candi Num", &mySurfaceOptimizer.mergingCandidateNum);

            // ImGui::Dummy(ImVec2(0.0f, 2.0f));

            ImGui::Text("Mini Block Cluster");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragInt("##Mini Block Cluster Num Threshold", &myCuttingPlaneOptimizer.miniBlockClusterThreshold);

            ImGui::Text("Target Block Cluster");
            ImGui::SameLine(half_width, p);
            ImGui::SetNextItemWidth(half_width);
            ImGui::DragInt("##Target Block Cluster Num", &myCuttingPlaneOptimizer.targetBlockClusterNum);

            // ImGui::Dummy(ImVec2(0.0f, 2.0f));

            // ImGui::Text("Tolerance");
            // ImGui::SameLine(half_width, p);
            // ImGui::SetNextItemWidth(half_width);
            // ImGui::DragFloat("##Tolerance", &myBaseSurface.tolerance, 1.0f, 0.0f, 1.0f, "%.7f", 1.0f);

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

            // ImGui::SameLine(0, button_horizontalGap);

            // if (ImGui::Button("Merge", ImVec2(button_width, 0)))
            // {
            //     mySurfaceOptimizer.PolygonOptimization_Merging();

            //     myRender.ClearViewer(viewer);

            //     myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
            //     myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
            //     myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            //     myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface, isShowEdgeClusterColor, polygonColorState, isShowOptimizedSurfaceEdgeNormal);
            //     myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
            // }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            // if (ImGui::Button("Eli Outlier", ImVec2(button_width, 0)))
            // {
            //     mySurfaceOptimizer.WorstCaseOptimization();

            //     myRender.ClearViewer(viewer);

            //     myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
            //     myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
            //     myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            //     myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface, isShowEdgeClusterColor, polygonColorState, isShowOptimizedSurfaceEdgeNormal);
            //     myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
            // }

            ImGui::SameLine(0, button_horizontalGap);

            // if (ImGui::Button("Save Surface", ImVec2(button_width, 0)))
            // {
            //     string outputFolderPath = igl::file_dialog_save();
            //     if( outputFolderPath.empty())
            //         return;

            //     mySurfaceOptimizer.CreateFolderName(outputFolderPath, inputFileName, tileFileName);
            //     mySurfaceOptimizer.SaveSurface(mySurfaceOptimizer.savingFullPath);
            // }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

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

            // ImGui::SameLine(0, button_horizontalGap);

            // if (ImGui::Button("Save Printing", ImVec2(button_width, 0)))
            // {
            //     string outputFolderPath = igl::file_dialog_save();
            //     if( outputFolderPath.empty())
            //         return;

            //     string currPath = myShellStructure.CreateSavingAllFolderName(outputFolderPath, inputFileName, tileFileName);
            //     // mySurfaceOptimizer.SaveInput(currPath);
            //     // mySurfaceOptimizer.SaveSurface(currPath);
            //     myShellStructure.SaveShell(currPath, true);

            //     mySurfaceOptimizer.SaveSurface(currPath);

            //     mySurfaceOptimizer.SaveParameter2File(currPath);
            //     mySurfaceOptimizer.SaveSurfaceOptimizationResult(currPath);
            //     myShellStructure.SaveShellOptimizationResult(currPath);
            //     SaveExcelTxt(currPath);
            // }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            // if (ImGui::Button("Generate Support", ImVec2(button_width, 0)))
            // {
            //     string outputFolderPath = igl::file_dialog_save();
            //     if( outputFolderPath.empty())
            //         return;

            //     myShellStructure.GenerateSupportForOrigShellBlocks(outputFolderPath);
            // }

            // ImGui::SameLine(0, button_horizontalGap);

            // if (ImGui::Button("Add Texcoord", ImVec2(button_width, 0)))
            // {
            //     string outputFolderPath = igl::file_dialog_open();
            //     if( outputFolderPath.empty())
            //         return;

            //     cout << outputFolderPath << endl;

            //     PolyMesh currMesh;

            //     OpenMesh::IO::read_mesh(currMesh, outputFolderPath);

            //     AddVertexTextureCoord2PolyMesh(currMesh);

            //     OpenMesh::IO::Options ropt;
            //     ropt += OpenMesh::IO::Options::VertexTexCoord;

            //     OpenMesh::IO::write_mesh(currMesh, outputFolderPath, ropt);
            // }

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            // if (ImGui::Button("Display", ImVec2(button_width, 0)))
            // {
            //     myRender.ClearViewer(viewer);

            //     myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
            //     myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
            //     myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface, isShowRemeshedSurfaceEdgeNormal);
            //     myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface, isShowEdgeClusterColor, polygonColorState, isShowOptimizedSurfaceEdgeNormal);
            //     myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
            //     myRender.DrawObjShellStructModel(viewer, myShellStructure.shellStructure, myShellStructure.shellStructureTri);
            //     myRender.DrawObjReplacedShellStructModel(viewer, myShellStructure.replacedShellStructure_LSM, myShellStructure.replacedShellStructureTri_LSM);
            //     myRender.DrawObjReplacedShellPlanarModel(viewer, myShellStructure.replacedShellStructure_LSM_planar, myShellStructure.replacedShellStructureTri_LSM_planar);
            //     myRender.DrawObjReplacedPenetrationVolumeModel(viewer, myShellStructure.replacedPenetrationVolumeTri);
            // }

            // ImGui::SameLine(0, button_horizontalGap);

            // if (ImGui::Button("Align Polygons", ImVec2(button_width, 0)))
//             {
//                 inputFileName = igl::file_dialog_open();
//                 if( inputFileName.empty() )
//                     return;

//                 string currFolderPath, currFileName;
//                 GetFolderPath(inputFileName, currFolderPath);
//                 GetFileName(inputFileName, currFileName);

// //                cout << currFolderPath << endl;
// //                cout << currFileName << endl;

//                 for (int i = 1; i <= 7; ++i)
//                 {
//                     /// Setup a folder to save puzzle files
//                     string command;
//                     command = "mkdir -p " + currFolderPath + "Polygon_" + to_string(i);
//                     system(command.c_str());

//                     string currSavingFolder = currFolderPath + "Polygon_" + to_string(i) + "/";

//                     string currFilePath = inputFileName;
//                     currFilePath = currFilePath.substr(0, currFilePath.size() - 5);
//                     currFilePath = currFilePath + to_string(i) + ".obj";

//                     cout << currFilePath << endl;

//                     PolyMesh currMesh;
//                     OpenMesh::IO::read_mesh(currMesh, currFilePath);

//                     vector<Vector3d> refVerList;

//                     {
//                         PolyMesh::VertexIter          v_it, v_end(currMesh.vertices_end());
//                         for (v_it=currMesh.vertices_begin(); v_it!=v_end; ++v_it)
//                         {
//                             refVerList.push_back(Vector3d(currMesh.point(*v_it)[0], currMesh.point(*v_it)[1], currMesh.point(*v_it)[2]));
//                         }
//                     }

//                     for (int j = 1; j <= 7; ++j )
//                     {
//                         if (i == j)
//                             continue;

//                         string alignFilePath = currFolderPath + "Polygon_" + to_string(j) + ".obj";

//                         cout << alignFilePath << endl;

//                         PolyMesh alignMesh;
//                         OpenMesh::IO::read_mesh(alignMesh, alignFilePath);

//                         vector<Vector3d> currVerList;

//                         {
//                             PolyMesh::VertexIter          v_it, v_end(alignMesh.vertices_end());
//                             for (v_it=alignMesh.vertices_begin(); v_it!=v_end; ++v_it)
//                             {
//                                 currVerList.push_back(Vector3d(alignMesh.point(*v_it)[0], alignMesh.point(*v_it)[1], alignMesh.point(*v_it)[2]));
//                             }
//                         }

//                         double currSimi;
//                         vector<Vector3d> bestAlignList;

//                         myBaseSurface.FindBestMatchingVerListWithFlipping(refVerList, currVerList, bestAlignList, currSimi);

//                         MatrixXd alignedP_mat;
//                         myBaseSurface.AlignPolygon_LSM_FixedConfig(refVerList, bestAlignList, alignedP_mat);

//                         std::ofstream out(currSavingFolder + "Polygon_" + to_string(j) + "_" + to_string(currSimi) + ".obj");
//                         std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
//                         std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

//                         for (int m = 0; m < alignedP_mat.cols(); ++m)
//                         {
//                             cout << "v " << alignedP_mat(0, m) << " " << alignedP_mat(1, m) << " " << alignedP_mat(2, m) << endl;
//                         }

//                         cout << "f 1 2 3 4" << endl;

//                         std::cout.rdbuf(coutbuf);
//                     }

//                     cout << endl;
//                 }
//             }

            ImGui::Dummy(ImVec2(0.0f, 3.0f));
        }

        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

//        ///////////////////////////////////////////////////////////////////////////
//        //// Operation Control - 6
//        ///////////////////////////////////////////////////////////////////////////
//
//        if (ImGui::CollapsingHeader("Replaced Surface", ImGuiTreeNodeFlags_DefaultOpen))
//        {
//            float half_width = (w - p) / 2.f;
//
//            //// gap between the button group and head
//
//            ImGui::Dummy(ImVec2(0.0f, 2.0f));
//            ImGui::Text("Tolerance");
//            ImGui::SameLine(half_width, p);
//            ImGui::SetNextItemWidth(half_width);
//            ImGui::DragFloat("##Tolerance", &myBaseSurface.tolerance, 1.0f, 0.0f, 1.0f, "%.7f", 1.0f);
//
//            ImGui::Text("Absolute Tolerance");
//            ImGui::SameLine(half_width, p);
//            ImGui::SetNextItemWidth(half_width);
//            ImGui::DragFloat("##Absolute Tolerance", &myBaseSurface.absoluteTolerance, 1.0f, 0.0f, 1.0f, "%.7f", 1.0f);
//
//            ImGui::Text("Fab Threshold");
//            ImGui::SameLine(half_width, p);
//            ImGui::SetNextItemWidth(half_width);
//            ImGui::DragFloat("##Fab Threshold", &myBaseSurface.fabricationThreshold, 1.0f, 0.0f, 1.0f, "%.7f", 1.0f);
//
//            //// gap between the button group and head
//            ImGui::Dummy(ImVec2(0.0f, 2.0f));
//
//            if (ImGui::Button("Replace & Check", ImVec2(button_width, 0)))
//            {
//                myBaseSurface.InitReplacedSurfaceAndToleranceParameters();
//                myBaseSurface.IsSurfaceMeetFabRequirement();
//
//                myRender.ClearViewer(viewer);
//
//                myRender.DrawObjModel(viewer, mySurface.inputSurfaceTriMesh);
//                myRender.DrawObjModelTexture(viewer, myTilePattern.inputTilePattern, myTilePattern.inputTriTilePattern);
//                myRender.DrawObjRemeshingModel(viewer, myBaseSurface.remeshedPolySurface, myBaseSurface.remeshedTriSurface);
//                myRender.DrawObjOptimizedModel(viewer, myBaseSurface.baseSurface, myBaseSurface.triBaseSurface);
//                myRender.DrawObjReplacedSurface(viewer, myBaseSurface.replacedSurface, myBaseSurface.replacedTriSurface, myBaseSurface.baseSurface);
//            }
//
//            ImGui::SameLine(0, button_horizontalGap);
//
//            if (ImGui::Button("TODO", ImVec2(button_width, 0)))
//            {
//                // TODO
//            }
//
//            ImGui::Dummy(ImVec2(0.0f, 3.0f));
//        }
//
//        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));

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

//            ImGui::Text("Show Ground");
//            ImGui::SameLine(half_width, p);
//            ImGui::Checkbox("##Show Ground", &visibleGround);
//
//            ImGui::Text("Show Axes");
//            ImGui::SameLine(half_width, p);
//            ImGui::Checkbox("##Show Axes", &visibleAxes);

            ImGui::Text("Show Input Surface");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Input Surface", &visibleObjModel);

            ImGui::Text("Show Texture");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Texture", &visibleObjTexture);

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            ImGui::Text("Show Remeshed Model");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Remeshed Model", &visibleObjRemeshingModel);

            // ImGui::Text("Show Remeshed Edge Normal");
            // ImGui::SameLine(half_width, p);
            // ImGui::Checkbox("##Show Remeshed Edge Normal", &isShowRemeshedSurfaceEdgeNormal);

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            ImGui::Text("Show Optimized Model");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Optimized Model", &visibleObjOptimizedModel);

            // ImGui::Text("Polygon Color State");
            // ImGui::SameLine(half_width / 1.2f, p * 0.5f);
            // ImGui::DragInt("##Polygon Color State", &polygonColorState);

            // ImGui::Text("Show Edge Color");
            // ImGui::SameLine(half_width, p);
            // ImGui::Checkbox("##Show Edge Color", &isShowEdgeClusterColor);

            // ImGui::Text("Show Optimized Edge Normal");
            // ImGui::SameLine(half_width, p);
            // ImGui::Checkbox("##Show Optimized Edge Normal", &isShowOptimizedSurfaceEdgeNormal);

            // ImGui::Dummy(ImVec2(0.0f, 3.0f));

            ImGui::Text("Show Replaced Surface");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Replaced Surface", &visibleObjReplacedSurface);

            ImGui::Text("Show Shell Structure");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Shell Structure", &visibleObjShellStructure);

            // ImGui::Text("Show Replaced Shell");
            // ImGui::SameLine(half_width, p);
            // ImGui::Checkbox("##Show Replaced Shell", &visibleObjReplacedShell);

            ImGui::Text("Show Replaced Shell");
            ImGui::SameLine(half_width, p);
            ImGui::Checkbox("##Show Replaced Shell", &visibleObjReplacedPlanarShell);

            // ImGui::Text("Show Penetration Volume");
            // ImGui::SameLine(half_width, p);
            // ImGui::Checkbox("##Show Penetration Volume", &visibleObjPenetrationVolume);

            ImGui::Dummy(ImVec2(0.0f, gap_between_renderGroups));

        }
        ImGui::Dummy(ImVec2(0.0f, gap_between_controlGroups));
    };

    viewer.plugins.push_back(&menu);
    viewer.data().face_based = true;
    viewer.core().background_color.setConstant(1.0f);
}
