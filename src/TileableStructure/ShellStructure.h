//
// Created by Linsanity on 27/6/22.
//

#ifndef TILEABLE_STRUCTURE_SHELLSTRUCTURE_H
#define TILEABLE_STRUCTURE_SHELLSTRUCTURE_H


#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>
#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"
#include "TileableStructure/BaseSurface.h"
#include "TileableStructure/ShellBlock.h"
#include "TileableStructure/CuttingPlane.h"
#include "Mesh/MeshBoolean.h"

#include <OpenMesh/Core/IO/MeshIO.hh>

using namespace std;
using namespace Eigen;

class ShellStructure {

public:

    /// linked to baseSurface
    BaseSurface * baseSurface;

    /// Openmesh Data
    PolyMesh shellStructure;
    PolyMesh shellStructureTri;

    PolyMesh overlappedShellStruct;
    PolyMesh overlappedShellStructTri;

    PolyMesh replacedShellStructure_LSM;
    PolyMesh replacedShellStructureTri_LSM;

    PolyMesh replacedShellStructure_LSM_planar;
    PolyMesh replacedShellStructureTri_LSM_planar;

    PolyMesh replacedShellStructure_combined;
    PolyMesh replacedShellStructureTri_combined;

    PolyMesh replacedPenetrationVolumeTri;

    /// Fabrication Support

    PolyMesh fullSupportSurface;
    PolyMesh fullSupportSurfaceTri;

    PolyMesh supportSurface;
    PolyMesh supportSurfaceTri;

    PolyMesh supportSurface_tolerance;
    PolyMesh supportSurfaceTri_tolerance;

    /// Parameters
    float cutUpper;
    float cutLower;

    float absoluteTolerance;

    /// Shell Analysis
    float penetrationVolumeSum;
    float worstOverlappedVolumeBlockRatio;
    float avgOverlappedVolumeBlockRatio;

    float avgBlockError;
    float worstBlockError;

    float avgContactAngleError;
    float worstContactAngleError;

    float gapVolumeSum;
    float avgGapVolumeRatio;
    float worstGapVolumeRatio;

    // Optimization time
    float shellOptimizationTime;

    /// ShellBlockList
    vector<ShellBlock> shellBlockList;

public:

    /// Initialization
    ShellStructure(BaseSurface * _baseSurface);
    ~ShellStructure();
    void InitOrigAndReplacedShellStructures();
    void InitShellStructure();
    void InitShellSupportSurface();
    void InitReplacedShellStructure_LSM();
    void InitReplacedShellStructure_LSM_planar();
    void UpdateTolerance();

    /// Support Generation
    void GenerateSupportForOrigShellBlocks(string folderPath);

    /// Result Analysis
    void WorstCaseAnalysis();
    void ComputeReplacedPenetrationVolumeTri();
    void GetReplacedBlockMatrixTri(const PolyMesh::FaceHandle & currFaceHandle, MatrixXd & V, MatrixXi & F);
    void ComputeBlockSimilarityTable_LSM();
    void ComputeWorstBlockSimilarityError();
    void ComputeWorstContactAngleError();
    double ComputeAngleBetweenTwoVectors(const PolyMesh::Point & v1, const PolyMesh::Point & v2);

    /// Save Files
    string Create3DPrintingFolderName(string folderPath, string surfacePath, string tilePath);
    string CreateSavingAllFolderName(string folderPath, string surfacePath, string tilePath);
    void SaveShell(string folderPath, bool isSavePrintingFiles);
    void SaveOrigOBJsFor3dPrinting(string folderPath);
    void SaveBaseSurface(string folderPath);
    void SaveOrigShellBlocks(string folderPath);
    void SaveReplacedShellBlocks(string folderPath);
    void SaveOrigShellBlocksMotionXMLFile(string folderPath);
    void SaveOrigShellBlocksWithDigits(string folderPath);
    void SaveReplacedShellBlocksWithDigits(string folderPath);
    void SaveOverlappedShellBlocks(string folderPath);
    void SaveOverlappedShellForSupportGeneration(string folderPath);
    void SaveOverlappedToleranceShellForSupportGeneration(string folderPath);
    void SaveShellSupportSurface(string folderPath);
    void SaveShellOptimizationResult(string folderPath);
    void SaveAllClusteringBlockOBJs(string folderPath, string templatePath);
    void SaveClusterBlockOBJs(string folderPath, string templatePath, int clusterID);

    /// Helper Function
    void UpperAndLowerPointList2PolyMesh(const vector<PolyMesh::Point> & currUpperPointList, const vector<PolyMesh::Point> & currLowerPointList, PolyMesh & myMesh);
    void UpperAndLowerPointList2PolyMesh(const vector<Vector3d> & upperPointList, const vector<Vector3d> & lowerPointList, PolyMesh & myMesh);
};


#endif //TILEABLE_STRUCTURE_SHELLSTRUCTURE_H
