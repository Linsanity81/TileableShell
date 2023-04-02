//
// Created by Linsanity on 27/6/22.
//

#ifndef TILEABLE_STRUCTURE_SHELLBLOCK_H
#define TILEABLE_STRUCTURE_SHELLBLOCK_H

#include <Eigen/Eigen>
#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>
#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"
#include "TileableStructure/BaseSurface.h"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/DefaultTriMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

using namespace std;
using namespace Eigen;

class ShellBlock {

public:

    PolyMesh::FaceHandle origFaceHandle;

    int assemblyID;
    int clusterID;
    float cutUpper;
    float cutLower;

    double xAxisAngle_assemblyDigits;
    double xAxisAngle_clusterDigits;

    Vector3d curaPos;

    Vector3d lowerFaceNormal;
    Vector3d upperFaceNormal;

    Vector3d lowerDiagonalStart;
    Vector3d lowerDiagonalEnd;

    Vector3d upperDiagonalStart;
    Vector3d upperDiagonalEnd;

    vector<int> assemblyDigitList;
    vector<Vector3d> assemblyDigitCenterList;
    Vector3d assemblyDigitSize;

    vector<int> clusterDigitList;
    vector<Vector3d> clusterDigitCenterList;
    Vector3d clusterDigitSize;

    Matrix3d rotMatForAssemblyDigitBoolean;
    Matrix3d scaleAssemblyMat;

    Matrix3d rotMatForClusterDigitBoolean;
    Matrix3d scaleClusterMat;

    PolyMesh origShellBlock;
    PolyMesh origShellBlockTri;
    PolyMesh origShellBlock_digits;
    PolyMesh origShellBlock_digits_cura;

    PolyMesh origShellBlock_overlap;
    PolyMesh origShellBlockTri_overlap;

    PolyMesh origShellBlock_overlap_tolerance;
    PolyMesh origShellBlockTri_overlap_tolerance;

    PolyMesh replacedShellBlock;
    PolyMesh replacedShellBlockTri;
    PolyMesh replacedShellBlock_digits;
    PolyMesh replacedShellBlock_digits_cura;

    PolyMesh replacedShellBlock_aligned;

public:

    /// Initialization
    ShellBlock(PolyMesh::FaceHandle _origFaceHandle, int _assemblyID, int _clusterID, float _cutUpper, float _cutLower,
               Vector3d _lowerFaceNormal, Vector3d _upperFaceNormal,
               Vector3d _lowerDiagonalStart, Vector3d _lowerDiagonalEnd,
               Vector3d _upperDiagonalStart, Vector3d _upperDiagonalEnd);
    ~ShellBlock();

    /// CSG Operation
    void AddDigit2OrigShellBlock();
    void AddDigit2ReplacedShellBlock();
    string GetDigitOBJFilePath(int digit);

    /// Save Files
    void SaveOrigShellBlock(string folderPath);
    void SaveReplacedShellBlock(string folderPath);
    void SaveOrigShellBlockWithDigits(string folderPath);
    void SaveOrigShellBlockWithDigits_cura(string folderPath);
    void SaveReplacedShellBlockWithDigits(string folderPath);
    void SaveReplacedShellBlockWithDigits_cura(string folderPath);
    void SaveOrigShellBlock_overlapped(string folderPath);
};


#endif //TILEABLE_STRUCTURE_SHELLBLOCK_H
