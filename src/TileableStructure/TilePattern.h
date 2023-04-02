//
// Created by Linsanity on 18/4/22.
//

#ifndef TILEABLE_STRUCTURE_TILEPATTERN_H
#define TILEABLE_STRUCTURE_TILEPATTERN_H

#include <Eigen/Eigen>
#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>

using namespace std;
using namespace Eigen;

class TilePattern {
public:
    /// Openmesh data
    PolyMesh inputTilePattern;
    PolyMesh inputTriTilePattern;

    PolyMesh origTilePattern;
    PolyMesh origTriTilePattern;

    /// bounding box
    BoundingBox bbox;

    /// Scalar
    float scalar;

    /// Rotation Angle
    float rotationAngle;

    /// String Info
    string folderPath;
    string patternName;

public:
    /// Initialization
    TilePattern();
    ~TilePattern();
    void ClearPattern();

    /// Texture Operation
    void LoadTexture(char * fileName);
    void ComputeBBox();
};


#endif //TILEABLE_STRUCTURE_TILEPATTERN_H
