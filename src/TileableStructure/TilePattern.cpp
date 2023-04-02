//
// Created by Linsanity on 18/4/22.
//

#include "TilePattern.h"
#include <OpenMesh/Core/IO/MeshIO.hh>

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

TilePattern::TilePattern()
{
    scalar = 7;

    rotationAngle = -15;
}

TilePattern::~TilePattern()
{
    ClearPattern();
}

void TilePattern::ClearPattern()
{
    inputTilePattern.clear();
    inputTriTilePattern.clear();

    origTilePattern.clear();
    origTriTilePattern.clear();
}




//**************************************************************************************//
//                                   Texture Operation
//**************************************************************************************//

void TilePattern::LoadTexture(char *fileName)
{
    OpenMesh::IO::read_mesh(origTilePattern, string(fileName));

    OpenMesh::IO::read_mesh(origTriTilePattern, string(fileName));

    inputTilePattern = origTilePattern;

    inputTriTilePattern = origTriTilePattern;

    ComputeBBox();

    folderPath = string(fileName);

    patternName = string(fileName);

    for (int i = patternName.size() - 1; i > 0; --i)
    {
        if (patternName[i] == '/')
        {
            patternName.erase(patternName.begin(), patternName.begin() + i + 1);
            break;
        }
    }
}

void TilePattern::ComputeBBox()
{
    MatrixXd V;
    MatrixXi F;

    PolyMesh2Matrix(inputTriTilePattern, V, F);

    if (V.rows() <= 0)
        return;

    float minU, minV, maxU, maxV;
    minU = maxU = V(0, 0);
    minV = maxV = V(0, 1);

    for (int i=1; i<V.rows(); i++)
    {
        if (maxU < V(i, 0))
            maxU = V(i, 0);
        if (minU > V(i, 0))
            minU = V(i, 0);

        if (maxV < V(i, 1))
            maxV = V(i, 1);
        if (minV > V(i, 1))
            minV = V(i, 1);
    }

    bbox.minPt = Vector3d(minU, minV, 0);
    bbox.maxPt = Vector3d(maxU, maxV, 0);
}



