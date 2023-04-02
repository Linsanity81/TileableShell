//
// Created by Linsanity on 18/4/22.
//

#ifndef TILEABLE_STRUCTURE_INITIALIZATION_H
#define TILEABLE_STRUCTURE_INITIALIZATION_H

#include <Eigen/Eigen>
#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>
#include "TileableStructure/Surface.h"
#include "TileableStructure/TilePattern.h"
#include "TileableStructure/BaseSurface.h"
#include "TileableStructure/CuttingPlane.h"

using namespace std;
using namespace Eigen;

class Surface;
class TilePattern;
class BaseSurface;
class BasePolygon;
class CuttingPlane;

class Initialization {

public:

    bool isPlanarization;

public:
    /// Initialization
    Initialization();
    ~Initialization();

    /// Init BaseSurface
    void InitBaseSurface(string inputFilePath, BaseSurface * baseSurface);

    /// Remeshing
    void Remeshing(Surface * surface, TilePattern * tilePattern, BaseSurface * baseSurface);
    bool ComputeSurfaceCoord(Surface * surface, TilePattern * tilePattern, BaseSurface * baseSurface, Vector3d ptTexCoord, Vector3d &ptSurfCoord);

    /// Planarization
    void ShapeOp_Planarization(BaseSurface * baseSurface);

    /// Properties Operation
    void AddProperty2BaseSurface(PolyMesh & baseSurface, PolyMesh & triBaseSurface);
};


#endif //TILEABLE_STRUCTURE_INITIALIZATION_H
