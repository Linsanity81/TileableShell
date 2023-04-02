//
// Created by Linsanity on 18/4/22.
//

#ifndef TILEABLE_STRUCTURE_SURFACE_H
#define TILEABLE_STRUCTURE_SURFACE_H

#include <Eigen/Eigen>
#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>

using namespace std;
using namespace Eigen;

class Surface {

public:
    /// Openmesh data
    PolyMesh inputSurfacePolyMesh;
    PolyMesh inputSurfaceTriMesh;

    /// For interpolation
    MatrixXd V;
    MatrixXi F;
    MatrixXd V_uv;
    BoundingBox bbox;
    BoundingBox uvBBox;

    vector<TexGeoTrianglePair> triPairlist;

    /// string info
    string folderPath;
    string surfaceName;

public:
    /// Initialization
    Surface();
    ~Surface();
    void ClearSurface();

    /// Read and Write Operation
    void LoadModel(char * fileName);
    void WriteOBJFile(char fileName[]);
    void ComputeBBox();
    void ComputeuvBBox();

    /// Parametrization
    void LSCM_Parametrization();
    void ARAP_Parametrization();

    /// For Barycentric Interpolation in Buckets
    void BuildTriangleBuckets();
};


#endif //TILEABLE_STRUCTURE_SURFACE_H
