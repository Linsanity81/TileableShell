//
// Created by Linsanity on 2/10/22.
//

#ifndef TILEABLE_STRUCTURE_CUTTINGPLANE_H
#define TILEABLE_STRUCTURE_CUTTINGPLANE_H

#include <Eigen/Eigen>
#include <vector>
#include <Utility/HelpTypedef.h>
#include <Utility/HelpFunc.h>
#include "BaseSurface.h"

class CuttingPlane {

public:
    BaseSurface * baseSurface;
    OpenMesh::EdgeHandle relatedEdgeHandle;

    int edgeLabel;
    int dihedralAngleClusteringLabel;
    double realDihedralAngle;
    double computationalDihedralAngle;
    Vector3d origEdgeNormal;
    Vector3d edgeNormal;   // Change
    Vector3d edgeCenter;
    Vector3d edgeNormalTo; // Change
    Vector3d dirFrom;
    Vector3d dirTo;
    Vector3d dirAxis;
    bool isBoundary;
    double assignedFreeAngle;

    OpenMesh::FaceHandle positiveFaceHandle;
    OpenMesh::FaceHandle negativeFaceHandle;

    float offsetAngle; // Change / degree

public:
    /// Initialization
    CuttingPlane();
    CuttingPlane(OpenMesh::EdgeHandle _inputEdgeHandle, BaseSurface * _baseSurface);
    ~CuttingPlane();

    /// Update
    void UpdateCuttingPlane(OpenMesh::FaceHandle faceHandle, float assignedAngle);
    void UpdateCuttingPlane(float currOffsetAngle);
    void UpdateCuttingPlane_boundary(float currOffsetAngle);

    /// Get Operation
    float GetCuttingPlaneAngle(const OpenMesh::FaceHandle & currFaceHandle);

};


#endif //TILEABLE_STRUCTURE_CUTTINGPLANE_H
