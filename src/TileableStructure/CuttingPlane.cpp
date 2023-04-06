//
// Created by Linsanity on 2/10/22.
//

#include "CuttingPlane.h"

///=========================================================================================///
///                                        Initialization
///=========================================================================================///

CuttingPlane::CuttingPlane()
{

}

CuttingPlane::CuttingPlane(OpenMesh::EdgeHandle _inputEdgeHandle, BaseSurface * _baseSurface)
{
    baseSurface = _baseSurface;
    relatedEdgeHandle = _inputEdgeHandle;

    assignedFreeAngle = 0;

    // Get dihedralAngle for each edge
    OpenMesh::EPropHandleT< double > dihedralAngle;
    baseSurface->baseSurface.get_property_handle(dihedralAngle, "dihedralAngle");

    // Get dihedralAngleLabel for each edge
    OpenMesh::EPropHandleT< int > dihedralAngleLabel;
    baseSurface->baseSurface.get_property_handle(dihedralAngleLabel, "dihedralAngleLabel");

    // Get angleClusterCentroid for each edge
    OpenMesh::EPropHandleT< double > angleClusterCentroid;
    baseSurface->baseSurface.get_property_handle(angleClusterCentroid, "angleClusterCentroid");

    // Get edgeClusteringLabel for each edge
    OpenMesh::EPropHandleT< int > edgeClusteringLabel;
    baseSurface->baseSurface.get_property_handle(edgeClusteringLabel, "edgeClusteringLabel");

    // Get normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface->baseSurface.get_property_handle( normal ,"normal");

    edgeLabel = baseSurface->baseSurface.property(edgeClusteringLabel, _inputEdgeHandle);

    dihedralAngleClusteringLabel = baseSurface->baseSurface.property(dihedralAngleLabel, relatedEdgeHandle);

    realDihedralAngle = baseSurface->baseSurface.property(dihedralAngle, relatedEdgeHandle);

    computationalDihedralAngle = baseSurface->baseSurface.property(angleClusterCentroid, relatedEdgeHandle);

    edgeNormal = baseSurface->baseSurface.property(normal, relatedEdgeHandle);
    edgeNormal.normalize();

    PolyMesh::Point from = baseSurface->baseSurface.point(baseSurface->baseSurface.from_vertex_handle(baseSurface->baseSurface.halfedge_handle(relatedEdgeHandle,0)));
    PolyMesh::Point to   = baseSurface->baseSurface.point(baseSurface->baseSurface.to_vertex_handle(baseSurface->baseSurface.halfedge_handle(relatedEdgeHandle,0)));

    dirFrom = Vector3d(from[0], from[1], from[2]);
    dirTo   = Vector3d(to[0], to[1], to[2]);

    edgeCenter = (dirFrom + dirTo) / 2.0;

    edgeNormalTo = edgeCenter + edgeNormal * 0.1;

    dirAxis = dirTo - dirFrom;
    dirAxis.normalize();

    offsetAngle = 0.0;
    origEdgeNormal = edgeNormal;

    // Define positive and negative face
    vector<OpenMesh::FaceHandle> fhList;
    for (PolyMesh::FaceIter f_it=baseSurface->baseSurface.faces_begin(); f_it!=baseSurface->baseSurface.faces_end(); ++f_it)
    {
        PolyMesh::FaceEdgeIter fe_it;

        for (fe_it = baseSurface->baseSurface.fe_iter(*f_it); fe_it.is_valid(); ++fe_it)
        {
            const PolyMesh::Point currTo   = baseSurface->baseSurface.point(baseSurface->baseSurface.to_vertex_handle(baseSurface->baseSurface.halfedge_handle(*fe_it,0)));
            const PolyMesh::Point currFrom = baseSurface->baseSurface.point(baseSurface->baseSurface.from_vertex_handle(baseSurface->baseSurface.halfedge_handle(*fe_it,0)));

            if ((currTo == to and currFrom == from) or (currTo == from and currFrom == to))
            {
                fhList.push_back(*f_it);
            }

            if (fhList.size() == 2)
                break;
        }

        if (fhList.size() == 2)
        {
            isBoundary = false;
        }
        else
        {
            isBoundary = true;
        }
    }

    positiveFaceHandle = fhList[0];
    PolyMesh::Point currFaceCenter = baseSurface->baseSurface.calc_face_centroid(positiveFaceHandle);
    Vector3d currFaceCentroid(currFaceCenter[0], currFaceCenter[1], currFaceCenter[2]);

    double origDist = (edgeNormalTo - currFaceCentroid).norm();

//    Matrix4d rotMat = GetRotationMatrix(15 * (M_PI / 180.0f), dirAxis);
//    RowVector4d currPoint(edgeNormalTo[0], edgeNormalTo[1], edgeNormalTo[2], 0);
//    auto rotVer = currPoint * rotMat;
//    Vector3d rotatedVer(rotVer[0], rotVer[1], rotVer[2]);

    Matrix3d rotMat = GetRotationMatrix3d(15 * (M_PI / 180.0f), dirAxis, edgeCenter);
//    Vector4d currPoint(edgeNormalTo[0], edgeNormalTo[1], edgeNormalTo[2], 0);
    Vector3d currPoint(edgeNormal[0], edgeNormal[1], edgeNormal[2]);
    auto rotVer = rotMat * currPoint;

    Vector3d newEdgeNormal = Vector3d(rotVer[0], rotVer[1], rotVer[2]);
    newEdgeNormal.normalize();
    Vector3d rotatedVer = edgeCenter + newEdgeNormal;

    double afterDist = (rotatedVer - currFaceCentroid).norm();

    if (afterDist < origDist)
    {
        if (!isBoundary)
        {
            negativeFaceHandle = fhList[1];
        }
        else
        {

        }
    }
    else
    {
        if (!isBoundary)
        {
            negativeFaceHandle = positiveFaceHandle;
            positiveFaceHandle = fhList[1];
        }
        else
        {
            negativeFaceHandle = positiveFaceHandle;

            OpenMesh::FaceHandle nullHandle;
            positiveFaceHandle = nullHandle;
        }
    }
}

CuttingPlane::~CuttingPlane()
{
    baseSurface = nullptr;
}




///=========================================================================================///
///                                        Update
///=========================================================================================///

void CuttingPlane::UpdateCuttingPlane(OpenMesh::FaceHandle faceHandle, float assignedAngle)
{
//    if (isBoundary)
//    {
//        assignedFreeAngle = assignedAngle;
//    }

    if (faceHandle == positiveFaceHandle)
    {
        if (isBoundary)
        {
//            cout << "positive. " << endl;
            if (GetCuttingPlaneAngle(faceHandle) == FREE_ANGLE)
            {
                float currAngleDiff = 90 - assignedAngle;
                UpdateCuttingPlane(-currAngleDiff);
            }
            else
            {
                float currAngleDiff = GetCuttingPlaneAngle(faceHandle) - assignedAngle;
                UpdateCuttingPlane(-currAngleDiff);
            }

            assignedFreeAngle = assignedAngle;
        }
        else
        {
            float currAngleDiff = GetCuttingPlaneAngle(faceHandle) - assignedAngle;
            UpdateCuttingPlane(-currAngleDiff);
        }

        return;
    }

    if (faceHandle == negativeFaceHandle)
    {
        if (isBoundary)
        {
//            cout << "negative. " << endl;
            if (GetCuttingPlaneAngle(faceHandle) == FREE_ANGLE)
            {
                float currAngleDiff = 90 - assignedAngle;
                UpdateCuttingPlane(currAngleDiff);
            }
            else
            {
                float currAngleDiff = GetCuttingPlaneAngle(faceHandle) - assignedAngle;
                UpdateCuttingPlane(currAngleDiff);
            }

            assignedFreeAngle = assignedAngle;
        }
        else
        {
            float currAngleDiff = GetCuttingPlaneAngle(faceHandle) - assignedAngle;
            UpdateCuttingPlane(currAngleDiff);
        }

        return;
    }
}

void CuttingPlane::UpdateCuttingPlane(float currOffsetAngle)
{
    offsetAngle += currOffsetAngle;

   if (fabs(offsetAngle) > 10)
   {
       offsetAngle -= currOffsetAngle;
       return;
   }

    if (offsetAngle == 0)
        return;

    Vector3d currDirAxis;

    if (offsetAngle < 0)
        currDirAxis = -dirAxis;
    else
        currDirAxis = dirAxis;

    Matrix3d rotMat = GetRotationMatrix3d(fabs(offsetAngle) * (M_PI / 180.0f), currDirAxis, edgeCenter);
//    Vector4d currPoint(edgeNormalTo[0], edgeNormalTo[1], edgeNormalTo[2], 0);
    Vector3d currPoint(origEdgeNormal[0], origEdgeNormal[1], origEdgeNormal[2]);
    auto rotVer = rotMat * currPoint;

    edgeNormal = Vector3d(rotVer[0], rotVer[1], rotVer[2]);
    edgeNormal.normalize();
    edgeNormalTo = edgeCenter + edgeNormal;

//    if (isBoundary)
//    {
//        cout << "origEdgeNormal: " << origEdgeNormal[0] << origEdgeNormal[1] << origEdgeNormal[2] << endl;
//        cout << "currEdgeNormal: " << edgeNormal[0] << edgeNormal[1] << edgeNormal[2] << endl << endl;
//    }

    // Get normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    baseSurface->baseSurface.get_property_handle( normal ,"normal");

    baseSurface->baseSurface.property(normal, relatedEdgeHandle) = edgeNormal;
}




///=========================================================================================///
///                                        Get Operation
///=========================================================================================///

float CuttingPlane::GetCuttingPlaneAngle(const OpenMesh::FaceHandle & currFaceHandle)
{
    if (isBoundary)
    {
        if (assignedFreeAngle == 0)
            return FREE_ANGLE;
        else
            return assignedFreeAngle;
    }

    else
    {
        if (currFaceHandle == positiveFaceHandle)
        {
            return computationalDihedralAngle / 2.0 + offsetAngle;
        }

        else if (currFaceHandle == negativeFaceHandle)
        {
            return computationalDihedralAngle / 2.0 - offsetAngle;
        }

        else
        {
            return ERROR_ANGLE;
        }
    }
}
