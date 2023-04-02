///////////////////////////////////////////////////////////////
//
// HelpDefine.h
//
//   General defines, parameter setting, etc.
//
// by Yingjie Cheng and Peng SONG
//
// 06/Dec/2020
//
//
///////////////////////////////////////////////////////////////


#ifndef _HELP_DEFINE_H
#define _HELP_DEFINE_H

#include <cmath>
#include <Eigen/Eigen>
#include <iostream>

using namespace Eigen;
using namespace std;

#ifndef vec
#define vec Vector3d
#endif

///=========================================================================================///
///                                     Math Related
///=========================================================================================///

// Max and min integer/float
#define  MIN_INT                   -10000000
#define  MAX_INT                    10000000
#define  MIN_FLOAT                 -10000000.0
#define  MAX_FLOAT                  10000000.0

#define FLOAT_ERROR_SMALL           0.0000001
#define FLOAT_ERROR_LARGE           0.0001

#define _ToRadian(X)		((X)/180.0*M_PI)
#define _ToDegree(X)		((X)*180.0/M_PI)

#ifndef _EPSILON
#define _EPSILON		1e-7
#endif

#ifndef _MAX
#define _MAX(a,b)		( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef _MIN
#define _MIN(a,b)		( ((a) < (b)) ? (a) : (b) )
#endif

#ifndef _IN_BETWEEN
#define _IN_BETWEEN(v,a,b)	( ((v) >= (a)) && ((v) <= (b)) )
#endif

#ifndef M_PI
#define M_PI			3.1415926535897932384626433832795
#endif

///=========================================================================================///
///                                     Geometry Related
///=========================================================================================///

////////////////////////////////////////////
/// Part Motion

struct Motion
{
    int type;                     // Motion type: R, T, 2T, 2R, T+R
    //pair<double, double>	range;    // Range (angle or position)
};




////////////////////////////////////////////
/// 3D Plane

struct Plane
{
    Vector3d point;
    Vector3d normal;

    void PrintPlane();
};




////////////////////////////////////////////
/// PolygonFace

struct Face
{
    vector<Vector3d> faceVerList;
};




////////////////////////////////////////////
/// 3D Box

struct Box
{
    Vector3d minPt;
    Vector3d maxPt;

    Vector3d cenPt;
    Vector3d size;

    vector<Vector3d> corners;   // 8 corners


    Box();
    Box & operator=(const Box &box);
    void PrintBox();

    Vector3d GetCenter();
    Vector3d GetSize();
    void GetMinMaxPts();
    vector<Vector3d> GetCorners();
    vector<vector<int>> GetEdges();
    vector<vector<int>> GetQuadFaces();
    vector<Vector3i> GetTriFaces();

    void Transform(Vector3d transVec, Vector3d scale);
    double GetQuadArea();
};




////////////////////////////////////////////
/// 3D Triangle

struct Triangle
{
    int id;          // For mesh deformation

    Vector3d v[3];
    Vector3d normal;
    float area;          // Face area
    Vector3d center;

    void Init(Vector3d _v0, Vector3d _v1, Vector3d _v2);
    Triangle & operator=(const Triangle &tri);
    bool IsEqual(Triangle *tri);
    void PrintTriangle();

    void ComputeCenter();
    void ComputeArea();
    void ComputeNormal();
    void CorrectNormal(Vector3d tagtNormal);

    Vector3d GetBBoxMinPt()
    {
        Vector3d bboxMinPt;

        bboxMinPt(0) = _MIN(v[0](0), _MIN(v[1](0), v[2](0)));
        bboxMinPt(1) = _MIN(v[0](1), _MIN(v[1](1), v[2](1)));
        bboxMinPt(2) = _MIN(v[0](2), _MIN(v[1](2), v[2](2)));

        return bboxMinPt;
    };

    Vector3d GetBBoxMaxPt()
    {
        Vector3d bboxMaxPt;

        bboxMaxPt(0) = _MAX(v[0](0), _MIN(v[1](0), v[2](0)));
        bboxMaxPt(1) = _MAX(v[0](1), _MIN(v[1](1), v[2](1)));
        bboxMaxPt(2) = _MAX(v[0](2), _MIN(v[1](2), v[2](2)));

        return bboxMaxPt;
    };
};


// 3D Circle
struct Circle
{
    Vector3d center;
    Vector3d normal;
    float radius;
};

// 3D bounding box struct
typedef struct BoundingBox
{
    Vector3d minPt;
    Vector3d maxPt;
    Vector3d centerPt;

    BoundingBox() :
            minPt(0, 0, 0),
            maxPt(0, 0, 0),
            centerPt(0, 0, 0)
    {}
}BoundingBox;

struct TexGeoTrianglePair
{
    Triangle texTriangle;       // Mesh triangle in texture domain.
    Triangle geoTriangle;       // Mesh triangle in geometrical space.
};

///=========================================================================================///
///                                     Math Related
///=========================================================================================///

Vector4d liftVec(Vector3d vec3);
Vector4d liftVec(RowVector3d vec3);
Matrix3d GetRotationFromAffine(Matrix4d AffineM);
Matrix4d GetAffine(Matrix3d RotM);
Matrix4d GetAffine(Matrix3d RotM, Vector3d TransVec);
void MultiplyPoint(vec inPt, const Matrix4d& inMat, vec &outPt);
void MultiplyVector(vec inVec, Matrix4d inMat, vec &outVec);
void MultiplyNormal(vec inNor, const Matrix4d& inMat, vec &outNor);

Matrix4d GetTransformMatrix(Vector3d eulerAngles, Vector3d position);
Matrix4d GetRotationMatrix(double rotAngle, Vector3d rotAxis);
Matrix2d GetRotationMatrix(double rotAngle);
Matrix4d GetRotationMatrix(double rotAngle, Vector3d rotAxis, const Vector3d& rotCenter);
Matrix3d GetRotationMatrix3d(double rotAngle, Vector3d rotAxis, const Vector3d& rotCenter);
Matrix4d GetRotationMatrix(Vector3d eulerAngles);
Matrix4d GetTranslateMatrix(Vector3d transVec);
Matrix3d GetTranslateMatrix3d(Vector3d transVec);
Matrix4d GetScaleMatrix(Vector3d scaleVec);
Matrix3d GetScaleMatrix3d(Vector3d scaleVec);
void RotateVectorAroundAxis(Vector3d sourceVec, Vector3d axis, double theta, Vector3d & outputVec);

bool IsPointInsideTriangle(vec point, const Triangle& triangle, bool isPrint = false);
float GetTriangleArea(Vector3d triPt0, Vector3d triPt1, Vector3d triPt2);

#endif
