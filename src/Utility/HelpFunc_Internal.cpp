//
// Created by Linsanity on 10/9/22.
//

#include "HelpFunc_Internal.h"

/////==============================================================================//
/////                              3D Plane
/////==============================================================================//

void Plane::PrintPlane()
{
    cout << "--------------------------" << endl;
    cout << "Point: " << endl <<  point << endl << endl;
    cout << "Normal: " << endl << normal << endl << endl;
    cout << "--------------------------" << endl << endl;
}

/////==============================================================================//
/////                              3D box
/////==============================================================================//

Box::Box()
{
    minPt = Vector3d(0, 0, 0);
    maxPt = Vector3d(0, 0, 0);
    cenPt = Vector3d(0, 0, 0);
}

Box & Box::operator=(const Box &box)
{
    if( this == &box )
        return *this;

    this->minPt = box.minPt;
    this->maxPt = box.maxPt;
    this->cenPt = box.cenPt;

    this->size  = box.size;

    return *this;
}

void Box::PrintBox()
{
    printf("Box: [%7.3f  %7.3f  %7.3f]      [%7.3f  %7.3f  %7.3f] \n",
           minPt(0), minPt(1), minPt(2), maxPt(0), maxPt(1), maxPt(2));
}


Vector3d Box::GetCenter()
{
    cenPt = 0.5f * ( minPt + maxPt );
    return cenPt;
}

Vector3d Box::GetSize()
{
    size = maxPt - minPt;
    return size;
}

void Box::GetMinMaxPts()
{
    minPt = cenPt - 0.5f * size;
    maxPt = cenPt + 0.5f * size;
}

vector<Vector3d> Box::GetCorners()
{
    Vector3d tempCorners[8];

    tempCorners[0] = Vector3d(minPt(0), minPt(1), minPt(2));
    tempCorners[1] = Vector3d(minPt(0), maxPt(1), minPt(2));
    tempCorners[2] = Vector3d(maxPt(0), maxPt(1), minPt(2));
    tempCorners[3] = Vector3d(maxPt(0), minPt(1), minPt(2));

    tempCorners[4] = Vector3d(minPt(0), minPt(1), maxPt(2));
    tempCorners[5] = Vector3d(minPt(0), maxPt(1), maxPt(2));
    tempCorners[6] = Vector3d(maxPt(0), maxPt(1), maxPt(2));
    tempCorners[7] = Vector3d(maxPt(0), minPt(1), maxPt(2));

    //vector<Vector3d> cornerList;
    corners.clear();
    for (int i = 0; i < 8; i++)
    {
        corners.push_back(tempCorners[i]);
    }

    return corners;
}

vector<vector<int>> Box::GetEdges()
{
    int edges[12][2] =
            {
                    0, 3,  // Along X-axis
                    1, 2,
                    5, 6,
                    4, 7,

                    0, 1,  // Along Y-axis
                    4, 5,
                    7, 6,
                    3, 2,

                    0, 4,  // Along Z-axis
                    3, 7,
                    2, 6,
                    1, 5
            };


    vector<vector<int>> edgeList;

    for (int i = 0; i < 12; i++)
    {
        vector<int> edge;
        for (int j = 0; j < 2; j++)
        {
            edge.push_back(edges[i][j]);
        }

        edgeList.push_back(edge);
    }

    return edgeList;
}


vector<vector<int>> Box::GetQuadFaces()
{
    int faces[6][4] =
            {
                    0, 4, 5, 1,  // Normal X-axis
                    3, 2, 6, 7,

                    0, 3, 7, 4,  // Normal Y-axis
                    1, 5, 6, 2,

                    0, 1, 2, 3,  // Normal Z-axis
                    4, 7, 6, 5
            };

    vector<vector<int>> faceList;
    for (int i = 0; i < 6; i++)
    {
        vector<int> face;
        for (int j = 0; j < 4; j++)
        {
            face.push_back(faces[i][j]);
        }

        faceList.push_back(face);
    }

    return faceList;
}


vector<Vector3i> Box::GetTriFaces()
{
    int faces[12][3] =
            {
                    0, 4, 5,  // Normal X-axis
                    5, 1, 0,
                    3, 2, 6,
                    6, 7, 3,

                    0, 3, 7,  // Normal Y-axis
                    7, 4, 0,
                    1, 5, 6,
                    6, 2, 1,

                    0, 1, 2,  // Normal Z-axis
                    2, 3, 0,
                    4, 7, 6,
                    6, 5, 4
            };

    vector<Vector3i> faceList;
    for (int i = 0; i < 12; i++)
    {
        Vector3i triFace = Vector3i(faces[i][0], faces[i][1], faces[i][2]);

        faceList.push_back(triFace);
    }

    return faceList;
}



void Box::Transform(Vector3d transVec, Vector3d scale)
{
    minPt(0) *= scale(0);  minPt(1) *= scale(1);  minPt(2) *= scale(2);
    maxPt(0) *= scale(0);  maxPt(1) *= scale(1);  maxPt(2) *= scale(2);
    cenPt(0) *= scale(0);  cenPt(1) *= scale(1);  cenPt(2) *= scale(2);

    minPt += transVec;
    maxPt += transVec;
    cenPt += transVec;
}

double Box::GetQuadArea()
{
    Vector3d dimen = maxPt - minPt;
    double quadArea = 0;

    if      ( dimen(0) == 0 && dimen(1)  > 0 &&  dimen(2)  > 0 )
        quadArea = dimen(1) * dimen(2);  // y-z plane quad
    else if ( dimen(0)  > 0 && dimen(1) == 0 &&  dimen(2)  > 0 )
        quadArea = dimen(0) * dimen(2);  // x-z plane quad
    else if ( dimen(0)  > 0 && dimen(1)  > 0 &&  dimen(2) == 0 )
        quadArea = dimen(0) * dimen(1);  // x-y plane quad
    else
        printf("Warning: The box is not degenerated into a quad. \n");

    return quadArea;
}




///==============================================================================//
///                              3D Triangle
///==============================================================================//

void Triangle::Init(Vector3d _v0, Vector3d _v1, Vector3d _v2)
{
    v[0] = _v0;
    v[1] = _v1;
    v[2] = _v2;
}

Triangle & Triangle::operator=(const Triangle &tri)
{
    if( this == &tri )
        return *this;

    for (int i=0; i<3; i++)
    {
        this->v[i] = tri.v[i];
    }

    this->normal = tri.normal;
    this->center = tri.center;
    this->area   = tri.area;

    return *this;
}

bool Triangle::IsEqual(Triangle *tri)
{
    if( this->v[0] == tri->v[0] &&
        this->v[1] == tri->v[1] &&
        this->v[2] == tri->v[2] )
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Triangle::PrintTriangle()
{
    printf("v0: [%.12f %.12f %.12f] \n", v[0](0), v[0](1), v[0](2));
    printf("v1: [%.12f %.12f %.12f] \n", v[1](0), v[1](1), v[1](2));
    printf("v2: [%.12f %.12f %.12f] \n", v[2](0), v[2](1), v[2](2));
    printf("\n");
}


void Triangle::ComputeCenter()
{
    center = (v[0]+v[1]+v[2])/3.0f;
}


void Triangle::ComputeArea()
{
    Vector3d normal  = (v[1] - v[0]).cross(v[2] - v[0]);
    area  = 0.5f * normal.norm();
}

void Triangle::ComputeNormal()
{
    Vector3d tempNor = (v[1] - v[0]).cross(v[2] - v[0]);  // Assume the vertices are saved in counter-clockwise
    double tempNorLen = (tempNor).norm();

    if ( tempNorLen != 0 )    normal = tempNor / tempNorLen;
    else                      normal = Vector3d(1,0,0);     // Note: this default vector also can be others
}

void Triangle::CorrectNormal(Vector3d tagtNormal)
{
    // Compute initial normal
    ComputeNormal();

    // Rearrange vertex order if needed
    double dotp = normal.dot(tagtNormal);
    if ( dotp < 0 )
    {
        Vector3d triVers[3];
        for (int i=0; i<3; i++)
        {
            triVers[i] = v[i];
        }

        v[0] = triVers[0];
        v[1] = triVers[2];
        v[2] = triVers[1];
    }

    // Recompute the normal
    ComputeNormal();
}




///======================================================================================//
///                              Matrix and Array Related
///======================================================================================//

///return vec4 from vec3 for computing AX+T
Vector4d liftVec(Vector3d vec3)
{
    Vector4d ColVec4;
    ColVec4(0) = vec3(0);
    ColVec4(1) = vec3(1);
    ColVec4(2) = vec3(2);
    ColVec4(3) = 1;
    return ColVec4;
}

Vector4d liftVec(RowVector3d vec3)
{
    Vector4d ColVec4;
    ColVec4(0) = vec3(0);
    ColVec4(1) = vec3(1);
    ColVec4(2) = vec3(2);
    ColVec4(3) = 1;
    return ColVec4;
}

///Affine matrix with Rotation and Translation matrix
Matrix3d GetRotationFromAffine(Matrix4d AffineM)
{
    Matrix3d RotM;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
            RotM(i,j) = AffineM(i,j);
    }
    return RotM;
}

Matrix4d GetAffine(Matrix3d RotM)
{
    Matrix4d AffineM;
    for(int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            AffineM(i, j) = RotM(i, j);
        }

        AffineM(i,3) = 0.0f;
    }

    AffineM(3,0) = 0.0f;
    AffineM(3,1) = 0.0f;
    AffineM(3,2) = 0.0f;
    AffineM(3,3) = 1.0f;

    return AffineM;
}

Matrix4d GetAffine(Matrix3d RotM, Vector3d TransVec)
{
    Matrix4d AffineM;
    for(int i=0;i<3;i++)
    {
        for (int j = 0; j < 3; j++)
        {
            AffineM(i, j) = RotM(i, j);
        }
        AffineM(i,3) = TransVec(i);
    }
    AffineM(3,3) = 1.0f;
    return AffineM;
}

// Note: Input matrix is Transform of OpenGL style matrix (Row major)
void MultiplyPoint(vec inPt, const Matrix4d& inMat, vec &outPt)
{
    Vector4d inPt4, outPt4;

    inPt4 = liftVec(inPt);
    outPt4 = inMat * inPt4;

    for(int i=0; i < 3; i++)
        outPt(i) = outPt4(i);
}

// Note: Input matrix is Transform of OpenGL style matrix (Row major)
void MultiplyVector(vec inVec, Matrix4d inMat, vec &outVec)
{
    outVec = GetRotationFromAffine(inMat) * inVec;
}

// Note: Input matrix is Transform of OpenGL style matrix (Row major)
void MultiplyNormal(vec inNor, const Matrix4d& inMat, vec &outNor)
{
    Matrix3d RotM;
    RotM = GetRotationFromAffine(inMat);
    if(RotM.determinant() == 0)
        printf("Inverse Matrix Error \n");
    outNor = RotM.inverse().transpose()*inNor;
    outNor.normalize();
}

/// rotation axis Z->Y->X , for fixed axis (world coordinate system)
Matrix4d GetRotationMatrix(Vector3d eulerAngles)
{
    Matrix3d RotM;
    RotM = AngleAxisd(eulerAngles[0], Vector3d::UnitX()) *
           AngleAxisd(eulerAngles[1], Vector3d::UnitY()) *
           AngleAxisd(eulerAngles[2], Vector3d::UnitZ());
    Matrix4d AffineM = GetAffine(RotM);
    return AffineM;
}

/// R = cos(x)*I + (1-cos(x))* (r * r^T) + sinx * ((0,-rz,ry);(rz,0,-rx);(-ry,rx,0))
/// Rodrigues
Matrix4d GetRotationMatrix(double rotAngle, Vector3d rotAxis)
{
    Matrix3d RotM;
    Matrix3d tempM;

    rotAxis.normalize();
    tempM << 0, -rotAxis.z(), rotAxis.y(),
            rotAxis.z(), 0, -rotAxis.x(),
            -rotAxis.y(), rotAxis.x(), 0;
    RotM = cos(rotAngle)*Matrix3d::Identity() + (1-cos(rotAngle))*(rotAxis*(rotAxis.transpose())) + sin(rotAngle)*tempM;

    Matrix4d AffineM;
    AffineM = GetAffine(RotM);
    return AffineM;
}

Matrix4d GetTranslateMatrix(Vector3d transVec)
{
    Matrix4d TransAffineM;
    TransAffineM.setIdentity();
    TransAffineM(0,3) = transVec(0);
    TransAffineM(1,3) = transVec(1);
    TransAffineM(2,3) = transVec(2);
    return TransAffineM;
}

Matrix3d GetTranslateMatrix3d(Vector3d transVec)
{
    Matrix3d TransAffineM;
    TransAffineM.setIdentity();
    TransAffineM(0,3) = transVec(0);
    TransAffineM(1,3) = transVec(1);
    TransAffineM(2,3) = transVec(2);
    return TransAffineM;
}

Matrix4d GetRotationMatrix(double rotAngle, Vector3d rotAxis, const Vector3d& rotCenter)
{
    return GetTranslateMatrix(rotCenter)*GetRotationMatrix(rotAngle,rotAxis)*GetTranslateMatrix(-rotCenter);
}

Matrix3d GetRotationMatrix3d(double rotAngle, Vector3d rotAxis, const Vector3d& rotCenter)
{
    Matrix4d temp;
    temp = GetRotationMatrix(rotAngle, rotAxis, rotCenter);

    Matrix3d result;
    result(0, 0) = temp(0, 0); result(0, 1) = temp(0, 1); result(0, 2) = temp(0, 2);
    result(1, 0) = temp(1, 0); result(1, 1) = temp(1, 1); result(1, 2) = temp(1, 2);
    result(2, 0) = temp(2, 0); result(2, 1) = temp(2, 1); result(2, 2) = temp(2, 2);

    return result;
}

Matrix4d GetTransformMatrix(Vector3d eulerAngles, Vector3d position)
{
    Matrix4d transRotM;
    transRotM = GetRotationMatrix(eulerAngles);
    transRotM(0,3) = position(0);
    transRotM(1,3) = position(1);
    transRotM(2,3) = position(2);
    return transRotM;
}

Matrix4d GetScaleMatrix(Vector3d scaleVec)
{
    Matrix4d scaleMat;

    scaleMat.setIdentity();
    scaleMat(0, 0) = scaleVec(0);
    scaleMat(1, 1) = scaleVec(1);
    scaleMat(2, 2) = scaleVec(2);

    return scaleMat;
}

Matrix3d GetScaleMatrix3d(Vector3d scaleVec)
{
    Matrix3d scaleMat;

    scaleMat.setIdentity();
    scaleMat(0, 0) = scaleVec(0);
    scaleMat(1, 1) = scaleVec(1);
    scaleMat(2, 2) = scaleVec(2);

    return scaleMat;
}

bool IsPointInsideTriangle(vec point, const Triangle& triangle, bool isPrint)
{
    // Consider the numerical error (double_ERROR value depends on triangle scale)
    //const double double_ERROR = 0.0000001;
    //const double double_ERROR = 0.000001;
    const double double_ERROR = 0.000002; // For Ring with VoxelSize = 0.15

    vec vec0 = triangle.v[2] - triangle.v[0] ;
    vec vec1 = triangle.v[1] - triangle.v[0] ;
    vec vec2 =      point  - triangle.v[0] ;

    // Note: DotResult should include the length of vec2 (when point is very close to triangle.v[0]).
    vec normal = vec0.cross(vec1);
    normal.normalize();
    double dotResult = normal.dot(vec2);

    if ( fabs(dotResult) > FLOAT_ERROR_SMALL )
    {
        if( isPrint )
        {
            printf("Warning: The point is not on the triangle's plane. \n");
            printf("error:  %.8f \n\n", fabs(dotResult));
        }
        return false;
    }

    double dot00 = vec0.dot(vec0) ;
    double dot01 = vec0.dot(vec1) ;
    double dot02 = vec0.dot(vec2) ;
    double dot11 = vec1.dot(vec1) ;
    double dot12 = vec1.dot(vec2) ;

    double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01) ;

    double u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
    //if (u < 0 || u > 1) // if u out of range, return directly
    if ( u < 0-double_ERROR || u > 1+double_ERROR  )
    {
        if( isPrint )
            printf("Warning: u=%.12f is out of range \n", u);
        return false ;
    }

    double v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
    //if (v < 0 || v > 1) // if v out of range, return directly
    if ( v < 0-double_ERROR || v > 1+double_ERROR  )
    {
        if( isPrint )
            printf("Warning: v=%.12f is out of range \n", v);
        return false ;
    }

    if( isPrint )
        printf( "u+v = %.12f \n", u+v);

    return u + v <= 1+double_ERROR ;
}

float GetTriangleArea(Vector3d triPt0, Vector3d triPt1, Vector3d triPt2)
{
    Vector3d triEdge1 = triPt1 - triPt0;
    Vector3d triEdge2 = triPt2 - triPt0;

    Vector3d crossVec = triEdge1.cross(triEdge2);

    return crossVec.norm();
}




