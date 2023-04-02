///////////////////////////////////////////////////////
//HelpFunc.cpp
//
// Additional Functions
//
// Created by Yingjie Cheng on 2020/11/05.
//
////////////////////////////////////////////////////////

#include "HelpFunc.h"

// Color scheme table
Vector3d colorTable[16] = {
        Vector3d(0.6, 0.6, 0.9),   //  1: Orange
        Vector3d(0.9, 0.6, 0.4),   //  2: Light Blue
        Vector3d(0.7, 0.3, 0.9),   //  3: Green
        Vector3d(0.4, 0.4, 0.9),   //  4: Blue
        Vector3d(0.8, 0.8, 0.8),   //  5: Light Gray

        Vector3d(0.9, 0.5, 0.5),   //  6: Red
        Vector3d(0.9, 0.9, 0.5),   //  7: Yellow
        Vector3d(0.4, 0.9, 0.4),   //  8: Purple
        Vector3d(0.4, 0.9, 0.9),   //  9: Cyan
        Vector3d(0.9, 0.3, 0.6),   // 10: Pink
        Vector3d(0.6, 0.4, 0.3),   // 11: Brown
        Vector3d(0.9, 0.6, 0.5),   // 12: LightSalmon

        Vector3d(0.5, 0.2, 0.5),   // 13: Dark Purple
        Vector3d(0.4, 0.8, 0.7),   // 14: Dark Cyan
        Vector3d(0.3, 0.4, 0.7),   // 15: Dark Blue
        Vector3d(0.6, 0.6, 0.3),   // 16: Dark Yellow
};




///======================================================================================//
///                                Color interpolation
///=====================================================================================//

void RedColorInterpolation(VectorXd planarityList, MatrixXd & colorMap)
{
    colorMap.resize(planarityList.size(), 3);

    for (int i = 0; i < planarityList.size(); ++i)
    {
        if (planarityList(i) <= 0.0005)
        {
            colorMap(i, 0) = 1;
            colorMap(i, 1) = 1;
            colorMap(i, 2) = 1;
        }
        else if (planarityList(i) >= 0.01)
        {
            colorMap(i, 1) = 0;
            colorMap(i, 2) = 0;
            colorMap(i, 0) = 0.9;
        }
        else
        {
            colorMap(i, 1) = 0;
            colorMap(i, 2) = 0;
            colorMap(i, 0) = 0.9 * (1.0 - ((planarityList(i) - 0.0005) / (0.01 - 0.0005)));
        }
    }
}




///======================================================================================//
///                                  Debug
///=====================================================================================//

void PrintVerList(vector<Vector3d> & verList, string str)
{
    cout << "----------------------------------" << endl;
    cout << str << endl;
    for (int i = 0; i < verList.size(); ++i)
    {
        cout << verList[i][0] << " " << verList[i][1] << " " << verList[i][2] << endl;
    }
    cout << "----------------------------------" << endl << endl;
}




///======================================================================================//
///                                 String Operations
///======================================================================================//

void GetFolderPath(string filePath, string &folderPath)
{
    // Subtract Puz file folder path
    size_t foundFolder = filePath.find_last_of('/'); // Note: this applies for Mac only
    folderPath = filePath.substr(0, foundFolder + 1);
}

void GetFolderPath(const char filePath[], char folderPath[])
{
    string sFilePath = string(filePath);

    // Subtract Puz file folder path
    size_t foundFolder = sFilePath.find_last_of('\\');
    string sFolderPath = sFilePath.substr(0, foundFolder + 1);
    sprintf(folderPath, "%s", sFolderPath.c_str());

}

string GetFileType(const char filePath[])
{
    string sFilePath = string(filePath);

    // Subtract file extension type
    size_t stringLen = sFilePath.size();
    size_t foundType = sFilePath.find_last_of('.');
    string sFileType = sFilePath.substr(foundType + 1, stringLen - (foundType + 1));

    return sFileType;
}

void GetFileName(string filePath, string & fileName)
{
    // Subtract Puz file folder path
    size_t foundFolder = filePath.find_last_of('/'); // Note: this applies for Mac only

    fileName = filePath.substr(foundFolder + 1, filePath.size() - 1);
}




///======================================================================================//
///                                    Mesh Volume
///======================================================================================//

Vector3d ComputePolygonCenter(vector<Vector3d> & verList)
{
    double xSum = 0;
    double ySum = 0;
    double zSum = 0;

    for (int i = 0; i < verList.size(); ++i)
    {
        xSum += verList[i][0];
        ySum += verList[i][1];
        zSum += verList[i][2];
    }

    Vector3d currCentroid;
    currCentroid(0) = xSum / double(verList.size() );
    currCentroid(1) = ySum / double(verList.size() );
    currCentroid(2) = zSum / double(verList.size() );

    return currCentroid;
}

void MoveToCenter(MatrixXd & verMat)
{
    // Move the model to the center (0, 0, 0)
    Vector3d minPoint = verMat.colwise().minCoeff();
    Vector3d maxPoint = verMat.colwise().maxCoeff();
    Vector3d currCenter = (maxPoint + minPoint) / 2;
    Vector3d offset = Vector3d(0,0,0) - currCenter;

    for (int i = 0; i < verMat.rows(); ++i)
    {
        verMat(i, 0) += offset(0);
        verMat(i, 1) += offset(1);
        verMat(i, 2) += offset(2);
    }
}

void MoveToCenter_ShapeOp(ShapeOp::Matrix3X & verMat)
{
    // Move the model to the center (0, 0, 0)
    Vector3d minPoint = verMat.rowwise().minCoeff();
    Vector3d maxPoint = verMat.rowwise().maxCoeff();
    Vector3d currCenter = (maxPoint + minPoint) / 2;
    Vector3d offset = Vector3d(0,0,0) - currCenter;

    for (int i = 0; i < verMat.cols(); ++i)
    {
        verMat(0, i) += offset(0);
        verMat(1, i) += offset(1);
        verMat(2, i) += offset(2);
    }
}

void MoveToSameCenter_ShapeOp(ShapeOp::Matrix3X & referP, ShapeOp::Matrix3X & currP)
{
    // Compute referCenter
    Vector3d minPoint_refer = referP.rowwise().minCoeff();
    Vector3d maxPoint_refer = referP.rowwise().maxCoeff();
    Vector3d referCenter = (minPoint_refer + maxPoint_refer) / 2.0f;

    // Compute currCenter
    Vector3d minPoint_curr = currP.rowwise().minCoeff();
    Vector3d maxPoint_curr = currP.rowwise().maxCoeff();
    Vector3d currCenter = (minPoint_curr + maxPoint_curr) / 2.0f;

    // Move the currP to the referCenter
    Vector3d offset = referCenter - currCenter;

    for (int i = 0; i < currP.cols(); ++i)
    {
        currP(0, i) += offset(0);
        currP(1, i) += offset(1);
        currP(2, i) += offset(2);
    }
}

bool IsVerListEqual(vector<Vector3d> & verList_1, vector<Vector3d> & verList_2)
{
    for (int i = 0; i < verList_1.size(); ++i)
    {
        if (find(verList_2.begin(), verList_2.end(), verList_1[i]) == verList_2.end())
            return false;
    }

    return true;
}

int ComputeMatchingNum(vector<int> v1, vector<int> v2)
{
    /// v1, v2 should be share the same size
    if (v1.size() != v2.size())
        return 0;

    int count = 0;

    for (int i = 0; i < v1.size(); ++i)
    {
        if (v1[i] == v2[i])
            count++;
    }

    return count;
}

int IsCurrListInExistingList(vector< vector<int> > existingList, vector<int> currList)
{
    for (int i = 0; i < currList.size(); ++i)
    {
        if (std::find(existingList.begin(), existingList.end(), currList) != existingList.end())
            return (std::find(existingList.begin(), existingList.end(), currList) - existingList.begin());

        currList.push_back(currList[0]);
        currList.erase(currList.begin());
    }

    return -1;
}

double ComputeAverageLength(const vector<Vector3d> & verList)
{
    double totalLength = 0;

    for (int i = 1; i < verList.size(); ++i)
    {
        totalLength += (verList[i] - verList[i - 1]).norm();
    }
    totalLength += (verList[0] - verList[verList.size() - 1]).norm();

    return totalLength / double(verList.size());
}

vector<Vector3d> CombineTwoVector(const vector<Vector3d> & upperList, const vector<Vector3d> & lowerList)
{
    vector<Vector3d> result;

    for (int i = 0; i < upperList.size(); ++i)
    {
        result.push_back(upperList[i]);
    }

    for (int j = 0; j < lowerList.size(); ++j)
    {
        result.push_back(lowerList[j]);
    }

    return result;
}

void SplitTwoVector(const vector<Vector3d> & combinedVector, vector<Vector3d> & upperList, vector<Vector3d> & lowerList)
{
    upperList.clear();
    lowerList.clear();

    int count = combinedVector.size();
    int half = count / 2;

    for (int i = 0; i < half; ++i)
    {
        upperList.push_back(combinedVector[i]);
        lowerList.push_back(combinedVector[i + half]);
    }
}

Vector3d PolyMeshPoint2EigenVector3d(const PolyMesh::Point & p1)
{
    return Vector3d(p1[0], p1[1], p1[2]);
}

vector<Vector3d> PolyMeshPointList2EigenVector3dList(const vector<PolyMesh::Point> & p1_list)
{
    vector<Vector3d> result;

    for (int i = 0; i < p1_list.size(); ++i)
    {
        result.push_back(Vector3d(p1_list[i][0], p1_list[i][1], p1_list[i][2]));
    }

    return result;
}

PolyMesh::Point EigenVector3d2PolyMeshPoint(const Vector3d & p1)
{
    return PolyMesh::Point(p1[0], p1[1], p1[2]);
}

void Matrix2VectorList(const MatrixXd & V, vector<Vector3d> & verList)
{
    for (int i = 0; i < V.rows(); ++i)
    {
        verList.push_back(Vector3d(V(i, 0), V(i, 1), V(i, 2)));
    }
}

bool IsTwoBlockMatrixEqual(const MatrixXd & V_1, const MatrixXd & V_2)
{
    if (V_1.rows() != V_2.rows())
        return false;

    vector<Vector3d> verList_1, verList_2;

    Matrix2VectorList(V_1, verList_1);
    Matrix2VectorList(V_2, verList_2);

    for (int i = 0; i < verList_2.size(); ++i)
    {
        if (std::find(verList_1.begin(), verList_1.end(), verList_2[i]) == verList_1.end())
            return false;
    }

    return true;
}

vector<Vector3d> ComputeMiddlePlane(const vector<Vector3d> & verList_1, const vector<Vector3d> & verList_2)
{
    vector<Vector3d> result;

    for (int i = 0; i < verList_1.size(); ++i)
    {
        result.push_back((verList_1[i] + verList_2[i]) / 2.0f);
    }

    return result;
}

vector<Vector3d> CombineTwoBlockVerLists(const vector<Vector3d> & verList_1, const vector<Vector3d> & verList_2)
{
    vector<Vector3d> result;

    for (int i = 0; i < verList_1.size(); ++i)
    {
        result.push_back(verList_1[i]);
        result.push_back(verList_2[i]);
    }

    return result;
}

vector<Vector3d> CombineTwoBlockVerLists_clockwise(const vector<Vector3d> & verList_1, const vector<Vector3d> & verList_2)
{
    vector<Vector3d> result;

    for (int i = 0; i < verList_1.size(); ++i)
    {
        result.push_back(verList_1[i]);
    }

    for (int i = verList_1.size() - 1; i >= 0; --i)
    {
        result.push_back(verList_2[i]);
    }

    return result;
}

Vector3d ComputePolygonCentroid(const vector<Vector3d> & verList)
{
    Vector3d result(0, 0, 0);

    for (int i = 0; i < verList.size(); ++i)
    {
        result += verList[i];
    }

    return (result / double(verList.size()));
}

Vector3d ComputePolygonCentroid(const MatrixXd & p)
{
    Vector3d result(0, 0, 0);

    for (int i = 0; i < p.cols(); ++i)
    {
        result += p.col(i);
    }

    return (result / double(p.cols()));
}

Vector3d ComputeMatrixCentroid(const MatrixXd & p)
{
    RowVector3d result(0.0, 0.0, 0.0);

    for (int i = 0; i < p.rows(); ++i)
    {
        result += p.row(i);
    }

    result = (result / double(p.rows()));

    return Vector3d(result[0], result[1], result[2]);
}

double AngleBetweenVectors(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
    double angle = 0.0;

    angle = std::atan2(a.cross(b).norm(), a.dot(b));

    return angle;
}




///==============================================================================//
///                             Mesh Volume Calculation
///==============================================================================//

double ComputeSignedVolumeOfTriangle(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3)
{
    double v321 = p3[0]*p2[1]*p1[2];
    double v231 = p2[0]*p3[1]*p1[2];
    double v312 = p3[0]*p1[1]*p2[2];
    double v132 = p1[0]*p3[1]*p2[2];
    double v213 = p2[0]*p1[1]*p3[2];
    double v123 = p1[0]*p2[1]*p3[2];
    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

double ComputeVolumeOfMesh(const MatrixXd & V_intersection, const MatrixXi & F_intersection)
{
    double totalVolume = 0;

    for (int i = 0; i < F_intersection.rows(); ++i)
    {
        Vector3d p1(V_intersection(F_intersection(i, 0), 0), V_intersection(F_intersection(i, 0), 1), V_intersection(F_intersection(i, 0), 2));
        Vector3d p2(V_intersection(F_intersection(i, 1), 0), V_intersection(F_intersection(i, 1), 1), V_intersection(F_intersection(i, 1), 2));
        Vector3d p3(V_intersection(F_intersection(i, 2), 0), V_intersection(F_intersection(i, 2), 1), V_intersection(F_intersection(i, 2), 2));

        totalVolume += ComputeSignedVolumeOfTriangle(p1, p2, p3);
    }

    return fabs(totalVolume);
}



///==============================================================================//
///                              Utility solver
///==============================================================================//

double SolveEqua(double a, double b, double c)
{
    return (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
}

double SolveEqua2(double a, double b, double c)
{
    return (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
}

double SolvePointToCircleK1(double Pointx, double Pointy, double Cx, double Cy, double r)
{
    Cx -= Pointx;
    Cy -= Pointy;
    return SolveEqua(r*r - Cx*Cx, 2 * Cx*Cy, r*r - Cy*Cy);
}

double SolvePointToCircleK2(double Pointx, double Pointy, double Cx, double Cy, double r)
{
    Cx -= Pointx;
    Cy -= Pointy;
    return SolveEqua2(r*r - Cx*Cx, 2 * Cx*Cy, r*r - Cy*Cy);
}

void UpIntersectionOfCircles(double ans[2], double O1_x, double O1_y, double O1_R, double O2_x, double O2_y, double O2_R)
{
    double x1 = O1_x - O2_x;
    double y1 = O1_y - O2_y;
    double a = 4 * y1*y1 + 4 * x1*x1;
    double b = -4 * y1*(O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1);
    double c = pow((O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1), 2) - pow(2 * O2_R*x1, 2);
    //printf("%f   %f  %f\n", a, b, c);

    ans[1] = (-b + sqrt(b*b - 4 * a*c)) / (2.0 * a);
    ans[0] = (O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1 - 2.0 * ans[1] * y1) / (2.0 * x1) + O2_x;
    ans[1] += +O2_y;
}

void DownIntersectionOfCircles(double ans[2], double O1_x, double O1_y, double O1_R, double O2_x, double O2_y, double O2_R)
{
    double x1 = O1_x - O2_x;
    double y1 = O1_y - O2_y;
    double a = 4 * y1*y1 + 4 * x1*x1;
    double b = -4 * y1*(O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1);
    double c = pow((O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1), 2) - pow(2 * O2_R*x1, 2);
    //printf("%f   %f  %f\n", a, b, c);

    ans[1] = (-b - sqrt(b*b - 4 * a*c)) / (2.0 * a);
    ans[0] = (O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1 - 2.0 * ans[1] * y1) / (2.0 * x1) + O2_x;
    ans[1] += +O2_y;
}

bool isIntersectOfCircles(double O1_x, double O1_y, double O1_R, double O2_x, double O2_y, double O2_R)
{
    double x1 = O1_x - O2_x;
    double y1 = O1_y - O2_y;
    double a = 4 * y1*y1 + 4 * x1*x1;
    double b = -4 * y1*(O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1);
    double c = pow((O2_R*O2_R - O1_R*O1_R + x1*x1 + y1*y1), 2) - pow(2 * O2_R*x1, 2);

    if (b*b - 4 * a*c < 0)    return false;
    else                      return true;
}




///==============================================================================//
///                         PolyMesh data to libigl rendering data
///==============================================================================//

// Only for triangle mesh
void Matrix2PolyMesh(PolyMesh & polyMesh, MatrixXd V, MatrixXi F)
{
    std::vector<PolyMesh::VertexHandle>  face_vhandles;

    int verNum = V.rows();
    PolyMesh::VertexHandle vHandle[verNum];

    // Add vertices
    for (int i = 0; i < V.rows(); ++i)
    {
        vHandle[i] = polyMesh.add_vertex(PolyMesh::Point(V(i, 0),V(i, 1),V(i, 2)));
    }

    // Add faces
    for (int i = 0; i < F.rows(); ++i)
    {
        face_vhandles.clear();
        for (int j = 0; j < 3; ++j)
        {
            face_vhandles.push_back(vHandle[F(i, j)]);
        }

        polyMesh.add_face(face_vhandles);
    }
}

void PolyMesh2Matrix(PolyMesh polyMesh, MatrixXd & V, MatrixXi & F)
{
    polyMesh.triangulate();

    V.resize(polyMesh.n_vertices(), 3);

    int i = 0;
    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        V(i, 0) = polyMesh.point(*v_it)[0];
        V(i, 1) = polyMesh.point(*v_it)[1];
        V(i, 2) = polyMesh.point(*v_it)[2];
        ++i;
    }

    F.resize(polyMesh.n_faces(), 3);

    i = 0;
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        int j = 0;
        for (fv_it=polyMesh.fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            F(i, j) = fv_it->idx();
            ++j;
        }
        ++i;
    }

//    cout << "done. " << endl;
}

void PolyMesh2Edge(PolyMesh polyMesh, MatrixXd & p1_mat, MatrixXd & p2_mat)
{
    p1_mat.resize(polyMesh.n_edges(), 3);
    p2_mat.resize(polyMesh.n_edges(), 3);

    int i = 0;
    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it,0)));
        const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it,0)));

        p1_mat(i, 0) = to[0];
        p1_mat(i, 1) = to[1];
        p1_mat(i, 2) = to[2];

        p2_mat(i, 0) = from[0];
        p2_mat(i, 1) = from[1];
        p2_mat(i, 2) = from[2];

        ++i;
    }
}

void PolyMesh2Planarity(PolyMesh polyMesh, VectorXd & planarityList)
{
    planarityList.resize(polyMesh.n_faces());

    OpenMesh::FPropHandleT< double > currPlanarity;
    polyMesh.get_property_handle(currPlanarity, "facePlanarity");

    int i = 0;
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
//        cout << polyMesh.property( currPlanarity , *f_it ) << endl;
        planarityList(i) = polyMesh.property( currPlanarity , *f_it );
        ++i;
    }
}

void PolyMesh2ClusteringLabel(PolyMesh polyMesh, VectorXd & labelList)
{
    labelList.resize(polyMesh.n_faces());

    OpenMesh::FPropHandleT< int > currLabel;
    polyMesh.get_property_handle(currLabel, "clusteringLabel");

    int i = 0;
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        labelList(i) = polyMesh.property( currLabel , *f_it );
        ++i;
    }
}

void PolyMesh2BlockClusterLabel(PolyMesh polyMesh, VectorXd & labelList)
{
    labelList.resize(polyMesh.n_faces());

    OpenMesh::FPropHandleT< int > currLabel;
    polyMesh.get_property_handle(currLabel, "blockClusteringLabel");

    int i = 0;
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        labelList(i) = polyMesh.property( currLabel , *f_it );
//        cout << "GetCurrBlockLabel: " << polyMesh.property( currLabel , *f_it ) << endl;
        ++i;
    }
}

void PolyMesh2EdgeClusteringLabel(PolyMesh polyMesh, VectorXd & labelList)
{
    labelList.resize(polyMesh.n_edges());

    OpenMesh::EPropHandleT< int > currLabel;
    polyMesh.get_property_handle(currLabel, "edgeClusteringLabel");

    int i = 0;
    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
    {
//        cout << polyMesh.property( currLabel , *f_it ) << endl;
        labelList(i) = polyMesh.property( currLabel , *e_it );
        ++i;
    }
}

void PolyMesh2EdgeNormal(PolyMesh polyMesh, MatrixXd & p1_mat, MatrixXd & p2_mat)
{
    p1_mat.resize(polyMesh.n_edges(), 3);
    p2_mat.resize(polyMesh.n_edges(), 3);

    int i = 0;
    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it,0)));
        const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it,0)));

        auto center = (to + from) / 2;
        Vector3d centerVec(center[0], center[1], center[2]);

        OpenMesh::EPropHandleT< Vector3d > currNormal;
        polyMesh.get_property_handle(currNormal, "normal");

        p1_mat(i, 0) = centerVec[0];
        p1_mat(i, 1) = centerVec[1];
        p1_mat(i, 2) = centerVec[2];

        p2_mat(i, 0) = (centerVec[0] + polyMesh.property(currNormal, *e_it)[0] * 0.1);
        p2_mat(i, 1) = (centerVec[1] + polyMesh.property(currNormal, *e_it)[1] * 0.1);
        p2_mat(i, 2) = (centerVec[2] + polyMesh.property(currNormal, *e_it)[2] * 0.1);

//        cout << "to: " << p1_mat(i, 0) << p1_mat(i, 1) << p1_mat(i, 2) << endl;
//        cout << "from: " << p2_mat(i, 0) << p2_mat(i, 1) << p2_mat(i, 2) << endl;

        ++i;
    }
}

void PolyMesh2FabTestLabel(PolyMesh polyMesh, MatrixXd & p1_mat, MatrixXd & p2_mat, VectorXd & fabLabelList)
{
    // Get problemVerPair to mesh
    OpenMesh::MPropHandleT< vector<std::tuple<Vector3d, Vector3d, int>> > problemVerPair;
    polyMesh.get_property_handle(problemVerPair, "problemVerPair");

    fabLabelList.resize(polyMesh.property(problemVerPair).size());
    p1_mat.resize(polyMesh.property(problemVerPair).size(), 3);
    p2_mat.resize(polyMesh.property(problemVerPair).size(), 3);

    for (int i = 0; i < polyMesh.property(problemVerPair).size(); ++i)
    {
        auto p1 = std::get<0>(polyMesh.property(problemVerPair)[i]);
        auto p2 = std::get<1>(polyMesh.property(problemVerPair)[i]);
        auto currLabel = std::get<2>(polyMesh.property(problemVerPair)[i]);

        p1_mat(i, 0) = p1[0];
        p1_mat(i, 1) = p1[1];
        p1_mat(i, 2) = p1[2];

        p2_mat(i, 0) = p2[0];
        p2_mat(i, 1) = p2[1];
        p2_mat(i, 2) = p2[2];

        fabLabelList(i) = currLabel;
    }
}




///==============================================================================//
///                      PolyMesh data and ShapeOp matrix data transfer
///==============================================================================//

void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat, vector< vector<int> > & faceVerList)
{
    /// Delete the duplicate vertices
    vector<Vector3d> verList;

    int i = 0;
    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        Vector3d currVer;
        currVer[0] = polyMesh.point(v_it)[0];
        currVer[1] = polyMesh.point(v_it)[1];
        currVer[2] = polyMesh.point(v_it)[2];
        ++i;

        if (std::find(verList.begin(), verList.end(), currVer) == verList.end())
        {
            verList.push_back(currVer);
        }
    }

    verMat.resize(3, verList.size());

    for (int j = 0; j < verList.size(); ++j)
    {
        verMat(0, j) = verList[j][0];
        verMat(1, j) = verList[j][1];
        verMat(2, j) = verList[j][2];
    }

    /// Find the index for each vertex of a face
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector<int> currFaceVerList;

        for (fv_it=polyMesh.fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = polyMesh.point(fv_it)[0];
            currVer[1] = polyMesh.point(fv_it)[1];
            currVer[2] = polyMesh.point(fv_it)[2];

            int currVerIndex = std::find(verList.begin(), verList.end(), currVer) - verList.begin();

            currFaceVerList.push_back(currVerIndex);
        }

        faceVerList.push_back(currFaceVerList);
    }
}

void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat, vector< vector<int> > & faceVerList, vector< vector<int> > & edgeVerList, vector<double> & targetLengthList, vector< vector<int> > & angleVerList, vector<double> & targetScaleList)
{
    /// Delete the duplicate vertices
    vector<Vector3d> verList;

    int i = 0;
    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        Vector3d currVer;
        currVer[0] = polyMesh.point(v_it)[0];
        currVer[1] = polyMesh.point(v_it)[1];
        currVer[2] = polyMesh.point(v_it)[2];
        ++i;

        if (std::find(verList.begin(), verList.end(), currVer) == verList.end())
        {
            verList.push_back(currVer);
        }
    }

    verMat.resize(3, verList.size());

    for (int j = 0; j < verList.size(); ++j)
    {
        verMat(0, j) = verList[j][0];
        verMat(1, j) = verList[j][1];
        verMat(2, j) = verList[j][2];
    }

    /// Find the index for each vertex of a face
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector<int> currFaceVerList;

        for (fv_it=polyMesh.fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = polyMesh.point(fv_it)[0];
            currVer[1] = polyMesh.point(fv_it)[1];
            currVer[2] = polyMesh.point(fv_it)[2];

            int currVerIndex = std::find(verList.begin(), verList.end(), currVer) - verList.begin();

            currFaceVerList.push_back(currVerIndex);
        }

        faceVerList.push_back(currFaceVerList);
    }

//    dbg(faceVerList);

    /// Find the index for each vertex of an edge
    OpenMesh::EPropHandleT< double > edgeCentroid;
    polyMesh.get_property_handle(edgeCentroid, "edgeClusterCentroid");

    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it,0)));
        const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it,0)));

        Vector3d v1(to[0], to[1], to[2]);
        Vector3d v2(from[0], from[1], from[2]);

        vector<int> currEdgeVerList;

        int currVerIndex_1 = std::find(verList.begin(), verList.end(), v1) - verList.begin();
        int currVerIndex_2 = std::find(verList.begin(), verList.end(), v2) - verList.begin();

        currEdgeVerList.push_back(currVerIndex_1);
        currEdgeVerList.push_back(currVerIndex_2);

        edgeVerList.push_back(currEdgeVerList);

        targetLengthList.push_back(polyMesh.property(edgeCentroid, *e_it));
    }

//    dbg(targetLengthList);

    /// Find the index for each vertex of a dihedral angle
    for (int m = 0; m < edgeVerList.size(); ++m)
    {
        int id1 = edgeVerList[m][0];
        int id2 = edgeVerList[m][1];

        vector<vector<int>> currFaces;

        for (int k = 0; k < faceVerList.size(); ++k)
        {
            if (find(faceVerList[k].begin(), faceVerList[k].end(), id1) != faceVerList[k].end() and find(faceVerList[k].begin(), faceVerList[k].end(), id2) != faceVerList[k].end())
            {
                currFaces.push_back(faceVerList[k]);
            }
        }

//        cout << "currFaces size: " << currFaces.size() << endl;

        if (currFaces.size() != 2)
            continue;

        vector<int> currAngleVerList;
        currAngleVerList.push_back(id1);
        currAngleVerList.push_back(id2);

        bool flag = false;

        for (int j = 0; j < currFaces[0].size(); ++j)
        {
            if (find(currAngleVerList.begin(), currAngleVerList.end(), currFaces[0][j]) != currAngleVerList.end())
            {
                continue;
            }

            currAngleVerList.push_back(currFaces[0][j]);

            for (int f = 0; f < currFaces[1].size(); ++f)
            {
                if (find(currAngleVerList.begin(), currAngleVerList.end(), currFaces[1][f]) != currAngleVerList.end())
                {
                    continue;
                }

                currAngleVerList.push_back(currFaces[1][f]);

//                cout << "currAngleVerList: ";
//                for (int t = 0; t < currAngleVerList.size(); ++t)
//                {
//                    cout << currAngleVerList[t] << " ";
//                }
//                cout << endl;

                angleVerList.push_back(currAngleVerList);

                flag = true;
                break;

//                currAngleVerList.erase(currAngleVerList.begin() + currAngleVerList.size() - 1);

//                cout << "Erased." << endl;
            }

            if (flag)
                break;

//            currAngleVerList.erase(currAngleVerList.end());
        }
    }

    /// Find target scale for each angle
    OpenMesh::EPropHandleT< double > angleCentroid;
    polyMesh.get_property_handle(angleCentroid, "angleClusterCentroid");

    OpenMesh::EPropHandleT< double > dihedralAngle;
    polyMesh.get_property_handle(dihedralAngle, "dihedralAngle");

//    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
//    {
//        dbg(polyMesh.property(dihedralAngle, *e_it));
//        dbg(polyMesh.property(angleCentroid, *e_it));
//        dbg(polyMesh.property(dihedralAngle, *e_it) / polyMesh.property(angleCentroid, *e_it));
//    }

//    dbg(angleVerList);

    for (int k = 0; k < angleVerList.size(); ++k)
    {
        for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
        {
            const PolyMesh::Point to = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it, 0)));
            const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it, 0)));

            Vector3d v1(to[0], to[1], to[2]);
            Vector3d v2(from[0], from[1], from[2]);

            if ((verList[angleVerList[k][0]] == v1 and verList[angleVerList[k][1]] == v2) or (verList[angleVerList[k][1]] == v1 and verList[angleVerList[k][0]] == v2))
            {
                targetScaleList.push_back(polyMesh.property(dihedralAngle, *e_it) / polyMesh.property(angleCentroid, *e_it));
                break;
            }
        }
    }

//    dbg(targetScaleList);
}

void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat,
                            vector< vector<int> > & faceVerList,
                            vector< vector<int> > & edgeVerList,vector<double> & targetLengthList,
                            vector< vector<int> > & angleVerList, vector<double> & targetScaleList,
                            vector< vector<int> > & diagonalVerList, vector<double> & targetDiagonalList)
{
    /// Delete the duplicate vertices
    vector<Vector3d> verList;

    int i = 0;
    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        Vector3d currVer;
        currVer[0] = polyMesh.point(v_it)[0];
        currVer[1] = polyMesh.point(v_it)[1];
        currVer[2] = polyMesh.point(v_it)[2];
        ++i;

        if (std::find(verList.begin(), verList.end(), currVer) == verList.end())
        {
            verList.push_back(currVer);
        }
    }

    verMat.resize(3, verList.size());

    for (int j = 0; j < verList.size(); ++j)
    {
        verMat(0, j) = verList[j][0];
        verMat(1, j) = verList[j][1];
        verMat(2, j) = verList[j][2];
    }

    /// Find the index for each vertex of a face
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector<int> currFaceVerList;

        for (fv_it=polyMesh.fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = polyMesh.point(fv_it)[0];
            currVer[1] = polyMesh.point(fv_it)[1];
            currVer[2] = polyMesh.point(fv_it)[2];

            int currVerIndex = std::find(verList.begin(), verList.end(), currVer) - verList.begin();

            currFaceVerList.push_back(currVerIndex);
        }

        faceVerList.push_back(currFaceVerList);
    }

    /// Find the index for each vertex of an edge
    OpenMesh::EPropHandleT< double > edgeCentroid;
    polyMesh.get_property_handle(edgeCentroid, "edgeClusterCentroid");

    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it,0)));
        const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it,0)));

        Vector3d v1(to[0], to[1], to[2]);
        Vector3d v2(from[0], from[1], from[2]);

        vector<int> currEdgeVerList;

        int currVerIndex_1 = std::find(verList.begin(), verList.end(), v1) - verList.begin();
        int currVerIndex_2 = std::find(verList.begin(), verList.end(), v2) - verList.begin();

        currEdgeVerList.push_back(currVerIndex_1);
        currEdgeVerList.push_back(currVerIndex_2);

        edgeVerList.push_back(currEdgeVerList);

        targetLengthList.push_back(polyMesh.property(edgeCentroid, *e_it));
    }

    /// Find the index for each vertex of a dihedral angle
    for (int m = 0; m < edgeVerList.size(); ++m)
    {
        int id1 = edgeVerList[m][0];
        int id2 = edgeVerList[m][1];

        vector<vector<int>> currFaces;

        for (int k = 0; k < faceVerList.size(); ++k)
        {
            if (find(faceVerList[k].begin(), faceVerList[k].end(), id1) != faceVerList[k].end() and find(faceVerList[k].begin(), faceVerList[k].end(), id2) != faceVerList[k].end())
            {
                currFaces.push_back(faceVerList[k]);
            }
        }

//        cout << "currFaces size: " << currFaces.size() << endl;

        if (currFaces.size() != 2)
            continue;

        vector<int> currAngleVerList;
        currAngleVerList.push_back(id1);
        currAngleVerList.push_back(id2);

        for (int j = 0; j < currFaces[0].size(); ++j)
        {
            if (find(currAngleVerList.begin(), currAngleVerList.end(), currFaces[0][j]) != currAngleVerList.end())
            {
                continue;
            }

            currAngleVerList.push_back(currFaces[0][j]);

            for (int f = 0; f < currFaces[1].size(); ++f)
            {
                if (find(currAngleVerList.begin(), currAngleVerList.end(), currFaces[1][f]) != currAngleVerList.end())
                {
                    continue;
                }
                currAngleVerList.push_back(currFaces[1][f]);

                angleVerList.push_back(currAngleVerList);

                currAngleVerList.pop_back();
            }

            currAngleVerList.pop_back();
        }
    }

    /// Find target scale for each angle
    OpenMesh::EPropHandleT< double > angleCentroid;
    polyMesh.get_property_handle(angleCentroid, "angleClusterCentroid");

    OpenMesh::EPropHandleT< double > dihedralAngle;
    polyMesh.get_property_handle(dihedralAngle, "dihedralAngle");

    for (int k = 0; k < angleVerList.size(); ++k)
    {
        for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
        {
            const PolyMesh::Point to = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it, 0)));
            const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it, 0)));

            Vector3d v1(to[0], to[1], to[2]);
            Vector3d v2(from[0], from[1], from[2]);

            if ((verList[angleVerList[k][0]] == v1 and verList[angleVerList[k][1]] == v2) or (verList[angleVerList[k][1]] == v1 and verList[angleVerList[k][0]] == v2))
            {
                targetScaleList.push_back(polyMesh.property(dihedralAngle, *e_it) / polyMesh.property(angleCentroid, *e_it));
                break;
            }
        }
    }

    /// Find the index of each vertex of each diagonal and targetDiagonalList
    // Get diagonalList to each face
    OpenMesh::FPropHandleT< vector<std::tuple<Vector3d, Vector3d, double>> > diagonalList;
    polyMesh.get_property_handle(diagonalList, "diagonalList");

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    polyMesh.get_property_handle(clusteringLabel, "clusteringLabel");

    // Get diagonalClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > diagonalClusterCentroidList;
    polyMesh.get_property_handle(diagonalClusterCentroidList, "diagonalClusterCentroidList");

    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        auto currTupleList = polyMesh.property(diagonalList, *f_it);

        int currLabel = polyMesh.property(clusteringLabel, *f_it);

        for (int k = 0; k < currTupleList.size(); ++k)
        {
            vector<int> currVerList;
            double currTarget;

            Vector3d p1 = std::get<0>(currTupleList[k]);
            Vector3d p2 = std::get<1>(currTupleList[k]);

            int p1_id = find(verList.begin(), verList.end(), p1) - verList.begin();
            int p2_id = find(verList.begin(), verList.end(), p2) - verList.begin();

            currVerList.push_back(p1_id);
            currVerList.push_back(p2_id);

            currTarget = polyMesh.property(diagonalClusterCentroidList)[currLabel][k];

            diagonalVerList.push_back(currVerList);
            targetDiagonalList.push_back(currTarget);
        }
    }
}

void PolyMesh2ShapeOpMatrix(PolyMesh polyMesh, Matrix3Xd & verMat,
                            vector< vector<int> > & faceVerList,
                            vector< vector<int> > & edgeVerList,vector<double> & targetLengthList,
                            vector< vector<int> > & angleVerList, vector<double> & targetScaleList,
                            vector< vector<int> > & diagonalVerList, vector<double> & targetDiagonalList,
                            int & worstFabVer, vector< vector<int> > & worstPolygonEdgeList, vector< vector<int> > & worstDiagonal,
                            vector<int> & worstPlanarityVerList, vector< vector<int> > & worstDihedralAngleVerList)
{
    // Get worstFabVerHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::VertexHandle > worstFabVerHandle;
    polyMesh.get_property_handle(worstFabVerHandle, "worstFabVerHandle");

    // Get worstPolygonSimiFaceHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::FaceHandle > worstPolygonSimiFaceHandle;
    polyMesh.get_property_handle(worstPolygonSimiFaceHandle, "worstPolygonSimiFaceHandle");

    // Get worstPlanarFaceHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::FaceHandle > worstPlanarFaceHandle;
    polyMesh.get_property_handle(worstPlanarFaceHandle, "worstPlanarFaceHandle");

    // Get worstDihedralAngleEdgeHandle to mesh
    OpenMesh::MPropHandleT< PolyMesh::EdgeHandle > worstDihedralAngleEdgeHandle;
    polyMesh.get_property_handle(worstDihedralAngleEdgeHandle, "worstDihedralAngleEdgeHandle");

    /// Delete the duplicate vertices
    /// Get worstFabVer
    vector<Vector3d> verList;

    int i = 0;
    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        Vector3d currVer;
        currVer[0] = polyMesh.point(v_it)[0];
        currVer[1] = polyMesh.point(v_it)[1];
        currVer[2] = polyMesh.point(v_it)[2];
        ++i;

        if (std::find(verList.begin(), verList.end(), currVer) == verList.end())
        {
            verList.push_back(currVer);
        }

        if (polyMesh.property(worstFabVerHandle) == *v_it)
        {
            worstFabVer = i;
        }
    }

    verMat.resize(3, verList.size());

    for (int j = 0; j < verList.size(); ++j)
    {
        verMat(0, j) = verList[j][0];
        verMat(1, j) = verList[j][1];
        verMat(2, j) = verList[j][2];
    }

    /// Find the index for each vertex of a face
    /// Get worstPlanarityVerList
    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector<int> currFaceVerList;

        for (fv_it=polyMesh.fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = polyMesh.point(fv_it)[0];
            currVer[1] = polyMesh.point(fv_it)[1];
            currVer[2] = polyMesh.point(fv_it)[2];

            int currVerIndex = std::find(verList.begin(), verList.end(), currVer) - verList.begin();

            currFaceVerList.push_back(currVerIndex);
        }

        if (polyMesh.property(worstPlanarFaceHandle) == *f_it)
        {
            worstPlanarityVerList = currFaceVerList;
        }

        faceVerList.push_back(currFaceVerList);
    }

    /// Find the index for each vertex of an edge
    /// Get worstDihedralAngleEdge

    vector<int> worstDihedralAngleEdge;

    OpenMesh::EPropHandleT< double > edgeCentroid;
    polyMesh.get_property_handle(edgeCentroid, "edgeClusterCentroid");

    for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it,0)));
        const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it,0)));

        Vector3d v1(to[0], to[1], to[2]);
        Vector3d v2(from[0], from[1], from[2]);

        vector<int> currEdgeVerList;

        int currVerIndex_1 = std::find(verList.begin(), verList.end(), v1) - verList.begin();
        int currVerIndex_2 = std::find(verList.begin(), verList.end(), v2) - verList.begin();

        currEdgeVerList.push_back(currVerIndex_1);
        currEdgeVerList.push_back(currVerIndex_2);

        edgeVerList.push_back(currEdgeVerList);

        targetLengthList.push_back(polyMesh.property(edgeCentroid, *e_it));

        if (polyMesh.property(worstDihedralAngleEdgeHandle) == *e_it)
        {
            worstDihedralAngleEdge = currEdgeVerList;
        }
    }

    /// Find the index for each vertex of a dihedral angle
    /// Get worstDihedralAngle
    for (int m = 0; m < edgeVerList.size(); ++m)
    {
        bool isWorstDihedralAngle = false;

        if (worstDihedralAngleEdge == edgeVerList[m])
        {
            isWorstDihedralAngle = true;
        }

        int id1 = edgeVerList[m][0];
        int id2 = edgeVerList[m][1];

        vector<vector<int>> currFaces;

        for (int k = 0; k < faceVerList.size(); ++k)
        {
            if (find(faceVerList[k].begin(), faceVerList[k].end(), id1) != faceVerList[k].end() and find(faceVerList[k].begin(), faceVerList[k].end(), id2) != faceVerList[k].end())
            {
                currFaces.push_back(faceVerList[k]);
            }
        }

        if (currFaces.size() != 2)
            continue;

        vector<int> currAngleVerList;
        currAngleVerList.push_back(id1);
        currAngleVerList.push_back(id2);

        for (int j = 0; j < currFaces[0].size(); ++j)
        {
            if (find(currAngleVerList.begin(), currAngleVerList.end(), currFaces[0][j]) != currAngleVerList.end())
            {
                continue;
            }

            currAngleVerList.push_back(currFaces[0][j]);

            for (int f = 0; f < currFaces[1].size(); ++f)
            {
                if (find(currAngleVerList.begin(), currAngleVerList.end(), currFaces[1][f]) != currAngleVerList.end())
                {
                    continue;
                }

                currAngleVerList.push_back(currFaces[1][f]);

                angleVerList.push_back(currAngleVerList);

                if (isWorstDihedralAngle)
                {
                    worstDihedralAngleVerList.push_back(currAngleVerList);
                }

                currAngleVerList.pop_back();
            }

            currAngleVerList.pop_back();
        }
    }

    /// Find target scale for each angle
    OpenMesh::EPropHandleT< double > angleCentroid;
    polyMesh.get_property_handle(angleCentroid, "angleClusterCentroid");

    OpenMesh::EPropHandleT< double > dihedralAngle;
    polyMesh.get_property_handle(dihedralAngle, "dihedralAngle");

    for (int k = 0; k < angleVerList.size(); ++k)
    {
        for (PolyMesh::EdgeIter e_it=polyMesh.edges_begin(); e_it!=polyMesh.edges_end(); ++e_it)
        {
            const PolyMesh::Point to = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(e_it, 0)));
            const PolyMesh::Point from = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(e_it, 0)));

            Vector3d v1(to[0], to[1], to[2]);
            Vector3d v2(from[0], from[1], from[2]);

            if ((verList[angleVerList[k][0]] == v1 and verList[angleVerList[k][1]] == v2) or (verList[angleVerList[k][1]] == v1 and verList[angleVerList[k][0]] == v2))
            {
                targetScaleList.push_back(polyMesh.property(dihedralAngle, *e_it) / polyMesh.property(angleCentroid, *e_it));
                break;
            }
        }
    }

    /// Find the index of each vertex of each diagonal and targetDiagonalList
    /// Get worstPolygonEdgeList and worstDiagonal

    // Get diagonalList to each face
    OpenMesh::FPropHandleT< vector<std::tuple<Vector3d, Vector3d, double>> > diagonalList;
    polyMesh.get_property_handle(diagonalList, "diagonalList");

    // Get clusterLabel to each face
    OpenMesh::FPropHandleT< int > clusteringLabel;
    polyMesh.get_property_handle(clusteringLabel, "clusteringLabel");

    // Get diagonalClusterCentroidList to mesh
    OpenMesh::MPropHandleT< vector<vector<double>> > diagonalClusterCentroidList;
    polyMesh.get_property_handle(diagonalClusterCentroidList, "diagonalClusterCentroidList");

    for (PolyMesh::FaceIter f_it=polyMesh.faces_begin(); f_it!=polyMesh.faces_end(); ++f_it)
    {
        auto currTupleList = polyMesh.property(diagonalList, *f_it);

        int currLabel = polyMesh.property(clusteringLabel, *f_it);

        for (int k = 0; k < currTupleList.size(); ++k)
        {
            vector<int> currVerList;
            double currTarget;

            Vector3d p1 = std::get<0>(currTupleList[k]);
            Vector3d p2 = std::get<1>(currTupleList[k]);

            int p1_id = find(verList.begin(), verList.end(), p1) - verList.begin();
            int p2_id = find(verList.begin(), verList.end(), p2) - verList.begin();

            currVerList.push_back(p1_id);
            currVerList.push_back(p2_id);

            currTarget = polyMesh.property(diagonalClusterCentroidList)[currLabel][k];

            diagonalVerList.push_back(currVerList);
            targetDiagonalList.push_back(currTarget);

            if (polyMesh.property(worstPolygonSimiFaceHandle) == *f_it) {
                worstDiagonal.push_back(currVerList);
            }


            if (polyMesh.property(worstPolygonSimiFaceHandle) == *f_it) {
                PolyMesh::FaceEdgeIter fe_it;

                for (fe_it = polyMesh.fe_iter(*f_it); fe_it.is_valid(); ++fe_it)
                {
                    const PolyMesh::Point currPolyTo   = polyMesh.point(polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*fe_it,0)));
                    const PolyMesh::Point currPolyFrom = polyMesh.point(polyMesh.from_vertex_handle(polyMesh.halfedge_handle(*fe_it,0)));

                    Vector3d currTo = PolyMeshPoint2EigenVector3d(currPolyTo);
                    Vector3d currFrom = PolyMeshPoint2EigenVector3d(currPolyFrom);

                    vector<int> currEdgeVerList;

                    currEdgeVerList.push_back(find(verList.begin(), verList.end(), currTo) - verList.begin());
                    currEdgeVerList.push_back(find(verList.begin(), verList.end(), currFrom) - verList.begin());

                    if (find(edgeVerList.begin(), edgeVerList.end(), currEdgeVerList) != edgeVerList.end())
                    {
                        worstPolygonEdgeList.push_back(currEdgeVerList);
                    }
                    else{
                        currEdgeVerList.push_back(currEdgeVerList[0]);
                        currEdgeVerList.erase(currEdgeVerList.begin());
                        worstPolygonEdgeList.push_back(currEdgeVerList);
                    }
                }
            }


        }
    }
}

void ShapeOpMatrix2PolyMesh(PolyMesh & polyMesh, Matrix3Xd verMat, vector< vector<int> > faceVerList)
{
    std::vector<PolyMesh::VertexHandle>  face_vhandles;

    int verNum = verMat.cols();
    PolyMesh::VertexHandle vHandle[verNum];

    // Add vertices
    for (int i = 0; i < verMat.cols(); ++i)
    {
        vHandle[i] = polyMesh.add_vertex(PolyMesh::Point(verMat(0, i),verMat(1, i),verMat(2, i)));
    }

    // Add faces
    for (int i = 0; i < faceVerList.size(); ++i)
    {
        face_vhandles.clear();
        for (int j = 0; j < faceVerList[i].size(); ++j)
        {
            face_vhandles.push_back(vHandle[faceVerList[i][j]]);
        }

        polyMesh.add_face(face_vhandles);
    }
}

void ShapeOpMatrix2VerList(Matrix3Xd verMat, vector<Vector3d> & verList)
{
    verList.clear();

    for (int i = 0; i < verMat.cols(); ++i)
    {
        Vector3d curr;
        curr[0] = verMat(0, i);
        curr[1] = verMat(1, i);
        curr[2] = verMat(2, i);

        verList.push_back(curr);
    }
}

void ShapeOpMatrix2OriPolyMesh(PolyMesh & polyMesh, Matrix3Xd verMat)
{
    int i = 0;
    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        polyMesh.set_point(*v_it, PolyMesh::Point(float(verMat(0, i)), float(verMat(1, i)), float(verMat(2, i))));
//        dbg(PolyMesh::Point(float(verMat(0, i)), float(verMat(1, i)), float(verMat(2, i))));
        ++i;
    }
}




///==============================================================================//
///                             PolyMesh property transfer
///==============================================================================//

PolyMesh::HalfedgeHandle FindHalfedge(const PolyMesh & m, PolyMesh::VertexHandle v1, PolyMesh::VertexHandle v2)
{
    PolyMesh::HalfedgeHandle heh = m.find_halfedge(v1, v2);
    if (heh.is_valid()) {
        return heh;
    }
    else {
        return PolyMesh::InvalidHalfedgeHandle;
    }
}

PolyMesh::EdgeHandle FindEdge(const PolyMesh& m, PolyMesh::VertexHandle v1, PolyMesh::VertexHandle v2)
{
    PolyMesh::HalfedgeHandle heh = m.find_halfedge(v1, v2);
    if (heh.is_valid()) {
        return m.edge_handle(heh);
    }
    else {
        return PolyMesh::InvalidEdgeHandle;
    }
}

void NormalListPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh)
{
    // Add normalList to each half edge
    OpenMesh::HPropHandleT< vector<Vector3d> > normalList;
    triMesh->add_property( normalList ,"normalList");

    for (PolyMesh::FaceIter f_it=triMesh->faces_begin(); f_it!=triMesh->faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        for (fv_it=triMesh->fv_iter( *f_it ); fv_it.is_valid(); ++fv_it) {
            auto curr_it = fv_it;
            PolyMesh::VertexHandle v1 = curr_it;
            PolyMesh::VertexHandle v2 = curr_it++;

            if (!v2.is_valid())
                v2=triMesh->fv_iter( *f_it );

            if (FindHalfedge(*polyMesh, v1, v2) != PolyMesh::InvalidHalfedgeHandle)
            {
                OpenMesh::HPropHandleT< vector<Vector3d> > currPolyMeshNormalList;
                polyMesh->get_property_handle(currPolyMeshNormalList, "normalList");

                OpenMesh::HPropHandleT< vector<Vector3d> > currTriMeshNormalList;
                triMesh->get_property_handle(currTriMeshNormalList, "normalList");

                triMesh->property(currTriMeshNormalList, FindHalfedge(*triMesh, v1, v2)) = polyMesh->property(currPolyMeshNormalList, FindHalfedge(*polyMesh, v1, v2));

                cout << "currNormal: " << triMesh->property(currTriMeshNormalList, FindHalfedge(*triMesh, v1, v2))[0] << endl;
            }
            else
            {
                vector<Vector3d> emptyList;
                OpenMesh::HPropHandleT< vector<Vector3d> > currTriMeshNormalList;
                triMesh->get_property_handle(currTriMeshNormalList, "normalList");

                triMesh->property(currTriMeshNormalList, FindHalfedge(*triMesh, v1, v2)) = emptyList;
            }
        }
    }
}

void EdgeNormalPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh)
{
    // Add normal to each edge
    OpenMesh::EPropHandleT< Vector3d > normal;
    triMesh->add_property( normal ,"normal");

    for (PolyMesh::EdgeIter e_it=triMesh->edges_begin(); e_it!=triMesh->edges_end(); ++e_it)
    {
        const PolyMesh::Point to   = triMesh->point(triMesh->to_vertex_handle(triMesh->halfedge_handle(e_it,0)));
        const PolyMesh::Point from = triMesh->point(triMesh->from_vertex_handle(triMesh->halfedge_handle(e_it,0)));

        for (PolyMesh::EdgeIter e_it_2=polyMesh->edges_begin(); e_it_2!=polyMesh->edges_end(); ++e_it_2)
        {
            const PolyMesh::Point currTo   = polyMesh->point(polyMesh->to_vertex_handle(polyMesh->halfedge_handle(e_it_2,0)));
            const PolyMesh::Point currFrom = polyMesh->point(polyMesh->from_vertex_handle(polyMesh->halfedge_handle(e_it_2,0)));

            if ((currTo == to and currFrom == from) or (currTo == from and currFrom == to))
            {
                OpenMesh::EPropHandleT< Vector3d > polyEdgeNormal;
                polyMesh->get_property_handle( polyEdgeNormal ,"normal");

                triMesh->property(normal, *e_it) = polyMesh->property(polyEdgeNormal, *e_it_2);

//                cout << "CurrNormal: " << endl << triMesh->property(normal, *e_it) << endl;
            }
        }
    }
}

void PlanarityPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh)
{
    // Add planarity to each face
    OpenMesh::FPropHandleT< double > triPlanarity;
    triMesh->add_property( triPlanarity ,"facePlanarity");

    // Get planarity to each face
    OpenMesh::FPropHandleT< double > polyPlanarity;
    polyMesh->get_property_handle(polyPlanarity, "facePlanarity");

    for (PolyMesh::FaceIter f_it=triMesh->faces_begin(); f_it!=triMesh->faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector< Vector3d > currFaceVerList;

        for (fv_it=triMesh->fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = triMesh->point(fv_it)[0];
            currVer[1] = triMesh->point(fv_it)[1];
            currVer[2] = triMesh->point(fv_it)[2];

            currFaceVerList.push_back(currVer);
        }

        for (PolyMesh::FaceIter f_it_poly=polyMesh->faces_begin(); f_it_poly!=polyMesh->faces_end(); ++f_it_poly)
        {
            PolyMesh::FaceVertexIter fv_it_poly;

            vector< Vector3d > currFaceVerList_poly;

            for (fv_it_poly=polyMesh->fv_iter( *f_it_poly ); fv_it_poly.is_valid(); ++fv_it_poly)
            {
                Vector3d currVer;
                currVer[0] = triMesh->point(fv_it_poly)[0];
                currVer[1] = triMesh->point(fv_it_poly)[1];
                currVer[2] = triMesh->point(fv_it_poly)[2];

                currFaceVerList_poly.push_back(currVer);
            }

            if (find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[0]) != currFaceVerList_poly.end()
            and find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[1]) != currFaceVerList_poly.end()
            and find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[2]) != currFaceVerList_poly.end() )
            {
                triMesh->property(triPlanarity, *f_it) = polyMesh->property(polyPlanarity, *f_it_poly);
//                cout << "Done." << endl;
//                cout << "currPlanarity: " << polyMesh->property(polyPlanarity, *f_it_poly)<< endl;
            }
        }
    }
}

void ClusteringLabelPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh)
{
    // Add clusteringLabel to each face
    OpenMesh::FPropHandleT< int > triLabel;
    triMesh->add_property( triLabel ,"clusteringLabel");

    // Get clusteringLabel to each face
    OpenMesh::FPropHandleT< int > polyLabel;
    polyMesh->get_property_handle(polyLabel, "clusteringLabel");

    for (PolyMesh::FaceIter f_it=triMesh->faces_begin(); f_it!=triMesh->faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector< Vector3d > currFaceVerList;

        for (fv_it=triMesh->fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = triMesh->point(*fv_it)[0];
            currVer[1] = triMesh->point(*fv_it)[1];
            currVer[2] = triMesh->point(*fv_it)[2];

            currFaceVerList.push_back(currVer);
        }

        for (PolyMesh::FaceIter f_it_poly=polyMesh->faces_begin(); f_it_poly!=polyMesh->faces_end(); ++f_it_poly)
        {
            PolyMesh::FaceVertexIter fv_it_poly;

            vector< Vector3d > currFaceVerList_poly;

            for (fv_it_poly=polyMesh->fv_iter( *f_it_poly ); fv_it_poly.is_valid(); ++fv_it_poly)
            {
                Vector3d currVer;
                currVer[0] = triMesh->point(*fv_it_poly)[0];
                currVer[1] = triMesh->point(*fv_it_poly)[1];
                currVer[2] = triMesh->point(*fv_it_poly)[2];

                currFaceVerList_poly.push_back(currVer);
            }

            if (find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[0]) != currFaceVerList_poly.end()
                and find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[1]) != currFaceVerList_poly.end()
                and find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[2]) != currFaceVerList_poly.end() )
            {
                triMesh->property(triLabel, *f_it) = polyMesh->property(polyLabel, *f_it_poly);
//                cout << "currLabel: " << triMesh->property(triLabel, *f_it) << endl;
            }
        }
    }
}

void BlockClusteringLabelPropertyTransfer(PolyMesh * polyMesh, PolyMesh * triMesh)
{
    // Add blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > triBlockClusteringLabel;

    if (!triMesh->get_property_handle(triBlockClusteringLabel, "blockClusteringLabel"))
        triMesh->add_property(triBlockClusteringLabel, "blockClusteringLabel");
    else
        triMesh->get_property_handle(triBlockClusteringLabel, "blockClusteringLabel");

    // Get blockClusteringLabel to each face
    OpenMesh::FPropHandleT< int > polyBlockClusteringLabel;
    polyMesh->get_property_handle(polyBlockClusteringLabel, "blockClusteringLabel");

    for (PolyMesh::FaceIter f_it=triMesh->faces_begin(); f_it!=triMesh->faces_end(); ++f_it)
    {
        PolyMesh::FaceVertexIter fv_it;

        vector< Vector3d > currFaceVerList;

        for (fv_it=triMesh->fv_iter( *f_it ); fv_it.is_valid(); ++fv_it)
        {
            Vector3d currVer;
            currVer[0] = triMesh->point(*fv_it)[0];
            currVer[1] = triMesh->point(*fv_it)[1];
            currVer[2] = triMesh->point(*fv_it)[2];

            currFaceVerList.push_back(currVer);
        }

        for (PolyMesh::FaceIter f_it_poly=polyMesh->faces_begin(); f_it_poly!=polyMesh->faces_end(); ++f_it_poly)
        {
            PolyMesh::FaceVertexIter fv_it_poly;

            vector< Vector3d > currFaceVerList_poly;

            for (fv_it_poly=polyMesh->fv_iter( *f_it_poly ); fv_it_poly.is_valid(); ++fv_it_poly)
            {
                Vector3d currVer;
                currVer[0] = triMesh->point(*fv_it_poly)[0];
                currVer[1] = triMesh->point(*fv_it_poly)[1];
                currVer[2] = triMesh->point(*fv_it_poly)[2];

                currFaceVerList_poly.push_back(currVer);
            }

            if (find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[0]) != currFaceVerList_poly.end()
                and find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[1]) != currFaceVerList_poly.end()
                and find(currFaceVerList_poly.begin(), currFaceVerList_poly.end(), currFaceVerList[2]) != currFaceVerList_poly.end() )
            {
                triMesh->property(triBlockClusteringLabel, *f_it) = polyMesh->property(polyBlockClusteringLabel, *f_it_poly);
//                cout << "currLabel: " << triMesh->property(triBlockClusteringLabel, *f_it) << endl;
            }
        }
    }
}




///==============================================================================//
///                         OpenMesh Get Operation
///==============================================================================//

void GetFaceVerList(PolyMesh * polyMesh, PolyMesh::FaceIter fh, vector<Vector3d> & verList)
{
    PolyMesh::FaceVertexIter fv_it_poly;

    for (fv_it_poly=polyMesh->fv_iter( *fh ); fv_it_poly.is_valid(); ++fv_it_poly)
    {
        Vector3d currVer;
        currVer[0] = polyMesh->point(*fv_it_poly)[0];
        currVer[1] = polyMesh->point(*fv_it_poly)[1];
        currVer[2] = polyMesh->point(*fv_it_poly)[2];

        verList.push_back(currVer);
    }
}




///==============================================================================//
///                             Least Squares Method
///==============================================================================//

void FindFittedPlane_LSM(MatrixXd points, Vector3d & normal, double & d)
{
    // 1、计算质心
    Eigen::RowVector3d centroid = points.colwise().mean();

    // 2、去质心
    Eigen::MatrixXd demean = points;
    demean.rowwise() -= centroid;

    // 3、SVD分解求解协方差矩阵的特征值特征向量
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(demean, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();

    // 5、平面的法向量a,b,c
    Eigen::RowVector3d rowNormal;
    rowNormal << V(0,2), V(1,2), V(2,2);
    normal(0) = rowNormal(0); normal(1) = rowNormal(1); normal(2) = rowNormal(2);

    // 6、原点到平面的距离d
    d = -rowNormal * centroid.transpose();
}

void FindFittedPlane_LSM(vector<Vector3d> verList, Vector3d & normal)
{
    MatrixXd points;
    points.resize(verList.size(), 3);

    for (int j = 0; j < verList.size(); ++j)
    {
        points(j, 0) = verList[j][0];
        points(j, 1) = verList[j][1];
        points(j, 2) = verList[j][2];
    }

    // 1、计算质心
    Eigen::RowVector3d centroid = points.colwise().mean();

    // 2、去质心
    Eigen::MatrixXd demean = points;
    demean.rowwise() -= centroid;

    // 3、SVD分解求解协方差矩阵的特征值特征向量
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(demean, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();

    // 5、平面的法向量a,b,c
    Eigen::RowVector3d rowNormal;
    rowNormal << V(0,2), V(1,2), V(2,2);
    normal(0) = rowNormal(0); normal(1) = rowNormal(1); normal(2) = rowNormal(2);
}




///==============================================================================//
///                             Add Property to PolyMesh
///==============================================================================//

void AddVertexTextureCoord2PolyMesh(PolyMesh & polyMesh)
{
    polyMesh.request_vertex_texcoords2D();

    PolyMesh triMesh = polyMesh;
    triMesh.triangulate();

    MatrixXd V_uv;

    MatrixXd V;
    MatrixXi F;
    PolyMesh2Matrix(triMesh, V, F);

    Vector3d minPoint = V.colwise().minCoeff();
    Vector3d maxPoint = V.colwise().maxCoeff();

    for (PolyMesh::VertexIter v_it=polyMesh.vertices_begin(); v_it!=polyMesh.vertices_end(); ++v_it)
    {
        float x, y, z, r;
        float theta, phi;

        x = polyMesh.point(*v_it)[0];
        y = polyMesh.point(*v_it)[1];
        z = polyMesh.point(*v_it)[2];

        r = sqrt(x*x+y*y);
        theta = atan2(y,x) + 3.1415926;
        phi = atan2(r,z);

        double t0 = theta/(2*3.1415926);
        double t1 = phi/3.1415926;

        polyMesh.set_texcoord2D(*v_it, PolyMesh::TexCoord2D(t0, t1));
    }
}




///==============================================================================//
///                             Random Stuff
///==============================================================================//

vector<int> GetRandomObjIndexList(vector<float> possibList, float alpha, int objNum)
{
    vector<int> objIndexList;
    if ( objNum > possibList.size() || objNum <= 0 )
    {
        printf("Warning: The input objNum is not correct %d ! \n\n", objNum);
        return objIndexList;
    }

    while( objIndexList.size() < objNum )
    {
        int tempIndex = GetRandomObjIndex(possibList, alpha);
        if ( std::find(objIndexList.begin(), objIndexList.end(), tempIndex) == objIndexList.end() )
        {
            objIndexList.push_back(tempIndex);
        }
    }

    return objIndexList;
}


// Input:  the selection possibility value of each object
// Output: randomly selected object index
int GetRandomObjIndex(vector<float> possibList, float alpha)
{
    if ( possibList.size() == 0)
        return -1;

    if ( possibList.size() == 1)
        return 0;

    vector<float> possibMapList = PossibExpMapping(possibList, alpha);

    // Compute possibility regions for objects with possibility value [P0, P1, P2, ..., P(k-1), Pk]
    // which is [P0, P0+P1, P0+P1+P2, ..., P0+P1+..+P(k-1), P0+P1+..+P(k-1)+Pk]
    vector<float> possibRegions;
    for (int i=0; i<possibMapList.size(); i++)
    {
        float possibSum = 0;
        for (int j=0; j<=i; j++)
        {
            possibSum += possibMapList[j];
        }
        possibRegions.push_back(possibSum);
    }

    // Generate a random value in range [0, P0+P1+..+P(k-1)+Pk)
    int lastObjIndex = possibMapList.size() - 1;
    float randValue = (rand()/(RAND_MAX+1.0)) * possibRegions[lastObjIndex];

    // Return object index by finding which region the random value falls into
    for (int i=0; i<possibRegions.size(); i++)
    {
        // Find each object's possibility range
        float regionMinValue, regionMaxValue;
        if ( i == 0 )
        {
            regionMinValue = 0;
            regionMaxValue = possibRegions[0];
        }
        else
        {
            regionMinValue = possibRegions[i-1];
            regionMaxValue = possibRegions[i];
        }

        // Return the randomly selected object index
        if ( randValue >= regionMinValue &&
             randValue <= regionMaxValue )
        {
            return i;
        }
    }

    printf("Warning: the return random value may not be correct. \n");

    return 0;
}


vector<float> PossibExpMapping(vector<float> possibList, float alpha)
{
    // Check if the input possibility values are correct
    for (int i=0; i<possibList.size(); i++)
    {
        if ( possibList[i] < 0 || possibList[i] > MAX_INT )
        {
            printf("Warning: The input possibility values [i=%2d  P=%.2f] are not correct! \n\n", i, possibList[i]);
        }
    }

    // Possibility value exponential mapping
    vector<float> possibMapList;
    for (int i=0; i<possibList.size(); i++)
    {
        float tempPossib = pow(possibList[i], alpha);
        possibMapList.push_back(tempPossib);
        //printf("i=%d  P1 %.2f  P2 %.2f \n", i, possibList[i], possibMapList[i]);
    }

    // Calculate the sum of all the possibility values in each list
    //float totalPossib1 = 0;
    //float totalPossib2 = 0;
    //for (int i=0; i<possibList.size(); i++)
    //{
    //	totalPossib1 += possibList[i];
    //	totalPossib2 += possibMapList[i];
    //}
    ////printf("Total Possib 1: %.2f \n", totalPossib1);
    ////printf("Total Possib 2: %.2f \n", totalPossib2);

    //// Normalize the possibility values in each list
    //for (int i=0; i<possibList.size(); i++)
    //{
    //	possibList[i]    /= totalPossib1;
    //	possibMapList[i] /= totalPossib2;
    //	//printf("i=%d  P1 %.2f  P2 %.2f \n", i, possibList[i], possibMapList[i]);
    //}

    return possibMapList;
}


// Return a float value in [0 1)
float GetRandomNumber(int seedIndex)
{
    if ( seedIndex < 1 )
        printf("Warning: seedIndex cannot be smaller than 1! \n\n");

    float randValue = seedIndex * rand()/(RAND_MAX+1.0);

    if( randValue > 1.0 )
        randValue = randValue - floor(randValue);
    else if( randValue == 1.0 )
        randValue = 0.999999;

    return randValue;
}