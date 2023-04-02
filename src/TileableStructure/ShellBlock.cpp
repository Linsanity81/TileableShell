//
// Created by Linsanity on 27/6/22.
//

#include "ShellBlock.h"
#include "Mesh/MeshBoolean.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

ShellBlock::ShellBlock(PolyMesh::FaceHandle _origFaceHandle, int _assemblyID, int _clusterID, float _cutUpper, float _cutLower,
        Vector3d _lowerFaceNormal, Vector3d _upperFaceNormal,
        Vector3d _lowerDiagonalStart, Vector3d _lowerDiagonalEnd,
        Vector3d _upperDiagonalStart, Vector3d _upperDiagonalEnd)
{
    origFaceHandle = _origFaceHandle;

    assemblyID = _assemblyID;
    clusterID = _clusterID + 1;
    cutLower = _cutLower;
    cutUpper = _cutUpper;

    lowerFaceNormal = _lowerFaceNormal;
    upperFaceNormal = _upperFaceNormal;

    lowerDiagonalStart = _lowerDiagonalStart;
    lowerDiagonalEnd = _lowerDiagonalEnd;

    upperDiagonalStart = _upperDiagonalStart;
    upperDiagonalEnd = _upperDiagonalEnd;

    string assemblyIDString = to_string(assemblyID);
    assemblyDigitList.clear();

    for (int i = 0; i < assemblyIDString.size(); ++i)
    {
        assemblyDigitList.push_back(int(assemblyIDString[i]) - 48);
    }

    string clusterIDString = to_string(clusterID);
    clusterDigitList.clear();

    for (int i = 0; i < clusterIDString.size(); ++i)
    {
        clusterDigitList.push_back(int(clusterIDString[i]) - 48);
    }

    assemblyDigitCenterList.clear();
    clusterDigitCenterList.clear();

    {
        Vector3d currMoveVec = lowerDiagonalEnd - lowerDiagonalStart;
        Vector3d currYPlaneVec(currMoveVec[0], 0, currMoveVec[2]);

        for (int i = 0; i < assemblyDigitList.size(); ++i)
        {
            assemblyDigitCenterList.push_back( lowerDiagonalEnd - double(i + 1) * currMoveVec / double(assemblyDigitList.size() + 1.0) );
        }

        xAxisAngle_assemblyDigits = std::atan2(0, -1)  - std::atan2(currMoveVec[2], currMoveVec[0]);

        assemblyDigitSize = Vector3d(0, 0, 0);

        assemblyDigitSize(0) = (currMoveVec / (assemblyDigitList.size() + 3)).norm();
        assemblyDigitSize(1) = cutLower * 0.2;
        assemblyDigitSize(2) = (assemblyDigitSize(0) / 0.58) * 1.0;

        rotMatForAssemblyDigitBoolean = igl::rotation_matrix_from_directions(Vector3d(0, -1, 0), lowerFaceNormal);
        scaleAssemblyMat = GetScaleMatrix3d(Vector3d(assemblyDigitSize(0) / 0.58, assemblyDigitSize(1) / 0.49,  assemblyDigitSize(2)));
    }

    {
        Vector3d currMoveVec = upperDiagonalEnd - upperDiagonalStart;
        Vector3d currYPlaneVec(currMoveVec[0], 0, currMoveVec[2]);

        for (int i = 0; i < clusterDigitList.size(); ++i)
        {
            clusterDigitCenterList.push_back( upperDiagonalStart + double(i + 1) * currMoveVec / double(clusterDigitList.size() + 1.0) );
        }

        xAxisAngle_clusterDigits = std::atan2(0, 1)  - std::atan2(currMoveVec[2], currMoveVec[0]);

        clusterDigitSize = Vector3d(0, 0, 0);

        clusterDigitSize(0) = (currMoveVec / (clusterDigitList.size() + 3)).norm();
        clusterDigitSize(1) = cutUpper * 0.2;
        clusterDigitSize(2) = (clusterDigitSize(0) / 0.58) * 1.0;

        rotMatForClusterDigitBoolean = igl::rotation_matrix_from_directions(Vector3d(0, 1, 0), upperFaceNormal);
        scaleClusterMat = GetScaleMatrix3d(Vector3d(clusterDigitSize(0) / 0.58, clusterDigitSize(1) / 0.49,  clusterDigitSize(2)));
    }

//    cout << "xAxisAngle_assemblyDigits: " << xAxisAngle_assemblyDigits << endl;
//    cout << "xAxisAngle_clusterDigits: " << xAxisAngle_clusterDigits << endl;

    int currIndex = assemblyID % 25 + 1;

    curaPos = Vector3d(0, 0, 0);

    curaPos[0] = (currIndex % 5) * 0.5;
    curaPos[1] = 0;
    curaPos[2] = ceil(currIndex / 5.0f) * 0.5;
}

ShellBlock::~ShellBlock()
{

}




//**************************************************************************************//
//                                   Save Files
//**************************************************************************************//

void ShellBlock::AddDigit2OrigShellBlock()
{
    Eigen::MatrixXd V_block;
    Eigen::MatrixXi F_block;

    PolyMesh2Matrix(origShellBlockTri, V_block, F_block);

//    for (int i = 0; i < clusterDigitList.size(); ++i)
//    {
//        string currDigitPath = GetDigitOBJFilePath(clusterDigitList[i]);
//
//        Eigen::MatrixXd V_digit;
//        Eigen::MatrixXi F_digit;
//
//        igl::readOBJ(currDigitPath, V_digit, F_digit);
//
//        V_digit = V_digit * scaleClusterMat;
//
//        Matrix3d tempXRotMat = GetRotationMatrix3d(xAxisAngle_clusterDigits, Vector3d(0, 1, 0), Vector3d(0, 0, 0) );
//
//        for (int j = 0; j < V_digit.rows(); ++j)
//        {
//            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));
//
//            auto currResult = tempXRotMat * currPoint;
//
//            V_digit(j, 0) = currResult[0];
//            V_digit(j, 1) = currResult[1];
//            V_digit(j, 2) = currResult[2];
//        }
//
//        for (int j = 0; j < V_digit.rows(); ++j)
//        {
//            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));
//
//            auto currResult = rotMatForClusterDigitBoolean * currPoint;
//
//            V_digit(j, 0) = currResult[0];
//            V_digit(j, 1) = currResult[1];
//            V_digit(j, 2) = currResult[2];
//        }
//
//        for (int j = 0; j < V_digit.rows(); ++j)
//        {
//            V_digit(j, 0) += clusterDigitCenterList[i][0];
//            V_digit(j, 1) += clusterDigitCenterList[i][1];
//            V_digit(j, 2) += clusterDigitCenterList[i][2];
//        }
//
//        Eigen::MatrixXd V_csg;
//        Eigen::MatrixXi F_csg;
//
//        MeshBoolean meshBoolean;
//        meshBoolean.MeshMinus(V_block, F_block, V_digit, F_digit, V_csg, F_csg);
//
//        PolyMesh currMesh;
//        Matrix2PolyMesh(currMesh, V_csg, F_csg);
//
//        V_block = V_csg;
//        F_block = F_csg;
//    }

    for (int i = 0; i < assemblyDigitList.size(); ++i)
    {
        string currDigitPath = GetDigitOBJFilePath(assemblyDigitList[i]);

        Eigen::MatrixXd V_digit;
        Eigen::MatrixXi F_digit;

        igl::readOBJ(currDigitPath, V_digit, F_digit);

        V_digit = V_digit * scaleAssemblyMat;

        Matrix3d reflectionMat = GetRotationMatrix3d(3.1415926, Vector3d(1, 0, 0), Vector3d(0, 0, 0));

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));

            auto currResult = reflectionMat * currPoint;

            V_digit(j, 0) = currResult[0];
            V_digit(j, 1) = currResult[1];
            V_digit(j, 2) = currResult[2];
        }

        Matrix3d tempXRotMat = GetRotationMatrix3d(xAxisAngle_assemblyDigits, Vector3d(0, 1, 0), Vector3d(0, 0, 0) );

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));

            auto currResult = tempXRotMat * currPoint;

            V_digit(j, 0) = currResult[0];
            V_digit(j, 1) = currResult[1];
            V_digit(j, 2) = currResult[2];
        }

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));

            auto currResult = rotMatForAssemblyDigitBoolean * currPoint;

            V_digit(j, 0) = currResult[0];
            V_digit(j, 1) = currResult[1];
            V_digit(j, 2) = currResult[2];
        }

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            V_digit(j, 0) += assemblyDigitCenterList[i][0];
            V_digit(j, 1) += assemblyDigitCenterList[i][1];
            V_digit(j, 2) += assemblyDigitCenterList[i][2];
        }

        Eigen::MatrixXd V_csg;
        Eigen::MatrixXi F_csg;

        MeshBoolean meshBoolean;
        meshBoolean.MeshMinus(V_block, F_block, V_digit, F_digit, V_csg, F_csg);

        PolyMesh currMesh;
        Matrix2PolyMesh(currMesh, V_csg, F_csg);

        V_block = V_csg;
        F_block = F_csg;
    }

    PolyMesh blockAfterCSG;
    Matrix2PolyMesh(blockAfterCSG, V_block, F_block);

    origShellBlock_digits = blockAfterCSG;

    // Rotate to align upperFaceNormal
    {
        MatrixXd curaMat = igl::rotation_matrix_from_directions(upperFaceNormal, Vector3d(0, 1, 0));

        for (int j = 0; j < V_block.rows(); ++j)
        {
            Vector3d currPoint(V_block(j, 0), V_block(j, 1), V_block(j, 2));

            auto currResult = curaMat * currPoint;

            V_block(j, 0) = currResult[0];
            V_block(j, 1) = currResult[1];
            V_block(j, 2) = currResult[2];
        }
    }

    // Move to (0, 0, 0)
    {
        Vector3d currCentroid = ComputeMatrixCentroid(V_block);

        Vector3d offset = Vector3d(0, 0, 0) - currCentroid;

        for (int j = 0; j < V_block.rows(); ++j)
        {
            V_block(j, 0) += offset[0];
            V_block(j, 1) += offset[1];
            V_block(j, 2) += offset[2];
        }
    }

    // Rotate to align (1, 0, 0)
    {
        Matrix3d tempXRotMat = GetRotationMatrix3d(xAxisAngle_clusterDigits, Vector3d(0, 1, 0), Vector3d(0, 0, 0) );

        for (int j = 0; j < V_block.rows(); ++j)
        {
            Vector3d currPoint(V_block(j, 0), V_block(j, 1), V_block(j, 2));

            auto currResult = tempXRotMat * currPoint;

            V_block(j, 0) = currResult[0];
            V_block(j, 1) = currResult[1];
            V_block(j, 2) = currResult[2];
        }
    }

    // Move to curaPos
    {
        Vector3d currCentroid = ComputeMatrixCentroid(V_block);

        Vector3d offset = curaPos - currCentroid;

        for (int j = 0; j < V_block.rows(); ++j)
        {
            V_block(j, 0) += offset[0];
            V_block(j, 1) += offset[1];
            V_block(j, 2) += offset[2];
        }
    }

    // Align to ground
    {
        double currMinY = V_block.col(1).minCoeff();

        double offset = -currMinY;

        for (int j = 0; j < V_block.rows(); ++j)
        {
            V_block(j, 1) += offset;
        }
    }

    PolyMesh blockAfterCSG_cura;
    Matrix2PolyMesh(blockAfterCSG_cura, V_block, F_block);

    origShellBlock_digits_cura = blockAfterCSG_cura;
}

void ShellBlock::AddDigit2ReplacedShellBlock()
{
    Eigen::MatrixXd V_block;
    Eigen::MatrixXi F_block;

    PolyMesh2Matrix(replacedShellBlockTri, V_block, F_block);

//    for (int i = 0; i < clusterDigitList.size(); ++i)
//    {
//        string currDigitPath = GetDigitOBJFilePath(clusterDigitList[i]);
//
//        Eigen::MatrixXd V_digit;
//        Eigen::MatrixXi F_digit;
//
//        igl::readOBJ(currDigitPath, V_digit, F_digit);
//
//        V_digit = V_digit * scaleClusterMat;
//
//        Matrix3d tempXRotMat = GetRotationMatrix3d(xAxisAngle_clusterDigits, Vector3d(0, 1, 0), Vector3d(0, 0, 0) );
//
//        for (int j = 0; j < V_digit.rows(); ++j)
//        {
//            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));
//
//            auto currResult = tempXRotMat * currPoint;
//
//            V_digit(j, 0) = currResult[0];
//            V_digit(j, 1) = currResult[1];
//            V_digit(j, 2) = currResult[2];
//        }
//
//        for (int j = 0; j < V_digit.rows(); ++j)
//        {
//            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));
//
//            auto currResult = rotMatForClusterDigitBoolean * currPoint;
//
//            V_digit(j, 0) = currResult[0];
//            V_digit(j, 1) = currResult[1];
//            V_digit(j, 2) = currResult[2];
//        }
//
//        for (int j = 0; j < V_digit.rows(); ++j)
//        {
//            V_digit(j, 0) += clusterDigitCenterList[i][0];
//            V_digit(j, 1) += clusterDigitCenterList[i][1];
//            V_digit(j, 2) += clusterDigitCenterList[i][2];
//        }
//
//        Eigen::MatrixXd V_csg;
//        Eigen::MatrixXi F_csg;
//
//        MeshBoolean meshBoolean;
//        meshBoolean.MeshMinus(V_block, F_block, V_digit, F_digit, V_csg, F_csg);
//
//        PolyMesh currMesh;
//        Matrix2PolyMesh(currMesh, V_csg, F_csg);
//
//        V_block = V_csg;
//        F_block = F_csg;
//    }

    for (int i = 0; i < assemblyDigitList.size(); ++i)
    {
        string currDigitPath = GetDigitOBJFilePath(assemblyDigitList[i]);

        Eigen::MatrixXd V_digit;
        Eigen::MatrixXi F_digit;

        igl::readOBJ(currDigitPath, V_digit, F_digit);

        V_digit = V_digit * scaleAssemblyMat;

        Matrix3d reflectionMat = GetRotationMatrix3d(3.1415926, Vector3d(1, 0, 0), Vector3d(0, 0, 0));

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));

            auto currResult = reflectionMat * currPoint;

            V_digit(j, 0) = currResult[0];
            V_digit(j, 1) = currResult[1];
            V_digit(j, 2) = currResult[2];
        }

        Matrix3d tempXRotMat = GetRotationMatrix3d(xAxisAngle_assemblyDigits, Vector3d(0, 1, 0), Vector3d(0, 0, 0) );

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));

            auto currResult = tempXRotMat * currPoint;

            V_digit(j, 0) = currResult[0];
            V_digit(j, 1) = currResult[1];
            V_digit(j, 2) = currResult[2];
        }

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            Vector3d currPoint(V_digit(j, 0), V_digit(j, 1), V_digit(j, 2));

            auto currResult = rotMatForAssemblyDigitBoolean * currPoint;

            V_digit(j, 0) = currResult[0];
            V_digit(j, 1) = currResult[1];
            V_digit(j, 2) = currResult[2];
        }

        for (int j = 0; j < V_digit.rows(); ++j)
        {
            V_digit(j, 0) += assemblyDigitCenterList[i][0];
            V_digit(j, 1) += assemblyDigitCenterList[i][1];
            V_digit(j, 2) += assemblyDigitCenterList[i][2];
        }

        Eigen::MatrixXd V_csg;
        Eigen::MatrixXi F_csg;

        MeshBoolean meshBoolean;
        meshBoolean.MeshMinus(V_block, F_block, V_digit, F_digit, V_csg, F_csg);

        PolyMesh currMesh;
        Matrix2PolyMesh(currMesh, V_csg, F_csg);

        V_block = V_csg;
        F_block = F_csg;
    }

    PolyMesh blockAfterCSG;
    Matrix2PolyMesh(blockAfterCSG, V_block, F_block);

    replacedShellBlock_digits = blockAfterCSG;

    // Rotate to align upperFaceNormal
    {
        MatrixXd curaMat = igl::rotation_matrix_from_directions(upperFaceNormal, Vector3d(0, 1, 0));

        for (int j = 0; j < V_block.rows(); ++j)
        {
            Vector3d currPoint(V_block(j, 0), V_block(j, 1), V_block(j, 2));

            auto currResult = curaMat * currPoint;

            V_block(j, 0) = currResult[0];
            V_block(j, 1) = currResult[1];
            V_block(j, 2) = currResult[2];
        }
    }

    // Move to (0, 0, 0)
    {
        Vector3d currCentroid = ComputeMatrixCentroid(V_block);

        Vector3d offset = Vector3d(0, 0, 0) - currCentroid;

        for (int j = 0; j < V_block.rows(); ++j)
        {
            V_block(j, 0) += offset[0];
            V_block(j, 1) += offset[1];
            V_block(j, 2) += offset[2];
        }
    }

    // Rotate to align (1, 0, 0)
    {
        Matrix3d tempXRotMat = GetRotationMatrix3d(xAxisAngle_clusterDigits, Vector3d(0, 1, 0), Vector3d(0, 0, 0) );

        for (int j = 0; j < V_block.rows(); ++j)
        {
            Vector3d currPoint(V_block(j, 0), V_block(j, 1), V_block(j, 2));

            auto currResult = tempXRotMat * currPoint;

            V_block(j, 0) = currResult[0];
            V_block(j, 1) = currResult[1];
            V_block(j, 2) = currResult[2];
        }
    }

    // Move to curaPos
    {
        Vector3d currCentroid = ComputeMatrixCentroid(V_block);

        Vector3d offset = curaPos - currCentroid;

        for (int j = 0; j < V_block.rows(); ++j)
        {
            V_block(j, 0) += offset[0];
            V_block(j, 1) += offset[1];
            V_block(j, 2) += offset[2];
        }
    }

    // Align to ground
    {
        double currMinY = V_block.col(1).minCoeff();

        double offset = -currMinY;

        for (int j = 0; j < V_block.rows(); ++j)
        {
            V_block(j, 1) += offset;
        }
    }

    PolyMesh blockAfterCSG_cura;
    Matrix2PolyMesh(blockAfterCSG_cura, V_block, F_block);

    replacedShellBlock_digits_cura = blockAfterCSG_cura;
}

string ShellBlock::GetDigitOBJFilePath(int digit)
{
    // return "/Users/sutd_cgl/Documents/DigitOBJs/Digit_" + to_string(digit) + ".obj";

    return "/Users/linsanity/Documents/DigitOBJs/Digit_" + to_string(digit) + ".obj";

//    return "../DigitOBJs/Digit_" + to_string(digit) + ".obj";

}




//**************************************************************************************//
//                                   Save Files
//**************************************************************************************//

void ShellBlock::SaveOrigShellBlock(string folderPath)
{
    string zeros;
    if (assemblyID < 10)
    {
        zeros = "0000";
    }
    else if (assemblyID >= 10 and assemblyID < 100)
    {
        zeros = "000";
    }
    else if (assemblyID >= 100 and assemblyID < 1000)
    {
        zeros = "00";
    }
    else
    {
        zeros = "0";
    }

    OpenMesh::IO::Options ropt;
    ropt += OpenMesh::IO::Options::VertexTexCoord;
    origShellBlock.request_vertex_texcoords2D();

    AddVertexTextureCoord2PolyMesh(origShellBlock);
    OpenMesh::IO::write_mesh(origShellBlock, folderPath + "/shell_block_" + zeros + to_string(assemblyID) + "_" + to_string(clusterID) + ".obj", ropt);
}

void ShellBlock::SaveReplacedShellBlock(string folderPath)
{
    string zeros;
    if (assemblyID < 10)
    {
        zeros = "0000";
    }
    else if (assemblyID >= 10 and assemblyID < 100)
    {
        zeros = "000";
    }
    else if (assemblyID >= 100 and assemblyID < 1000)
    {
        zeros = "00";
    }
    else
    {
        zeros = "0";
    }

    OpenMesh::IO::Options ropt;
    ropt += OpenMesh::IO::Options::VertexTexCoord;
    replacedShellBlock.request_vertex_texcoords2D();

    AddVertexTextureCoord2PolyMesh(replacedShellBlock);
    OpenMesh::IO::write_mesh(replacedShellBlock, folderPath + "/replaced_block_" + zeros + to_string(assemblyID) + "_" + to_string(clusterID) + ".obj", ropt);
}

void ShellBlock::SaveOrigShellBlockWithDigits(string folderPath)
{
    string zeros;
    if (assemblyID < 10)
    {
        zeros = "0000";
    }
    else if (assemblyID >= 10 and assemblyID < 100)
    {
        zeros = "000";
    }
    else if (assemblyID >= 100 and assemblyID < 1000)
    {
        zeros = "00";
    }
    else
    {
        zeros = "0";
    }

    OpenMesh::IO::write_mesh(origShellBlock_digits, folderPath + "/shell_block_" + zeros + to_string(assemblyID) + "_" + to_string(clusterID) + ".obj");
    OpenMesh::IO::write_mesh(origShellBlock_digits_cura, folderPath + "/_cura_shell_block_" + zeros + to_string(assemblyID) + "_" + to_string(clusterID)+ ".obj");
}

void ShellBlock::SaveReplacedShellBlockWithDigits(string folderPath)
{
    string zeros;
    if (assemblyID < 10)
    {
        zeros = "0000";
    }
    else if (assemblyID >= 10 and assemblyID < 100)
    {
        zeros = "000";
    }
    else if (assemblyID >= 100 and assemblyID < 1000)
    {
        zeros = "00";
    }
    else
    {
        zeros = "0";
    }

    OpenMesh::IO::write_mesh(replacedShellBlock_digits, folderPath + "/replaced_block_" + zeros + to_string(assemblyID) + "_" + to_string(clusterID) + ".obj");
    OpenMesh::IO::write_mesh(replacedShellBlock_digits_cura, folderPath + "/_cura_replaced_block_" + zeros + to_string(assemblyID) + "_" + to_string(clusterID) + ".obj");
}

void ShellBlock::SaveOrigShellBlock_overlapped(string folderPath)
{
    string zeros;
    if (assemblyID < 10)
    {
        zeros = "0000";
    }
    else if (assemblyID >= 10 and assemblyID < 100)
    {
        zeros = "000";
    }
    else if (assemblyID >= 100 and assemblyID < 1000)
    {
        zeros = "00";
    }
    else
    {
        zeros = "0";
    }

    OpenMesh::IO::write_mesh(origShellBlockTri_overlap, folderPath + "/shell_block_overlapped_" + zeros + to_string(assemblyID) + ".obj");
}