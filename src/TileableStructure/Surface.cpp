//
// Created by Linsanity on 18/4/22.
//

#include "Surface.h"
#include <OpenMesh/Core/IO/MeshIO.hh>

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

Surface::Surface()
{

}

Surface::~Surface()
{
    ClearSurface();
}

void Surface::ClearSurface()
{
    inputSurfacePolyMesh.clear();
    inputSurfaceTriMesh.clear();
    triPairlist.clear();
}




//**************************************************************************************//
//                              Read and Write Operation
//**************************************************************************************//

void Surface::LoadModel(char *fileName)
{
    OpenMesh::IO::read_mesh(inputSurfacePolyMesh, string(fileName));

    OpenMesh::IO::read_mesh(inputSurfaceTriMesh, string(fileName));
    inputSurfaceTriMesh.triangulate();

    PolyMesh2Matrix(inputSurfaceTriMesh, V, F);

    MoveToCenter(V);

//    LSCM_Parametrization();
    ARAP_Parametrization();

    BuildTriangleBuckets();

    ComputeBBox();

    ComputeuvBBox();

    folderPath = string(fileName);

    surfaceName = string(fileName);

    for (int i = surfaceName.size() - 1; i > 0; --i)
    {
        if (surfaceName[i] == '/')
        {
            surfaceName.erase(surfaceName.begin(), surfaceName.begin() + i + 1);
            break;
        }
    }
}

void Surface::WriteOBJFile(char *fileName)
{

}

void Surface::ComputeBBox()
{
    float minX, minY, minZ, maxX, maxY, maxZ;
    minX = maxX = V(0, 0);
    minY = maxY = V(0, 1);
    minZ = maxZ = V(0, 2);

    for (int i = 1; i < V.rows(); ++i)
    {
        if (maxX < V(i, 0))
            maxX = V(i, 0);
        if (minX > V(i, 0))
            minX = V(i, 0);

        if (maxY < V(i, 1))
            maxY = V(i, 1);
        if (minY > V(i, 1))
            minY = V(i, 1);

        if (maxZ < V(i, 2))
            maxZ = V(i, 2);
        if (minZ > V(i, 2))
            minZ = V(i, 2);
    }

    bbox.minPt = Vector3d(minX, minY, minZ);
    bbox.maxPt = Vector3d(maxX, maxY, maxZ);

//    printf("BBox min:    %f %f %f\n", bbox.minPt[0], bbox.minPt[1], bbox.minPt[2]);
//    printf("BBox max:    %f %f %f\n", bbox.maxPt[0], bbox.maxPt[1], bbox.maxPt[2]);
}

void Surface::ComputeuvBBox()
{
    float minX, minY, minZ, maxX, maxY, maxZ;
    minX = maxX = V_uv(0, 0);
    minY = maxY = V_uv(0, 1);
    minZ = maxZ = V_uv(0, 2);

    for (int i = 1; i < V_uv.rows(); ++i)
    {
        if (maxX < V_uv(i, 0))
            maxX = V_uv(i, 0);
        if (minX > V_uv(i, 0))
            minX = V_uv(i, 0);

        if (maxY < V_uv(i, 1))
            maxY = V_uv(i, 1);
        if (minY > V_uv(i, 1))
            minY = V_uv(i, 1);
    }

    uvBBox.minPt = Vector3d(minX, minY, 0);
    uvBBox.maxPt = Vector3d(maxX, maxY, 0);

//    printf("uvBBox min:    %f %f %f\n", uvBBox.minPt[0], uvBBox.minPt[1], uvBBox.minPt[2]);
//    printf("uvBBox max:    %f %f %f\n", uvBBox.maxPt[0], uvBBox.maxPt[1], uvBBox.maxPt[2]);
}




//**************************************************************************************//
//                              Parametrization
//**************************************************************************************//

void Surface::LSCM_Parametrization()
{
    // Fix two points on the boundary
    VectorXi bnd, b(2, 1);
    igl::boundary_loop(F, bnd);
    b(0) = bnd(0);
    b(1) = bnd(bnd.size() / 2);
    MatrixXd bc(2, 2);
    bc << 0, 0, 1, 0;

    // LSCM parametrization
    igl::lscm(V, F, b, bc, V_uv);
}

void Surface::ARAP_Parametrization()
{
    Eigen::MatrixXd initial_guess;

    // Compute the initial solution for ARAP (harmonic parametrization)
    Eigen::VectorXi bnd;
    igl::boundary_loop(F,bnd);
    Eigen::MatrixXd bnd_uv;
    igl::map_vertices_to_circle(V,bnd,bnd_uv);

    igl::harmonic(V,F,bnd,bnd_uv,1,initial_guess);

    // Add dynamic regularization to avoid to specify boundary conditions
    igl::ARAPData arap_data;
    arap_data.energy = igl::ARAP_ENERGY_TYPE_ELEMENTS;
    arap_data.with_dynamics = true;
    Eigen::VectorXi b  = Eigen::VectorXi::Zero(0);
    Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(0,0);

//    Eigen::VectorXi b  = Eigen::VectorXi::Zero(1000);
//    Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(b.size(),V.cols());

    // Initialize ARAP
    arap_data.max_iter = 1000;
    // 2 means that we're going to *solve* in 2d
    arap_precomputation(V,F,2,b,arap_data);

    // Solve arap using the harmonic map as initial guess
    V_uv = initial_guess;

    arap_solve(bc,arap_data,V_uv);
}




//**************************************************************************************//
//                              Barycentric Interpolation in Buckets
//**************************************************************************************//

void Surface::BuildTriangleBuckets()
{
    /////////////////////////////////////////////////////////////
    // Initialize each bucket location in texture domain

    for (int i=0; i<F.rows(); i++)
    {
        TexGeoTrianglePair tempPair;

        /////////////////////////////////////////////////////////////
        // Compute the texture triangles inside each bucket

        // Get the triangle in texture domain
        Triangle texTriangle;
        for (int j=0; j<3; j++)
        {
            int texIndex = F(i, j);
            texTriangle.v[j](0) = V_uv(texIndex, 0);
            texTriangle.v[j](1) = V_uv(texIndex, 1);
            texTriangle.v[j](2) = 0;
        }

        // Get the triangle in geometrical space
        Triangle geoTriangle;
        for (int j=0; j<3; j++)
        {
            int vertexIndex = F(i, j);
            geoTriangle.v[j](0) = V(vertexIndex, 0);
            geoTriangle.v[j](1) = V(vertexIndex, 1);
            geoTriangle.v[j](2) = V(vertexIndex, 2);
        }

        tempPair.texTriangle = texTriangle;
        tempPair.geoTriangle = geoTriangle;

        triPairlist.push_back( tempPair );
    }
}


