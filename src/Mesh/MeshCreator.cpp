///////////////////////////////////////////////////////////////
//
// MeshCreator.cpp
//
//   Construct libigl mesh of various primitive shapes
//
// by Yingjie Cheng and Peng Song
//
// 28/Nov/2020
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////////////////


#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include "Mesh.h"
#include "Utility/HelpFunc.h"
#include "MeshCreator.h"
#include "ConVexHull.h"
#include "MeshBoolean.h"


///=========================================================================================///
///                                        Cuboid
///=========================================================================================///

/// Create an axis-aligned cuboid with the given parameters
///   minPt: corner point with the small x, y, z values
///   maxPt: corner point with the large x, y, z values

Mesh* MeshCreator::CreateCuboid(Vector3d minPt, Vector3d maxPt)
{
    // Create a cuboid with the computed size
    Vector3d size = maxPt - minPt;
    Mesh* mesh = CreateCuboid(size);

    // Compute the translation matrix
    Vector3d center = 0.5f * (minPt + maxPt);
    Matrix4d mat = GetTranslateMatrix( center );

    // Translate the cuboid to its computed position
    mesh->TransformMesh( mat );

    return mesh;
}

Mesh* MeshCreator::CreateCuboid(Vector3d size)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the cuboid

    double minX = -0.5f * size[0];
    double minY = -0.5f * size[1];
    double minZ = -0.5f * size[2];

    double maxX =  0.5f * size[0];
    double maxY =  0.5f * size[1];
    double maxZ =  0.5f * size[2];

    verList.emplace_back( minX, minY, maxZ );
    verList.emplace_back( maxX, minY, maxZ );
    verList.emplace_back( maxX, maxY, maxZ );
    verList.emplace_back( minX, maxY, maxZ );

    verList.emplace_back( minX, minY, minZ );
    verList.emplace_back( maxX, minY, minZ );
    verList.emplace_back( maxX, maxY, minZ );
    verList.emplace_back( minX, maxY, minZ );


    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the cuboid

    triList.emplace_back(5,4,7 );
    triList.emplace_back(5,7,6 );
    triList.emplace_back(7,2,6 );
    triList.emplace_back(7,3,2 );

    triList.emplace_back(1,0,4 );
    triList.emplace_back(1,3,0 );
    triList.emplace_back(5,6,2 );
    triList.emplace_back(5,2,1 );

    triList.emplace_back(4,5,1 );
    triList.emplace_back(1,2,3 );
    triList.emplace_back(0,3,4 );
    triList.emplace_back(4,3,7 );


    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the sphere

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}




///=========================================================================================///
///                                         Sphere
///=========================================================================================///

/// Create a sphere with the given parameters
///   radius:     radius of the sphere
///   position:   position of the sphere center
///   polarSamp:  number of samples for the polar angle [-PI/2.0, PI/2.0]
///   azimuSamp:  number of samples for the azimuthal angle [0, 2.0*PI]

Mesh* MeshCreator::CreateSphere(double radius, Vector3d position, int polarSamp, int azimuSamp)
{
    // Create a sphere with the given radius, centered at the origin
    Mesh* mesh = CreateSphere(radius, polarSamp, azimuSamp);

    // Compute the translation matrix
    Matrix4d mat = GetTranslateMatrix( position );

    // Translate the sphere to its target position
    mesh->TransformMesh( mat );

    return mesh;
}

Mesh* MeshCreator::CreateSphere(double radius, int polarSamp, int azimuSamp)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the sphere

    verList.emplace_back(0, 0, radius);
    for(int i = 1; i < polarSamp; i++)
    {
        double alpha = M_PI/2.0 - i*M_PI/polarSamp;
        for(int j = 0; j < azimuSamp; j++)
        {
            double beta = j*2.0*M_PI/azimuSamp;
            double x,y,z;
            x = radius * cos(alpha) * cos(beta);
            y = radius * cos(alpha) * sin(beta);
            z = radius * sin(alpha);
            verList.emplace_back(x, y, z);
        }
    }
    verList.emplace_back(0, 0, -radius);


    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the sphere

    // Top ring of triangles
    for(int j = 0; j < azimuSamp-1; j++)
    {
        triList.emplace_back(0,j+1,j+2);
    }
    triList.emplace_back(0, azimuSamp, 1);

    // Middle rings of triangles
    int StartId = 1;
    for(int i = 1; i < polarSamp-1; i++)
    {
        for(int j = 0; j < azimuSamp-1; j++)
        {
            triList.emplace_back(StartId + j, StartId + azimuSamp + j,     StartId + azimuSamp + j + 1);
            triList.emplace_back(StartId + j, StartId + azimuSamp + j + 1, StartId + 1 + j);
        }
        triList.emplace_back(StartId + azimuSamp - 1, StartId + 2*azimuSamp - 1, StartId + azimuSamp);
        triList.emplace_back(StartId + azimuSamp - 1, StartId + azimuSamp, StartId);
        StartId += azimuSamp;
    }

    // Bottom ring of triangles
    int verNum = verList.size();
    for(int j=0; j<azimuSamp-1; j++)
    {
        triList.emplace_back(verNum - 1, j + 1 + StartId, j + StartId);
    }
    triList.emplace_back(verNum - 1, StartId, StartId + azimuSamp -1);


    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the sphere

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}




///=========================================================================================///
///                                         Cylinder
///=========================================================================================///

/// Create a cylinder with the given parameters
///   capCenterA:  center point of one cap
///   capCenterB:  center point of the other cap
///   radius:      radius of the two caps
///   radSamp:     number of samples for the boundary of each cap

Mesh* MeshCreator::CreateCylinder(Vector3d capCenterA, Vector3d capCenterB, double radius, int radSamp)
{
    // Create a cylinder with the given radius and computed length
    double length = (capCenterB - capCenterA).norm();
    Mesh* mesh = CreateCylinder(length, radius, radSamp);

    // Compute the translation matrix
    Vector3d center = 0.5f * (capCenterA + capCenterB);
    Matrix4d traMat = GetTranslateMatrix( center );

    // Compute the rotation matrix (note: default cylinder is aligned with x-axis)
    Vector3d xAxis = Vector3d(1, 0, 0);
    Vector3d vector = (capCenterB - capCenterA).normalized();
    Vector3d rotAxis = xAxis.cross( vector );
    double dotP = xAxis.dot( vector );
    double rotAngle = acos( dotP );
    Matrix4d rotMat = GetRotationMatrix(rotAngle, rotAxis);

    // if para with x-axis, use y-axis
    if( rotAxis.norm() < 0.0001)
    {
        rotAxis = Vector3d (0,1,0).cross( vector );
        rotAngle = acos( Vector3d(0,1,0).dot(vector));
        rotMat = GetRotationMatrix(rotAngle, rotAxis) * GetRotationMatrix(Vector3d(0,0,M_PI/2));
    }

    // Compute the trannsform matrix on the cylinder
    Matrix4d transMat = traMat * rotMat;

    // Transform the cylinder to its target position and orientation
    mesh->TransformMesh( transMat );

    return mesh;
}


/// Create a cylinder that is centered at the origin and aligned with the x-axis)

Mesh* MeshCreator::CreateCylinder(double length, double radius, int radSamp)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the cylinder

    double halfLen = 0.5f * length;

    // Center point of the right cap
    verList.emplace_back(halfLen, 0, 0);

    // Sampled points on the right cap
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        double x, y, z;
        x = halfLen;
        y = radius * cos(beta);
        z = radius * sin(beta);

        verList.emplace_back(x, y, z);
    }

    // Sampled points on the left cap
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        double x, y, z;
        x = -halfLen;
        y = radius * cos(beta);
        z = radius * sin(beta);

        verList.emplace_back(x, y, z);
    }

    // Center point of the left cap
    verList.emplace_back(-halfLen, 0, 0);


    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the cylinder

    // Triangles of the right cap
    for(int j = 0; j < radSamp - 1; j++)
    {
        triList.emplace_back(0, j+1, j+2);
    }
    triList.emplace_back(0, radSamp, 1);

    // Triangles of the cylinder body
    int StartId = 1;
    for(int j=0;j<radSamp-1;j++)
    {
        triList.emplace_back(StartId + j, StartId + radSamp + j,     StartId + radSamp + j + 1);
        triList.emplace_back(StartId + j, StartId + radSamp + j + 1, StartId + 1 + j);
    }
    triList.emplace_back(StartId+radSamp - 1, StartId + 2*radSamp - 1, StartId + radSamp);
    triList.emplace_back(StartId+radSamp - 1, StartId + radSamp,       StartId);
    StartId += radSamp;

    // Triangles of the left cap
    int verNum = verList.size();
    for(int j = 0; j < radSamp - 1; j++)
    {
        triList.emplace_back(verNum - 1, j + 1 + StartId, j + StartId);
    }
    triList.emplace_back(verNum - 1, StartId, StartId + radSamp - 1);


    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the cylinder

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}




///=========================================================================================///
///                                          Cone
///=========================================================================================///

/// Create a cone with the given parameters
///   baseCenter:  center point of the cone base
///   apexPoint:   point of the cone apex
///   radius:      radius of the cone base
///   radSamp:     number of samples for the base boundary

Mesh* MeshCreator::CreateCone(Vector3d baseCenter, Vector3d apexPoint, double radius, int radSamp)
{
    // Create a cylinder with the given radius and computed length
    double length = (apexPoint - baseCenter).norm();
    Mesh* mesh = CreateCone(length, radius, radSamp);

    // Compute the translation matrix
    Matrix4d traMat = GetTranslateMatrix( baseCenter );

    // Compute the rotation matrix (note: default cylinder is aligned with x-axis)
    Vector3d xAxis = Vector3d(1, 0, 0);
    Vector3d vector = (apexPoint - baseCenter).normalized();
    Vector3d rotAxis = xAxis.cross( vector );
    double dotP = xAxis.dot( vector );
    double rotAngle = acos( dotP );
    Matrix4d rotMat = GetRotationMatrix(rotAngle, rotAxis);

    // Compute the trannsform matrix on the cylinder
    Matrix4d transMat = traMat * rotMat;

    // Transform the cylinder to its target position and orientation
    mesh->TransformMesh( transMat );

    return mesh;
}

Mesh* MeshCreator::CreateCone(Vector3d apexPoint, Vector3d axis_n, double height, double rangeAngle, int radSamp)
{
    double radius = height*tan(rangeAngle/2.0);
    Mesh* mesh = CreateCone(height, radius, radSamp);
    axis_n.normalize();
    Matrix4d rotM = GetRotationMatrix(Vector3d(1,0,0), Vector3d(-axis_n));
    Vector3d centerP = apexPoint + axis_n*height;
    Matrix4d transM = GetTranslateMatrix(centerP);
    mesh->TransformMesh(transM*rotM);
    return mesh;
}

/// Create a cone that is oriented along the +x-axis; its base is centered at the origin

Mesh* MeshCreator::CreateCone(double length, double radius, int radSamp)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the cone

    // Base center point
    verList.emplace_back(0, 0, 0);

    // Sampled points on the base
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        double x, y, z;
        x = 0;
        y = radius * cos(beta);
        z = radius * sin(beta);

        verList.emplace_back(x,y,z);
    }

    // Apex point
    verList.emplace_back(length, 0, 0);


    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the cone

    // Triangles of the base
    for(int j = 0; j < radSamp - 1; j++)
    {
        triList.emplace_back(0, j+2, j+1);
    }
    triList.emplace_back(0, 1, radSamp);

    // Triangles of the cone body
    int StartID = 1+radSamp;
    for(int j = 0; j < radSamp - 1; j++)
    {
        triList.emplace_back(StartID, j+1, j+2);
    }
    triList.emplace_back(StartID, radSamp, 1);


    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the cylinder

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}




///=========================================================================================///
///                                          Fan
///=========================================================================================///

/// Create a fan with the given parameters
///   radius:     large radius of the fan
///   length:     difference between the large radius and the small radius
///   thickness:  thickness of the fan (along y-axis)
///   angle:      opening angle of the fan

Mesh* MeshCreator::CreateFan(double radius, double length, double thickness, double angle)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the fan

    double r = thickness/2.0;

    double x1 = (radius - length) * sin(angle) + r * cos(angle);
    double x2 = 1.1*(radius/cos(angle) + r*tan(angle))*sin(angle) + r*cos(angle);

    double z1 = (radius - length) * cos(angle) - r * sin(angle);
    double z2 = 1.1*(radius/cos(angle) + r*tan(angle))*cos(angle) - r*sin(angle);

    verList.emplace_back( x1, -r,  z1);
    verList.emplace_back( x2, -r,  z2);
    verList.emplace_back(-x2, -r,  z2);
    verList.emplace_back(-x1, -r,  z1);

    verList.emplace_back( x1,  r,  z1);
    verList.emplace_back( x2,  r,  z2);
    verList.emplace_back(-x2,  r,  z2);
    verList.emplace_back(-x1,  r,  z1);


    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the fan

    triList.emplace_back(0, 1, 2);
    triList.emplace_back(0, 2, 3);
    triList.emplace_back(1, 5, 6);
    triList.emplace_back(1, 6, 2);
    triList.emplace_back(0, 4, 5);
    triList.emplace_back(0, 5, 1);
    triList.emplace_back(2, 6, 7);
    triList.emplace_back(2, 7, 3);
    triList.emplace_back(0, 3, 7);
    triList.emplace_back(0, 7, 4);
    triList.emplace_back(4, 7, 6);
    triList.emplace_back(4, 6, 5);


    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the cylinder

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}

Mesh* MeshCreator::CreateFan(double radius, double thick, double start_angle, double end_angle, int radSamp)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the fan
    verList.emplace_back(0, thick/2, 0);
    for(int i=0;i<=radSamp;i++)
    {
        double theta = (end_angle - start_angle)/radSamp * 1.0 * i + start_angle;
        verList.emplace_back(sin(theta)*radius, thick/2, cos(theta)*radius );
    }
    verList.emplace_back(0, -thick/2, 0);
    for(int i=0;i<=radSamp;i++)
    {
        double theta = (end_angle - start_angle)/radSamp * 1.0 * i + start_angle;
        verList.emplace_back(sin(theta)*radius, -thick/2, cos(theta)*radius );
    }

    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the fan
    for(int i=0;i<radSamp;i++)
        triList.emplace_back(0, i+1, i+2);
    for(int i=radSamp+2; i<radSamp*2+2;i++)
        triList.emplace_back(radSamp+2, i+2, i+1);
    for(int i=0;i<=radSamp;i++)
    {
        triList.emplace_back(i, i+radSamp+2, i+radSamp+3);
        triList.emplace_back(i, i+radSamp+3, i+1);
    }
    triList.emplace_back(radSamp+1, radSamp+radSamp+3, radSamp+2);
    triList.emplace_back(radSamp+1, radSamp+2, 0);

    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the cylinder

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}


///=========================================================================================///
///                                          Frustum
///=========================================================================================///
/// Create a Frusrtum
/// centerA : center of bottom circle surface
/// centerB : center of top circle surface
/// rA      : radius of bottom circle
/// rB      : radius of top circle
/// radSamp : number of samples

Mesh *MeshCreator::CreateFrustum(Vector3d centerA, Vector3d centerB, double rA, double rB, int radSamp)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the frustum

    Vector3d normalN = centerB - centerA;
    if(normalN.norm()>0) normalN.normalize();
    Vector3d N_vert1, N_vert2;
    N_vert1 = Vector3d (1,0,1).cross(normalN);
    if(N_vert1.norm() == 0) N_vert1 = Vector3d (0,1,0).cross(normalN);
    N_vert1.normalize();
    N_vert2 = -normalN.cross(N_vert1);

    // Center point of the bottom cap
    verList.push_back(centerA);

    // Sampled points on the bottom cap
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        Vector3d pj;
        pj = centerA + rA*(cos(beta)*N_vert1 + sin(beta)*N_vert2);
        verList.push_back(pj);
    }

    // Sampled points on the left cap
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        Vector3d pj;
        pj = centerB + rB*(cos(beta)*N_vert1 + sin(beta)*N_vert2);
        verList.push_back(pj);
    }

    // Center point of the left cap
    verList.push_back(centerB);


    /////////////////////////////////////////////////////////////////////////
    /// 2. Computer triangles of the frustum

    // Triangles of the right cap
    for(int j = 0; j < radSamp - 1; j++)
    {
        triList.emplace_back(0, j+1, j+2);
    }
    triList.emplace_back(0, radSamp, 1);

    // Triangles of the cylinder body
    int StartId = 1;
    for(int j=0;j<radSamp-1;j++)
    {
        triList.emplace_back(StartId + j, StartId + radSamp + j,     StartId + radSamp + j + 1);
        triList.emplace_back(StartId + j, StartId + radSamp + j + 1, StartId + 1 + j);
    }
    triList.emplace_back(StartId+radSamp - 1, StartId + 2*radSamp - 1, StartId + radSamp);
    triList.emplace_back(StartId+radSamp - 1, StartId + radSamp,       StartId);
    StartId += radSamp;

    // Triangles of the left cap
    int verNum = verList.size();
    for(int j = 0; j < radSamp - 1; j++)
    {
        triList.emplace_back(verNum - 1, j + 1 + StartId, j + StartId);
    }
    triList.emplace_back(verNum - 1, StartId, StartId + radSamp - 1);


    /////////////////////////////////////////////////////////////////////////
    /// 3. Construct a triangular mesh of the frustum

    Mesh *mesh = new Mesh(verList, triList);

    return mesh;
}

Mesh *MeshCreator::CreateFrustum(Vector3d centerA, Vector3d normalA, Vector3d centerB, Vector3d normalB,
                                 double rA, double rB, int radSamp)
{
    vector<Vector3d> verList;
    /////////////////////////////////////////////////////////////////////////
    /// 1. Computer vertices of the frustum

    /// frame (normalA, A_vert1, A_vert2)
    if(normalA.norm()>0) normalA.normalize();
    Vector3d A_vert1, A_vert2;
    A_vert1 = Vector3d (1,0,0).cross(normalA);
    if(A_vert1.norm() == 0) A_vert1 = Vector3d (0,1,0).cross(normalA);
    if(A_vert1.norm() > 0) A_vert1.normalize();
    A_vert2 = normalA.cross(A_vert1);

    // Center point of the bottom cap
    verList.push_back(centerA);

    // Sampled points on the bottom cap
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        Vector3d pj;
        pj = centerA + rA*(cos(beta)*A_vert1 + sin(beta)*A_vert2);
        verList.push_back(pj);
    }

    /// frame (normalB, B_vert1, B_vert2)
    if(normalB.norm()>0) normalB.normalize();
    Vector3d B_vert1, B_vert2;
    B_vert1 = Vector3d (1,0,0).cross(normalB);
    if(B_vert1.norm() == 0) B_vert1 = Vector3d (0,1,0).cross(normalB);
    if(B_vert1.norm() > 0) B_vert1.normalize();
    B_vert2 = normalB.cross(B_vert1);

    // Sampled points on the left cap
    for(int j = 0; j < radSamp; j++)
    {
        double beta = j * 2.0 * M_PI / radSamp;

        Vector3d pj;
        pj = centerB + rB*(cos(beta)*B_vert1 + sin(beta)*B_vert2);
        verList.push_back(pj);
    }

    // Center point of the left cap
    verList.push_back(centerB);

    MatrixXd verM;
    verM.resize(verList.size(),3);
    for(int i=0;i<verList.size();i++)
    {
        verM(i,0) = verList[i].x();
        verM(i,1) = verList[i].y();
        verM(i,2) = verList[i].z();
    }

    /////////////////////////////////////////////////////////////////////////
    /// 2. Construct a triangular mesh of the frustum
    auto conVexHull = new ConVexHull();
    Mesh* mesh = conVexHull->GetConvexHull(verM);
    return mesh;
}

Mesh *MeshCreator::CreateSaddle(int nSize)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    /// vert
    for(int i=0;i<=nSize;i++)
    {
        for(int j=0;j<=nSize;j++)
        {
            double x,y;
            x = 2.0/nSize*i - 1;
            y = 2.0/nSize*i - 1;
            double z = 0.1*(y*y - x*x);
            verList.emplace_back(x,y,z);
        }
    }

    /// tri
    for(int i=0;i<nSize;i++)
    {
        for(int j=0;j<nSize;j++)
        {
            int st = i+j*nSize;
            triList.emplace_back(st, st+1, st+nSize+1);
            triList.emplace_back(st, st+nSize+1, st+nSize);
        }
    }

    Mesh* mesh = new Mesh(verList, triList);
    return mesh;
}

Mesh *MeshCreator::CreateMobius(int nSize)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    double x, y, z;
    for(int i=0;i<2*nSize;i++)
    {
        double t = i * 4.0 * M_PI / (2*nSize);
        x = (2+1*cos(0.5*t))*cos(t);
        y = (2+1*cos(0.5*t))*sin(t);
        z = 1*sin(0.5*t);
        verList.emplace_back(x, y, z);
    }

    for(int i=0;i<nSize-1;i++)
    {
        triList.emplace_back(i, i+nSize, i+nSize+1);
        triList.emplace_back(i, i+nSize+1, i+1);
    }
    triList.emplace_back(nSize-1, 2*nSize-1, 0);
    triList.emplace_back(nSize-1, 0, nSize);

    Mesh* mesh = new Mesh(verList, triList);
    return mesh;
}

Mesh *MeshCreator::Create4Frame(int nSize)
{
    vector<Vector3d> verList;
    vector<Vector3i> triList;

    Vector3d pA, pB, pC, pD;
    pA << 0,0,0;
    pB << 0,1,0;
    pC << sqrt(3)/2, 1.0/2, 0;
    pD << sqrt(3)/6, 1.0/2, sqrt(6)/3;
/*
    Mesh* spA = CreateSphere(rl, pA, nSize, nSize);
    Mesh* spB = CreateSphere(rl, pB, nSize, nSize);
    Mesh* spC = CreateSphere(rl, pC, nSize, nSize);
    Mesh* spD = CreateSphere(rl, pD, nSize, nSize);

    Mesh* cylAB = CreateCylinder(pA, pB, rl, nSize);
    Mesh* cylAC = CreateCylinder(pA, pC, rl, nSize);
    Mesh* cylAD = CreateCylinder(pA, pD, rl, nSize);
    Mesh* cylCB = CreateCylinder(pC, pB, rl, nSize);
    Mesh* cylDB = CreateCylinder(pD, pB, rl, nSize);
    Mesh* cylCD = CreateCylinder(pC, pD, rl, nSize);

    MeshBoolean meshBoolean;
    Mesh* mesh = meshBoolean.MeshConnect
            */
    verList.push_back(pA);
    verList.push_back(pB);
    verList.push_back(pC);
    verList.push_back(pD);

    triList.emplace_back(0, 1, 2);
    triList.emplace_back(0, 3, 1);
    triList.emplace_back(0, 2, 3);
    triList.emplace_back(3, 2, 1);

    Mesh* mesh = new Mesh(verList, triList);
    return mesh;
}

Mesh *MeshCreator::CreateOpenCurve(vector<Vector3d> pList, double radius, int sampleN) {
    /// tangent
    Vector3d verCenter = Vector3d(0, 0, 0);
    //for (const auto &p: pList)
    //    verCenter += p;
    int pNum = pList.size();
    //verCenter = verCenter / pNum + Vector3d(0, 0, 0);
    verCenter = Vector3d (-300, 80, 0);
    Vector3d tanCurve, norCurve;
    vector<Vector3d> tangentPitchCurve;
    vector<Vector3d> normalPitchCurve;

    for (int i = 0; i < pNum - 1; i++) {
        tanCurve = (pList[i + 1] - pList[i]).normalized();
        tangentPitchCurve.push_back(tanCurve);

        norCurve = (pList[i] - verCenter);
        norCurve = norCurve - norCurve.dot(tanCurve) * tanCurve;
        norCurve.normalize();
        normalPitchCurve.push_back(norCurve);
    }
    tangentPitchCurve.push_back((pList[pNum - 1] - pList[pNum - 2]).normalized());
    normalPitchCurve.push_back(((pList[pNum - 1] - verCenter) - (pList[pNum - 1] - verCenter).dot(tanCurve) * tanCurve).normalized());

    ///verList
    vector<Vector3d> G_verList;
    for (int i = 0; i < pNum; i++) {
        Vector3d N_1, N_2;
        N_1 = normalPitchCurve[i];
        N_2 = -tangentPitchCurve[i].cross(N_1);

        for (int j = 0; j < sampleN; j++) {
            Vector3d TubeP;
            double theta = j * 2.0 * M_PI / sampleN;

            TubeP = pList[i] + (radius) * (N_1 * cos(theta) + N_2 * sin(theta));
            G_verList.push_back(TubeP);
        }
    }

    ///triList
    vector<Vector3i> G_triList;
    int StartId = 0;
    for (int itr = 0; itr < pNum - 1; itr++) {
        for (int j = 0; j < sampleN - 1; j++) {
            G_triList.emplace_back(StartId + j, StartId + sampleN + j, StartId + sampleN + 1 + j);
            G_triList.emplace_back(StartId + j, StartId + sampleN + 1 + j, StartId + 1 + j);
        }
        G_triList.emplace_back(StartId + sampleN - 1, StartId + 2 * sampleN - 1, StartId + sampleN);
        G_triList.emplace_back(StartId + sampleN - 1, StartId + sampleN, StartId);
        StartId += sampleN;
    }

    G_verList.push_back(pList[0]);
    G_verList.push_back(pList[pList.size()-1]);

    StartId += sampleN;
    for(int i=0;i<sampleN-1;i++)
        G_triList.emplace_back(StartId, i, i+1);
    G_triList.emplace_back(StartId, sampleN-1, 0);

    StartId++;
    for(int i = 0;i<sampleN-1;i++)
        G_triList.emplace_back(StartId, StartId-sampleN+i, StartId-1-sampleN+i);
    G_triList.emplace_back(StartId, StartId-1-sampleN, StartId-2);


    /// Mesh
    Mesh *curMesh = new Mesh(G_verList, G_triList);
    //Mesh *endSph1 = CreateSphere(radius, pList[0], sampleN, sampleN);
    //Mesh *endSph2 = CreateSphere(radius, pList[pNum - 1], sampleN, sampleN);
    return curMesh;
}