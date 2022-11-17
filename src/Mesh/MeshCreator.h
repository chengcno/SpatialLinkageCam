///////////////////////////////////////////////////////////////
//
// MeshCreator.h
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


#ifndef _MESH_CREATOR_H
#define _MESH_CREATOR_H


using namespace std;
using namespace Eigen;

class Mesh;

class MeshCreator{

public:
    MeshCreator() {};
    ~MeshCreator() {};

    /// Cuboid
    Mesh* CreateCuboid(Vector3d minPt, Vector3d maxPt);
    Mesh* CreateCuboid(Vector3d size);

    /// Sphere
    Mesh* CreateSphere(double radius, Vector3d position, int polarSamp, int azimuSamp);
    Mesh* CreateSphere(double radius, int polarSamp, int azimuSamp);

    /// Cylinder
    Mesh* CreateCylinder(Vector3d capCenterA, Vector3d capCenterB, double radius, int radSamp);
    Mesh* CreateCylinder(double length, double radius, int radSamp);

    /// Cone
    Mesh* CreateCone(Vector3d baseCenter, Vector3d apexPoint, double radius, int radSamp);
    Mesh* CreateCone(double length, double radius, int radSamp);
    Mesh* CreateCone(Vector3d apexPoint, Vector3d axis_n, double height, double rangeAngle, int radSamp);

    /// Fan
    Mesh* CreateFan(double radius, double length, double thickness, double angle);
    Mesh* CreateFan(double radius, double thick, double start_angle,double end_angle, int radSamp);

    /// Round table
    Mesh* CreateFrustum(Vector3d centerA, Vector3d centerB, double rA, double rB, int radSamp);
    Mesh* CreateFrustum(Vector3d centerA, Vector3d normalA, Vector3d centerB, Vector3d normalB, double rA, double rB, int radSamp);

    /// z = y^2 - x^2
    Mesh* CreateSaddle(int nSize);
    Mesh* CreateMobius(int nSize);
    Mesh* Create4Frame(int nSize);

    /// tube curve
    Mesh* CreateOpenCurve(vector<Vector3d> pList, double radius, int sampleN);
};


#endif //_MESH_CREATOR_H
