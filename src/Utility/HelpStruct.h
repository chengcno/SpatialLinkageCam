///////////////////////////////////////////////////////////////
//
// HelpStruct.h
//
//   Common Structures
//
// by Peng SONG ( songpenghit@gmail.com )
//
// 02/Sep/2015
//
// Modified by CHENG 2020/11/04
//
///////////////////////////////////////////////////////////////

#ifndef HELPSTRUCT_H
#define HELPSTRUCT_H

#include <vector>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;

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
    Vector3d v[3];
    Vector3d normal;
    double area;          // Face area
    Vector3d center;

    void Init(Vector3d _v0, Vector3d _v1, Vector3d _v2);
    Triangle & operator=(const Triangle &tri);
    bool IsEqual(Triangle *tri);
    void PrintTriangle();

    void ComputeCenter();
    void ComputeArea();
    void ComputeNormal();
    void CorrectNormal(Vector3d tagtNormal);
};



#endif //HELPSTRUCT_H
