///////////////////////////////////////////////////////////////
//
// MeshObject.h
//
//   An object represented as a group of meshes, mainly for rendering
//
// by Peng Song
//
// 29/Nov/2020
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////////////////


#ifndef _MESH_OBJECT_H
#define _MESH_OBJECT_H

#include <igl/opengl/ViewerData.h>

using namespace std;
using namespace Eigen;

class Mesh;

class MeshObject
{

public:
    MeshObject() {};
    ~MeshObject() {};

    /// Ground
    vector<igl::opengl::ViewerData> CreateGround(Vector3d origin, double size, int gridNum, int sampNum);

    /// Axes
    vector<igl::opengl::ViewerData> CreateAxes(Vector3d origin, double size, int sampNum);

    /// Fuction for testing MeshCreator
    vector<igl::opengl::ViewerData> Function_Test();

};


#endif //_MESH_OBJECT_H
