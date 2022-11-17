///////////////////////////////////////////////////////////////
//
// Mesh.h
//
//   libigl mesh data structure
//
// by Yingjie Cheng and Peng Song
//
// 28/Nov/2020
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////////////////


#ifndef _MESH_H
#define _MESH_H

#include <vector>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

struct Triangle;

class Mesh
{
public:
    MatrixX3d verM;                            // Store vertices in a matrix (n,3)
    MatrixX3i triM;                            // Store triangles in a matrix (m,3)
    MatrixX3d norM;                            // Store per triangle normal in a matrix (m,3)

    vector<Triangle*> triangles;               // Triangles (vertex positions)


public:
    Mesh();
    Mesh(vector<Vector3d> verList, vector<Vector3i> triList);
    Mesh(const MatrixXd& _verM, MatrixXi _tirM);
    Mesh(vector<Vector3d> verList, int rows, int cols);
    ~Mesh();
    Mesh* DeepCopy();
    void ClearMesh();

    /// Vector to Eigen Matrix and inverse
    void VerList2VerMat(vector<Vector3d> verList);
    void TriList2TriMat(vector<Vector3i> triList);
    void VerMat2VerList(vector<Vector3d> &verList);
    void TriMat2TriList(vector<Vector3i> &triList);
    void RestoreNormal();


    /// Mesh Operations
    void GetMeshTriangles();
    void ReverseNormal();

    /// Transform Mesh
    void TransformMesh(Matrix4d transMat);                           // Modify mesh vertices
    void TransformMesh(Matrix4d transMat, MatrixXd &newVerM);        // Do not modify mesh vertices

    /// save .obj
    void saveOBJ(string _filename);

    double GetVolume();
};


#endif //_MESH_H