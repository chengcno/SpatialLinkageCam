//////////////////////////////////////////////////
//MeshBoolean.h
//
// MeshBoolean Class, AUB A∆B A\B
//
// Created by Yingjie Cheng on 2020/11/20.
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////

#include "MeshBoolean.h"
#include "Mesh.h"

MeshBoolean::MeshBoolean()
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_UNION;
}

MeshBoolean::MeshBoolean(MatrixXd _VA, MatrixXi _FA, MatrixXd _VB, MatrixXi _FB)
{
    VA = _VA;
    VB = _VB;
    FA = _FA;
    FB = _FB;

    boolean_type = igl::MESH_BOOLEAN_TYPE_UNION;
}
MeshBoolean::~MeshBoolean() {}

///==============================================================================//
///                              Boolean
///==============================================================================//

void MeshBoolean::MeshUnion(MatrixXd &VC, MatrixXi &FC)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_UNION;
    igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,boolean_type,VC,FC,J);
}

void MeshBoolean::MeshUnion(const MatrixXd _VA, const MatrixXi _FA, const MatrixXd _VB,
        const MatrixXi _FB, MatrixXd &VC, MatrixXi &FC)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_UNION;
    igl::copyleft::cgal::mesh_boolean(_VA,_FA,_VB,_FB,boolean_type,VC,FC,J);
}

void MeshBoolean::MeshIntersect(MatrixXd &VC, MatrixXi &FC)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_INTERSECT;
    igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,boolean_type,VC,FC,J);
}

void MeshBoolean::MeshIntersect(const MatrixXd _VA, const MatrixXi _FA,
        const MatrixXd _VB, const MatrixXi _FB, MatrixXd &VC, MatrixXi &FC)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_INTERSECT;
    igl::copyleft::cgal::mesh_boolean(_VA,_FA,_VB,_FB,boolean_type,VC,FC,J);
}

void MeshBoolean::MeshMinus(MatrixXd &VC, MatrixXi &FC)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_MINUS;
    igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,boolean_type,VC,FC,J);
}

void MeshBoolean::MeshMinus(const MatrixXd _VA, const MatrixXi _FA,
        const MatrixXd _VB, const MatrixXi _FB, MatrixXd &VC, MatrixXi &FC)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_MINUS;
    igl::copyleft::cgal::mesh_boolean(_VA,_FA,_VB,_FB,boolean_type,VC,FC,J);
}

Mesh* MeshBoolean::MeshUnion(Mesh *A, Mesh *B)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_UNION;
    MatrixXd VC;
    MatrixXi FC, J;

    igl::copyleft::cgal::mesh_boolean(A->verM, A->triM, B->verM, B->triM, boolean_type, VC, FC, J);
    Mesh *UnionMesh = new Mesh(VC, FC);
    return UnionMesh;
}

Mesh* MeshBoolean::MeshIntersect(Mesh *A, Mesh *B)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_INTERSECT;
    MatrixXd VC;
    MatrixXi FC, J;

    igl::copyleft::cgal::mesh_boolean(A->verM, A->triM, B->verM, B->triM, boolean_type, VC, FC, J);
    Mesh *InterMesh = new Mesh(VC, FC);
    return InterMesh;
}

Mesh* MeshBoolean::MeshMinus(Mesh *A, Mesh *B)
{
    boolean_type = igl::MESH_BOOLEAN_TYPE_MINUS;
    MatrixXd VC;
    MatrixXi FC, J;

    igl::copyleft::cgal::mesh_boolean(A->verM, A->triM, B->verM, B->triM, boolean_type, VC, FC, J);
    Mesh *MinusMesh = new Mesh(VC, FC);
    return MinusMesh;
}

Mesh *MeshBoolean::MeshConnect(Mesh *A, Mesh *B)
{
    MatrixXd VC;
    MatrixXi FC, fB;

    long numVertA, numVertB, numFaceA, numFaceB;
    numVertA = A->verM.rows();
    numVertB = B->verM.rows();
    numFaceA = A->triM.rows();
    numFaceB = B->triM.rows();

    VC.resize(numVertA + numVertB, 3);
    FC.resize(numFaceA + numFaceB, 3);

    VC.block(0,0,numVertA, 3) = A->verM;
    FC.block(0,0,numFaceA, 3) = A->triM;

    VC.block(numVertA, 0, numVertB,3) = B->verM;
    fB.resize(numFaceB, 3);
    fB.setOnes();
    fB = fB * numVertA;
    FC.block(numFaceA, 0, numFaceB, 3) = B->triM + fB;

    Mesh *Mc = new Mesh(VC,FC);
    return Mc;
}

Mesh *MeshBoolean::MeshConnect(vector<Mesh *> meshList)
{
    int size_n = meshList.size();
    if(size_n == 0) cout<< " meshList is Empty!"<<endl;
    Mesh* resM;
    resM = meshList[0];

    for(int i=1;i<size_n;i++)
    {
        resM = MeshConnect(resM, meshList[i]);
    }
    return resM;
}