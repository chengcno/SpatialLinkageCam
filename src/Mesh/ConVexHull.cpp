//
// Created by cheng on 7/7/21.
// Modified by CHENG 2022/Aug/08
//

#include "ConVexHull.h"

///////////////////////////// Init /////////////////////////////////

ConVexHull::ConVexHull()
{
    modelM = nullptr;
    convexM = nullptr;
}

ConVexHull::ConVexHull(Mesh *_modelM)
{
    modelM = _modelM->DeepCopy();
    convexM = nullptr;
}

ConVexHull::~ConVexHull()
{
    if(!modelM)
        modelM->ClearMesh();
    if(!convexM)
        convexM->ClearMesh();
}



///////////////////////////// Convex Hull /////////////////////////////////

Mesh *ConVexHull::GetConvexHull(Mesh *_modelM)
{
    modelM = _modelM->DeepCopy();

    MatrixXd reVerM;
    MatrixXi reTriM;
    igl::copyleft::cgal::convex_hull(modelM->verM, reVerM, reTriM);

    convexM = new Mesh(reVerM, reTriM);
    convexM->GetMeshTriangles();

    return convexM;
}

Mesh *ConVexHull::GetConvexHull(MatrixXd _verM)
{
    MatrixXd reVerM;
    MatrixXi reTriM;
    igl::copyleft::cgal::convex_hull(_verM, reVerM, reTriM);

    Mesh* reM = new Mesh(reVerM, reTriM);

    return reM;
}

Mesh *ConVexHull::GetConvexHull(vector<Vector3d> verList)
{
    MatrixXd reVerM;
    reVerM.resize(verList.size(), 3);
    for(int i=0;i<verList.size();i++)
    {
        reVerM(i,0) = verList[i].x();
        reVerM(i,1) = verList[i].y();
        reVerM(i,2) = verList[i].z();
    }
    return GetConvexHull(reVerM);
}

void ConVexHull::GetConvexHull_2D(MatrixXd _verM, MatrixXd& re_verM)
{
    MatrixXi reLineM;
    igl::copyleft::cgal::convex_hull(_verM, re_verM, reLineM);
}
