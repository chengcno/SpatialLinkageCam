//
// Created by cheng on 29/4/22.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_READMESH_H
#define SPATIALLINKAGES_READMESH_H

#include "Mesh/Mesh.h"
#include "CamPart/Groove.h"
#include <igl/readOBJ.h>

class ReadMesh {
public:
    ReadMesh();
    ~ReadMesh();

public:
    Mesh* curveM;
    vector<Vector3d> curvePts;
    Mesh* curveNew;
    Mesh* c0Mesh;
    int idKey;

public:
    void ReadMesh2Curve();
    void FindC0Points();
    void ReSample();
    void UniSample();
    void FindCentre();
    void ScaleM(Vector3d scale_V);
    void MoveTo(Vector3d cenNew);


private:
    Vector3d maxV, minV;
    Vector3d centre;
    vector<Vector3d> c0pts;
    vector<int>      c0Id;
    vector<double>   lens;

    double rBall;
    double rCur;

};


#endif //SPATIALLINKAGES_READMESH_H
