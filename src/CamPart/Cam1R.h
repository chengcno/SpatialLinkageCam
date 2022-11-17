//
// Created by cheng on 20/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_CAM1R_H
#define SPATIALLINKAGES_CAM1R_H

#include <Eigen/Eigen>
#include <vector>
#include "Mesh/Mesh.h"

using namespace Eigen;
using namespace std;

class Cam1R {
public:
    Cam1R();
    ~Cam1R();

public:
    Mesh*               camMesh;
    Mesh*               folMesh;
    double              rCFball, rFol;
    vector<Vector3d>    PitchCurve;
    Vector3d            folCen;
    Vector3d            folAxisAlpha;
    vector<double>      folAlpha;
    Vector3d            pJoint_0, pAnkle_0;

    Matrix4d            worldMat;
    int                 numN, numM;

private:
    vector<pair<double,Vector3d>>   FollOrient;          // Follower orient
    vector<Quaterniond>             outQuaters;          // follower quats

public:
    void CreateCamMesh();
    void CreateFolMesh();
    void ComputeEMechMotion(Matrix4d driMat, Matrix4d &folMat);

private:
    void GetPitchCurve();
    void ComputeFollowOrient();
};


#endif //SPATIALLINKAGES_CAM1R_H
