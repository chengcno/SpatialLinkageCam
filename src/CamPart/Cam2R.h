//
// Created by cheng on 20/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_CAM2R_H
#define SPATIALLINKAGES_CAM2R_H

#include <Eigen/Eigen>
#include <vector>
#include "Mesh/Mesh.h"

using namespace Eigen;
using namespace std;

class Cam2R {
public:
    Cam2R();
    ~Cam2R();

public:
    Mesh*               camMesh;
    Mesh*               folMesh;
    double              rCFball, rFol;
    vector<Vector3d>    PitchCurve;
    Vector3d            folCen;
    Vector3d            folAxisAlpha, folAxisBeta;
    vector<double>      folAlpha, folBeta;
    Vector3d            pJoint_0, pAnkle_0;

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


#endif //SPATIALLINKAGES_CAM2R_H
