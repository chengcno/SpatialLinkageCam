//
// Created by cheng on 30/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_CAMP_H
#define SPATIALLINKAGES_CAMP_H


#include <Eigen/Eigen>
#include <vector>
#include "Mesh/Mesh.h"

using namespace Eigen;
using namespace std;

class CamP {
public:
    CamP();
    ~CamP();

public:
    Mesh*               camMesh;
    Mesh*               folMesh;
    double              rCFball, rFol;
    vector<Vector3d>    PitchCurve;
    Vector3d            folCen;
    Vector3d            folAxis;
    vector<double>      folDisp;
    Vector3d            pJoint_0, pAnkle_0;

    Matrix4d            worldMat;
    int                 numN, numM;

private:
    vector<pair<double,Vector3d>>   FollOrient;          // Follower orient
    vector<Quaterniond>             outQuaters;          // follower quats
    vector<double>                  outTrans;            // follower translate

public:
    void CreateCamMesh();
    void CreateFolMesh();
    void ComputeEMechMotion(Matrix4d driMat, Matrix4d &folMat);

private:
    void GetPitchCurve();
    void ComputeFollowOrient();
};


#endif //SPATIALLINKAGES_CAMP_H
