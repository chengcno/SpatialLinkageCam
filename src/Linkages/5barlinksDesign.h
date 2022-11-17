//
// Created by cheng on 28/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_FIVEDESIGN_H
#define SPATIALLINKAGES_FIVEDESIGN_H


#include <vector>
#include <Eigen/Eigen>
#include "Mesh/Mesh.h"
#include "Opt/LMeigen.h"
#include "Opt/IKLMeigen.h"
#include "Opt/PSO_Design.h"
#include <string>

using namespace std;
using namespace Eigen;

class fiveDesign {
public:
    fiveDesign();
    ~fiveDesign();

public:
    vector<Vector3d> jointPos;
    vector<Vector3d> jointOri;
    vector<Matrix4d> jointMat;

    vector<Mesh*> linksMesh;
    vector<Matrix4d> linksMat;

    vector<Vector3d> endCurveA;
    Vector3d EndA;

    vector<Vector3d> targetCurve;

    LMFunctor functor;
    LMFunctorIK IKfunc;
    int         EEFid;

    PSO_Design* _designPSO;
    VectorXd    preX;
    vector<char> jTypes;

    // one circle motion
    string gBestFile;

public:
    void SetTopologyANDitr(int k, int maxTimes);
    void initKineOpt();
    void CreateModel();

    void InputDriver(int frame);
    void UpdateModel(int link_ID, MatrixXd &VerM);

    void SetTargetC(MatrixXd posM);
    bool OptimizePSO();
};


#endif //SPATIALLINKAGES_FIVEDESIGN_H
