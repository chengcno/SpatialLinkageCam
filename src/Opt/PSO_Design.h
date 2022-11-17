//
// Created by cheng on 28/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_PSO_DESIGN_H
#define SPATIALLINKAGES_PSO_DESIGN_H

#include <vector>
#include <Eigen/Eigen>
#include "Opt/LMeigen.h"
#include "Opt/IKLMeigen.h"
#include <string>
#include <chrono>


using namespace std;
using namespace Eigen;
using namespace chrono;

class PSO_Design {
public:
    PSO_Design();
    PSO_Design(int _kind, int _itrMax);
    ~PSO_Design();

public:
    vector<Vector3d> targetCurve;
    int              sizeCurve;
    int              EndEffectorID;

private:
    VectorXd         oneX;      // one-vectors
    vector<Matrix4d> linksMat;

private:
    double w;
    double c1;
    double c2;

    int dim;
    int groupSize;
    int itr_max;
    VectorXd vel_max;
    int numCtrs;
    int timeClip;
    int singleCtrNum;

    int saveID;

public:
    int kind;

    VectorXd boundX_low, boundX_up;
    MatrixXd pBest;     // groupSize x dim
    MatrixXd pBestCtrs; // groupSize x #ctrs
    MatrixXd pBestFx;   // groupSize x 1
    RowVectorXd gBest;
    RowVectorXd gBestCtrs;
    double      gBestFx;

    MatrixXd posiX;
    MatrixXd velcX;     //  groupSize x dim
    MatrixXd xCtrs;     //  groupSize x #ctrs
    MatrixXd xFx;       //  groupSize x 1

    // for one point in PSO
    vector<char>     jTypes;
    vector<Vector3d> jointPos;
    vector<Vector3d> jointOri;
    vector<Matrix4d> jointMat;


public:
    void SetTargetCurve(const vector<Vector3d>& _targetCurve);
    void SetTargetCurve(const MatrixXd &posM);
    void SetEndEffectorID(int id);

    void init();
    bool Minimal();
    void oneStep();
    void saveGbest(string fname);
    void GetGbestInfo(string fname);

    double durationTime;

private:
    bool CompareBetter(RowVectorXd ctr_new, RowVectorXd ctr_old, double f_new, double f_old);
    void outPrint();
    int  noneZeroCtrs();
    int  numNoZeroCtr();

    void computeFxCtrX(MatrixXd _x);

    void updateVelocity(MatrixXd &_V, MatrixXd _X);
    void updatePosition(MatrixXd _V, MatrixXd &_X);

    void updatePbest(MatrixXd pNew, MatrixXd pNewCtrs, MatrixXd pNewFx);
    void updateGbest();

    void SetTopology();
    void SetLMopt();

    void UpdateJointMats(VectorXd _x);

};


#endif //SPATIALLINKAGES_PSO_DESIGN_H
