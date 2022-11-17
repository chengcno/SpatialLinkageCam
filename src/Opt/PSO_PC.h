//
// Created by cheng on 29/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_PSO_PC_H
#define SPATIALLINKAGES_PSO_PC_H

#include <vector>
#include <Eigen/Eigen>
#include "Opt/LMeigen.h"
#include "Opt/IKLMeigen.h"
#include <string>
#include <chrono>


using namespace std;
using namespace Eigen;
using namespace chrono;

class PSO_PC {
public:
    PSO_PC();
    ~PSO_PC();

public:
    vector<char> jTypes;
    Vector3d folCen1, folCen2;
    Vector3d folAxisAlpha, folAxisBeta, folAxisGamma;
    vector<double> alphas, betas, gammas;
    vector<Vector3d> pc1, pc2;
    int sizeC;

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

    double resistR;
    double resistR2;
    double depthL;
    //int singleCtrNum;

public:
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

public:
    void init();
    void Minimal();
    void oneStep();
    void saveGbest(string fname);

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

    void UpdatePitchC(VectorXd _x);
    double minialRadius(const vector<Vector3d> &pCur);
    double lengPcur(const vector<Vector3d> &pCur);
    double camMaxR (const vector<Vector3d> &pCur);
    double camMinR(const vector<Vector3d> &pCur);


};


#endif //SPATIALLINKAGES_PSO_PC_H
