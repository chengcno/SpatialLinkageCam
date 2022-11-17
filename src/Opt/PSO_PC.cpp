//
// Created by cheng on 29/12/21.
// Modified by CHENG 2022/Aug/08
//

#include "PSO_PC.h"
#include "Utility/HelpFunc.h"
#include <ctime>
#include <random>
#include <iostream>
#include <fstream>
#include "Utility/HelpDefine.h"

PSO_PC::PSO_PC()
{
    w = 1;
    c1 = c2 = 2;

    dim = 3 + 3 + 1;
    groupSize = 40;
    itr_max = 200;
    vel_max.resize(dim);

    timeClip = 80;
    //singleCtrNum = 1 + 9 + 5 + 1; // Jaco, s, dis, dis
    numCtrs = 4 + 2;// + timeClip;
    resistR = 2.8;
    resistR2 = 2.8;
    depthL = 8;
}

PSO_PC::~PSO_PC() {

}

void PSO_PC::UpdatePitchC(VectorXd _x)
{
    Vector3d end1 = Vector3d (_x(0), _x(1), _x(2));
    Vector3d end2 = Vector3d (_x(3), _x(4), _x(5));

    Matrix4d rM1, rM2;
    sizeC = alphas.size();
    pc1.clear();
    pc2.clear();
    folAxisAlpha.normalize();
    folAxisGamma.normalize();
    for(int i=0;i<sizeC;i++)
    {
        if(jTypes[0] == 'u')
            rM1 = GetTranslateMatrix(Vector3d(0, _x(6), 0))
              * GetRotationMatrix(alphas[i], folAxisAlpha, folCen1)
              * GetRotationMatrix(betas[i], folAxisBeta, folCen1);
        if(jTypes[0] == 'c')
            rM1 = GetTranslateMatrix(Vector3d(0, _x(6), 0))
                  * GetTranslateMatrix(betas[i]*folAxisAlpha)
                  * GetRotationMatrix(alphas[i], folAxisAlpha, folCen1);

        if(jTypes[4] == 'r')
            rM2 = GetTranslateMatrix(Vector3d(0, _x(6), 0))
              * GetRotationMatrix(gammas[i], folAxisGamma, folCen2);
        if(jTypes[4] == 'p')
            rM2 = GetTranslateMatrix(Vector3d(0, _x(6), 0))
                  * GetTranslateMatrix(gammas[i]*folAxisGamma);

        double theta = i*2*M_PI/sizeC;
        Vector3d pend1, pend2;
        MultiplyPoint(end1, rM1, pend1);
        MultiplyPoint(end2, rM2, pend2);
        ///for manip -theta
        Matrix4d rM = GetRotationMatrix(Vector3d(0, 0, -theta));
        MultiplyPoint(pend1, rM, pend1);
        MultiplyPoint(pend2, rM, pend2);
        pc1.push_back(pend1);
        pc2.push_back(pend2);
    }
    pc1.push_back(pc1[0]);
    pc2.push_back(pc2[0]);
}

double PSO_PC::minialRadius(const vector<Vector3d> &pCur)
{
    double rad;
    Vector3d lin1,lin2;
    lin1 = pCur[0] - pCur[sizeC-1];
    lin2 = pCur[1] - pCur[0];
    double theta = acos(lin1.dot(lin2)/(lin1.norm() * lin2.norm()));
    rad = _MIN(lin2.norm()/tan(theta + 0.00001), lin1.norm()/tan(theta+0.00001));
    for(int i=1;i<sizeC;i++)
    {
        lin1 = pCur[i] - pCur[i-1];
        lin2 = pCur[i+1] - pCur[i];
        theta = acos(lin1.dot(lin2)/(lin1.norm() * lin2.norm()));
        double rad_m = _MIN(lin2.norm()/tan(theta+0.00001), lin1.norm()/tan(theta+0.00001));
        rad = _MIN(rad, rad_m);
    }
    return rad;
}

void PSO_PC::computeFxCtrX(MatrixXd _x)
{
    xFx.resize(groupSize, 1);
    xCtrs.resize(groupSize, numCtrs);

    for(int i=0;i<groupSize;i++)
    {
        VectorXd oneX = _x.row(i).transpose();
        UpdatePitchC(oneX);

        xFx(i,0) = lengPcur(pc1) + lengPcur(pc2) + 6*oneX(6);
        xCtrs(i, 0) = resistR - minialRadius(pc1);
        xCtrs(i, 1) = resistR2 - minialRadius(pc2);

        xCtrs(i,2) = camMaxR(pc1) - folCen1.y() - oneX(6) + depthL;
        xCtrs(i,3) = camMaxR(pc2) - folCen2.y() - oneX(6) + depthL;

        xCtrs(i,4) = 14 - camMinR(pc1);
        xCtrs(i,5) = 14 - camMinR(pc2);


        for(int j=0; j<timeClip;j++) {
            int Id = j * (sizeC / timeClip);
            Matrix4d rM1 = GetTranslateMatrix(Vector3d(0, oneX(6), 0))
                           * GetRotationMatrix(alphas[Id%sizeC], folAxisAlpha, folCen1)
                           * GetRotationMatrix(betas[Id%sizeC], folAxisBeta, folCen1);
            Vector3d pend1;
            MultiplyPoint(Vector3d(oneX(0), oneX(1), oneX(2)), rM1, pend1);

        }

    }
}

void PSO_PC::updateVelocity(MatrixXd &_V, MatrixXd _X)
{
    default_random_engine rng(time(nullptr));
    uniform_real_distribution<double> disRdom1(0, 1);

    for(int i=0;i<groupSize;i++)
    {
        _V.row(i) = w*_V.row(i) + c1*disRdom1(rng)*(pBest.row(i) - _X.row(i))
                    + c2*disRdom1(rng)*(gBest - _X.row(i));
    }

    //min < V < max
    for(int i=0;i<groupSize;i++)
    {
        for(int j=0;j<dim;j++)
        {
            if(_V(i,j) < -vel_max(j))
                _V(i,j) = -vel_max(j);
            if(_V(i,j) > vel_max(j))
                _V(i,j) = vel_max(j);
        }
    }
}

void PSO_PC::init()
{
    boundX_low.resize(dim);
    boundX_up.resize(dim);

    boundX_low[0] = -15;
    boundX_up[0] = 25;
    boundX_low[1] = 15;
    boundX_up[1] = folCen1.y() - 7;
    boundX_low[2] = folCen1.z() - 23;
    boundX_up[2] = folCen1.z() - 10.0;

    boundX_low[3] = -10;
    boundX_up[3] = 10;
    boundX_low[4] = 10;
    boundX_up[4] = folCen2.y() - 10;
    boundX_low[5] = folCen2.z() - 5;
    boundX_up[5] = folCen2.z() + 5;

    cout<< "blow= " <<  boundX_low[5] <<" up =" << boundX_up[5]<<endl;

    boundX_low[6] = -15;
    boundX_up[6] = 150;

    // 60 < y < 1

    vel_max = (boundX_up - boundX_low)*0.15;

    posiX.resize(groupSize, dim);
    velcX.resize(groupSize, dim);
    xCtrs.resize(groupSize, numCtrs);
    xFx.resize(groupSize, 1);

    default_random_engine rng(time(nullptr));
    uniform_real_distribution<double> disRdom1(0, 1);
    for(int i=0;i<groupSize;i++)
    {
        for(int j=0;j<dim;j++)
        {
            velcX(i,j) = 2*vel_max(j)*(disRdom1(rng)-0.5);
            posiX(i,j) = (boundX_up(j) - boundX_low(j))*disRdom1(rng) + boundX_low(j);
        }
    }

    cout<<"finish init PSO" <<endl;
    computeFxCtrX(posiX);

    pBest = posiX;
    pBestFx = xFx;
    pBestCtrs = xCtrs;

    updateGbest();
}

void PSO_PC::updatePosition(MatrixXd _V, MatrixXd &_X)
{
    _X = _X + _V;

    // X in bound
    default_random_engine rng(time(nullptr));
    uniform_real_distribution<double> disRdom1(0, 1);
    for(int i=0;i<groupSize;i++)
    {
        for(int j=0;j<dim;j++)
        {
            if(_X(i,j) < boundX_low(j))
                _X(i, j) = (boundX_up(j) - boundX_low(j))*disRdom1(rng)*0.4 + boundX_low(j);
            else if(_X(i,j) > boundX_up(j))
                _X(i, j) = (boundX_up(j) - boundX_low(j))*disRdom1(rng)*(-0.4) + boundX_up(j);
        }
    }
}

void PSO_PC::updatePbest(MatrixXd pNew, MatrixXd pNewCtrs, MatrixXd pNewFx)
{
    for(int i=0;i<groupSize;i++)
    {
        bool is_better = CompareBetter(pNewCtrs.row(i), pBestCtrs.row(i), pNewFx(i,0), pBestFx(i,0));
        if(is_better)
        {
            pBest.row(i) = pNew.row(i);
            pBestCtrs.row(i) = pNewCtrs.row(i);
            pBestFx(i,0) = pNewFx(i,0);
        }
    }
}

void PSO_PC::updateGbest()
{
    int keyID = 0;
    double preBest = gBestFx;
    for(int i=1;i<groupSize;i++)
    {
        bool is_better = CompareBetter(pBestCtrs.row(i), pBestCtrs.row(keyID),
                                       pBestFx(i,0), pBestFx(keyID,0));
        if(is_better)
            keyID = i;
    }
    gBest = pBest.row(keyID);
    gBestCtrs = pBestCtrs.row(keyID);
    gBestFx = pBestFx(keyID, 0);
    //if(preBest != gBestFx) {
    //    cout << " global BEST update!" << endl;
    //    if(noneZeroCtrs() == 0)
    //        cout<< " all in ctrs" <<endl;
    //}
}

bool PSO_PC::CompareBetter(RowVectorXd ctr_new, RowVectorXd ctr_old, double f_new, double f_old)
{
    /// ToDo; replace with #ctrs!=0, & SUM(L_i*ctr_i) for timeClip
    int sizeC = numCtrs;
    bool is_better = (f_new < f_old);

    for(int i=0;i<sizeC;i++)
    {
        if(ctr_new(i) < 0.000001 && ctr_old(i) >= 0.000001)
        {
            is_better = true;
            break;
        }
        else if(ctr_new(i) >= 0.000001 && ctr_old(i) < 0.000001)
        {
            is_better = false;
            break;
        }
        else if(ctr_new(i) >= 0.000001 && ctr_old(i) >= 0.000001)
        {
            is_better = (ctr_new(i) < ctr_old(i));
            break;
        }
    }
    return is_better;
}

void PSO_PC::outPrint()
{
    cout<<"finish PSO PC"<<endl;
    cout<< "gBest " <<gBest<<endl;
    UpdatePitchC(gBest.transpose());
    cout<< minialRadius(pc1) <<" "<<minialRadius(pc2)<<endl;
}

int PSO_PC::noneZeroCtrs()
{
    int ID = numCtrs;
    for(int i=0;i<numCtrs;i++)
    {
        if(gBestCtrs(i) >= 0.000001)
        {
            ID = i;
            break;
        }
    }
    return numCtrs-ID;
}

int PSO_PC::numNoZeroCtr()
{
    int countNo = 0;
    for(int i=0;i<numCtrs;i++)
    {
        if(gBestCtrs(i) >= 0.000001)
        {
            countNo ++;
        }
    }
    return  countNo;
}

void PSO_PC::oneStep()
{
    updateVelocity(velcX, posiX);
    updatePosition(velcX, posiX);
    computeFxCtrX(posiX);
    updatePbest(posiX, xCtrs, xFx);
    updateGbest();

}

void PSO_PC::Minimal()
{
    auto staTime = system_clock::now();
    cout<<"start minimal "<<endl;
    for(int itr = 0;itr<itr_max;itr++)
    {
        oneStep();
        w = -itr*0.6/itr_max + 1;
    }
    while (noneZeroCtrs() > 0)
    {
        cout<< "none zero constraints num = " << noneZeroCtrs() <<endl;
        for(int itr = 0;itr<itr_max;itr++)
        {
            oneStep();
            w = -itr*0.6/itr_max + 1;
        }
    }
    auto endTime = system_clock::now();
    durationTime = double(duration_cast<microseconds>(endTime - staTime).count()) * microseconds::period::num / microseconds::period::den;
    //cout<< "cost " << durationTime << " sec" <<endl;
    outPrint();
}

void PSO_PC::saveGbest(string fname)
{
    ofstream ifile;
    ifile.open(fname, ios::out);

    // geometry parameters
    for(int i=0;i<dim;i++)
        ifile << gBest(i) << endl;

    // end-effector& TargetCurve
    UpdatePitchC(gBest.transpose());
    ifile << pc1.size() << endl;
    for(auto p : pc1)
        ifile << p.x() << " " << p.y() << " " << p.z() << endl;
    for(auto p : pc2)
        ifile << p.x() << " " << p.y() << " " << p.z() << endl;

    // opt info
    ifile << durationTime <<endl;

    // energy
    ifile << gBestFx << endl;

    // Num ctrs
    ifile << numCtrs << endl;

    // length
    ifile << "leng1 = " << lengPcur(pc1) <<endl;
    ifile << "leng2 = " << lengPcur(pc2) <<endl;
    ifile.close();
}

double PSO_PC::lengPcur(const vector<Vector3d> &pCur)
{
    double len = 0;
    for(int i=0;i<sizeC;i++)
    {
        len += (pCur[i] - pCur[i+1]).norm();
    }
    return len;
}

double PSO_PC::camMaxR(const vector<Vector3d> &pCur)
{
    double r = 0;
    for(int i=0;i<sizeC;i++)
    {
        double r1 = sqrt(pCur[i].x()*pCur[i].x() + pCur[i].y()*pCur[i].y());
        r = _MAX(r, r1);
    }
    return r;
}

double PSO_PC::camMinR(const vector<Vector3d> &pCur)
{
    double r = 1000;
    for(int i=0;i<sizeC;i++)
    {
        double r1 = sqrt(pCur[i].x()*pCur[i].x() + pCur[i].y()*pCur[i].y());
        r = _MIN(r, r1);
    }
    return r;
}