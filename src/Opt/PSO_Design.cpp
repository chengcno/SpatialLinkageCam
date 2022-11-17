//
// Created by temp on 28/12/21.
// Modified by CHENG 2022/Aug/08
//

#include "PSO_Design.h"
#include "Utility/HelpFunc.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include <ctime>
#include <random>
#include <iostream>
#include <fstream>

PSO_Design::PSO_Design()
{
    PSO_Design(1, 100);
}

PSO_Design::PSO_Design(int _kind, int _itrMax)
{
    w = 1;
    c1 = c2 = 2;

    dim = 18+1+2;
    groupSize = 60;
    itr_max = _itrMax;
    vel_max.resize(dim);

    timeClip = 60;
    singleCtrNum = 1 + 9 + 5 + 1 + 2 + 1; // Jaco, s, dis, dis, dis, sing
    numCtrs = singleCtrNum * timeClip;

    saveID = 0;
    kind = _kind;

    SetTopology();
    SetLMopt();
}

PSO_Design::~PSO_Design() {

}

void PSO_Design::SetTargetCurve(const vector<Vector3d>& _targetCurve)
{
    sizeCurve = _targetCurve.size();
    targetCurve = _targetCurve;
}

void PSO_Design::SetTargetCurve(const MatrixXd& _posM)
{
    sizeCurve = _posM.rows();
    targetCurve.clear();
    for(int i=0;i<sizeCurve;i++)
    {
        targetCurve.emplace_back(_posM(i,0),_posM(i,1),_posM(i,2));
    }
}

void PSO_Design::computeFxCtrX(MatrixXd _x)
{
    xFx.resize(groupSize, 1);
    xCtrs.resize(groupSize, numCtrs);

    for(int i=0;i<groupSize;i++)
    {
        oneX = _x.row(i).transpose();
        UpdateJointMats(oneX);

        Vector3d jp1 , jp2 , jp3 ,jp4 ,jp5;
        LMFunctor functorFK;
        LMFunctorIK functorIK;

        functorIK.effector_ID = EndEffectorID;
        functorIK.effector_point = targetCurve[0];


        functorFK.JointInitMat = jointMat;
        functorIK.JointInitMat = jointMat;

        functorFK.m = 15;
        functorFK.n = 9;
        functorIK.m = 15;
        functorIK.n = 9;

        functorFK.JointType = jTypes;
        functorIK.JointType = jTypes;

        functorFK.InputBarX.resize(3);

        VectorXd kM_km, IKM_IKm;// IKuniM_m, FKuniM_m;
        kM_km.resize(timeClip);
        IKM_IKm.resize(timeClip);
        VectorXd preX;
        preX.setConstant(9, 0.0);

        double oriDis = 0;

        for(int j=0; j<timeClip;j++)
        {

            int Id = j*(sizeCurve/timeClip);
            functorIK.InputEndP = targetCurve[Id];
            Eigen::LevenbergMarquardt<LMFunctorIK, double> lmIK(functorIK);

            lmIK.minimize(preX);
            functorIK.LoopMatrix(preX, linksMat);
            VectorXd fX;
            functorIK(preX, fX);
            xCtrs(i, j*singleCtrNum) = fX.norm() - 0.0001;

            Vector3d oriNow;
            MultiplyVector(Vector3d(0,0,-1), linksMat[2], oriNow);
            oriDis += abs(acos(Vector3d(0,0,-1).dot(oriNow)));

            int ttj = 1;
            //j0 = u
            if(jTypes[0] == 'u')
            {
                xCtrs(i, j * singleCtrNum + ttj) = abs(preX(ttj - 1)) - M_PI * 0.15;
                ttj++;
                xCtrs(i, j * singleCtrNum + ttj) = abs(preX(ttj - 1)) - M_PI * 0.2;
                ttj++;
            }
            if(jTypes[0] == 'c')
            {
                xCtrs(i, j * singleCtrNum + ttj) = abs(preX(ttj - 1)) - M_PI * 0.16;
                ttj++;
                xCtrs(i, j * singleCtrNum + ttj) = abs(preX(ttj - 1)) - 25;
                ttj++;
            }

            for(int kkt = 1; kkt<=3;kkt++)
            {
            if(jTypes[kkt] == 'r') {
                xCtrs(i, j * singleCtrNum + ttj) = abs(preX(ttj-1)) - M_PI_2;
                ttj++;
            }
            else if(jTypes[kkt] == 'p')
            {
                xCtrs(i, j * singleCtrNum + ttj) = abs(preX(ttj-1)) - 30;
                ttj++;
            }
            else if(jTypes[kkt] == 's')
            {
                Matrix4d rM = GetRotationMatrix(Vector3d(preX(ttj), preX(ttj+1), preX(ttj+2)));
                Vector3d enP;
                MultiplyVector(Vector3d(1,0,0), rM, enP);
                double coEn = Vector3d (1,0,0).dot(enP);
                xCtrs(i, j*singleCtrNum + ttj) = cos(M_PI*0.305) - coEn; ttj++;
                xCtrs(i, j*singleCtrNum + ttj) = -1; ttj++;
                xCtrs(i, j*singleCtrNum + ttj) = -1; ttj++;
            }
            else if(jTypes[kkt] == 'u')
            {
                xCtrs(i, j*singleCtrNum + ttj) = abs(preX(ttj-1)) - M_PI/2; ttj++;
                xCtrs(i, j*singleCtrNum + ttj) = abs(preX(ttj-1)) - M_PI*0.22; ttj++;
            }
            else if(jTypes[kkt] == 'c')
            {
                xCtrs(i, j*singleCtrNum + ttj) = abs(preX(ttj-1)) - M_PI/2; ttj++;
                xCtrs(i, j*singleCtrNum + ttj) = abs(preX(ttj-1)) - 30; ttj++;
            }
            }

            //j4 = r;
            if(jTypes[4] == 'r')
                xCtrs(i, j*singleCtrNum + ttj) = abs(preX(ttj-1)) - M_PI/4.0;
            else if(jTypes[4] == 'p')
                xCtrs(i, j*singleCtrNum + ttj) = abs(preX(ttj-1)) - 25;


            //dist
            jp1 = jointMat[0].block(0,3,3,1);
            jp2 = jointMat[1].block(0,3,3,1);
            jp3 = jointMat[2].block(0,3,3,1);
            jp4 = jointMat[3].block(0,3,3,1);
            jp5 = jointMat[4].block(0,3,3,1);
            MultiplyPoint(jp1, linksMat[0], jp1);
            MultiplyPoint(jp2, linksMat[1], jp2);
            MultiplyPoint(jp3, linksMat[2], jp3);
            MultiplyPoint(jp4, linksMat[3], jp4);
            MultiplyPoint(jp5, linksMat[4], jp5);
            xCtrs(i, j*singleCtrNum + 15) = 50 - jp3.y();

            xCtrs(i, j*singleCtrNum + 10) = 35 - (jp1 - jp3).norm();
            xCtrs(i, j*singleCtrNum + 11) = 35 - (jp1 - jp4).norm();
            xCtrs(i, j*singleCtrNum + 12) = 35 - (jp2 - jp4).norm();
            xCtrs(i, j*singleCtrNum + 13) = 35 - (jp2 - jp5).norm();
            xCtrs(i, j*singleCtrNum + 14) = 35 - (jp3 - jp5).norm();

            xCtrs(i, j*singleCtrNum + 16) = 35 - (jp2 - jp3).norm();
            xCtrs(i, j*singleCtrNum + 17) = 35 - (jp4 - jp3).norm();

            MatrixXd jacIK;
            functorIK.df(preX, jacIK);

            JacobiSVD<MatrixXd> svdIK(jacIK, ComputeThinU | ComputeThinV);
            MatrixXd ValIK = svdIK.singularValues();
            IKM_IKm(j) = ValIK.maxCoeff() / ValIK.minCoeff();


            vector<double> inputBar;
            inputBar.push_back(preX(0));
            inputBar.push_back(preX(1));
            inputBar.push_back(preX(8));
            functorFK.InputBarX = inputBar;
            MatrixXd jacFK;
            functorFK.df(preX, jacFK);
            MatrixXd jacM = jacFK.block(0, 2, 12, 6);
            JacobiSVD<MatrixXd> svdFK(jacM, ComputeThinU | ComputeThinV);
            MatrixXd ValFK = svdFK.singularValues();

            kM_km(j) = ValFK.maxCoeff() / ValFK.minCoeff();

            xCtrs(i, j*singleCtrNum + 18) = 0.01 - ValFK.minCoeff();
        }
        double l1 =(jp1-jp2).norm() ;
        double l2 = (jp3-jp2).norm() ;
        double l3 = (jp3-jp4).norm() ;
        double l4 = (jp5-jp4).norm() ;
        double l5 =  (jp5 - jp1).norm();

        double avg = (l1+l2+l3+l4+l5)/5.0;
        double var = pow(l1-avg, 2)+ pow(l2-avg, 2)+pow(l3-avg, 2)+pow(l4-avg, 2)+pow(l5-avg, 2);
        var = sqrt(0.2*var);

        xFx(i,0) =
                0*oriDis + 1*kM_km.maxCoeff() + 4*IKM_IKm.maxCoeff()
                + (avg)*5
                + 5*var;
    }
}

void PSO_Design:: SetTopology()
{
    jTypes.clear();
    jTypes.push_back('u');

    switch (kind) {
        case 1:
            jTypes.push_back('s');
            jTypes.push_back('r');
            jTypes.push_back('u');
            break;

        case 2:
            jTypes.push_back('s');
            jTypes.push_back('u');
            jTypes.push_back('r');
            break;

        case 3:
            jTypes.push_back('u');
            jTypes.push_back('s');
            jTypes.push_back('r');
            break;

        case 4:
            jTypes.push_back('u');
            jTypes.push_back('r');
            jTypes.push_back('s');
            break;

        case 5:
            jTypes.push_back('r');
            jTypes.push_back('s');
            jTypes.push_back('u');
            break;

        case 6:
            jTypes.push_back('r');
            jTypes.push_back('u');
            jTypes.push_back('s');
            break;

        case 7:
            jTypes.push_back('u');
            jTypes.push_back('u');
            jTypes.push_back('u');
            break;

        default:
            break;
    }
    jTypes.push_back('r');
}

void PSO_Design::SetLMopt()
{
    jointPos.clear();
    jointPos.emplace_back(0, 40, 30);
    jointPos.emplace_back(-10, 75, 35);
    jointPos.emplace_back(0, 110, 0);
    jointPos.emplace_back(-10, 80, -25);
    jointPos.emplace_back(-15, 40, -30);

    jointOri.clear();
    jointOri.emplace_back(0,M_PI,-M_PI_2);
    jointOri.emplace_back(0,M_PI*1.2,M_PI_2);
    jointOri.emplace_back(0,M_PI_2,M_PI);
    jointOri.emplace_back(0,0,-M_PI_2);
    jointOri.emplace_back(0,M_PI,M_PI_2);

    jointMat.clear();
    for(int i=0;i<5;i++)
    {
        jointMat.emplace_back(GetTranslateMatrix(jointPos[i])* GetRotationMatrix(jointOri[i]));
    }

}

void PSO_Design::UpdateJointMats(VectorXd _x)
{
    jointPos.clear();
    jointPos.emplace_back(0, 40, 30);
    for(int i=0;i<3;i++)
        jointPos.emplace_back(_x(3*i), _x(3*i+1), _x(3*i+2));
    jointPos.emplace_back(-15, 40, -30);

    jointOri.clear();
    if(jTypes[0] == 'u')
        jointOri.emplace_back(0,M_PI,-M_PI_2);
    else if(jTypes[0] == 'c')
        jointOri.emplace_back(0,-M_PI_2, M_PI_2);

    for(int i=3;i<6;i++)
        jointOri.emplace_back(_x(3*i), _x(3*i+1), _x(3*i+2));

    if(jTypes[4] == 'r')
        jointOri.emplace_back(0,M_PI,M_PI_2);
    else if(jTypes[4] == 'p')
        jointOri.emplace_back(0, M_PI_2 ,0);


    jointMat.clear();
    for(int i=0;i<5;i++)
    {
        jointMat.emplace_back(GetTranslateMatrix(jointPos[i])* GetRotationMatrix(jointOri[i]));
    }
    if(jTypes[4] == 'r')
        jointMat[4] = GetRotationMatrix(_x(18), Vector3d(0,0,1), Vector3d(-15,40,-30))
            *jointMat[4];
    if(jTypes[4] == 'p')
        jointMat[4] = GetRotationMatrix(_x(18), Vector3d(0,1,0), Vector3d(-15,40,-30))
                      *jointMat[4];
    jointMat[4] = GetTranslateMatrix(Vector3d(_x(19), 0, _x(20)))*jointMat[4];
}

void PSO_Design::updateVelocity(MatrixXd &_V, MatrixXd _X)
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

void PSO_Design::init()
{
    boundX_low.resize(dim);
    boundX_up.resize(dim);

    //-40 < x < 40
    double x_bd = 100;
    boundX_low(0) = -x_bd;
    boundX_low(3) = -x_bd;
    boundX_low(6) = -x_bd;
    boundX_up(0) = x_bd;
    boundX_up(3) = x_bd;
    boundX_up(6) = x_bd;

    // 60 < y < 140
    double y_lo = 40;
    double y_up = 200;
    boundX_low(1) = y_lo;
    boundX_low(4) = y_lo;
    boundX_low(7) = y_lo;
    boundX_up(1) = y_up;
    boundX_up(4) = y_up;
    boundX_up(7) = y_up;

    // -20 < z1 < 60; -40 < z2 < 40; -70 < z3 < 20;
    boundX_low(2) = -60;
    boundX_low(5) = -80;
    boundX_low(8) = -100;
    boundX_up(2) = 100;
    boundX_up(5) = 80;
    boundX_up(8) = 60;


    // -2Pi < angle < 2Pi;
    for(int i=9;i<18;i++)
    {
        boundX_low(i) = -2*M_PI;
        boundX_up(i) = 2*M_PI;
    }

    // [pi/8]
    boundX_low[18] = -M_PI/5;
    boundX_up[18] = M_PI/5;
    // -10 < J4x < 10
    boundX_low[19] = -10;
    boundX_up[19] = 10;
    // -40 < J4z < 15
    boundX_low[20] = -40;
    boundX_up[20] = 15;

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

void PSO_Design::updatePosition(MatrixXd _V, MatrixXd &_X)
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

void PSO_Design::updatePbest(MatrixXd pNew, MatrixXd pNewCtrs, MatrixXd pNewFx)
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

void PSO_Design::updateGbest()
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
    if(preBest != gBestFx) {
        cout << " global BEST update!" << endl;
        if(noneZeroCtrs() == 0)
        {
            cout << " all in ctrs" << endl;
            //saveGbest("../data/0123/mid_0/kine_1_"+ to_string(saveID)+".txt");
            saveID++;
        }
    }
}

bool PSO_Design::CompareBetter(RowVectorXd ctr_new, RowVectorXd ctr_old, double f_new, double f_old)
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

void PSO_Design::outPrint()
{

}

int PSO_Design::noneZeroCtrs()
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

int PSO_Design::numNoZeroCtr()
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

void PSO_Design::oneStep()
{
    updateVelocity(velcX, posiX);

    updatePosition(velcX, posiX);

    computeFxCtrX(posiX);

    updatePbest(posiX, xCtrs, xFx);

    updateGbest();
}

bool PSO_Design::Minimal()
{
    auto staTime = system_clock::now();
    cout<<"start minimal "<<endl;
    for(int itr = 0;itr<itr_max;itr++)
    {
        oneStep();
        w = -itr*0.6/itr_max + 1;
        cout<<"step " << itr << "/" << itr_max <<endl;
    }
    int ids = 0;
    while (noneZeroCtrs() > 0 && ids < 3)
    {
        cout<< "none zero constraints num = " << noneZeroCtrs() <<endl;
        for(int itr = 0;itr<itr_max;itr++)
        {
            oneStep();
            w = -itr*0.6/itr_max + 1;
            cout<<"step " << itr << "/" << itr_max <<endl;
        }
        ids++;
    }
    auto endTime = system_clock::now();
    durationTime = double(duration_cast<microseconds>(endTime - staTime).count()) * microseconds::period::num / microseconds::period::den;
    cout<< "cost " << durationTime << " sec" <<endl;
    outPrint();

    if(ids == 3)
        return false;
    else
        return true;
}

void PSO_Design::SetEndEffectorID(int id)
{
    EndEffectorID = id;
}

void PSO_Design::saveGbest(string fname)
{
    ofstream ifile;
    ifile.open(fname, ios::out);

    // topology
    for(auto j : jTypes)
    {
        switch (j)
        {
            case 'r':
                ifile << 1 <<endl;
                break;

            case 'p':
                ifile << 2 <<endl;
                break;

            case 'u':
                ifile << 3 << endl;
                break;

            case 'c':
                ifile << 4 <<endl;
                break;

            case 's':
                ifile << 5 <<endl;

            default:
                ifile << 0 <<endl;
                break;
        }
        cout<<j;
    }
    cout<<endl;

    // geometry mat
    UpdateJointMats(gBest.transpose());
    for(int i=0;i<5;i++)
    {
        for(int ii=0;ii<4;ii++)
        {
            for(int jj=0;jj<4;jj++)
            {
                ifile << jointMat[i](ii,jj) << " ";
            }
            ifile << "\n";
        }
    }

    // geometry parameters
    for(int i=0;i<dim;i++)
        ifile << gBest(i) << endl;

    // end-effector& TargetCurve
    ifile << targetCurve.size() << endl;
    for(auto p : targetCurve)
        ifile << p.x() << " " << p.y() << " " << p.z() << endl;

    // opt info
    ifile <<durationTime <<endl;

    // energy
    ifile << gBestFx << endl;

    // Num ctrs
    ifile << numCtrs << endl;

    //EEFid
    ifile << EndEffectorID << endl;

    ifile.close();

    GetGbestInfo(fname);
}

void PSO_Design::GetGbestInfo(string fname)
{
    ofstream ifile;
    ifile.open(fname, ios::app);

    UpdateJointMats(gBest.transpose());

    Vector3d jp1 , jp2 , jp3 ,jp4 ,jp5;

    LMFunctor functorFK;
    LMFunctorIK functorIK;
    LMFunctorIK functorIk_uni;
    functorIK.effector_ID = EndEffectorID;
    functorIK.effector_point = targetCurve[0];

    functorIk_uni.effector_ID = EndEffectorID;
    functorIk_uni.effector_point = targetCurve[0]/80;

    functorFK.JointInitMat = jointMat;
    functorIK.JointInitMat = jointMat;

    jp1 = jointMat[0].block(0,3,3,1);
    jp2 = jointMat[1].block(0,3,3,1);
    jp3 = jointMat[2].block(0,3,3,1);
    jp4 = jointMat[3].block(0,3,3,1);
    jp5 = jointMat[4].block(0,3,3,1);
    double l1 =(jp1-jp2).norm() ;
    double l2 = (jp3-jp2).norm() ;
    double l3 = (jp3-jp4).norm() ;
    double l4 = (jp5-jp4).norm() ;
    double l5 =  (jp5 - jp1).norm();

    double avg = (l1+l2+l3+l4+l5)/5.0;
    double var = pow(l1-avg, 2)+ pow(l2-avg, 2)+pow(l3-avg, 2)+pow(l4-avg, 2)+pow(l5-avg, 2);
    var = sqrt(0.2*var);

    functorIk_uni.JointInitMat.clear();
    for(auto p : jointMat)
    {
        p(0, 3) = p(0, 3)/80;
        p(1, 3) = p(1, 3)/80;
        p(2, 3) = p(2, 3)/80;
        functorIk_uni.JointInitMat.push_back(p);
    }

    functorFK.m = 15;
    functorFK.n = 9;
    functorIK.m = 15;
    functorIK.n = 9;

    functorIk_uni.m = 15;
    functorIk_uni.n = 9;

    functorFK.JointType = jTypes;
    functorIK.JointType = jTypes;

    functorIk_uni.JointType = jTypes;

    functorFK.InputBarX.resize(3);

    VectorXd kM_km, IKM_IKm, IKuniM_m, FKuniM_m, minFKkesi, minFKUnikesi;
    kM_km.resize(timeClip);
    IKM_IKm.resize(timeClip);
    IKuniM_m.resize(timeClip);
    FKuniM_m.resize(timeClip);
    minFKkesi.resize(timeClip);
    minFKUnikesi.resize(timeClip);
    VectorXd preX;
    preX.setConstant(9, 0.0);

    for(int j=0; j<timeClip;j++)
    {

        int Id = j*(sizeCurve/timeClip);
        functorIK.InputEndP = targetCurve[Id];
        Eigen::LevenbergMarquardt<LMFunctorIK, double> lmIK(functorIK);

        lmIK.minimize(preX);
        functorIK.LoopMatrix(preX, linksMat);

        MatrixXd jacIK;
        functorIK.df(preX, jacIK);
        JacobiSVD<MatrixXd> svdIK(jacIK, ComputeThinU | ComputeThinV);
        MatrixXd ValIK = svdIK.singularValues();
        IKM_IKm(j) = ValIK.maxCoeff() / ValIK.minCoeff();

        VectorXd uniPreX;
        uniPreX = preX;
        int ttk = 0;
        if(jTypes[0] == 'c')
            uniPreX(1) = uniPreX(1)/80;
        ttk += 2;
        for(int l = 1;l<4;l++)
        {
            if(jTypes[l] == 'r')
                ttk++;
            if(jTypes[l] == 'p')
            {
                uniPreX(ttk) = uniPreX(ttk)/80;
                ttk++;
            }
            if(jTypes[l] == 'u')
                ttk += 2;
            if(jTypes[l] == 'c')
            {
                uniPreX(ttk + 1) = uniPreX(ttk+1)/80;
                ttk += 2;
            }
            if(jTypes[l] == 's')
                ttk += 3;
        }
        if(jTypes[4] == 'p')
            uniPreX(ttk) = uniPreX(ttk)/80;

        MatrixXd uniJaco;
        functorIk_uni.df(uniPreX, uniJaco);

        for(int l = 9; l < 15; l++)
        {
            uniJaco.row(l) = uniJaco.row(l)*60;
        }
        JacobiSVD<MatrixXd> svdUni(uniJaco, ComputeThinU | ComputeThinV);
        MatrixXd ValUni = svdUni.singularValues();
        IKuniM_m(j) = ValUni.maxCoeff()/ ValUni.minCoeff();

        MatrixXd uniFKJac = uniJaco.block(0, 2, 12, 6);
        JacobiSVD<MatrixXd> svdFKUni(uniFKJac, ComputeThinU | ComputeThinV);
        MatrixXd ValFKUni = svdFKUni.singularValues();
        FKuniM_m(j) = ValFKUni.maxCoeff() / ValFKUni.minCoeff();
        minFKUnikesi(j) = ValFKUni.minCoeff();

        vector<double> inputBar;
        inputBar.push_back(preX(0));
        inputBar.push_back(preX(1));
        inputBar.push_back(preX(8));
        functorFK.InputBarX = inputBar;
        MatrixXd jacFK;
        functorFK.df(preX, jacFK);
        MatrixXd jacM = jacFK.block(0, 2, 12, 6);
        JacobiSVD<MatrixXd> svdFK(jacM, ComputeThinU | ComputeThinV);
        MatrixXd ValFK = svdFK.singularValues();

        kM_km(j) = ValFK.maxCoeff() / ValFK.minCoeff();
        minFKkesi(j) = ValFK.minCoeff();

    }
    ifile << "ValFK = " << kM_km.maxCoeff() << endl;
    ifile << "ValIK = " << IKM_IKm.maxCoeff() << endl;
    ifile << "ValUni = " << FKuniM_m.maxCoeff() <<endl;
    ifile << "ValIKUni = " << IKuniM_m.maxCoeff() << endl;

    ifile << "min kesi = " << minFKkesi.minCoeff() << endl;
    ifile << "min Unikesi = " << minFKUnikesi.minCoeff() << endl;

    ifile << "len_avg = " <<avg <<endl;
    ifile << "len_var = " <<var <<endl;

    ifile.close();
}
