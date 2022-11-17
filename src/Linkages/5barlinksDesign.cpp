//
// Created by cheng on 27/12/21.
// Modified by CHENG 2022/Aug/08
//

#include "5barlinksDesign.h"
#include "Mesh/MeshBoolean.h"
#include "Mesh/MeshCreator.h"
#include "Utility/HelpFunc.h"
#include "Joint.h"
#include "Mesh/ConVexHull.h"
#include "fstream"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Utility/HelpDefine.h"

fiveDesign::fiveDesign()
{
    auto _pso = new PSO_Design();
    _designPSO = _pso;
    EEFid = 2;
}

fiveDesign::~fiveDesign()
{

}

void fiveDesign::SetTopologyANDitr(int k, int maxTimes)
{
    _designPSO = new PSO_Design(k, maxTimes);
}

void fiveDesign::SetTargetC(MatrixXd posM)
{

    int pNum = posM.rows();
    cout<< "pos rows = " << pNum <<endl;
    targetCurve.clear();
    for(int i=0;i<pNum;i++)
        targetCurve.emplace_back(posM(i,0), posM(i,1), posM(i,2));

    _designPSO->SetEndEffectorID(EEFid);
    _designPSO->SetTargetCurve(posM);

}

bool fiveDesign::OptimizePSO()
{
    _designPSO->init();
    if(!_designPSO->Minimal())
        return false;
    _designPSO->saveGbest(gBestFile);

    jTypes = _designPSO->jTypes;
    jointMat = _designPSO->jointMat;
    return true;
}

void fiveDesign::initKineOpt()
{
    functor.m = 15;
    functor.n = 9;
    IKfunc.m = 15;
    IKfunc.n = 9;

    functor.JointType = jTypes;
    IKfunc.JointType = jTypes;

    functor.JointInitMat = jointMat;
    functor.InputBarX.resize(3);
    IKfunc.JointInitMat = jointMat;

    preX.resize(functor.n);
    preX.setZero();
}

void fiveDesign::CreateModel()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double rl = 2.1; //3.5
    int N = CSG_NUM_N;

    auto uJ1 = new Joint(jTypes[0]);
    uJ1->CreateSimpleModel();
    uJ1->InitPose(0,1,jointMat[0]);

    auto rJ2 = new Joint(jTypes[1]);
    rJ2->CreateSimpleModel();
    rJ2->InitPose(2, 1, jointMat[1]);

    auto sJ3 = new Joint(jTypes[2]);
    sJ3->CreateSimpleModel();
    sJ3->InitPose(2,3,jointMat[2]);

    auto uJ4 = new Joint(jTypes[3]);
    uJ4->CreateSimpleModel();
    uJ4->InitPose(3, 4, jointMat[3]);

    auto rJ5 = new Joint(jTypes[4]);
    rJ5->CreateSimpleModel();
    rJ5->InitPose(4, 0, jointMat[4]);

    Mesh* lM0 = meshCreator.CreateCylinder(uJ1->partB_touch, rJ5->partB_touch, rl, N);
    //Mesh* lM1 = meshCreator.CreateCylinder(uJ1->partA_touch, rJ2->partB_touch, rl ,N);
    //Mesh* lM2 = meshCreator.CreateCylinder(rJ2->partA_touch, sJ3->partA_touch, rl, N);
    Mesh* lM1 = meshCreator.CreateCylinder(uJ1->partA_touch, rJ2->partA_touch, rl ,N);
    Mesh* lM2 = meshCreator.CreateCylinder(rJ2->partB_touch, sJ3->partA_touch, rl, N);
    Mesh* lM3 = meshCreator.CreateCylinder(sJ3->partB_touch, uJ4->partA_touch, rl, N);
    Mesh* lM4 = meshCreator.CreateCylinder(uJ4->partB_touch, rJ5->partA_touch, rl, N);

    lM0 = meshBoolean.MeshUnion(lM0, uJ1->partB);
    lM0 = meshBoolean.MeshUnion(lM0, rJ5->partB);
    //lM0 = meshBoolean.MeshConnect(uJ1->partB, rJ5->partB);
    lM4 = meshBoolean.MeshUnion(lM4, rJ5->partA);
    lM1 = meshBoolean.MeshUnion(lM1, rJ2->partA);
    lM2 = meshBoolean.MeshUnion(lM2, rJ2->partB);
    lM1 = meshBoolean.MeshUnion(lM1, uJ1->partA);
    lM4 = meshBoolean.MeshUnion(lM4, uJ4->partB);
    lM3 = meshBoolean.MeshUnion(lM3, uJ4->partA);
    lM2 = meshBoolean.MeshUnion(lM2, sJ3->partA);
    lM3 = meshBoolean.MeshUnion(lM3, sJ3->partB);

    ///eef
    Mesh* cubeL0 = meshCreator.CreateCuboid(Vector3d(-rl*1.6, -rl*1.2 ,-rl*1.2), Vector3d(rl*1.6, rl*1.2 ,rl*1.2));
    Mesh* cubeL1 = meshCreator.CreateCuboid(Vector3d(-rl*0.8, -rl*1.3, -rl*1.3), Vector3d(rl*0.8, 0, rl*1.3));
    Vector3d eA = sJ3->partB_touch;
    Vector3d eB = uJ4->partA_touch;
    Matrix4d cubeMat3 = GetTranslateMatrix((eA+eB)/2)* GetRotationMatrix(M_PI*0, eB-eA)*GetRotationMatrix(Vector3d(1,0,0), eB-eA);
    cubeL0->TransformMesh(cubeMat3);
    cubeL1->TransformMesh(cubeMat3);
    //lM3 = meshBoolean.MeshConnect(lM3, cubeL0);
    //lM3 = meshBoolean.MeshMinus(lM3, cubeL1);

    cubeL0 = meshCreator.CreateCuboid(Vector3d(-rl*1.6, -rl*1.2 ,-rl*1.2), Vector3d(rl*1.6, rl*1.2 ,rl*1.2));
    cubeL1 = meshCreator.CreateCuboid(Vector3d(-rl*0.8, -rl*1.3, -rl*1.3), Vector3d(rl*0.8, 0, rl*1.3));
    eA = rJ2->partB_touch;
    eB = sJ3->partA_touch;
    Matrix4d cubeMat2 = GetTranslateMatrix((eA+eB)/2)* GetRotationMatrix(M_PI*1, eB-eA)*GetRotationMatrix(Vector3d(1,0,0), eB-eA);
    cubeL0->TransformMesh(cubeMat2);
    cubeL1->TransformMesh(cubeMat2);
    lM2 = meshBoolean.MeshConnect(lM2, cubeL0);
    lM2 = meshBoolean.MeshMinus(lM2, cubeL1);

    double tol = 0.06;
    Mesh* cubeL0_h = meshCreator.CreateCuboid(Vector3d(-rl*0.8 + tol, -rl*1.2, -rl*2.4), Vector3d (rl*0.8-tol, rl*1.2, rl*2.4));
    Mesh* cubeL1_h = meshCreator.CreateCuboid(Vector3d(-rl*1.6, 0-tol, -rl*1.2-tol), Vector3d(rl*1.6, rl*1.6, rl*1.2+tol));

    Mesh* endFM = meshCreator.CreateCuboid(Vector3d(-rl*0.8+tol, -10, -rl*0.8), Vector3d(rl*0.8-tol, -rl, rl*0.8));

    endFM = meshBoolean.MeshConnect(endFM, cubeL0_h);
    endFM = meshBoolean.MeshMinus(endFM, cubeL1_h);

    if(EEFid == 2)
    {
        endFM->TransformMesh(cubeMat2);
        MultiplyPoint(Vector3d(0, -10, 0), cubeMat2, EndA);
        Mesh *endFM2 = meshCreator.CreateCylinder(targetCurve[0] + Vector3d (0, 0, 0), EndA, rl * 0.8, CSG_NUM_N);
        endFM = meshBoolean.MeshConnect(endFM, endFM2);
        lM2 = meshBoolean.MeshConnect(endFM, lM2);
    }
    else if(EEFid == 3)
    {
        endFM->TransformMesh(cubeMat3);
        MultiplyPoint(Vector3d(0, -10, 0), cubeMat3, EndA);
        Mesh *endFM3 = meshCreator.CreateCylinder(targetCurve[0], EndA, rl * 0.8, CSG_NUM_N);
        endFM = meshBoolean.MeshConnect(endFM, endFM3);
        lM3 = meshBoolean.MeshConnect(endFM, lM3);
    }
    EndA = targetCurve[0];

    IKfunc.effector_ID = EEFid;
    IKfunc.effector_point = EndA;
    endCurveA.push_back(EndA);

    linksMesh.push_back(lM0);
    linksMesh.push_back(lM1);
    linksMesh.push_back(lM2);
    linksMesh.push_back(lM3);
    linksMesh.push_back(lM4);

    cout<<"finish create linkages"<<endl;
}

void fiveDesign::InputDriver(int frame)
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    int Num = targetCurve.size();
    int Id = frame%Num;

    if(frame == 1)
    {
        preX.resize(functor.n);
        preX.setZero();
    }

    ///IK
    IKfunc.InputEndP = targetCurve[Id];
    Eigen::LevenbergMarquardt<LMFunctorIK, double> lmIK(IKfunc);
    lmIK.minimize(preX);
    IKfunc.LoopMatrix(preX, linksMat);

    MatrixXd jacIK;
    IKfunc.df(preX, jacIK);
    JacobiSVD<MatrixXd> svdIK(jacIK, ComputeThinU | ComputeThinV);
    MatrixXd B = svdIK.singularValues();


    Vector3d outA;
    MultiplyPoint(EndA, linksMat[EEFid], outA);
    endCurveA.push_back(outA);
}

void fiveDesign::UpdateModel(int link_ID, MatrixXd &VerM)
{
    linksMesh[link_ID]->TransformMesh(linksMat[link_ID], VerM);
}




