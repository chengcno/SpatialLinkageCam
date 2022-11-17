//
// Created by cheng on 20/12/21.
// Modified by CHENG 2022/Aug/08
//

#include "Cam2R.h"
#include <iostream>
#include "Utility/HelpFunc.h"
#include "Cam.h"
#include "Mesh/MeshCreator.h"
#include "Mesh/MeshBoolean.h"
#include "Utility/HelpDefine.h"

Cam2R::Cam2R() {
    rCFball = 2.5; //4.5, 2.5
    rFol = 1.8; //3, 1.8
    numN = numM = CSG_NUM_N;
}

Cam2R::~Cam2R() {

}

void Cam2R::GetPitchCurve()
{
    int N = folAlpha.size();
    if(folAlpha.size() != folBeta.size())
        cout<< "motion size not match"<<endl;

    Vector3d pcPoint;
    for(int i=0;i<N;i++)
    {
        Matrix4d rM = GetRotationMatrix(folAlpha[i], folAxisAlpha)* GetRotationMatrix(folBeta[i], folAxisBeta);
        Matrix3d rotM = GetRotationFromAffine(rM);
        Quaterniond quaternion(rotM);
        outQuaters.push_back(quaternion);

        rM = GetRotationMatrix(folAlpha[i], folAxisAlpha, folCen)* GetRotationMatrix(folBeta[i], folAxisBeta, folCen);
        MultiplyPoint(pJoint_0, rM, pcPoint);

        /// for manip
        double theta = -i*2.0*M_PI/N;
        Matrix4d cMat = GetRotationMatrix(Vector3d(0,0,theta));
        MultiplyPoint(pcPoint, cMat, pcPoint);
        PitchCurve.push_back(pcPoint);
    }
    cout<<"finish Pitch Curve"<<endl;
}

void Cam2R::CreateCamMesh()
{
    GetPitchCurve();
    ComputeFollowOrient();
    auto _cam = new Cam(rCFball, PitchCurve, FollOrient);
    _cam->CreatPartMesh();
    camMesh = _cam->camMesh;
    cout<<"finish cam mesh"<<endl;
}

void Cam2R::ComputeEMechMotion(Matrix4d driMat, Matrix4d &folMat)
{
    RowVector3d trans = driMat.matrix().block(0,3,3,1).transpose();
    if(trans.norm() > 0.0001)
        cout<<"Cam_2R driver matrix includes translation"<<endl;
    if(abs(driMat(3,3)-1) > 0.0001)
        cout<<"Cam_2R driver rotation axis is NOT z-axis"<<endl;

    double theta;
    theta = acos(driMat(0,0));
    if(driMat(0,1)>0) theta = 2*M_PI - theta;
    ///for manip
    //theta = 2*M_PI - theta;
    int id = floor(theta/(2*M_PI) * PitchCurve.size());
    double weight = (theta - id*2.0*M_PI/PitchCurve.size())/(2.0*M_PI/PitchCurve.size());
    if(id == PitchCurve.size())
    {
        id = 0;
        weight = 0;
    }

    Quaternion<double> quad, qa, qb;
    qa = outQuaters[id%PitchCurve.size()];
    qb = outQuaters[(id+1)%PitchCurve.size()];
    quad = qa.slerp(weight,qb);

    Matrix3d RotM = quad.toRotationMatrix();
    folMat = GetTranslateMatrix(folCen) * GetAffine(RotM) * GetTranslateMatrix(-folCen);
}

void Cam2R::ComputeFollowOrient()
{
    Vector3d follOrient;
    //pJoint_0, pAnkle_0;
    follOrient = pAnkle_0 - pJoint_0;

    FollOrient.emplace_back(0,follOrient);
    double Time = 0;
    int LineNUM = 11*PitchCurve.size();
    for(int i=1;i<LineNUM;i++)
    {
        ///for manip
        Time = i*2.0*M_PI/LineNUM;
        //driver
        Matrix4d CamTrans;
        Matrix3d RotM;
        RotM = AngleAxisd(Time, Vector3d::UnitZ());
        CamTrans = GetAffine(RotM);

        Matrix4d FolTrans;
        ComputeEMechMotion(CamTrans, FolTrans);
        MultiplyVector(pAnkle_0 - pJoint_0, FolTrans, follOrient);

        ///try
        MultiplyVector(follOrient,GetRotationMatrix(Vector3d(0,0,-Time)),follOrient);
        FollOrient.emplace_back(Time,follOrient);
    }
    cout<<"finish Fol ori"<<endl;
}

void Cam2R::CreateFolMesh()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    //pJoint_0, pAnkle_0;
    Mesh* cfBall = meshCreator.CreateSphere(rCFball, pJoint_0, numN, numM);
    Mesh* oriCyl = meshCreator.CreateCylinder(pJoint_0, pAnkle_0, rFol, numN);
    Mesh* ankleSph = meshCreator.CreateSphere(rFol, pAnkle_0, numN, numM);
    ///for kinema (0, -5, 15)
    /// (0,0,10) five-bar (0,8,0) parallel
    Mesh* cylLink = meshCreator.CreateCylinder(pAnkle_0, folCen - Vector3d (0,0,10.00), rFol, numM);
    Mesh* cylSph = meshCreator.CreateSphere(rFol, folCen - Vector3d (0,0,10.00), numN, numN);
    Mesh* cylLink2 = meshCreator.CreateCylinder(folCen, folCen - Vector3d (0,0,10.00), rFol, numM);
    cylLink = meshBoolean.MeshConnect(cylLink, cylLink2);
    cylLink = meshBoolean.MeshConnect(cylLink, cylSph);

    oriCyl = meshBoolean.MeshConnect(oriCyl, ankleSph);
    folMesh = meshBoolean.MeshConnect(cfBall, oriCyl);
    folMesh = meshBoolean.MeshConnect(folMesh, cylLink);
}



