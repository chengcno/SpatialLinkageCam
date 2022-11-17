//////////////////////////////////////////////////
//Cam.cpp
//
// Create model of Cam
//
// Created by Yingjie Cheng on 2020/11/15.
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////

#include "Cam.h"
#include "Mesh/Mesh.h"
#include "Mesh/MeshCreator.h"
#include "Mesh/MeshBoolean.h"
#include "Utility/HelpFunc.h"
#include "Utility/HelpDefine.h"
#include "Mesh/ConVexHull.h"
///==============================================================================//
///                             Initilization
///==============================================================================//

Cam::Cam()
{
    _groove = nullptr;
    PitchTube = nullptr;

    GrooveRadius = 4;
    CurveSize = 360;
    Pitchcurve.clear();

    numN = CSG_NUM_N;
    rCamAxis = 6;
    rCFball = 4.5;
    angleTor = M_PI/90;
}

// no need
Cam::Cam(double radius, int Nsize)
{
    _groove = nullptr;
    PitchTube = nullptr;

    numN = CSG_NUM_N;
    rCamAxis = 6;
    rCFball = 4.5;
    angleTor = M_PI/90;

    GrooveRadius = radius;
    CurveSize = Nsize;
    Pitchcurve.clear();
    alphas.clear();
    betas.clear();
}

Cam::Cam(double radius, const vector<Vector3d>& _pitchCurve) {

    _groove = nullptr;
    PitchTube = nullptr;

    numN = CSG_NUM_N;
    rCamAxis = 6;
    rCFball = 4.5;
    angleTor = M_PI/90;

    GrooveRadius = radius;
    CurveSize = _pitchCurve.size();
    Pitchcurve = _pitchCurve;
    alphas.clear();
    betas.clear();
    Ualpha = Dbeta = 0;
}

Cam::Cam(double radius, const vector<Vector3d>& _pitchCurve, double _ualpha, double _Dbeta)
{

    _groove = nullptr;
    PitchTube = nullptr;

    numN = CSG_NUM_N;
    rCamAxis = 6;
    rCFball = 4.5;
    angleTor = M_PI/90;

    GrooveRadius = radius;
    CurveSize = _pitchCurve.size();
    Pitchcurve = _pitchCurve;
    alphas.clear();
    betas.clear();
    Ualpha = _ualpha;
    Dbeta = _Dbeta;
}

Cam::Cam(double radius, vector<Vector3d> _pitchCurve, vector<double> _alphas, vector<double> _betas)
{
    _groove = nullptr;
    PitchTube = nullptr;

    numN = CSG_NUM_N;
    rCamAxis = 6;
    rCFball = 4.5;
    angleTor = M_PI/90;

    GrooveRadius = radius;
    CurveSize = _pitchCurve.size();
    Pitchcurve = _pitchCurve;

    alphas = _alphas;
    betas = _betas;
    Ualpha = Dbeta = 0;
}

Cam::Cam(double radius, vector<Vector3d> _pitchCurve, vector<pair<double,Vector3d>> _FollOrient)
{
    _groove = nullptr;
    PitchTube = nullptr;

    numN = CSG_NUM_N;
    rCamAxis = 3;   //6, 3
    rCFball = 2.5; //4.5, 2.5
    angleTor = M_PI/90;

    GrooveRadius = radius;
    CurveSize = _pitchCurve.size();
    Pitchcurve = _pitchCurve;

    Ualpha = Dbeta = 0;
    FollOrient = _FollOrient;

    ComputeCrossAngle();
}
Cam::~Cam()
{
    delete _groove;
}

///==============================================================================//
///                            Create Mesh
///==============================================================================//

void Cam::CreatPartMesh()
{
    if(_groove == nullptr)
    {
        if(Pitchcurve.empty())
        {
            _groove = new Groove(GrooveRadius, CurveSize, Vector3d(0, 0, 1), Vector3d(0, 0, 0));
            _groove->CreateGrooveMesh();
            Pitchcurve = _groove->pitchCurve;
        }

        else if(Ualpha > Dbeta)
        {
            _groove = new Groove(GrooveRadius, CurveSize, Vector3d(0, 0, 1), Vector3d(0, 0, 0));
            _groove->Ualpha = Ualpha;
            _groove->Dbeta = Dbeta;
            _groove->GenaratePitchCurve(Pitchcurve);
            _groove->CreateGrooveMesh();
            _groove->GrooveMesh_->ReverseNormal();
        }
        else if(alphas.empty() && betas.empty())
        {
            _groove = new Groove(GrooveRadius, CurveSize, Vector3d(0, 0, 1), Vector3d(0, 0, 0));
            _groove->GenaratePitchCurve(Pitchcurve);
            _groove->CreateGrooveMesh();
            _groove->GrooveMesh_->ReverseNormal();
        }
        else
        {
            _groove = new Groove(GrooveRadius, CurveSize, Vector3d(0, 0, 1), Vector3d(0, 0, 0));
            _groove->GenaratePitchCurve(Pitchcurve);
            _groove->CreateGrooveMesh(alphas,betas);
            _groove->GrooveMesh_->ReverseNormal();
        }

    }
    cout<<"finish groove"<<endl;
    /// thick edge = 4
    _groove->radius_ = _groove->radius_ + 2.3; //4, 3, 2
    camMesh = _groove->CreatClosedMesh();

    auto _conH = new ConVexHull();
    camMesh = _conH->GetConvexHull(camMesh);

    MeshBoolean meshBoolean;
    camMesh = meshBoolean.MeshMinus(camMesh, _groove->MinusMesh);
    cout<<"finish convexhull"<<endl;
}

Groove* Cam::GetGroove()
{
    return _groove;
}

Mesh* Cam::GetMinusGroove()
{
    return _groove->MinusMesh;
}

void Cam::FromGrooveToCam()
{

    /// ******************** UP part ************************//

    vector<Vector3d> verList;
    vector<Vector3i> triList;
    _groove->GrooveMesh_->TriMat2TriList(triList);
    _groove->GrooveMesh_->VerMat2VerList(verList);

    int Groove_Mesh_Size = numN;
    double AxisLen = 20;

    /// Up circle
    int StartId = verList.size();
    int Startri = StartId;
    int Size = _groove->pitchCurve.size();
    int addSize = _groove->additionalSize;
    zmax_ = MIN_FLOAT;

    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc = verList[i * (Groove_Mesh_Size + addSize + 1)];
        zmax_ = _MAX(zmax_, Pc.z());

    }
    verList.emplace_back(0,0,zmax_+AxisLen);
    double Rcy = rCamAxis;
    vector<double> Angles;
    for(int i=0;i<Size;i++)
    {
        Vector3d Bc = verList[(i)*(Groove_Mesh_Size+addSize+1)];
        double AngeB = acos(Bc.x()/sqrt(Bc.x()*Bc.x() + Bc.y()*Bc.y()));
        if(Bc.y()<0) AngeB = 2*M_PI-AngeB;

        Angles.push_back(AngeB);
    }
    for(int i=0;i<Size-1;i++)
    {
        if(Angles[i]>Angles[i+1] && Angles[i] < Angles[i+1]+1.57)
        {
            Angles[i + 1] = Angles[i] + 0.01;
        }
    }
    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc(0) = Rcy*cos(Angles[i]);
        Pc(1) = Rcy*sin(Angles[i]);
        Pc(2) = zmax_+AxisLen;
        verList.push_back(Pc);
    }
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(StartId, StartId + i + 1, StartId + i + 2);
    }
    triList.emplace_back(StartId,StartId+Size,StartId+1);

    ///UP cylinder
    double camth = 0.04;
    int EndId = StartId+Size;
    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc = verList[i+1+Startri];
        Pc(2) = zmax_+camth;
        verList.push_back(Pc);
    }
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(Startri+1+i,EndId+1+i,EndId+2+i);
        triList.emplace_back(Startri+1+i,EndId+2+i,Startri+2+i);
    }
    triList.emplace_back(Startri+Size,EndId+Size,EndId+1);
    triList.emplace_back(Startri+Size,EndId+1,Startri+1);
    EndId = EndId + Size;

    ///UP torus
    vector<Vector3d> Rxy;
    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc = verList[i+1+Startri+Size];
        Vector3d Bc = verList[(i)*(Groove_Mesh_Size+addSize+1)];
        double r_ = sqrt(Bc.x()*Bc.x() + Bc.y()*Bc.y());

        Pc(0) = 0.65*r_/Rcy*verList[i+1+Startri+Size](0);
        Pc(1) = 0.65*r_/Rcy*verList[i+1+Startri+Size](1);
        Rxy.push_back(Pc);
    }

    ///smooth
    double weight = 0.05;
    Rxy.push_back(Rxy[0]);
    for(int t=0;t<200;t++)
    {
        for (int i = 1; i < Size ; i++)
        {
            Vector3d acc = Rxy[i-1] + Rxy[i+1] - 2*Rxy[i];
            Rxy[i] += acc * weight;
        }
        Vector3d acc = Rxy[1] + Rxy[Size-1] - 2*Rxy[0];
        Rxy[0] += acc*weight;
        Rxy[Size] = Rxy[0];
    }
    Rxy.pop_back();

    for(int i=0;i<Size;i++)
    {
        verList.push_back(Rxy[i]);
    }
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(Startri+1+i+Size,EndId+1+i,EndId+2+i);
        triList.emplace_back(Startri+1+i+Size,EndId+2+i,Startri+2+i+Size);
    }
    triList.emplace_back(EndId,EndId+Size,EndId+1);
    triList.emplace_back(EndId,EndId+1,Startri+1+Size);

    /// Connection
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(EndId+1+i,i*(Groove_Mesh_Size+addSize+1),
                             (i+1)*(Groove_Mesh_Size+addSize+1));
        triList.emplace_back(EndId+1+i,(i+1)*(Groove_Mesh_Size+addSize+1),
                             EndId+2+i);
    }
    triList.emplace_back(EndId+Size,(Size-1)*(Groove_Mesh_Size+addSize+1),
                         0);
    triList.emplace_back(EndId+Size,0,EndId+1);

    ///*******************************down part ************************************//

    Angles.clear();
    for(int i=0;i<Size;i++)
    {
        Vector3d Bc = verList[(i+1)*(Groove_Mesh_Size+addSize+1)-1];
        double AngeB = acos(Bc.x()/sqrt(Bc.x()*Bc.x() + Bc.y()*Bc.y()));
        if(Bc.y()<0) AngeB = 2*M_PI-AngeB;

        Angles.push_back(AngeB);
    }
    for(int i=0;i<Size-1;i++)
    {
        if(Angles[i]>Angles[i+1] && Angles[i+1]>Angles[i]-1.57)
        {
            Angles[i + 1] = Angles[i] + 0.01;
        }
    }

    StartId = EndId+Size+1;
    /// down circle
    zmin_ = MAX_FLOAT;

    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;

        Pc = verList[(i + 1) * (Groove_Mesh_Size + addSize + 1) - 1];
        zmin_ = _MIN(zmin_,Pc.z());

    }
    verList.emplace_back(0,0,zmin_ - AxisLen );
    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc(0) = Rcy*cos(Angles[i]);
        Pc(1) = Rcy*sin(Angles[i]);
        Pc(2) = zmin_- AxisLen ;
        verList.push_back(Pc);
    }
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(StartId, StartId + i + 2, StartId + i + 1);
    }
    triList.emplace_back(StartId,StartId+1,StartId+Size);

    ///down cylinder
    EndId = StartId+Size;
    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc = verList[i+1+Startri];
        Pc(2) = zmin_-camth;
        verList.push_back(Pc);
    }
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(StartId+1+i,EndId+2+i,EndId+1+i);
        triList.emplace_back(StartId+1+i,StartId+2+i,EndId+2+i);
    }
    triList.emplace_back(StartId+Size,EndId+1,EndId+Size);
    triList.emplace_back(StartId+Size,StartId+1,EndId+1);

    ///down torus
    EndId = EndId + Size;
    Rxy.clear();
    for(int i=0;i<Size;i++)
    {
        Vector3d Pc;
        Pc = verList[i+1+StartId+Size];
        Vector3d Bc = verList[(i+1)*(Groove_Mesh_Size+addSize+1)-1];
        double r_ = sqrt(Bc.x()*Bc.x() + Bc.y()*Bc.y());
        Pc(0) = r_*0.65/Rcy*Pc(0);
        Pc(1) = r_*0.65/Rcy*Pc(1);
        Rxy.push_back(Pc);
    }

    ///smooth
    Rxy.push_back(Rxy[0]);
    for(int t=0;t<200;t++)
    {
        for (int i = 1; i < Size ; i++)
        {
            Vector3d acc = Rxy[i-1] + Rxy[i+1] - 2*Rxy[i];
            Rxy[i] += acc * weight;
        }
        Vector3d acc = Rxy[1] + Rxy[Size-1] - 2*Rxy[0];
        Rxy[0] += acc*weight;
        Rxy[Size] = Rxy[0];
    }
    Rxy.pop_back();
    for(int i=0;i<Size;i++)
    {
        verList.push_back(Rxy[i]);
    }
    for(int i=0;i<Size-1;i++) {
        triList.emplace_back(StartId+1+i+Size,EndId+2+i,EndId+1+i);
        triList.emplace_back(StartId+1+i+Size,StartId+2+i+Size,EndId+2+i);
    }
    triList.emplace_back(EndId,EndId+1,EndId+Size);
    triList.emplace_back(EndId,StartId+1+Size,EndId+1);

    ///connection
    for(int i=0;i<Size-1;i++)
    {
        triList.emplace_back(EndId+1+i,(i+2)*(Groove_Mesh_Size+addSize+1)-1,
                             (i+1)*(Groove_Mesh_Size+addSize+1)-1);
        triList.emplace_back(EndId+1+i,EndId+2+i,
                             (i+2)*(Groove_Mesh_Size+addSize+1)-1);
    }
    triList.emplace_back(EndId+Size,Groove_Mesh_Size+addSize,
                         (Size)*(Groove_Mesh_Size+addSize+1)-1);
    triList.emplace_back(EndId+Size,EndId+1,Groove_Mesh_Size+addSize);

    camMesh = new Mesh(verList, triList);
    cout<<"finish inside cam mesh"<<endl;

}

void Cam::GetCamThickness(double &zmax, double &zmin)
{
    zmax = zmax_;
    zmin = zmin_;
}

Mesh* Cam::GetPitchTube()
{
    return PitchTube;
}

Mesh* Cam::CreatePitchTube()
{
    if (Pitchcurve.empty())
        printf("PitchCurve are empty!\n");

    Mesh* cyMesh;
    vector<Vector3d> verList;
    vector<Vector3i> triList;
    MeshCreator meshCreator;

    double rtube = 0.15;
    int Vrows = 0;
    for(int i=0;i<Pitchcurve.size();i++)
    {
        cyMesh = meshCreator.CreateCylinder(Pitchcurve[i], Pitchcurve[(i+1)%CurveSize], rtube, 16);
        for(int j=0;j<cyMesh->verM.rows();j++)
            verList.emplace_back(cyMesh->verM(j,0), cyMesh->verM(j,1), cyMesh->verM(j,2));
        for(int j=0;j<cyMesh->triM.rows();j++)
        {
            triList.emplace_back(cyMesh->triM(j,0)+Vrows, cyMesh->triM(j,1)+Vrows, cyMesh->triM(j,2)+Vrows);
        }

        Vrows += cyMesh->verM.rows();
        delete cyMesh;
    }
    Mesh* cysMesh = new Mesh(verList, triList);
    return cysMesh;
}


//////////
////// Open angle
/////////////////////////////

void Cam::ComputeCrossAngle() {
    alphas.clear();
    betas.clear();
    int Size = Pitchcurve.size();
    int Lines = FollOrient.size();

    for (int i = 0; i < Size; i++) {
        alphas.push_back(-4.0f);
        betas.push_back(4.0f);
    }
    for (int i = 0; i < Lines; i++) {
        bool tag = false;
        Vector3d LinePO = GetPolyLine(FollOrient[i].first, Pitchcurve);
        Vector3d LineVec = FollOrient[i].second;
        LineVec.normalize();

        Vector3d LineN1, LineN2;
        Vector3d drivAxis = Vector3d(0, 0, 1);
        LineN1 = drivAxis - drivAxis.dot(LineVec) * LineVec;
        LineN1.normalize();
        LineN2 = LineVec.cross(LineN1);
        LineN2.normalize();
        for (int j = 0; j < Size; j++) {

            Vector3d CyP = Pitchcurve[j];
            Vector3d CyT = GetPolyT(j);
            Vector3d CyN = GetPolyN(j);
            //CyT.normalize();
            CyN.normalize();


            double t1, t2;
            if (LineToCylinder(LinePO, LineVec, CyP, CyT, rCFball, t1, t2)) {
                if (t2 > 0.85 * rCFball) {
                    Vector3d LcrossCy = LinePO + LineVec * t2;
                    double paraCy = (LcrossCy - CyP).dot(CyT.normalized());
                    if (paraCy <= CyT.norm() * 1.0 && paraCy >= -CyT.norm() * 0.00001) {

                        Vector3d ProCross = (LcrossCy - CyP) - paraCy * (CyT.normalized());

                        ProCross.normalize();
                        double deg = getDegree(ProCross.dot(CyN), ProCross.dot(CyN.cross(CyT.normalized())));
                        if(false)
                        {
                            cout << "deg = " << deg << endl;
                            cout << "Process = " << ProCross <<endl;
                            cout << "CyN = " << CyN << endl;
                            cout << "CyT = " << CyT << endl;
                        }

                        alphas[j] = _MAX(alphas[j], deg + asin(3/4.5));
                        betas[j] = _MIN(betas[j], deg - asin(3/4.5));
                        tag = true;
                    }
                }
            }


            int Lnum = 13;
        }

    }

    //check
    vector<int> failListUP, failListDown;
    for (int i = 0; i < Size; i++) {
        if (alphas[i] == -4.0f || betas[i] == 4.0f) {
            if (alphas[i] == -4.0f)
                alphas[i] = M_PI / 3;
            if (betas[i] == 4.0f)
                betas[i] = -M_PI / 3;
            printf("Cut open angle fail! \n");
        }
        bool SomeWraning = false;
        if (i < Size - 2) {
            Vector3d CyP = Pitchcurve[i];
            Vector3d CyT = GetPolyT(i).normalized();
            Vector3d CyN = GetPolyN(i).normalized();

            Vector3d CyP2 = Pitchcurve[i + 1];
            Vector3d CyT2 = GetPolyT(i + 1).normalized();
            Vector3d CyN2 = GetPolyN(i + 1).normalized();


            if (abs(alphas[i] - 2 * alphas[i + 1] + alphas[i + 2]) > 2.5 * angleTor) {
                if (alphas[i] - alphas[i + 1] > 1.5 * angleTor) {
                    cout << " UP i" << i + 1 << " ,BE careful!\n";
                    failListUP.push_back(i);
                    //CorssAngleUp[i+1] = CorssAngleUp[i];
                }

                Vector3d Pnow = (cos(alphas[i]) * CyN + sin(alphas[i]) * (CyN.cross(CyT))) * rCFball +
                                CyP;
                Vector3d Pnew = (cos(alphas[i + 1]) * CyN2 + sin(alphas[i + 1]) * (CyN2.cross(CyT2))) *
                                rCFball + CyP2;
                if ((Pnow - Pnew).norm() >= 2 * (CyP - CyP2).norm()) {
                    SomeWraning = true;
                    Vector3d LcrossCy = Pnow - CyP + CyP2;
                    double paraCy = (LcrossCy - CyP2).dot(CyT2.normalized());
                    Vector3d ProCross = (LcrossCy - CyP2) - paraCy * (CyT2.normalized());
                    ProCross.normalize();
                    double deg = getDegree(ProCross.dot(CyN2), ProCross.dot(CyN2.cross(CyT2)));
                    //CorssAngleUp[i+1] = deg + 0.1*_Groove_Open_Angle_Tor;
                    cout << "Up angle i " << i << " or " << i + 1 << "may fail.\n";
                }

            }
            if (abs(betas[i] - 2 * betas[i + 1] + betas[i + 2]) >
                2.5 * angleTor) {
                if (betas[i] - betas[i + 1] < -1.5 * angleTor) {
                    cout << " Low i" << i + 1 << " ,BE careful!\n";
                    failListDown.push_back(i+1);
                }
                Vector3d Pnow = (cos(betas[i]) * CyN + sin(betas[i]) * (CyN.cross(CyT))) *
                                rCFball + CyP;
                Vector3d Pnew = (cos(betas[i + 1]) * CyN2 + sin(betas[i + 1]) * (CyN2.cross(CyT2))) *
                                rCFball + CyP2;
                if ((Pnow - Pnew).norm() >= 2 * (CyP - CyP2).norm()) {
                    SomeWraning = true;
                    Vector3d LcrossCy = Pnow - CyP + CyP2;
                    double paraCy = (LcrossCy - CyP2).dot(CyT2.normalized());
                    Vector3d ProCross = (LcrossCy - CyP2) - paraCy * (CyT2.normalized());
                    ProCross.normalize();
                    double deg = getDegree(ProCross.dot(CyN2), ProCross.dot(CyN2.cross(CyT2)));
                    cout << "Low angle i " << i << " or " << i + 1 << " may fail\n";

                }
            }
        }
    }

    for (auto i:failListUP) {
        alphas[i] = (alphas[i + 1] + alphas[i - 1]) / 2;
    }
    for (auto i:failListDown) {
        //betas[i] = (betas[i + 1] + betas[i - 1]) / 2;
    }

    //tor
    for (int i = 0; i < Size; i++) {
        alphas[i] += 2 * angleTor;
        betas[i] -= 2 * angleTor;
    }
    for (int i = 0; i < Size; i++) {
        if (alphas[i] - betas[i] > M_PI * 0.9)
            cout << "CUT ANGLE TOO LARGE!" << endl;
    }
    cout<<"finish open angles"<<endl;

}

Vector3d Cam::GetPolyT(int i)
{
    if(i<Pitchcurve.size()-1)
        return Pitchcurve[i+1]- Pitchcurve[i];
    if(i == Pitchcurve.size() - 1)
        return Pitchcurve[0] - Pitchcurve[i];
    else
        printf("GetPolyT error!\n");
    return Vector3d (1,0,0);
}

Vector3d Cam::GetPolyN(int i)
{
    Vector3d norCurve,tanCurve;
    norCurve = (Pitchcurve[i]).normalized();
    if(i<Pitchcurve.size()-1)
    {
        tanCurve = (Pitchcurve[i+1] - Pitchcurve[i]);
        tanCurve.normalize();

    }
    if(i == Pitchcurve.size()-1)
    {
        tanCurve = (Pitchcurve[0] - Pitchcurve[i]);
        tanCurve.normalize();
    }
    norCurve = norCurve - norCurve.dot(tanCurve)*tanCurve;
    norCurve.normalize();
    return norCurve;
}

///[-PI,PI]
double Cam::getDegree(double cosx, double sinx)
{
    double theta;
    if(cosx > 1) cosx = 1;
    if(cosx < -1) cosx = -1;
    theta = acos(cosx);
    if(sinx < 0)
        theta = -theta;
    return theta;
}
