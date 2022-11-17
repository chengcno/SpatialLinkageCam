//////////////////////////////////////////////////
//Groove.cpp
//
// Groove on Cam
//
// Created by Yingjie Cheng on 2020/11/9.
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////

#include "Groove.h"
#include "Utility/HelpDefine.h"
#include <iostream>

///==============================================================================//
///                              Initialization
///==============================================================================//

Groove::Groove()
{
    vecOrient = Vector3d (0,0,1);
    verCenter = Vector3d (0,0,0);

    GrooveMesh_ = NULL;
    NSize = 0;

    Gap_tor_L = 0.4;
    thickEdge = 4;
    numM = numN = CSG_NUM_N;
}

Groove::Groove(double _radius, int _Nsize, Vector3d _orient, Vector3d _center)
{
    Gap_tor_L = 0.275 * FAB_SCALE; ///0.01, 0.3
    thickEdge = 4;
    numM = numN = CSG_NUM_N;

    radius_ = _radius + Gap_tor_L;
    NSize = _Nsize;
    vecOrient = _orient;
    verCenter = _center;

    GrooveMesh_ = new Mesh();
    GrooveMesh_->verM.resize(_Nsize*numN,3);
    //GrooveMesh_->triM.resize(2*_Nsize*_Groove_Mesh_Size,3);
    Ualpha = M_PI/4;
    Dbeta = -M_PI/4;

    MinusMesh = new Mesh();
}

Groove::~Groove()
{
    delete GrooveMesh_;
}

///==============================================================================//
///                              pitch Curve
///==============================================================================//

void Groove::GenaratePitchCurve(vector<Vector3d> _pitchcurve)
{
    NSize = _pitchcurve.size();
    GrooveMesh_->verM.resize(NSize*numN,3);
    GrooveMesh_->triM.resize(2*NSize*numN,3);

    pitchCurve = _pitchcurve;

    //compute tangent and normal
    //TODO:modify to minimal error
    //use cencoid replace Center
    Vector3d tanCurve, norCurve;
    tangentPitchCurve.clear();
    normalPitchCurve.clear();
    curvePara.clear();

    for(int i=0;i<NSize;i++)
    {
        if(i<NSize-1)
            tanCurve = (_pitchcurve[i+1] - _pitchcurve[i]).normalized();
        else if(i==NSize-1)
            tanCurve = (_pitchcurve[0] - _pitchcurve[i]).normalized();
        tangentPitchCurve.push_back(tanCurve);

        norCurve = (_pitchcurve[i] - verCenter);
        norCurve = norCurve - norCurve.dot(tanCurve)*tanCurve;
        norCurve.normalize();
        normalPitchCurve.push_back(norCurve);

        double theta = i*2.0*M_PI/NSize;
        curvePara.push_back(theta);
    }
    tangentPitchCurve.push_back((_pitchcurve[0] - _pitchcurve[NSize-1]).normalized());
    normalPitchCurve.push_back((_pitchcurve[NSize-1] - verCenter).normalized());
}

void Groove::GenaratePitchCurve(vector<Vector3d> _pitchcurve, vector<double> _paras)
{
    GenaratePitchCurve(_pitchcurve);
    curvePara.clear();
    curvePara = _paras;
}

/// Init pitch curve with (0.5cosx,0.45sinx+0.05,0.1sinx)
void Groove::GeneratePitchCurve()
{
    pitchCurve.clear();
    tangentPitchCurve.clear();
    normalPitchCurve.clear();
    curvePara.clear();

    for(int i=0;i<NSize;i++)
    {
        Vector3d CenterPoint,tanCurve, normalCurve;
        double theta = i*2*M_PI/NSize;

        CenterPoint << 0.5*cos(theta), 0.45*sin(theta)+0.05, 0.1*sin(theta);
        tanCurve << -0.5*sin(theta), 0.45*cos(theta), 0.1*cos(theta);
        normalCurve << -0.5*cos(theta), -0.45*sin(theta), -0.1*sin(theta);

        tanCurve.normalize();
        normalCurve.normalize();

        pitchCurve.emplace_back(CenterPoint);
        tangentPitchCurve.emplace_back(tanCurve);
        normalPitchCurve.emplace_back(-normalCurve);
        curvePara.push_back(theta);
    }
}

///==============================================================================//
///                             Groove Mesh
///==============================================================================//

void Groove::CreateGrooveMesh()
{
    if(pitchCurve.empty())
        GeneratePitchCurve();
    CreateGrooveMesh(Ualpha, Dbeta);
}
void Groove::CreateGrooveMesh(vector<double> alphas, vector<double> betas)
{
    if(pitchCurve.empty())
    {
        GeneratePitchCurve();
        printf("pitch curve is empty! import init\n");
    }
    int addSize = CreateverList(alphas,betas);
    CreatetriList(addSize);
    //update mesh
    GrooveMesh_->GetMeshTriangles();
    MinusMesh->GetMeshTriangles();
}

int Groove::CreateverList(double alpha, double beta)
{

    vector<Vector3d> G_verList, MG_verList;
    additionalSize = 0;
    int _Groove_Mesh_Size = numN;
    for(int itr=0;itr<NSize;itr++)
    {
        Vector3d Surface;
        double theta = (beta - alpha + 2*M_PI)/_Groove_Mesh_Size;
        Vector3d N_1,N_2;
        N_1 = normalPitchCurve[itr];
        N_2 = -tangentPitchCurve[itr].cross(N_1);
        Vector3d OT;
        OT = cos(alpha)*N_1 + sin(alpha)*N_2;
        Eigen::Vector3d EdgeStart = radius_*OT + pitchCurve[itr];
        Eigen::Vector3d EdgeStart1 ;
        Vector3d HT;
        HT = vecOrient - vecOrient.dot(OT)*OT;
        HT.normalize();
        HT = HT + OT;
        HT.normalize();
        EdgeStart1 = EdgeStart + HT*thickEdge;
        Vector3d EdgeStartAno = EdgeStart + HT*thickEdge*10;
        Eigen::Vector3d EdgeStart2 = EdgeStart1 + vecOrient*radius_;

        //GrooveMesh_->verList.push_back(EdgeStart2);
        G_verList.push_back(EdgeStart1);
        MG_verList.push_back(EdgeStartAno);
        if(itr == 0) additionalSize += 1; //do once

        for(int j=0;j<=_Groove_Mesh_Size;j++)
        {
            Surface = radius_*cos(j*theta+alpha)*N_1 + radius_*sin(j*theta+alpha)*N_2 + pitchCurve[itr];
            G_verList.push_back(Surface);
            Surface = (radius_+0.15*Gap_tor_L)*cos(j*theta+alpha)*N_1 + (radius_+Gap_tor_L*0.15)*sin(j*theta+alpha)*N_2 + pitchCurve[itr];
            MG_verList.push_back(Surface);
        }
        OT = cos(beta)*N_1 + sin(beta)*N_2;
        Eigen::Vector3d EdgeEnd = radius_*OT + pitchCurve[itr];
        Eigen::Vector3d EdgeEnd1 ;
        HT = -vecOrient + vecOrient.dot(OT)*OT;
        HT.normalize();
        HT = HT + OT;
        HT.normalize();
        EdgeEnd1 = EdgeEnd + HT*thickEdge;
        Vector3d EdgeEndAno = EdgeEnd + HT*thickEdge*10;
        Eigen::Vector3d EdgeEnd2 = EdgeEnd1 - vecOrient*radius_;

        G_verList.push_back(EdgeEnd1);
        MG_verList.push_back(EdgeEndAno);

        if(itr == 0) additionalSize += 1; //do once
    }
    GrooveMesh_->VerList2VerMat(G_verList);
    MinusMesh->VerList2VerMat(MG_verList);

    return additionalSize;
}

int Groove::CreateverList(vector<double> alphas, vector<double> betas)
{
    additionalSize = 0;
    vector<Vector3d> G_verList, MG_verList;
    Vector3d UpEdge = pitchCurve[0] + radius_*(cos(alphas[0])*normalPitchCurve[0] +
                                               sin(alphas[0])*(-tangentPitchCurve[0].cross(normalPitchCurve[0])));
    for(int itr=0;itr<NSize;itr++)
    {
        /// surface point = R*(N1*cos(t) + N2*sin(t)) + position
        Vector3d Surface;
        double theta = (betas[itr] - alphas[itr] + 2*M_PI)/numN;
        //std::cout<< "i = "<<itr<< ", angle = " <<theta << ", alpha = "<< alphas[itr] << ", beta = "<<betas[itr]<< std::endl;
        Vector3d N_1,N_2;
        N_1 = normalPitchCurve[itr];
        N_2 = -tangentPitchCurve[itr].cross(N_1);
        Vector3d OT;
        OT = cos(alphas[itr])*N_1 + sin(alphas[itr])*N_2;
        Vector3d EdgeStart = radius_*OT + pitchCurve[itr];
        Vector3d EdgeStart1 = EdgeStart + OT*thickEdge;
        Vector3d EdgeStartAno = EdgeStart1 + 2*OT*thickEdge;

        MG_verList.push_back(EdgeStartAno);
        G_verList.push_back(EdgeStart1);
        if(itr == 0) additionalSize += 1; //do once

        for(int j=0;j<=numN;j++)
        {
            Surface = radius_*cos(j*theta+alphas[itr])*N_1 + radius_*sin(j*theta+alphas[itr])*N_2 + pitchCurve[itr];
            G_verList.push_back(Surface);
            Surface = (radius_+0.15*Gap_tor_L)*cos(j*theta+alphas[itr])*N_1 + (radius_+Gap_tor_L*0.15)*sin(j*theta+alphas[itr])*N_2 + pitchCurve[itr];
            MG_verList.push_back(Surface);
        }
        OT = cos(betas[itr])*N_1 + sin(betas[itr])*N_2;
        Vector3d EdgeEnd = radius_*OT + pitchCurve[itr];
        Vector3d EdgeEnd1 = EdgeEnd + OT*thickEdge;
        Vector3d EdgeEndAno = EdgeEnd1 + 2*OT*thickEdge;

        G_verList.push_back(EdgeEnd1);
        MG_verList.push_back(EdgeEndAno);
        if(itr == 0) additionalSize += 1; //do once
    }
    GrooveMesh_->VerList2VerMat(G_verList);
    MinusMesh->VerList2VerMat(MG_verList);

    return additionalSize;
}

void Groove::CreatetriList(int addSize)
{
    int StartId = 0;
    vector<Vector3i> G_triList, MG_triList;
    for(int itr=0;itr<NSize-1;itr++)
    {
        for(int j=0;j<numN+addSize;j++)
        {
            G_triList.emplace_back(StartId+j,StartId+numN+addSize+1+j,StartId+numN+addSize+2+j);
            G_triList.emplace_back(StartId+j, StartId+numN+addSize+2+j,StartId+1+j);

            MG_triList.emplace_back(StartId+j,StartId+numN+addSize+1+j,StartId+numN+addSize+2+j);
            MG_triList.emplace_back(StartId+j, StartId+numN+addSize+2+j,StartId+1+j);
        }
        MG_triList.emplace_back(StartId+numN+addSize,StartId+numN+addSize+1+numN+addSize,StartId+numN+addSize+1);
        MG_triList.emplace_back(StartId+numN+addSize, StartId+numN+addSize+1,StartId);
        StartId += numN+addSize+1;
    }

    for(int j=0;j<numN+addSize;j++)
    {
        G_triList.emplace_back(StartId+j, j,j+1);
        G_triList.emplace_back(StartId+j, j+1,StartId+j+1);
        MG_triList.emplace_back(StartId+j, j,j+1);
        MG_triList.emplace_back(StartId+j, j+1,StartId+j+1);
    }
    MG_triList.emplace_back(StartId+numN+addSize, numN+addSize,0);
    MG_triList.emplace_back(StartId+numN+addSize, 0,StartId);
    GrooveMesh_->TriList2TriMat(G_triList);
    MinusMesh->TriList2TriMat(MG_triList);
}

void Groove::CreateGrooveMesh(double alpha, double beta)
{
    if(pitchCurve.empty())
        printf("pitch Curve is empty!\n");

    //Create verList
    int addSize = CreateverList(alpha,beta);

    //give per triangle
    CreatetriList(addSize);

    //update mesh
    GrooveMesh_->GetMeshTriangles();
}

//******************************************************************************************//
//                              Groove open angle
//******************************************************************************************//


void Groove::RefineGrooveOpen(vector<pair<double, double> > _openAngles)
{
    openAngles = _openAngles;
}

///==============================================================================//
///                             Get C,T,N
///==============================================================================//

double Groove::ParaIn0to2Pi(double s)
{
    while(s>=2*M_PI)
        s = s - 2*M_PI;
    while(s<0)
        s = s + 2*M_PI;
    return s;
}

int Groove::FindInParas(double s)
{
    int id = -1;
    for(int i=0;i<NSize-1;i++)
    {
        if(curvePara[i+1]>s && curvePara[i]<=s)
        {
            id = i;
            break;
        }
    }
    if(s>curvePara[NSize-1] && s<=2*M_PI)
        id = NSize - 1;

    if(id == -1) printf("curve paras are wrong!\n");

    return id;
}

void Groove::GetParaId(double s, int &id, int &id2, double &wL, double &wR)
{
    s = ParaIn0to2Pi(s);
    id = FindInParas(s);
    double pre = curvePara[id];
    double next;

    if(id<NSize-1)
    {
        next = curvePara[id+1];
        id2 = id+1;
    }
    else if(id == NSize - 1)
    {
        next = 2*M_PI;
        id2 = 0;
    }

    wL = (next - s)/(next - pre);
    wR = (s - pre)/(next - pre);
}

Vector3d Groove::GetPitchCurve(double s)
{
    Vector3d Position;

    int id,id2;
    double weightLeft,weightRight;

    GetParaId(s,id,id2,weightLeft,weightRight);

    Position = weightLeft*pitchCurve[id] + weightRight*pitchCurve[id2];
    return Position;
}

Vector3d Groove::GetCurveTangent(double s)
{
    Vector3d TangentCurve;

    int id,id2;
    double weightLeft,weightRight;

    GetParaId(s,id,id2,weightLeft,weightRight);

    TangentCurve = weightLeft*tangentPitchCurve[id] + weightRight*tangentPitchCurve[id2];

    return TangentCurve;
}

Vector3d Groove::GetCurveNormal(double s)
{
    Vector3d NormalCur;

    int id,id2;
    double weightLeft,weightRight;

    GetParaId(s,id,id2,weightLeft,weightRight);

    NormalCur = weightLeft*normalPitchCurve[id] + weightRight*normalPitchCurve[id2];
    return NormalCur;
}


Mesh * Groove::CreatClosedMesh()
{
    ///verList
    vector<Vector3d> G_verList;
    for(int i=0;i<NSize;i++)
    {
        Vector3d N_1,N_2;
        N_1 = normalPitchCurve[i];
        N_2 = -tangentPitchCurve[i].cross(N_1);

        for(int j=0;j<numM;j++)
        {
            Vector3d TubeP;
            double theta = j*2.0*M_PI/numM;

            TubeP = pitchCurve[i] + (radius_-Gap_tor_L)*(N_1*cos(theta) + N_2*sin(theta));
            G_verList.push_back(TubeP);
        }
    }

    ///triList
    vector<Vector3i> G_triList;
    int StartId = 0;
    for(int itr=0;itr<NSize-1;itr++)
    {
        for(int j=0;j<numM-1;j++)
        {
            G_triList.emplace_back(StartId+j,StartId+numM+j,StartId+numM+1+j);
            G_triList.emplace_back(StartId+j, StartId+numM+1+j,StartId+1+j);
        }
        G_triList.emplace_back(StartId+numM-1, StartId+2*numM-1, StartId+numM);
        G_triList.emplace_back(StartId+numM-1, StartId+numM, StartId);
        StartId += numM;
    }

    for(int j=0;j<numM-1;j++)
    {
        G_triList.emplace_back(StartId+j, j,j+1);
        G_triList.emplace_back(StartId+j, j+1,StartId+j+1);
    }
    G_triList.emplace_back(StartId+numM-1, numM-1, 0);
    G_triList.emplace_back(StartId+numM-1, 0, StartId);

    /// Mesh
    Mesh *gmesh = new Mesh(G_verList, G_triList);
    return gmesh;
}
