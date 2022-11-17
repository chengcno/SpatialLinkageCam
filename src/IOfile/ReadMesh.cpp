//
// Created by cheng on 29/4/22.
// Modified by CHENG 2022/Aug/08
//

#include "ReadMesh.h"
#include "Mesh/MeshCreator.h"
#include "Mesh/MeshBoolean.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include <string>

ReadMesh::ReadMesh() {
    idKey = 11;
    rCur = 0.75;
    rBall = 1.5;
}

ReadMesh::~ReadMesh() {

}

void ReadMesh::ReadMesh2Curve()
{
    MatrixXd V;
    MatrixXi F;
    igl::readOBJ("../data/closed_random_curves/C0_" + to_string(idKey) + ".obj", V, F);

    curveM = new Mesh(V,F);
    int sizeN = V.rows();
    cout<< "size = " << sizeN <<endl;

    for(int i=0;i<sizeN;i+=40)
    {
        Vector3d pt;
        pt.setZero();
        for(int j=i;j<i+40;j++)
        {
            pt += V.row(j).transpose();
        }
        pt = pt/40;
        curvePts.push_back(pt*100);
    }

    auto gr = new Groove(1, sizeN, Vector3d(0,0,1), Vector3d(10, 0, 0));
    gr->GenaratePitchCurve(curvePts);
    curveNew = gr->CreatClosedMesh();
}

void ReadMesh::FindC0Points()
{
    int sizeN = curvePts.size();
    Vector3d tanPre, tanNex;
    tanPre = curvePts[1] - curvePts[0];
    tanPre.normalize();
    for(int i=1;i<sizeN-1;i++)
    {
        tanNex = curvePts[i+1] - curvePts[i];
        tanNex.normalize();

        if(abs(acos(tanNex.dot(tanPre))) > 0.1) //todo, acos > ?
        {
            c0Id.push_back(i);
            c0pts.push_back(curvePts[i]);
        }
        tanPre = tanNex;
    }

    cout<< "c0 points num = " << c0Id.size() << endl;

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;
    if(c0Id.size() > 0)
    {
        c0Mesh = meshCreator.CreateSphere(rBall, c0pts[0], 26, 28);
        for (auto p: c0pts)
        {
            Mesh *c0_iM = meshCreator.CreateSphere(rBall, p, 26, 28);
            c0Mesh = meshBoolean.MeshConnect(c0Mesh, c0_iM);
        }
    }

    if(c0Id.size() > 0)
    {
        ReSample();
        UniSample();
    }
    else
    {
        // all lengths
        vector<double> length_all;
        double sum = 0;
        length_all.push_back(sum);
        int sizeN = curvePts.size();
        for (int i = 1; i < sizeN; i++)
        {
            double len;
            len = (curvePts[i] - curvePts[i-1]).norm();
            sum += len;
            length_all.push_back(sum);
        }
        double len_last = (curvePts[sizeN-1] - curvePts[0]).norm();
        sum += len_last;
        length_all.push_back(sum);

        int Ns = 720;
        vector<Vector3d> pList;
        pList.emplace_back(curvePts[0]);
        double len = 0;
        int id = 0;
        while(id<sizeN && len < sum - 1.5*sum/Ns)
        {
            len += (sum)/Ns;
            while (length_all[id] < len)
            {
                id++;
            }
            double resL = len - length_all[id - 1];
            double weight = resL / (length_all[id] - length_all[id - 1]);
            Vector3d pNew = (1-weight)*curvePts[id-1] + weight*curvePts[id];
            pList.push_back(pNew);
        }

        cout<< "after uniform"<<endl;
        curvePts.clear();
        curvePts = pList;

        auto gr = new Groove(1, curvePts.size(), Vector3d(0,0,1), Vector3d(10, 0, 0));
        gr->GenaratePitchCurve(curvePts);
        curveNew = gr->CreatClosedMesh();
        cout<<" creating mesh"<<endl;
    }
}

void ReadMesh::ReSample()
{
    vector<Vector3d> reCurve;
    int st;
    if(c0Id.size() > 0)
        st = c0Id[0];
    else
        st = 0;

    int ptsN = curvePts.size();
    for(int i=0;i<ptsN;i++)
    {
        int id = (i+st)%ptsN;
        reCurve.push_back(curvePts[id]);
    }

    for(auto &p : c0Id)
        p = p - st;
    curvePts.clear();
    curvePts = reCurve;

    cout<< "finish resample, c0 points ID = " <<endl;
    for(auto p : c0Id)
        cout<< " " << p ;
    cout<<endl;
}

void ReadMesh::UniSample()
{
    // all lengths
    vector<double> length_all;
    double sum = 0;
    length_all.push_back(sum);
    int sizeN = curvePts.size();
    for (int i = 1; i < sizeN; i++)
    {
        double len;
        len = (curvePts[i] - curvePts[i-1]).norm();
        sum += len;
        length_all.push_back(sum);
    }
    double len_last = (curvePts[sizeN-1] - curvePts[0]).norm();
    sum += len_last;
    length_all.push_back(sum);

    cout<< "finish compute length" <<endl;

    // c0 length
    for(auto i : c0Id)
    {
        lens.push_back(length_all[i]);
    }
    lens.push_back(length_all[sizeN]);
    c0Id.push_back(sizeN);
    curvePts.push_back(curvePts[0]);

    cout<< "finish c0 path length" <<endl;

    // P new
    vector<Vector3d> pList;
    vector<int> c0_pList_Id;

    int nSize = 1800;
    vector<int> path_size;
    for(int i=0;i<lens.size()-1;i++)
    {
        path_size.push_back(floor((lens[i+1]-lens[i])/sum * nSize) + 1);
    }
    nSize = 0;
    for(auto si : path_size)
    {nSize += si;cout<<si<<" ";}
    cout<<endl;

    for(int path_i = 0; path_i < lens.size() - 1; path_i++)
    {
        pList.emplace_back(curvePts[c0Id[path_i]]);
        c0_pList_Id.push_back(pList.size()-1);
        double len = lens[path_i];
        int id = c0Id[path_i];
        //cout<< "path_i = " <<path_i <<endl;
        int times = 1;
        while(id<c0Id[path_i+1] && times<path_size[path_i])
        {
            len += (lens[path_i+1]-lens[path_i])/path_size[path_i];
            times++;
            while (length_all[id] < len)
            {
                id++;
            }

            //    cout<<"find id "<<endl;
            double resL = len - length_all[id - 1];
            double weight = resL / (length_all[id] - length_all[id - 1]);
            Vector3d pNew = (1-weight)*curvePts[id-1] + weight*curvePts[id];
            pList.push_back(pNew);

            //    cout<<"push back pnew"<<endl;
        }
    }
    pList.push_back(curvePts[0]);
    c0_pList_Id.push_back(pList.size()-1);

    cout<<"pList size = " <<pList.size() << " , c0 ID = ";
    for(auto p : c0_pList_Id)
        cout<< " " << p;
    cout<<endl;

    //check
    double dTMax = 0; double dTMin = 1000;
    for(int i=1;i<=nSize;i++)
    {
        double T = (pList[i] - pList[i-1]).norm();
        dTMax = _MAX(dTMax, T);
        dTMin = _MIN(dTMin, T);
    }
    cout<< "dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;

    // sin sample
    vector<Vector3d> sinList;
    for(int path_i = 0; path_i < lens.size() - 1; path_i++)
    {
        sinList.push_back(pList[c0_pList_Id[path_i]]);
        for(int i=c0_pList_Id[path_i] + 1; i<c0_pList_Id[path_i+1]; i++)
        {
            int k = i - c0_pList_Id[path_i];
            double key = 0.5*(1 - cos(k*M_PI/(c0_pList_Id[path_i+1] - c0_pList_Id[path_i])))*(c0_pList_Id[path_i+1] - c0_pList_Id[path_i]) + c0_pList_Id[path_i];
            int keyId = floor(key);
            double resId = key - keyId;
            Vector3d sinPt = (1-resId)*pList[keyId] + (resId)*pList[keyId+1];
            sinList.push_back(sinPt);
        }
    }

    curvePts.clear();
    curvePts = sinList;

    auto gr = new Groove(rCur, curvePts.size(), Vector3d(0,0,1), Vector3d(10, 0, 0));
    gr->GenaratePitchCurve(curvePts);
    curveNew = gr->CreatClosedMesh();

    //curveNew->saveOBJ("../data/0429/curve.obj");
}

void ReadMesh::FindCentre()
{
    double xMax, yMax, zMax, xMin, yMin, zMin;
    xMax = yMax = zMax = -1000;
    xMin = yMin = zMin = 1000;
    for(auto p : curvePts)
    {
        xMax = _MAX(xMax, p.x());
        yMax = _MAX(yMax, p.y());
        zMax = _MAX(zMax, p.z());

        xMin = _MIN(xMin, p.x());
        yMin = _MIN(yMin, p.y());
        zMin = _MIN(zMin, p.z());
    }
    maxV << xMax, yMax, zMax;
    minV << xMin, yMin, zMin;

    centre = (maxV + minV)/2;
}

void ReadMesh::ScaleM(Vector3d scale_V)
{
    FindCentre();
    Matrix4d transM;
    transM = GetTranslateMatrix(centre)*GetScaleMatrix(scale_V)*GetTranslateMatrix(-centre);

    MultiplyPoint(maxV, transM, maxV);
    MultiplyPoint(minV, transM, minV);
    centre = (maxV + minV)/2;

    for(auto &p : curvePts)
        MultiplyPoint(p, transM, p);
    curveNew->TransformMesh(transM);
    if(c0Id.size() > 0)
        c0Mesh->TransformMesh(transM);
}

void ReadMesh::MoveTo(Vector3d cenNew)
{
    Matrix4d transM;
    transM = GetTranslateMatrix(cenNew - centre);

    MultiplyPoint(maxV, transM, maxV);
    MultiplyPoint(minV, transM, minV);
    centre = (maxV + minV)/2;

    for(auto &p : curvePts)
        MultiplyPoint(p, transM, p);
    curveNew->TransformMesh(transM);
    if(c0Id.size() > 0)
        c0Mesh->TransformMesh(transM);

    //curveNew->saveOBJ("0429/optimal/"+ to_string(idKey) +"/curve.obj");
    //if(c0Id.size() > 0)
    //    c0Mesh->saveOBJ("0429/optimal/" + to_string(idKey)+"/c0points.obj");
}

