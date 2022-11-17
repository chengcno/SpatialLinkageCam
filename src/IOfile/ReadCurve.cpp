//
// Created by cheng on 27/12/21.
// Modified by CHENG 2022/Aug/08
//

#include "ReadCurve.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include <igl/readOBJ.h>
#include "CamPart/Groove.h"
#include "Mesh/MeshCreator.h"
#include "Mesh/Mesh.h"
#include "Mesh/MeshBoolean.h"
#include "Mesh/ConVexHull.h"
#include "ReadMesh.h"

ReadCurve::ReadCurve()
{
    pNum = 0;
    TranM.setIdentity();
}


void ReadCurve::readFile(string fname)
{
    fstream fileC;
    fileC.open(fname, ios::in);

    fileC >> pNum;
    posM.resize(pNum, 3);
    for (int i=0;i<pNum;i++)
    {
        double x,y,z;
        fileC >> x >> y >> z;
        posM(i,0) = x;
        posM(i,1) = y;
        posM(i,2) = z;
    }
    fileC.close();
    std::cout<< "finish read" <<endl;

    getMinMax();
    placeCen();
    moveTo(Vector3d(-60, 100, 50));
    generateMesh(0.7);
}


void ReadCurve::AppendReadMechs(iglViewer &viewer)
{
    iglViewerData partMech;
    partMech.set_mesh(curMesh->verM, curMesh->triM);
    partMech.set_colors(RowVector3d (0.8, 0.9, 0.9));
    partMech.face_based = true;
    partMech.show_lines = unsigned (0);
    partMech.compute_normals();
    viewer.data_list.push_back(partMech);
}

void ReadCurve::ClearViewerList(iglViewer &viewer)
{
    for(int i=viewer.data_list.size()-1; i>=21; i--)
        viewer.erase_mesh(i);
}

void ReadCurve::getMinMax()
{
    box_max = posM.colwise().maxCoeff().transpose();
    box_min = posM.colwise().minCoeff().transpose();

    length = (box_max - box_min).norm();
    cout<< "input curve length " << length <<endl;
}

void ReadCurve::scaleTo(double _len)
{
    placeCen();
    double sca = _len/length;
    posM = posM*sca;
    TranM = GetScaleMatrix(Vector3d(sca, sca, sca))*TranM;
}

void ReadCurve::placeCen()
{
    RowVector3d disp = -(box_min + box_max)/2;
    posM = posM.rowwise() + disp;
    TranM = GetTranslateMatrix(disp.transpose())*TranM;
}

int ReadCurve::closeID()
{
    int KeyID = 0;
    double dist = posM.row(KeyID).norm();
    for(int i=1;i<pNum;i++)
    {
        double dist_2 = posM.row(i).norm();
        if(dist > dist_2)
        {
            KeyID = i;
            dist = dist_2;
        }
    }
    return KeyID;
}

void ReadCurve::ReplaceOrder()
{
    int keyID = closeID();
    MatrixXd posNew;
    posNew.resize(pNum, 3);
    for(int i=0;i<pNum;i++)
    {
        int id = (keyID + i)%pNum;
        posNew.row(i) = posM.row(id);
    }
    posM = posNew;
}

void ReadCurve::moveTo(Vector3d startP)
{
    ///ReplaceOrder();
    RowVector3d disp = startP.transpose();// - posM.row(0);
    posM = posM.rowwise() + disp;
    TranM = GetTranslateMatrix(disp.transpose())*TranM;
}

void ReadCurve::rotateTo(Vector3d angleE)
{
    Matrix4d roM = GetRotationMatrix(angleE);
    Vector3d rP;
    for(int i=0;i<pNum;i++)
    {
        MultiplyPoint(posM.row(i).transpose(), roM, rP);
        posM.row(i) = rP.transpose();
    }
    TranM = roM*TranM;
}

void ReadCurve::rotateTo(Matrix4d rM)
{
    Vector3d rP;
    for(int i=0;i<pNum;i++)
    {
        MultiplyPoint(posM.row(i).transpose(), rM, rP);
        posM.row(i) = rP.transpose();
    }
    TranM = rM*TranM;
}

void ReadCurve::generateMesh(double _radius)
{
    fstream inC;
    inC.open("../data/0_InputCurves/"+ to_string(_type)+".txt", ios::out);
    inC << posM.rows() <<endl;
    for(int i=0;i<posM.size();i++)
        inC << posM(i, 0) <<" " << posM(i, 1) <<" "<<posM(i, 2) <<endl;
    inC.close();

    /// tangent
    Vector3d verCenter = Vector3d (0,0,0);
    vector<Vector3d> pitchCurve;
    for(int i=0;i<pNum;i++)
        pitchCurve.emplace_back(posM(i,0), posM(i,1), posM(i,2));
    for(const auto& p : pitchCurve)
        verCenter += p;
    verCenter = verCenter/pNum;
    Vector3d tanCurve, norCurve;
    vector<Vector3d> tangentPitchCurve;
    vector<Vector3d> normalPitchCurve;

    for(int i=0;i<pNum;i++)
    {
        if(i<pNum-1)
            tanCurve = (pitchCurve[i+1] - pitchCurve[i]).normalized();
        else if(i==pNum-1)
            tanCurve = (pitchCurve[0] - pitchCurve[i]).normalized();
        tangentPitchCurve.push_back(tanCurve);

        norCurve = (pitchCurve[i] - verCenter);
        norCurve = norCurve - norCurve.dot(tanCurve)*tanCurve;
        norCurve.normalize();
        normalPitchCurve.push_back(norCurve);

    }
    tangentPitchCurve.push_back((pitchCurve[0] - pitchCurve[pNum - 1]).normalized());
    normalPitchCurve.push_back((pitchCurve[pNum - 1] - verCenter).normalized());

    ///verList
    vector<Vector3d> G_verList;
    for(int i=0;i<pNum;i++)
    {
        Vector3d N_1,N_2;
        N_1 = normalPitchCurve[i];
        N_2 = -tangentPitchCurve[i].cross(N_1);

        for(int j=0;j<CSG_NUM_N;j++)
        {
            Vector3d TubeP;
            double theta = j*2.0*M_PI/CSG_NUM_N;

            TubeP = pitchCurve[i] + (_radius)*(N_1*cos(theta) + N_2*sin(theta));
            G_verList.push_back(TubeP);
        }
    }

    ///triList
    vector<Vector3i> G_triList;
    int StartId = 0;
    for(int itr=0;itr<pNum-1;itr++)
    {
        for(int j=0;j<CSG_NUM_N-1;j++)
        {
            G_triList.emplace_back(StartId+j,StartId+CSG_NUM_N+j,StartId+CSG_NUM_N+1+j);
            G_triList.emplace_back(StartId+j, StartId+CSG_NUM_N+1+j,StartId+1+j);
        }
        G_triList.emplace_back(StartId+CSG_NUM_N-1, StartId+2*CSG_NUM_N-1, StartId+CSG_NUM_N);
        G_triList.emplace_back(StartId+CSG_NUM_N-1, StartId+CSG_NUM_N, StartId);
        StartId += CSG_NUM_N;
    }

    for(int j=0;j<CSG_NUM_N-1;j++)
    {
        G_triList.emplace_back(StartId+j, j,j+1);
        G_triList.emplace_back(StartId+j, j+1,StartId+j+1);
    }
    G_triList.emplace_back(StartId+CSG_NUM_N-1, CSG_NUM_N-1, 0);
    G_triList.emplace_back(StartId+CSG_NUM_N-1, 0, StartId);

    /// Mesh
    curMesh = new Mesh(G_verList, G_triList);
    std::cout<< "finish curve Mesh" <<endl;
}

void ReadCurve::funcGene(int type)
{
    int Nsize = 720;
    pNum = Nsize;
    posM.resize(pNum, 3);
    _type = type;

    if(type == -5)
    {
        auto _readM = new ReadMesh();
        _readM->ReadMesh2Curve();
        _readM->FindC0Points();

        Vector3d spaceMax, spaceMin;
        _readM->ScaleM(Vector3d(0.7, 0.7, 0.7));
        _readM->MoveTo(Vector3d (-151.707, 25.459, 52.4472));

        pNum = _readM->curvePts.size();
        posM.resize(pNum, 3);
        for(int i=0;i<pNum;i++)
        {
            posM.row(i) = _readM->curvePts[i].transpose();
        }
        cout<<"finish curve"<<endl;
    }

    if(type == -4) ///chair
    {
        vector<Vector3d> polyVec;
        polyVec.emplace_back(0, 0, 10);
        polyVec.emplace_back(0, 0, 4);
        polyVec.emplace_back(0, 5, 4);
        polyVec.emplace_back(0, 5, 0);
        polyVec.emplace_back(0, 0, 0);
        polyVec.emplace_back(5, 0, 0);
        polyVec.emplace_back(5, 5, 0);
        polyVec.emplace_back(5, 5, 4);
        polyVec.emplace_back(5, 0, 4);
        polyVec.emplace_back(5, 0, 10);
        polyVec.emplace_back(0, 0, 10);

        vector<Vector3d> beCtr;
        beCtr.emplace_back(5, 0, 10);
        beCtr.emplace_back(2.5, -1.4, 10);
        beCtr.emplace_back(0, 0, 10);

        int N = 10;
        vector<double> lens;
        double lenSum = 0;
        lens.push_back(lenSum);
        for(int i=0;i<N;i++)
        {
            lenSum += (polyVec[i+1] - polyVec[i]).norm();
            lens.push_back(lenSum);
        }
        for(int i=0;i<N+1;i++)
            lens[i] = lens[i]/lenSum;

        cout<< "lens =" <<endl;
        for(auto len : lens)
            cout<< len << " ";

        int key_id = 0;
        for(int i=0;i<Nsize;i++)
        {
            double t = i*1.0/Nsize;
            while(t > lens[key_id+1])
                key_id ++;
            double t1 = (t-lens[key_id])/(lens[key_id+1] - lens[key_id]);
            t1 = (1-cos(M_PI*t1))/2.0;
            Vector3d pVec;
            if(key_id < N - 1)
                pVec = polyVec[key_id]*(1-t1) + polyVec[key_id+1]*t1;
            else
                pVec = (1-t1)*(1-t1)*beCtr[0] + 2*t1*(1-t1)*beCtr[1] + t1*t1*beCtr[2];//Bezier(t1, beCtr);

            posM(i,0) = pVec.x();
            posM(i,1) = pVec.y();
            posM(i,2) = pVec.z();
        }


    }

    /// tree
    if(type == -3)
    {
        vector<Vector2d> polyVec;
        vector<double> lens;
        polyVec.emplace_back(-1, 0);
        polyVec.emplace_back(1, 0);
        polyVec.emplace_back(1, 4);
        polyVec.emplace_back(5, 4);
        polyVec.emplace_back(2, 7);
        polyVec.emplace_back(4, 7);
        polyVec.emplace_back(1.5, 9.5);
        polyVec.emplace_back(3, 9.5);
        polyVec.emplace_back(0, 12.5);
        polyVec.emplace_back(-3, 9.5);
        polyVec.emplace_back(-1.5, 9.5);
        polyVec.emplace_back(-4, 7);
        polyVec.emplace_back(-2, 7);
        polyVec.emplace_back(-5, 4);
        polyVec.emplace_back(-1, 4);
        polyVec.emplace_back(-1, 0);

        int Nvec = 15;
        int lenSum = 0;
        lens.push_back(0);
        for(int i=0;i<Nvec;i++)
        {
            double len_i = (polyVec[i+1] - polyVec[i]).norm();
            lenSum += len_i;
            lens.push_back(lenSum);
        }
        for(int i=0;i<=Nvec;i++)
        {
            lens[i] = lens[i]/lenSum;
        }

        int key_id = 0;
        for(int i=0;i<Nsize;i++)
        {
            double t = i*1.0/Nsize;
            while(t > lens[key_id+1])
                key_id ++;
            double t1 = (t-lens[key_id])/(lens[key_id+1] - lens[key_id]);
            t1 = (1-cos( M_PI*t1))/2.0;
            Vector2d pVec = polyVec[key_id]*(1-t1) + polyVec[key_id+1]*t1;

            posM(i,0) = 0;
            posM(i,1) = pVec.y();
            posM(i,2) = pVec.x();
        }
    }

    // heart parametric
    if(type == -2)
    {
        double x,y,z;
        vector<double> varT;
        for(int i=0;i<=Nsize/2;i++) {
            double t = i * 2.0 * M_PI / Nsize;
            x = 0;
            y = 13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t);
            z = 16 * pow(sin(t), 3);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
            varT.push_back(t);
        }

        //resample half
        for(int times = 0;times < 1; times ++)
        {
            vector<double> length_h;
            double sum = 0;
            length_h.push_back(sum);
            for (int i = 1; i < pNum / 2 + 1; i++) {
                double len;
                len = (posM.row(i) - posM.row(i - 1)).norm();
                sum += len;
                length_h.push_back(sum);
            }

            int nSize = pNum / 2;
            vector<Vector3d> pList;
            vector<double> Tnew;
            pList.emplace_back(posM.row(0).transpose());
            Tnew.push_back(0);
            double len = 0;
            int id = 0;
            for (int i = 1; i < nSize; i++) {
                len += sum / nSize;
                while (length_h[id] < len) {
                    id++;
                }
                double resL = len - length_h[id - 1];
                double weight = resL / (length_h[id] - length_h[id - 1]);
                double t = (1 - weight) * varT[id - 1] + weight * varT[id];
                x = 0;
                y = 13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t);
                z = 16 * pow(sin(t), 3);
                pList.emplace_back(x, y, z);
                Tnew.push_back(t);
            }
            pList.emplace_back(posM.row(nSize));
            varT.emplace_back(M_PI);

            for (int i = 0; i <= nSize; i++) {
                posM.row(i) = pList[i];
                varT[i] = Tnew[i];
            }
        }

        double dTMax = 0; double dTMin = 1000;
        for(int i=1;i<=Nsize/2;i++)
        {
            double T = (posM.row(i) - posM.row(i-1)).norm();
            dTMax = _MAX(dTMax, T);
            dTMin = _MIN(dTMin, T);
        }
        //pNum = pList.size();
        cout<< "dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;

        cout<<"pNum/2 = "<<pNum/2 <<endl;

        //sin sample
        vector<Vector3d> pList;
        pList.emplace_back(posM.row(0));
        for(int i=1;i<Nsize/2;i++)
        {
            double t = i * 2.0 * M_PI / Nsize;
            t = M_PI_2*(1-cos(t));
            //
            //t = M_PI*(-2*pow(t/M_PI,3) + 3*pow(t/M_PI,2));
            int id = floor(t/(2.0*M_PI/Nsize));
            double resT = (t - id*2.0 * M_PI / Nsize)/(2.0*M_PI/Nsize);
            pList.emplace_back((1-resT)*posM.row(id) + resT*posM.row(id+1));
        }
        pList.emplace_back(posM.row(Nsize/2));
        for (int i = 0; i <= Nsize/2; i++)
            posM.row(i) = pList[i];

        // another half
        for(int i=0;i<Nsize/2;i++)
        {
            posM.row(i+Nsize/2) = posM.row(Nsize/2-i);
            posM(i+Nsize/2,2) = -posM(-i+Nsize/2,2);
        }

        dTMax = 0; dTMin = 1000;
        for(int i=1;i<Nsize;i++)
        {
            double T = (posM.row(i) - posM.row(i-1)).norm();
            dTMax = _MAX(dTMax, T);
            dTMin = _MIN(dTMin, T);
            cout<< " i =" << i <<" , |T| = " << T <<endl;
        }
        //pNum = pList.size();
        cout<< "sin dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;

        cout<<"pNum = "<<pNum <<endl;
    }

    if(type == -1)
    {
        double x,y,z;
        for(int i=0;i<Nsize;i++)
        {
            double t = i*2.0*M_PI/Nsize;
            x = 2*cos(t);
            y = 2*sin(t);
            z = cos(2*t);

            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
    }

    if(type == 0)
    {
        double x,y,z;
        for(int i=0;i<Nsize;i++)
        {
            double t = i*2.0*M_PI/Nsize;
            t = (1-cos(t))/2;
            double theta = 4*M_PI*t;
            double r = 0.5 + t;
            x = r*cos(theta);
            y = r*sin(theta);
            z = -4*t;

            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
        //normalization
        ReSample();

        double dTMax = 0; double dTMin = 1000;
        for(int i=1;i<Nsize;i++)
        {
            double T = (posM.row(i) - posM.row(i-1)).norm();
            dTMax = _MAX(dTMax, T);
            dTMin = _MIN(dTMin, T);
        }
        //pNum = pList.size();
        cout<< "dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;
        cout<<"pNum = "<< pNum <<endl;
    }

    //knot
    if(type == 1)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double theta = i*2.0*M_PI/Nsize;
            x = cos(theta)+2*cos(2*theta);
            y = sin(theta)-2*sin(2*theta);
            z = 1.2*sin(3*theta);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }

        //normalization
        ReSample();

        double dTMax = 0; double dTMin = 1000;
        for(int i=1;i<Nsize;i++)
        {
            double T = (posM.row(i) - posM.row(i-1)).norm();
            dTMax = _MAX(dTMax, T);
            dTMin = _MIN(dTMin, T);
        }
        //pNum = pList.size();
        cout<< "dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;

        cout<<"pNum = "<<pNum <<endl;
    }

    //kinem
    if(type == 2)
    {
        double zScale = 1.2;
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double theta = i*2.0*M_PI/Nsize;
            x = 2*cos(theta) + cos(3*theta);
            y = 2*sin(theta) - sin(3*theta);
            z = zScale*sin(2*theta);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
        //normalization
        ReSample();

        double dTMax = 0; double dTMin = 1000;
        for(int i=1;i<Nsize;i++)
        {
            double T = (posM.row(i) - posM.row(i-1)).norm();
            dTMax = _MAX(dTMax, T);
            dTMin = _MIN(dTMin, T);
        }
        //pNum = pList.size();
        cout<< "dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;

        cout<<"pNum = "<<pNum <<endl;
    }

    //Ma An
    if(type == 3)
    {
        double a, b, c;
        a = 2.5;
        b = 2;
        c = 1.2;
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double theta = i*2.0*M_PI/Nsize;
            x = 2*(cos(a)*cos(theta)*cos(b*theta) - sin(theta)*sin(b*theta));
            y = 2*(cos(a)*sin(theta)*cos(b*theta) + cos(theta)*sin(b*theta));
            z = c*sin(a)*cos(b*theta);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }

        // no normalization
        cout<<"pNum = "<<pNum <<endl;
    }

    //mobius
    if(type == 4)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double t = i * 4.0 * M_PI / Nsize;
            x = (2+1*cos(0.5*t))*cos(t);
            y = (2+1*cos(0.5*t))*sin(t);
            z = 1*sin(0.5*t);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
        //
        ReSample();

        double dTMax = 0; double dTMin = 1000;
        for(int i=1;i<Nsize;i++)
        {
            double T = (posM.row(i) - posM.row(i-1)).norm();
            dTMax = _MAX(dTMax, T);
            dTMin = _MIN(dTMin, T);
        }
        //pNum = pList.size();
        cout<< "dT in [ " << dTMin << " , " <<dTMax <<" ]"<<endl;

        cout<<"pNum = "<<pNum <<endl;
    }
    //rose
    if(type == 5)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double t = i * 2.0 * M_PI / Nsize;
            x = 2*cos(2*t)*cos(t);
            y = 2*cos(2*t)*sin(t);
            z = 1.4*cos(2*t);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
        // no normalization
        cout<<"pNum = "<<pNum <<endl;
    }
    //ginger
    if(type == 6)
    {}
    //saddle
    if(type == 7)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double t = i * 2.0 * M_PI / Nsize;
        }
    }
    //t tt ttt
    if(type == 8)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double t = 4.0*i/Nsize;
            if(t <= 2 )
            {
                x = pow(t-1, 1);
                y = pow(t-1, 2);
                z = pow(t-1, 3);
            }
            else
            {
                x = pow(3-t, 1);
                y = pow(3-t, 2);
                z = pow(3-t, 3);
            }
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }

        //normalization
        double lengCur = 0;
        for(int i=1;i<Nsize;i++)
        {
            RowVector3d lineV = posM.row(i) - posM.row(i-1);
            lengCur += lineV.norm();
        }
        double t = 0;
        vector<Vector3d> pList;
        while(t <= 4)
        {
            if(t <= 2 )
            {
                x = pow(t-1, 1);
                y = pow(t-1, 2);
                z = pow(t-1, 3);
            }
            else
            {
                x = pow(3-t, 1);
                y = pow(3-t, 2);
                z = pow(3-t, 3);
            }
            pList.emplace_back(x,y,z);

            double dx,dy,dz;
            if(t <= 2 )
            {
                dx = 1;
                dy = 2*(t-1);
                dz = 3*pow(t-1, 2);
            }
            else
            {
                dx = -1;
                dy = -2*pow(3-t, 1);
                dz = -3*pow(3-t, 2);
            }

            Vector3d dT = Vector3d (dx,dy,dz);
            t += lengCur/dT.norm()/Nsize;
        }
        //
        posM.resize(pList.size(), 3);
        for(int i=0;i<pList.size();i++)
        {
            posM(i, 0) = pList[i].x();
            posM(i, 1) = pList[i].y();
            posM(i, 2) = pList[i].z();
        }
        pNum = pList.size();
        cout<<"pNum = "<<pNum <<endl;
    }
    //Bs-cube
    if(type == 9) {
        Vector3d pA, pB, pC, pD;
        pA << 0, 0, 0;
        pB << 0, 1, 0;
        pC << sqrt(3) / 2, 1.0 / 2, 0;
        pD << sqrt(3) / 6, 1.0 / 2, sqrt(6) / 3;
        //double x, y, z;
        for (int i = 0; i < Nsize ; i++) {
            double t = 2.0*M_PI * i / Nsize ;
            t = (cos(t)+1)/2;
            Vector3d pBez =
                    pA * pow(1 - t, 3) + pB * pow(1 - t, 2) * t * 3 + pC * pow(t, 2) * (1 - t) * 3 + pD * pow(t, 3);
            posM.row(i) = pBez.transpose();
        }
    }

    //
    if(type == 10)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double t = i * 2.0 * M_PI / Nsize;
            x = 2*cos(t);
            y = 2*sin(t);
            z = 1*sin(3*t);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
        // no normalization
        cout<<"pNum = "<<pNum <<endl;
    }

    //
    if(type == 11)
    {
        double x, y, z;
        for(int i=0;i<Nsize;i++)
        {
            double t = 2.0*M_PI * i / Nsize + 0*M_PI_2;
            x = cos(t);
            y = pow(cos(t),2);
            z = pow(cos(t),3);
            posM(i,0) = x;
            posM(i,1) = y;
            posM(i,2) = z;
        }
        cout<<"pNum = "<<pNum <<endl;
    }

    //smile
    if(type == 12)
    {
        SmileCurve();
        ReSample();
        Smooth();
        cout<<"pNum = "<<pNum <<endl;
    }

    getMinMax();
}

void ReadCurve::ReSample()
{
    vector<double> length;
    double sum = 0;
    length.push_back(sum);
    for(int i=1;i<pNum;i++)
    {
        double len;
        len = (posM.row(i) - posM.row(i-1)).norm();
        sum += len;
        length.push_back(sum);
    }
    sum += (posM.row(pNum-1) - posM.row(0)).norm();
    length.push_back(sum);

    int nSize = 720;
    vector<Vector3d> pList;
    pList.push_back(posM.row(0).transpose());
    double len = 0;
    int id = 0;
    for(int i=1;i<nSize;i++)
    {
        len += sum/720;
        while(length[id] < len)
        {
            id++;
        }
        double resL = len - length[id-1];
        double weight = resL/(length[id] - length[id-1]);
        RowVector3d interP = posM.row(id-1)*(1-weight) + posM.row(id%pNum)*weight;
        pList.push_back(interP.transpose());
    }

    posM.resize(720, 3);
    for(int i=0;i<nSize;i++)
    {
        posM.row(i) = pList[i].transpose();
    }
    pNum = posM.rows();
    cout<<"finish reSample"<<endl;
}

void ReadCurve::Smooth()
{

    getMinMax();

    int itr = 0;
    while(itr<300)
    {
        itr++;
        double wh = 0.05;
        posM.row(0) += wh * 0.5 * (posM.row(1) + posM.row(pNum - 1) - 2 * posM.row(0));
        for (int i = 1; i < pNum - 1; i++) {
            RowVector3d dh = 0.5 * (posM.row(i - 1) + posM.row(i + 1) - 2 * posM.row(i));
            posM.row(i) += wh * dh;
        }
        posM.row(pNum - 1) += wh * 0.5 * (posM.row(pNum - 2) + posM.row(0) - 2 * posM.row(pNum - 1));

        Vector3d scMax, scMin;
        scMax = posM.colwise().maxCoeff().transpose();
        scMin = posM.colwise().minCoeff().transpose();

        //Vector3d traV = -(scMax + scMin)/2 + (box_max + box_min)/2;
        double scale = (box_max - box_min).norm() / (scMax - scMin).norm();
        Matrix4d tM = GetTranslateMatrix((box_max + box_min) / 2)
                      * GetScaleMatrix(Vector3d(scale, scale, scale))
                      * GetTranslateMatrix(-(scMax + scMin) / 2);
        for (int i = 0; i < pNum; i++) {
            Vector3d pV;
            pV = posM.row(i).transpose();
            MultiplyPoint(pV, tM, pV);
            posM.row(i) = pV.transpose();
        }
    }

    getMinMax();
}

void ReadCurve::SmileCurve() {
    vector<Vector3d> pFace;
    //circle
    int faceSize = 120;
    for (int i = 0; i <= faceSize; i++)
    {
        double t = i*2.0*M_PI/faceSize;
        pFace.emplace_back(cos(t+M_PI_2), sin(t+M_PI_2), 0);
    }

    //Left arc
    int mSize = 30;
    vector<Vector3d> leftP;
    for(int i=0;i<=mSize;i++)
    {
        double t = -i*M_PI*0.6/mSize + M_PI*0.8;
        leftP.emplace_back(0.35*cos(t) - 0.35, 0.35*sin(t) + 0.2, 0);
    }

    //RightArc
    vector<Vector3d> rightP;
    for(int i=0;i<=mSize;i++)
    {
        double t = -i*M_PI*0.6/mSize + M_PI*0.8;
        rightP.emplace_back(0.35*cos(t) + 0.35, 0.35*sin(t) + 0.2, 0);
    }

    //Mouth
    int mouthSize = 90;
    vector<Vector3d> mouthP;
    for(int i=0;i<=mouthSize;i++)
    {
        double t = i*2.0*M_PI/mouthSize + M_PI_2;
        if(sin(t)>0)
            mouthP.emplace_back(0.5*cos(t), 0.2*sin(t) - 0.3, 0);
        else
            mouthP.emplace_back(0.5*cos(t), 0.35*sin(t) - 0.3, 0);
    }

    vector<Vector3d> stP, edP;
    stP.push_back(mouthP[mouthSize]);
    edP.push_back(leftP[0]);
    stP.push_back(leftP[mSize]);
    edP.push_back(rightP[0]);
    stP.push_back(rightP[mSize]);
    edP.push_back(pFace[0]);
    stP.push_back(pFace[faceSize]);
    edP.push_back(mouthP[0]);

    vector<Vector3d> DstP, DedP;
    double dZ = 1.3;
    DstP.emplace_back(-1, 0, dZ);
    DedP.emplace_back(0.7, 0.7, -dZ);
    DstP.emplace_back(0.7, -0.7, dZ);
    DedP.emplace_back(0.7, 0.7, -dZ);
    DstP.emplace_back(0.7, -0.7, dZ);
    DedP.emplace_back(-1, 0, -dZ);
    DstP.emplace_back(-1, 0, dZ);
    DedP.emplace_back(-1, 0, -dZ);

    vector<Vector3d> conn1,conn2,conn3,conn4;
    int connSize = 30;
    conn1 = HermiteCur(stP[0], edP[0], DstP[0], DedP[0], connSize);
    conn2 = HermiteCur(stP[1], edP[1], DstP[1], DedP[1], connSize);
    conn3 = HermiteCur(stP[2], edP[2], DstP[2], DedP[2], connSize);
    conn4 = HermiteCur(stP[3], edP[3], DstP[3], DedP[3], connSize);

    pNum = faceSize + mSize*2 + mouthSize + 4*connSize;
    int rows = 0;
    posM.resize(pNum, 3);
    for(int i=0;i<mouthSize;i++)
    {
        posM.row(rows) = mouthP[i].transpose();
        rows++;
    }
    for(int i = 0;i<connSize;i++)
    {
        posM.row(rows) = conn1[i].transpose();
        rows++;
    }
    for(int i = 0;i<mSize;i++)
    {
        posM.row(rows) = leftP[i].transpose();
        rows++;
    }
    for(int i = 0;i<connSize;i++)
    {
        posM.row(rows) = conn2[i].transpose();
        rows++;
    }
    for(int i = 0;i<mSize;i++)
    {
        posM.row(rows) = rightP[i].transpose();
        rows++;
    }
    for(int i = 0;i<connSize;i++)
    {
        posM.row(rows) = conn3[i].transpose();
        rows++;
    }
    for(int i = 0;i<faceSize;i++)
    {
        posM.row(rows) = pFace[i].transpose();
        rows++;
    }
    for(int i = 0;i<connSize;i++)
    {
        posM.row(rows) = conn4[i].transpose();
        rows++;
    }
}

vector<double> ReadCurve::Hermite(double x, double y, double dx, double dy, int size)
{
    double a = dy+dx+2*x-2*y;
    double b = 3*y - 3*x - 2*dx - dy;
    double c = dx;
    double d = x;

    vector<double> ft;
    for(int i=0;i<size;i++)
    {
        double t = i*1.0/size;
        double f_t = a*pow(t,3) + b*t*t + c*t + d;
        ft.push_back(f_t);
    }
    return ft;
}

vector<Vector3d> ReadCurve::HermiteCur(Vector3d X, Vector3d Y, Vector3d dX, Vector3d dY, int size)
{
    vector<Vector3d> pList;
    vector<double> fx,fy,fz;
    fx = Hermite(X.x(), Y.x(), dX.x(), dY.x(), size);
    fy = Hermite(X.y(), Y.y(), dX.y(), dY.y(), size);
    fz = Hermite(X.z(), Y.z(), dX.z(), dY.z(), size);

    for(int i=0;i<size;i++)
    {
        pList.emplace_back(fx[i], fy[i], fz[i]);
    }
    return pList;
}

void ReadCurve::readMesh()
{
    MatrixXd bM, cM, dM;
    MatrixXi bT, cT, dT;

    igl::readOBJ("../data/0123/opti/mid_1/Model/camFol1.obj", bM, bT);
    Mesh* M1 = new Mesh(bM, bT);
    igl::readOBJ("../data/0123/opti/mid_1/Model/camFol2.obj", cM, cT);
    Mesh* M2 = new Mesh(cM, cT);

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;
    M1 = meshBoolean.MeshUnion(M1,M1);
    M2 = meshBoolean.MeshUnion(M2,M2);

    M1->saveOBJ("../data/0123/opti/mid_1/camFol1_1.obj");
    M2->saveOBJ("../data/0123/opti/mid_1/camFol2_1.obj");

}

Vector3d ReadCurve::Bezier(double t, vector<Vector3d> CtrPts)
{
    /// c(3,i) (1-t)^(n-i) t^i P_i
    // n = 2
    return 1*pow(1-t, 2)*CtrPts[0] + 2*t*(1-t)*CtrPts[1] + t*t*CtrPts[2];
}

