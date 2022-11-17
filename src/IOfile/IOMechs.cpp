//
// Created by cheng on 6/7/22.
// Modified by CHENG 2022/Aug/08
//

#include "IOMechs.h"
#include "igl/readOBJ.h"
#include <sstream>
#include "Utility/HelpFunc.h"

IOMechs::IOMechs()
{
    sizeMesh = 0;
    motionFrames = 360;
    isCurve = true;
    motionVel = 1.0f;
}

IOMechs::~IOMechs() {

}

void IOMechs::ReadInputCurve(string curveName)
{
    inputCurveName = curveName;
    MatrixXd cM;
    MatrixXi cT;
    igl::readOBJ(curveName, cM, cT);
    Mesh* curveMesh = new Mesh(cM, cT);

    meshList.clear();
    meshList.push_back(curveMesh);
    sizeMesh = meshList.size();

    colorList.clear();
    colorList.emplace_back(0.7, 0.85, 0.75);
    isCurve = true;
}

void IOMechs::ClearViewerList(iglViewer &viewer)
{
    int n = viewer.data_list.size();
    for(int i=n-1;i>=21;i--)
        viewer.erase_mesh(i);

    sizeMesh = 0;
}

void IOMechs::AppendReadMechs(iglViewer &viewer)
{
    // origin
    for(int i=0;i<sizeMesh;i++)
    {
        iglViewerData partMech;
        partMech.set_mesh(meshList[i]->verM, meshList[i]->triM);
        partMech.set_colors(colorList[i]);
        partMech.face_based = true;
        partMech.show_lines = unsigned (0);
        partMech.compute_normals();

        viewer.data_list.push_back(partMech);
    }
}

void IOMechs::ReadInputMechs(string mechsName)
{
    fstream fin;
    fin.open(mechsName, ios::in);

    if(!fin)
    {
        std::cerr<<"cannot open the file";
    }

    int lens = mechsName.length();
    cout<< " mechsName length = " << lens <<endl;
    mechsName.erase(lens - 8);
    cout<< " new file load path = " << mechsName << endl;

    sizeMesh = 0;
    inModelNameList.clear();
    inMotionNameList.clear();
    int sizeMotion = 0;

    char buf[1024]={0};
    vector<string> modelName;
    vector<string> motionName;
    bool is_keyModel_found = false;
    bool is_keyMotion_found = false;
    int modelID = -1;
    int motionID = -2;
    while (fin.getline(buf, sizeof(buf)))
    {
        //cout << buf << endl;
        stringstream word(buf);
        string key_Model;
        word >> key_Model;

        if(key_Model == "Model")
        {
            cout<< "find Key word Model" <<endl;
            word >> sizeMesh;
            is_keyModel_found = true;
            is_keyMotion_found = false;
        }
        if(is_keyModel_found)
        {
            if(modelID >=0 && modelID<sizeMesh)
            {
                inModelNameList.push_back(mechsName+key_Model);
            }
            modelID++;
            if( modelID == sizeMesh)
                is_keyModel_found = false;
        }

        if(key_Model == "Matrix")
        {
            cout<< "find Key word Motion" <<endl;
            word >> sizeMotion;
            is_keyModel_found = false;
            is_keyMotion_found = true;
        }
        if(key_Model == "Frame")
        {
            word >> motionFrames;
        }
        if(is_keyMotion_found)
        {
            if(motionID >= 0 && motionID < sizeMotion)
            {
                inMotionNameList.push_back(mechsName+key_Model);
            }
            motionID++;
            if(motionID == sizeMotion)
                is_keyMotion_found = false;
        }
    }

    for(const auto& fs : inModelNameList)
        cout<< fs <<endl;
    ReadModels();
    for(const auto& fs : inMotionNameList)
        cout<< fs <<endl;
    ReadMotion();

    isCurve = false;
}

void IOMechs::UpdateMotion(iglViewer &viewer, double frame)
{
    frame = frame*(motionFrames/360.0);
    frame = frame - int(floor(frame/motionFrames))*motionFrames;

    if(!isCurve) {
        // origin
        for (int i = 0; i < sizeMesh; i++) {
            MatrixXd verM;
            Matrix4d preM, aftM;
            int preId = int(floor(frame)) % motionFrames;
            int aftId = int(ceil(frame)) % motionFrames;
            preM = motionMats[sizeMesh - i - 1][preId];
            aftM = motionMats[sizeMesh - i - 1][aftId];
            Matrix4d tM = InterpolationMat4d(preM, aftM, frame - int(floor(frame)));

            meshList[sizeMesh - i - 1]->TransformMesh(tM, verM);
            viewer.data_list[viewer.data_list.size() - 1 - i].set_vertices(verM);
            viewer.data_list[viewer.data_list.size() - 1 - i].compute_normals();
        }
    }
}

void IOMechs::ReadModels()
{
    meshList.clear();
    for(int i=0;i<sizeMesh;i++)
    {
        MatrixXd cM;
        MatrixXi cT;
        igl::readOBJ(inModelNameList[i], cM, cT);
        Mesh* partMesh = new Mesh(cM, cT);
        meshList.push_back(partMesh);
    }
}

void IOMechs::ReadMotion()
{
    motionMats.clear();
    for(auto & ms : inMotionNameList)
    {
        fstream matin;
        vector<Matrix4d> mats;
        matin.open(ms, ios::in);
        for(int i=0;i<motionFrames;i++)
        {
            Matrix4d mat;
            for(int ii=0;ii<4;ii++){
                for(int jj=0;jj<4;jj++){
                    matin >> mat(ii, jj);
                }
            }
            mats.push_back(mat);
        }
        motionMats.push_back(mats);
    }

    for(int i=inMotionNameList.size();i<sizeMesh;i++)
    {
        vector<Matrix4d> mats;
        for(int itr=0;itr<motionFrames;itr++)
        {
            mats.push_back(Matrix4d::Identity());
        }
        motionMats.push_back(mats);
    }

    colorList.clear();
    for(int i=0;i<sizeMesh;i++)
    {
        if(i < inMotionNameList.size() )
            colorList.push_back(colorTable[i%11]);
        else if(i == sizeMesh-1)
            colorList.push_back(colorTable[11]);
        else
            colorList.push_back(colorTable[12]);
    }
}
