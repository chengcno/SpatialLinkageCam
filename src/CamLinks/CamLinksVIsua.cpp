//
// Created by cheng on 28/12/21.
// Modified by CHENG 2022/Aug/08
//

#include "CamLinksVIsua.h"
#include <fstream>
#include "Mesh/MeshBoolean.h"
#include "Mesh/MeshCreator.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Linkages/Joint.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include "Mesh/ConVexHull.h"
#include "CamPart/Groove.h"

CamLinksVisua::CamLinksVisua()
{
    IKfunc.n = 9;
    IKfunc.m = 15;

    cam2r = new Cam2R();
    cam1r = new Cam1R();
    camP = new CamP();
    camC = new CamC;
    pso_pc = new PSO_PC();

    fileLoad = "tmp_data/";
}

CamLinksVisua::~CamLinksVisua()
{
}


void CamLinksVisua::ReadLinks()
{
    fstream linkf;
    linkf.open("../data/tmp_data/tmpdata"+to_string(fileID)+".txt", ios::in);

    for(int i=0;i<5;i++)
    {
        int type;
        linkf >> type;
        if(type == 1)
            jTypes.push_back('r');
        else if( type == 2 )
            jTypes.push_back('p');
        else if( type == 3 )
            jTypes.push_back('u');
        else if( type == 4 )
            jTypes.push_back('c');
        else if( type == 5 )
            jTypes.push_back('s');

        // strange bug!
        if(type == 0)
            i--;

        cout<< type <<endl;
    }

    IKfunc.JointType = jTypes;
    for(auto p : jTypes)
        cout<< p << endl;

    for(int itr=0;itr<5;itr++)
    {
        Matrix4d tempMat;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                linkf >> tempMat(i,j);
            }
        }
        geoMats.push_back(tempMat);
    }

     IKfunc.JointInitMat = geoMats;
    for(int i=0;i<21;i++)
    {
        double para_best;
        linkf >> para_best;
        gBestPara.push_back(para_best);
    }

    linkf >> sizeCur;
    cout<< sizeCur <<endl;
    for(int i=0;i<sizeCur;i++)
    {
        double x, y, z;
        linkf >> x >> y >> z;
        targetCurve.emplace_back(x, y, z);
    }

    linkf >> duraTime;
    linkf >> energy;
    linkf >> numCtrs;

    IKfunc.effector_ID = 2;
    IKfunc.effector_point = targetCurve[0];

    linkf.close();
}

void CamLinksVisua::InitialLinksMesh()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double rl = 2.1; ///2.5
    int N = CSG_NUM_N;

    auto J1 = new Joint(jTypes[0]);
    J1->CreateSimpleModel(Vector3d(0, 10, 0), Vector3d(0,0,-10));
    J1->InitPose(0,1,geoMats[0]);

    auto J2 = new Joint(jTypes[1]);
    J2->CreateSimpleModel();//Vector3d(0,-10,0), Vector3d(0,0,-10));
    J2->InitPose(2, 1, geoMats[1]);

    auto J3 = new Joint(jTypes[2]);
    J3->CreateSimpleModel(Vector3d(0, 10, 0), Vector3d(0,0,10));
    J3->InitPose(2,3,geoMats[2]);

    auto J4 = new Joint(jTypes[3]);
    J4->CreateSimpleModel(Vector3d(0, -10, 0), Vector3d(0, 0, -10));
    J4->InitPose(3, 4, geoMats[3]);

    auto J5 = new Joint(jTypes[4]);
    J5->CreateSimpleModel(Vector3d(0, 10, 0), Vector3d(0, 0, -10));
    J5->InitPose(4, 0, geoMats[4]);

    Mesh* lM0 = meshBoolean.MeshConnect(J1->partB, J5->partB);

    linkSupVecA = J1->partB_touch;
    linkSupVecB = J5->partB_touch;

    Mesh* lM1 = meshBoolean.MeshConnect(J1->partA, J2->partA);
    Mesh* lM2 = meshBoolean.MeshConnect(J3->partA, J2->partB);
    Mesh* lM3 = meshBoolean.MeshConnect(J3->partB, J4->partA);
    Mesh* lM4 = meshBoolean.MeshConnect(J4->partB, J5->partA);

    //cylinder:
    Mesh* lM0_cyl = meshCreator.CreateCylinder(J1->partB_touch, J5->partB_touch, rl, N);
    Mesh* lM1_cyl = meshCreator.CreateCylinder(J1->partA_touch, J2->partA_touch, rl ,N);

    Mesh* lM2_cyl = meshCreator.CreateCylinder(J2->partB_touch, J3->partA_touch, rl, N);
    Mesh* lM3_cyl = meshCreator.CreateCylinder(J3->partB_touch, J4->partA_touch, rl, N);
    Mesh* lM4_cyl = meshCreator.CreateCylinder(J4->partB_touch, J5->partA_touch, rl, N);

    lM0 = meshBoolean.MeshConnect(lM0, lM0_cyl);
    lM1 = meshBoolean.MeshConnect(lM1, lM1_cyl);
    lM2 = meshBoolean.MeshConnect(lM2, lM2_cyl);
    lM3 = meshBoolean.MeshConnect(lM3, lM3_cyl);
    lM4 = meshBoolean.MeshConnect(lM4, lM4_cyl);

    link1_cam = meshBoolean.MeshConnect(lM1_cyl, J1->partA);
    link4_cam = meshBoolean.MeshConnect(lM4_cyl, J5->partA);

    Vector3d eA = J2->partB_touch;
    Vector3d eB = J3->partA_touch;

    Mesh* eefCyl = meshCreator.CreateCylinder(eA*1.0 + eB*0.0, targetCurve[0], rl, N);
    lM2 = meshBoolean.MeshConnect(lM2, eefCyl);

    EndA = targetCurve[0];

    linksMesh.push_back(lM0);
    linksMesh.push_back(lM1);
    linksMesh.push_back(lM2);
    linksMesh.push_back(lM3);
    linksMesh.push_back(lM4);

    cout<<"finish create linkages"<<endl;

    InitialCamMesh();

    cout<<"finish create cams"<<endl;
}

void CamLinksVisua::InitialCamMesh()
{
    //opt pc
    pso_pc->jTypes = jTypes;
    pso_pc->folCen1 = geoMats[0].block(0,3,3,1);
    pso_pc->folCen2 = geoMats[4].block(0,3,3,1);
    if(jTypes[0] == 'u')
        MultiplyVector(Vector3d(0,1,0), geoMats[0], pso_pc->folAxisAlpha);
    if(jTypes[0] == 'c')
        MultiplyVector(Vector3d(0,1,0), geoMats[0], pso_pc->folAxisAlpha);

    MultiplyVector(Vector3d(0,0,1), geoMats[0], pso_pc->folAxisBeta);

    if(jTypes[4] == 'r')
        MultiplyVector(Vector3d(0,1,0), geoMats[4], pso_pc->folAxisGamma);
    if(jTypes[4] == 'p')
        MultiplyVector(Vector3d(1,0,0), geoMats[4], pso_pc->folAxisGamma);

    preX.resize(IKfunc.n);
    preX.setZero();
    for(int i=0;i<sizeCur;i++)
    {
        IKfunc.InputEndP = targetCurve[i];
        Eigen::LevenbergMarquardt<LMFunctorIK, double> lmIK(IKfunc);
        lmIK.minimize(preX);
        pso_pc->alphas.push_back(preX(0));
        pso_pc->betas.push_back(preX(1));
        pso_pc->gammas.push_back(-preX(8));
    }
    pso_pc->sizeC = sizeCur;

    pso_pc->init();
    pso_pc->Minimal();
    cout<< "gBest = " <<pso_pc->gBest<<endl;

    pso_pc->saveGbest("../data/"+fileLoad+"camInfo_2.txt");
    //lift
    LiftDis = pso_pc->gBest(6);
    Matrix4d LiftM = GetTranslateMatrix(Vector3d(0, LiftDis, 0));
    for(auto l : linksMesh)
        l->TransformMesh(LiftM);
    link1_cam->TransformMesh(LiftM);
    link4_cam->TransformMesh(LiftM);
    //EndA = EndA + Vector3d(0, LiftDis, 0);
    endCurveA.push_back(EndA + Vector3d(0, LiftDis, 0));


    Vector3d camLeftFolCen = geoMats[0].block(0, 3, 3, 1);
    MultiplyPoint(camLeftFolCen, LiftM, camLeftFolCen);

    if(jTypes[0] == 'u')
    {
        cam2r->folCen = camLeftFolCen;
        MultiplyVector(Vector3d(0, 1, 0), geoMats[0], cam2r->folAxisAlpha);
        MultiplyVector(Vector3d(0, 0, 1), geoMats[0], cam2r->folAxisBeta);
        cam2r->pJoint_0 = Vector3d(pso_pc->gBest(0), pso_pc->gBest(1) + pso_pc->gBest(6), pso_pc->gBest(2));
        cam2r->pAnkle_0 = Vector3d(camLeftFolCen - Vector3d(0,0,12));//pso_pc->gBest(0) + 5, pso_pc->gBest(1) + pso_pc->gBest(6) + 10, pso_pc->gBest(2));
    }
    else if(jTypes[0] == 'c')
    {
        camC->folCen = camLeftFolCen;
        MultiplyVector(Vector3d(0, 1, 0), geoMats[0], camC->folAxis);
        camC->pJoint_0 = Vector3d(pso_pc->gBest(0), pso_pc->gBest(1) + pso_pc->gBest(6), pso_pc->gBest(2));
        camC->pAnkle_0 = Vector3d(pso_pc->gBest(0) + 7, pso_pc->gBest(1) + pso_pc->gBest(6) + 14, pso_pc->gBest(2));
    }

    Vector3d camRightFolCen = geoMats[4].block(0, 3, 3, 1);
    MultiplyPoint(camRightFolCen, LiftM, camRightFolCen);
    if(jTypes[4] == 'r')
    {
        cam1r->folCen = camRightFolCen;
        MultiplyVector(Vector3d(0, 1, 0), geoMats[4], cam1r->folAxisAlpha);
        cam1r->pJoint_0 = Vector3d(pso_pc->gBest(3), pso_pc->gBest(4) + pso_pc->gBest(6), pso_pc->gBest(5));
        MultiplyPoint(Vector3d(0, 10, 0), GetTranslateMatrix(Vector3d(0, LiftDis, 0)) * geoMats[4], cam1r->pAnkle_0);
    }
    if(jTypes[4] == 'p')
    {
        camP->folCen = camRightFolCen;
        MultiplyVector(Vector3d(1, 0, 0), geoMats[4], camP->folAxis);
        camP->pJoint_0 = Vector3d(pso_pc->gBest(3), pso_pc->gBest(4) + pso_pc->gBest(6), pso_pc->gBest(5));
        camP->pAnkle_0 = Vector3d(pso_pc->gBest(3) , pso_pc->gBest(4) + pso_pc->gBest(6) + 10, pso_pc->gBest(5));

    }

    preX.resize(IKfunc.n);
    preX.setZero();
    for(int i=0;i<sizeCur;i++)
    {
        IKfunc.InputEndP = targetCurve[i];
        Eigen::LevenbergMarquardt<LMFunctorIK, double> lmIK(IKfunc);
        lmIK.minimize(preX);
        cam2r->folAlpha.push_back(preX(0));
        cam2r->folBeta.push_back(preX(1));

        camC->folTheta.push_back(preX(0));
        camC->folTrans.push_back(preX(1));

        cam1r->folAlpha.push_back(-preX(8));
        camP->folDisp.push_back(-preX(8));
    }

    if(jTypes[0] == 'u') cam2r->CreateCamMesh();
    if(jTypes[0] == 'u') cam2r->CreateFolMesh();
    if(jTypes[4] == 'r') cam1r->CreateFolMesh();
    if(jTypes[4] == 'r') cam1r->CreateCamMesh();

    if(jTypes[0] == 'c') camC->CreateCamMesh();
    if(jTypes[0] == 'c') camC->CreateFolMesh();
    if(jTypes[4] == 'p') camP->CreateFolMesh();
    if(jTypes[4] == 'p') camP->CreateCamMesh();
}

void CamLinksVisua::ClearViewerList(iglViewer &viewer)
{
    for(int i=viewer.data_list.size()-1; i>=21; i--)
        viewer.erase_mesh(i);
}

void CamLinksVisua::AppendMechs(iglViewer &viewer)
{
    igl::opengl::ViewerData mydata;
    mydata.set_mesh(cam2Mesh->verM, cam2Mesh->triM);
    mydata.show_lines = unsigned (0);
    mydata.face_based = true;
    mydata.compute_normals();
    mydata.set_colors(RowVector3d(0.86, 0.85, 0.95));
    viewer.data_list.push_back(mydata);

    vector<RowVector3d> colors = {
            RowVector3d(0.86, 0.86, 0.86),
            RowVector3d(0.86, 0.85, 0.95),
            RowVector3d(0.86, 0.9, 0.75),
            RowVector3d(0.9, 0.9, 0.75),
            RowVector3d(0.9, 0.8, 0.85)
    };

    for(int i=0;i<5;i++)
    {
        mydata.clear();
        mydata.set_mesh(linksMesh[i]->verM, linksMesh[i]->triM);
        mydata.show_lines = unsigned (0);
        mydata.face_based = true;
        mydata.compute_normals();
        mydata.set_colors(colors[i]);
        viewer.data_list.push_back(mydata);
    }
    mydata.clear();
    mydata.show_lines = unsigned (0);
    mydata.face_based = true;
    mydata.set_mesh(camSups[1]->verM, camSups[1]->triM);
    mydata.set_colors(RowVector3d(0.86, 0.86, 0.86));
    viewer.data_list.push_back(mydata);

    mydata.clear();
    mydata.show_lines = unsigned (0);
    mydata.face_based = true;
     mydata.set_colors(RowVector3d(0.8, 0.7, 0.6));
    viewer.data_list.push_back(mydata);
}

void CamLinksVisua::AppendCurve(iglViewer &viewer)
{
    MatrixXd verM;
    MatrixXi triM;
    addCurveToViewer(verM, triM);
    viewer.data_list.rbegin()->clear();
    viewer.data_list.rbegin()->set_mesh(verM, triM);
    viewer.data_list.rbegin()->compute_normals();
}

void CamLinksVisua::UpdateMotion(iglViewer &viewer, int frame)
{
    vector<MatrixXd> partsVec;
    UpdateModel(frame, partsVec);
    int n = viewer.data_list.size();
    for(int i=0;i<6;i++)
    {
        viewer.data_list[n - 3 - i].set_vertices(partsVec[5 - i]);
        viewer.data_list[n - 3 - i].compute_normals();
    }
}

void CamLinksVisua::UpdateModel(int frame, vector<MatrixXd> &partsVec)
{
    partsVec.clear();
    int Id = frame % sizeCur;
    if(frame == 1)
    {
        preX.resize(IKfunc.n);
        preX.setZero();
    }
    partsVec.clear();
    MatrixXd camVert;
    double theta = frame*2.0*M_PI/sizeCur;
    Matrix4d driM = GetRotationMatrix(Vector3d(0,0,theta));
    cam2Mesh->TransformMesh(driM, camVert);
    partsVec.push_back(camVert);

    IKfunc.InputEndP = targetCurve[Id];
    Eigen::LevenbergMarquardt<LMFunctorIK, double> lmIK(IKfunc);
    lmIK.minimize(preX);
    IKfunc.LoopMatrix(preX, motionMats);

    Vector3d outA;
    MultiplyPoint(EndA, motionMats[2], outA);
    endCurveA.push_back(outA + Vector3d(0, LiftDis, 0));

    MatrixXd linkVert;
    for(int i=0;i<5;i++)
    {
        linksMesh[i]->TransformMesh(GetTranslateMatrix(Vector3d(0, LiftDis, 0))*motionMats[i]*GetTranslateMatrix(Vector3d(0, -LiftDis, 0)), linkVert);
        partsVec.push_back(linkVert);
    }

    if(frame == 1)
    {
        fstream matf;
        matf.open("../data/"+fileLoad+"cam.pmt", ios::out);
        matf.close();
        for(int itr = 0;itr<5;itr++) {
            matf.open("../data/" + fileLoad + "link" + to_string(itr) + ".pmt", ios::out);
            matf.close();
        }
        for(int i=1;i<10;i++)
        {
            matf.open("../data/" + fileLoad + "s"+to_string(i)+".ang", ios::out);
            matf.close();
        }
    }

    if(frame <= sizeCur)
    {
        fstream matf;

        for(int i=1;i<10;i++)
        {
            matf.open("../data/" + fileLoad + "s"+to_string(i)+".ang", ios::app);
            matf << preX(i-1) <<endl;
            matf.close();
        }

        matf.open("../data/"+fileLoad+"cam.pmt", ios::app);
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                matf << driM(i,j) <<" ";
            }
        }
        matf << "\n";
        matf.close();

        for(int itr = 0;itr<5;itr++)
        {
            matf.open("../data/"+fileLoad+"link"+ to_string(itr)+".pmt", ios::app);
            Matrix4d linkMat = GetTranslateMatrix(Vector3d(0, LiftDis, 0))*motionMats[itr]*GetTranslateMatrix(Vector3d(0, -LiftDis, 0));
            for(int i=0;i<4;i++)
            {
                for(int j=0;j<4;j++)
                {
                    matf << linkMat(i,j) <<" ";
                }
            }
            matf << "\n";
            matf.close();
        }
    }

    if(frame == sizeCur+1)
    {
        auto outC = new Groove(0.5, sizeCur, Vector3d(0,1,0), Vector3d(0,0,0));
        outC->GenaratePitchCurve(endCurveA);
        Mesh* curM = outC->CreatClosedMesh();
        curM->saveOBJ(fileLoad+"curve.obj");

        MeshCreator meshCtrator;
        Mesh* endBall = meshCtrator.CreateSphere(2.1, endCurveA[0], 30, 40);
        endBall->saveOBJ(fileLoad+"endBall.obj");

        for(int i=0;i<5;i++)
            linksMesh[i]->saveOBJ(fileLoad+"link"+ to_string(i)+".obj");

        for(int i=0;i<2;i++)
            camSups[i]->saveOBJ(fileLoad+"camSup"+ to_string(i)+".obj");

        cam2Mesh->saveOBJ(fileLoad+"cam.obj");

        ofstream curFile;
        curFile.open("../data/"+fileLoad+"curve.txt", ios::out);
        curFile<<sizeCur<<endl;
        for(auto p : endCurveA)
        {
            curFile << p.x() <<" "<<p.y()<<" "<<p.z() <<endl;
        }
        curFile.close();
    }
}

void CamLinksVisua::ConnectMesh()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double thickG = 2;
    double lenX = 25;
    double lenZ = 10+abs(geoMats[0](2,3) - geoMats[4](2,3))/2;

    /// z -2,
    double cenZ = (geoMats[0](2,3) + geoMats[4](2,3))/2 - 4;
    double overLen = -2;

    Mesh* CamAXIS = meshCreator.CreateCylinder(Vector3d(0,0,cenZ-lenZ-15-overLen), Vector3d(0,0,cenZ + lenZ + 10 + overLen), 4, 80);

    Mesh* camSup1 = meshCreator.CreateCuboid(Vector3d(0,0,-lenZ + cenZ - 10 - overLen) - Vector3d(8, 8, 0), Vector3d(0,0,-lenZ + cenZ - 10 - overLen) + Vector3d(8, 8, 2*thickG));
    Mesh* camSup2 = meshCreator.CreateCuboid(Vector3d(0,0,lenZ + cenZ + 5 + overLen ) - Vector3d(8, 8, 2*thickG), Vector3d(0,0,lenZ + cenZ + 5 + overLen) + Vector3d(8, 8, 0));

    double gDepth = -45 - LiftDis;
    Vector3d supGrdSize = Vector3d (100, thickG, 120);
    Mesh* supGrd = meshCreator.CreateCuboid(Vector3d(0,gDepth,cenZ)-supGrdSize/2, Vector3d(0,gDepth,cenZ)+supGrdSize/2);
    Mesh* camSup1Grd = meshCreator.CreateCuboid( Vector3d(-8, gDepth, -thickG-lenZ + cenZ - 10 - overLen), Vector3d(8, 8, thickG-lenZ + cenZ - 10 - overLen));
    Mesh* camSup2Grd = meshCreator.CreateCuboid( Vector3d(-8, gDepth, -thickG+lenZ + cenZ + 5 + overLen), Vector3d(8, 8, thickG+lenZ + cenZ + 5 + overLen));
    //Mesh* linkSupGrd = meshCreator.CreateCylinder(link0SupVec+Vector3d(0, LiftDis,0), Vector3d(link0SupVec.x(), gDepth, link0SupVec.z()), 2.1, CSG_NUM_N);
    Vector3d link0SupVec = (linkSupVecA + linkSupVecB)/2.0 + Vector3d(0, LiftDis, 0);
    Vector3d link0Ori = (linkSupVecB - linkSupVecA).normalized();
    Vector3d link0VecA, link0VecB;
    link0VecA = link0SupVec - (-7)*link0Ori;
    link0VecB = link0SupVec + 15*link0Ori;
    Mesh* linkSupGrd1 = meshCreator.CreateCylinder(link0VecA,Vector3d(link0VecA.x(), gDepth, link0VecA.z()), 2.1, CSG_NUM_N);
    Mesh* linkSupGrd2 = meshCreator.CreateCylinder(link0VecB,Vector3d(link0VecB.x(), gDepth, link0VecB.z()), 2.1, CSG_NUM_N);
    linkSupGrd1 = meshBoolean.MeshConnect(linkSupGrd1, linkSupGrd2);
    auto conH = new ConVexHull();
    Mesh* linkSupGrd = conH->GetConvexHull(linkSupGrd1);

    //shift
    Vector3d shiftDir;
    double shiftDis = 25.0;
    MultiplyVector(link0Ori, GetRotationMatrix(Vector3d(0,M_PI_2,0)), shiftDir);
    linkSupGrd->TransformMesh(GetTranslateMatrix(shiftDir*shiftDis));
    Vector3d link0VecA_sf, link0VecB_sf;
    MultiplyPoint(link0VecA, GetTranslateMatrix(shiftDir*(shiftDis+_MIN(2.1, shiftDis+0.001))), link0VecA_sf);
    MultiplyPoint(link0VecB, GetTranslateMatrix((shiftDis+_MIN(2.1, shiftDis+0.001))*shiftDir), link0VecB_sf);
    Mesh* sup2Grd = meshCreator.CreateCylinder(link0VecA, link0VecA_sf, 2.1, CSG_NUM_N);
    Mesh* sup1Grd = meshCreator.CreateCylinder(link0VecB, link0VecB_sf, 2.1, CSG_NUM_N);
    sup1Grd = meshBoolean.MeshConnect(sup1Grd, sup2Grd);
    Mesh* supGrdShift = conH->GetConvexHull(sup1Grd);
    linkSupGrd = meshBoolean.MeshConnect(linkSupGrd, supGrdShift);

    supGrd = meshBoolean.MeshConnect(supGrd, camSup1Grd);
    supGrd = meshBoolean.MeshConnect(supGrd, camSup2Grd);
    supGrd = meshBoolean.MeshConnect(supGrd, linkSupGrd);
    camSup1 = meshBoolean.MeshConnect(camSup1, camSup2);

    camSups.push_back(camSup1);
    camSups.push_back(supGrd);

    linksMesh[1]->saveOBJ(fileLoad+"link1_0.obj");
    linksMesh[4]->saveOBJ(fileLoad+"link4_0.obj");

    if(jTypes[0] == 'u') link1_cam = meshBoolean.MeshConnect(link1_cam, cam2r->folMesh);
    if(jTypes[0] == 'c') link1_cam = meshBoolean.MeshConnect(link1_cam, camC->folMesh);

    if(jTypes[4] == 'r') link4_cam = meshBoolean.MeshConnect(link4_cam, cam1r->folMesh);
    if(jTypes[4] == 'p') link4_cam = meshBoolean.MeshConnect(link4_cam, camP->folMesh);

    link1_cam->saveOBJ(fileLoad+"camFol1.obj");
    link4_cam->saveOBJ(fileLoad+"camFol2.obj");

    if(jTypes[0] == 'u')
        cam2Mesh = meshBoolean.MeshConnect(CamAXIS, cam2r->camMesh);
    if(jTypes[0] == 'c')
        cam2Mesh = meshBoolean.MeshConnect(CamAXIS, camC->camMesh);

    if(jTypes[4] == 'p')
        cam2Mesh = meshBoolean.MeshUnion(cam2Mesh, camP->camMesh);
    if(jTypes[4] == 'r')
        cam2Mesh = meshBoolean.MeshConnect(cam2Mesh, cam1r->camMesh);

    if(jTypes[0] == 'u') linksMesh[1] = meshBoolean.MeshConnect(linksMesh[1], cam2r->folMesh);
    if(jTypes[0] == 'c') linksMesh[1] = meshBoolean.MeshUnion(linksMesh[1], camC->folMesh);

    if(jTypes[4] == 'r') linksMesh[4] = meshBoolean.MeshConnect(linksMesh[4], cam1r->folMesh);
    if(jTypes[4] == 'p') linksMesh[4] = meshBoolean.MeshUnion(linksMesh[4], camP->folMesh);

    cout<<"finish connect cams"<<endl;

    AddWall();
}

void CamLinksVisua::AddGround()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double thickG = 2;
    double lenX = 25;
    double lenZ = 47;
    Vector3d cenLow = Vector3d (0, -40 - LiftDis, -6);
    Vector3d cenTop = Vector3d (0, 40 + LiftDis, -6);
    Mesh* G1 = meshCreator.CreateCuboid(cenLow - Vector3d(lenX, thickG , lenZ - 2 * thickG), cenLow + Vector3d(lenX, thickG, lenZ - 2 * thickG));
    Mesh* G2 = meshCreator.CreateCuboid(cenTop - Vector3d(lenX, thickG ,lenZ - 2*thickG), cenTop + Vector3d(lenX, thickG, lenZ - 2*thickG));
    Mesh* G3 = meshCreator.CreateCuboid(Vector3d(cenLow.x() - lenX, cenLow.y() - thickG*3, cenLow.z() - lenZ ), Vector3d(cenLow.x() + lenX, cenTop.y() + thickG * 3, cenLow.z() - lenZ + 2 * thickG));
    Mesh* G4 = meshCreator.CreateCuboid(Vector3d(cenLow.x() - lenX, cenLow.y() - thickG*3, cenLow.z() + lenZ - 2 * thickG), Vector3d(cenLow.x() + lenX, cenTop.y() + thickG * 3, cenLow.z() + lenZ));

    Mesh* G1JS = meshCreator.CreateCuboid(cenLow - Vector3d(lenX * 0.7, thickG * 1, lenZ), cenLow + Vector3d(lenX * 0.7, 1 * thickG, lenZ));
    G1 = meshBoolean.MeshUnion(G1, G1JS);

    double tol = 0.1;
    Mesh* G1J_tor = meshCreator.CreateCuboid(cenLow - Vector3d(lenX * 0.7, thickG * 1, lenZ) - Vector3d (tol, tol , tol), cenLow + Vector3d(lenX * 0.7, thickG, lenZ) + Vector3d(tol, tol, tol));
    G3 = meshBoolean.MeshMinus(G3, G1J_tor);
    G4 = meshBoolean.MeshMinus(G4, G1J_tor);

    Mesh* G2JR = meshCreator.CreateCuboid(cenTop - Vector3d(lenX*0.7, thickG*1.0, lenZ), cenTop + Vector3d(lenX*0.7, thickG*1, -lenZ + 2*thickG));
    G2 = meshBoolean.MeshUnion(G2, G2JR);
    Mesh* G2JR_tor = meshCreator.CreateCuboid(cenTop - Vector3d(lenX*0.7, thickG*1.0, lenZ) - Vector3d (tol, tol ,tol), cenTop + Vector3d(lenX*0.7, thickG*1, -lenZ + 3*thickG) + Vector3d (tol, tol ,tol));
    G3 = meshBoolean.MeshMinus(G3, G2JR_tor);

    Mesh* G2JL1 = meshCreator.CreateCuboid(cenTop - Vector3d(lenX*0.85, thickG, -lenZ + 2*thickG),cenTop + Vector3d(-lenX*0.5, thickG, lenZ));
    Mesh* G2JL1_tor = meshCreator.CreateCuboid(cenTop - Vector3d(lenX*0.85, thickG, -lenZ + 2*thickG)- Vector3d (tol, tol ,tol),cenTop + Vector3d(-lenX*0.5, thickG, lenZ) + Vector3d (tol, tol ,tol));
    G2 = meshBoolean.MeshUnion(G2, G2JL1);
    G4 = meshBoolean.MeshMinus(G4, G2JL1_tor);

    Mesh* G2JL2 = meshCreator.CreateCuboid(cenTop - Vector3d(-lenX*0.5, thickG, -lenZ + 2*thickG),cenTop + Vector3d(lenX*0.85, thickG, lenZ));
    Mesh* G2JL2_tor = meshCreator.CreateCuboid(cenTop - Vector3d(-lenX*0.5, thickG, -lenZ + 2*thickG)- Vector3d (tol, tol ,tol),cenTop + Vector3d(lenX*0.85, thickG, lenZ) + Vector3d (tol, tol ,tol));
    G2 = meshBoolean.MeshUnion(G2, G2JL2);
    G4 = meshBoolean.MeshMinus(G4, G2JL2_tor);

    Mesh* CamAXIS = meshCreator.CreateCylinder(Vector3d(0,0,-70), Vector3d(0,0,60), 5.5, 40);
    G3 = meshBoolean.MeshMinus(G3, CamAXIS);
    G4 = meshBoolean.MeshMinus(G4, CamAXIS);

    Mesh* MCube1 = meshCreator.CreateCuboid(cam1r->folCen - Vector3d(30, 15, 12), cam1r->folCen + Vector3d(30, 15, 12));
    Mesh* MCube2 = meshCreator.CreateCuboid(cam2r->folCen - Vector3d(9, 15, 25), cam2r->folCen + Vector3d(9, 15, 25));
    Mesh* MCube3 = meshCreator.CreateCuboid(cam2r->folCen - Vector3d(17, 15, 40), cam2r->folCen + Vector3d(17, 15, -2));
    Mesh* MCube4 = meshCreator.CreateCuboid(cam1r->folCen - Vector3d(3.5, thickG, 14), cam1r->folCen + Vector3d(3.5, thickG, -8.5));

    G2 = meshBoolean.MeshMinus(G2, MCube1);
    G2 = meshBoolean.MeshMinus(G2, MCube2);
    G2 = meshBoolean.MeshMinus(G2, MCube3);
    G2 = meshBoolean.MeshUnion(G2, MCube4);
    G4 = meshBoolean.MeshMinus(G4, MCube2);

    grdM.push_back(G1);
    grdM.push_back(G2);
    grdM.push_back(G3);
    grdM.push_back(G4);

    linksMesh[0] = meshBoolean.MeshUnion(linksMesh[0], G2);

    Mesh* linksM = meshBoolean.MeshConnect(linksMesh);
    linksM = meshBoolean.MeshConnect(linksM, cam2Mesh);
    cout<<"finish add grd"<<endl;
}

void CamLinksVisua::AddWall()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    Mesh* cubM = meshCreator.CreateCuboid(Vector3d(0, 0, 0), Vector3d(100, 100, 100));
    double thick = 1;
    double radius = 10;

    Mesh* cyl1 = meshCreator.CreateCylinder(Vector3d(thick+radius, thick+radius, thick+radius), Vector3d(thick+radius, 100, thick+radius), radius+thick, CSG_NUM_N);
    Mesh* cyl2 = meshCreator.CreateCylinder(Vector3d(thick+radius, thick+radius, thick+radius), Vector3d(thick+radius, radius+thick, 100), radius+thick, CSG_NUM_N);
    Mesh* cyl3 = meshCreator.CreateCylinder(Vector3d(thick+radius, thick+radius, thick+radius), Vector3d(100, radius+thick, thick+radius), radius+thick, CSG_NUM_N);

    Mesh* sph = meshCreator.CreateSphere(radius+thick, Vector3d(thick+radius, thick+radius, thick+radius), CSG_NUM_N, CSG_NUM_N);

    //curve center
    double mx,my,mz,Mx,My,Mz;
    mx = my = mz = 1000;
    Mx = My = Mz = -1000;
    for(auto p : targetCurve)
    {
        mx = _MIN(mx, p.x());
        my = _MIN(my, p.y());
        mz = _MIN(mz, p.z());

        Mx = _MAX(Mx, p.x());
        My = _MAX(My, p.y());
        Mz = _MAX(Mz, p.z());
    }

    /// new walls;
    //y, z, x
    Mesh* cyl1_in = meshCreator.CreateCylinder(Vector3d(thick+radius, thick+0, thick+radius), Vector3d(thick+radius, 100+thick, thick+radius), radius, CSG_NUM_N);
    Mesh* cyl2_in = meshCreator.CreateCylinder(Vector3d(thick+radius, thick+radius, thick), Vector3d(thick+radius, radius+thick, thick+100), radius, CSG_NUM_N);
    Mesh* cyl3_in = meshCreator.CreateCylinder(Vector3d(thick, thick+radius, thick+radius), Vector3d(thick+100, radius+thick, thick+radius), radius, CSG_NUM_N);
    cyl1 = meshBoolean.MeshMinus(cyl1, cyl1_in);
    cyl2 = meshBoolean.MeshMinus(cyl2, cyl2_in);
    cyl3 = meshBoolean.MeshMinus(cyl3, cyl3_in);

    Mesh* cubeLL = meshCreator.CreateCuboid(Vector3d(0,0,0), Vector3d(110, 110, 110));
    cubeLL->TransformMesh(GetTranslateMatrix(Vector3d(thick+radius,0,0)));
    cyl1 = meshBoolean.MeshMinus(cyl1, cubeLL);
    cyl2 = meshBoolean.MeshMinus(cyl2, cubeLL);
    sph = meshBoolean.MeshMinus(sph, cubeLL);

    cubeLL->TransformMesh(GetTranslateMatrix(Vector3d(-thick-radius,thick+radius,0)));
    cyl3 = meshBoolean.MeshMinus(cyl3, cubeLL);
    cyl2 = meshBoolean.MeshMinus(cyl2, cubeLL);
    sph = meshBoolean.MeshMinus(sph, cubeLL);

    cubeLL->TransformMesh(GetTranslateMatrix(Vector3d(0,-thick-radius,thick+radius)));
    cyl1 = meshBoolean.MeshMinus(cyl1, cubeLL);
    cyl3 = meshBoolean.MeshMinus(cyl3, cubeLL);
    sph = meshBoolean.MeshMinus(sph, cubeLL);

    Mesh* sph_in = meshCreator.CreateSphere(radius, Vector3d(thick+radius, thick+radius, thick+radius), CSG_NUM_N, CSG_NUM_N);
    sph = meshBoolean.MeshMinus(sph, sph_in);

    Mesh* cubeM1 = meshCreator.CreateCuboid(Vector3d(0,thick+radius, thick+radius), Vector3d(thick, 100, 100));
    Mesh* cubeM2 = meshCreator.CreateCuboid(Vector3d(thick+radius, 0, thick+radius), Vector3d(100, thick, 100));
    Mesh* cubeM3 = meshCreator.CreateCuboid(Vector3d(thick+radius, thick+radius, 0), Vector3d(100, 100, thick));

    wallM = meshBoolean.MeshConnect(cyl1, cyl2);
    wallM = meshBoolean.MeshConnect(wallM, cyl3);
    wallM = meshBoolean.MeshConnect(wallM, sph);
    wallM = meshBoolean.MeshConnect(wallM, cubeM1);
    wallM = meshBoolean.MeshConnect(wallM, cubeM2);
    wallM = meshBoolean.MeshConnect(wallM, cubeM3);

    //hide wall
    double scale = 0.9;
    wallM->TransformMesh(GetTranslateMatrix(Vector3d((mx+Mx)/2, (my+My)/2, (mz+Mz)/2))* GetRotationMatrix(Vector3d(0,M_PI_2,0))
                * GetScaleMatrix(Vector3d (scale, scale, scale))*GetTranslateMatrix(Vector3d(-50,-50,-50)));
    wallM->TransformMesh(GetTranslateMatrix(Vector3d(0, LiftDis, 0)));
    wallM->saveOBJ(fileLoad+"walls.obj");

}

void CamLinksVisua::addCurveToViewer(MatrixXd &verM, MatrixXi &triM)
{
    auto outC = new Groove(0.5, sizeCur, Vector3d(1,0,0), Vector3d(0,0,0));
    outC->GenaratePitchCurve(endCurveA);
    Mesh* curM = outC->CreatClosedMesh();

    verM = curM->verM;
    triM = curM->triM;
}
