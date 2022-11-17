//
// Created by cheng on 29/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_CAMLINKSVISUA_H
#define SPATIALLINKAGES_CAMLINKSVISUA_H

#include "CamPart/Cam1R.h"
#include "CamPart/Cam2R.h"
#include "CamPart/CamC.h"
#include "CamPart/CamP.h"
#include "Mesh/Mesh.h"
#include "Opt/IKLMeigen.h"
#include "Opt/PSO_PC.h"
#include "Utility/HelpTypedef.h"

class CamLinksVisua {

public:
    CamLinksVisua();
    ~CamLinksVisua();

public:
    string        fileLoad;
    vector<Mesh*> linksMesh;
    Mesh        * cam2Mesh;
    vector<Mesh*> grdM;
    Mesh        * wallM;

    Cam1R   *cam1r;
    Cam2R   *cam2r;
    CamC    *camC;
    CamP    *camP;
    vector<Mesh*> camSups;
    PSO_PC *pso_pc;

    vector<Vector3d> endCurveA;
    Vector3d EndA;

    vector<Matrix4d> motionMats;

    double LiftDis;
    int sizeCur;

    int fileID;

private:
    vector<char> jTypes;
    vector<Matrix4d> geoMats;
    vector<double> gBestPara;


    vector<Vector3d> targetCurve;

    double duraTime;
    double energy;
    int numCtrs;

    VectorXd preX;
    LMFunctorIK IKfunc;

    Vector3d linkSupVecA, linkSupVecB;
    Mesh* link1_cam, *link4_cam;


public:
    void ReadLinks();
    void InitialLinksMesh();
    void InitialCamMesh();
    void ConnectMesh();
    void AddWall();
    void AddGround();
    void UpdateModel(int frame, vector<MatrixXd> &partsVec);
    void UpdateMotion(iglViewer &viewer, int frame);
    void addCurveToViewer(MatrixXd &verM, MatrixXi &triM);
    void AppendMechs(iglViewer &viewer);
    void AppendCurve(iglViewer &viewer);
    void ClearViewerList(iglViewer &viewer);
};


#endif //SPATIALLINKAGES_CAMLINKSVISUA_H
