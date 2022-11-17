//
// Created by cheng on 5/8/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_JOINT_H
#define SPATIALLINKAGES_JOINT_H

#include "Mesh/Mesh.h"

class Joint {

public:
    enum JointType
    {
        R_joint,
        U_joint,
        P_joint,
        S_joint,
        RR_joint,
        RP_joint,
        PR_joint,
        RRP_joint,
        PRR_joint,
        SP_joint,
        PS_joint,
        C_joint
    };

public:
    Joint(JointType _type);
    Joint(char jType);
    ~Joint();

public:
    JointType jointType;
    Vector3d initPosition;
    Matrix3d initRotationM;

    Mesh* partA;
    Mesh* partB;
    Mesh* partMid;

    Mesh* partAll;

    int linkA_id,linkB_id;
    Vector3d partA_touch, partB_touch;
    int N_size, M_size;

public:
    void CreateModel();
    void CreateModel(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateSimpleModel();
    void CreateSimpleModel(const Vector3d &touchPA, const Vector3d &touchPB);
    void InitPose(int _linkA_id, int _linkB_id, Matrix4d initMatrix);
    void ResetPose();

public:
    void CreateSjoint();
    void CreateSjoint(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateSimpleS();
    void CreateSimpleS(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateRjoint();
    void CreateRjoint(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateSimpleR();
    void CreateSimpleR(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreatePjoint();
    void CreateSimpleP();
    void CreateSimpleP(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateCjoint();
    void CreateSimpleC();
    void CreateSimpleC(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateUjoint();
    void CreateUjoint(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateSimpleU();
    void CreateSimpleU(const Vector3d &touchPA, const Vector3d &touchPB);
    void CreateRRjoint();
    void CreatePRjoint();
    void CreateRRPjoint();
};


#endif //SPATIALLINKAGES_JOINT_H
