//
// Created by cheng on 26/11/21.
// Modified by CHENG 2022/Aug/08
//


#include "LMeigen.h"
#include "Utility/HelpFunc.h"

int LMFunctor::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{

        fvec.resize(m);
        //loop
        vector<Matrix4d> linkMats;
        Matrix4d linkM5 = LoopMatrix(x, linkMats);
        linkM5 = linkM5 - Matrix4d::Identity();
        for(int i=0;i<3;i++)
        {
            for (int j = 0; j < 3; ++j)
            {
                fvec(i*3+j) = linkM5(i,j);
            }

            fvec(9+i) = linkM5(i,3)/Uniscale;
        }
        fvec(12) = InputBarX[0] - x(0);
        fvec(13) = InputBarX[1] - x(1);
        fvec(14) = InputBarX[2] - x(8);

    return 0;
}

int LMFunctor::df(const VectorXd &x, MatrixXd &fjac) const {

        fjac.resize(m, n);
        fjac.setZero();
        fjac(12,0) = -1;
        fjac(13, 1) = -1;
        fjac(14, 8) = -1;

        //Loop
        VectorXd epc;
        double tol = 0.00001;
        epc.resize(n);
        vector<Matrix4d> linkMats;
        for(int itr=0;itr<n;itr++)
        {
            epc.setZero();
            epc(itr) = tol;
            Matrix4d M5plus = LoopMatrix(x+epc, linkMats);
            Matrix4d M5mini = LoopMatrix(x-epc, linkMats);
            Matrix4d D_M5_i = (M5plus - M5mini)/(2.0*tol);

            for(int i=0;i<3;i++)
            {
                for (int j = 0; j < 3; ++j)
                {
                    fjac(i*3+j, itr) = D_M5_i(i,j);
                }
                fjac(9+i, itr) = D_M5_i(i,3)/Uniscale;
            }
        }

        return 0;
}

Matrix4d LMFunctor::LoopMatrix(const VectorXd &x, vector<Matrix4d> &linkMats) const
{
    linkMats.clear();
    linkMats.push_back(Matrix4d::Identity());

    int itr = 0;
    //J1
    Matrix4d linkM1;
    if(JointType[0] == 'u')
    {
        double alpha = x[itr]; itr++;
        double beta = x[itr]; itr++;
        Vector3d U_initPinN, U_initRotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[0], U_initPinN);
        MultiplyVector(Vector3d(0,0,1), JointInitMat[0], U_initRotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[0].block(0,3,3,1), linkMats[0], rotCen);
        MultiplyVector(U_initRotN, linkMats[0], U_initRotN);
        MultiplyVector(U_initPinN, linkMats[0], U_initPinN);
        linkM1 = GetRotationMatrix(alpha, U_initPinN, rotCen) * GetRotationMatrix(beta, U_initRotN, rotCen);
    }
    if(JointType[0] == 'c')
    {
        double theta = x[itr]; itr++;
        double len = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[0], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[0].block(0,3,3,1), linkMats[0], rotCen);
        MultiplyVector(R_rotN, linkMats[0], R_rotN);
        linkM1 = GetTranslateMatrix(R_rotN*len)* GetRotationMatrix(theta, R_rotN, rotCen);
    }
    linkMats.push_back(linkM1);

    //J2
    Matrix4d linkM2;
    if(JointType[1] == 'r')
    {
        double theta = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[1], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[1].block(0,3,3,1), linkMats[1], rotCen);
        MultiplyVector(R_rotN, linkMats[1], R_rotN);
        linkM2 = GetRotationMatrix(theta, R_rotN, rotCen)*linkM1;
    }
    if(JointType[1] == 'p')
    {
        double len = x[itr];itr++;
        Vector3d T_line;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[1], T_line);
        MultiplyVector(T_line, linkMats[1], T_line);
        linkM2 = GetTranslateMatrix(T_line*len) * linkM1;
    }
    if(JointType[1] == 's')
    {
        double alpha, beta, gamma;
        alpha = x[itr]; itr++;
        beta = x[itr]; itr++;
        gamma = x[itr]; itr++;
        Vector3d S_RotX, S_RotY, S_RotZ;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[1], S_RotX);
        MultiplyVector(Vector3d(0,1,0), JointInitMat[1], S_RotY);
        MultiplyVector(Vector3d(0,0,1), JointInitMat[1], S_RotZ);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[1].block(0,3,3,1), linkMats[1], rotCen);
        linkM2 = GetRotationMatrix(alpha, S_RotX, rotCen)* GetRotationMatrix(beta, S_RotY, rotCen)*
                 GetRotationMatrix(gamma, S_RotZ, rotCen)*linkM1;
    }
    if(JointType[1] == 'u')
    {
        double alpha = x[itr]; itr++;
        double beta = x[itr]; itr++;
        Vector3d U_initPinN, U_initRotN;
        MultiplyVector(Vector3d(0,0,1), JointInitMat[1], U_initPinN);
        MultiplyVector(Vector3d(0,1,0), JointInitMat[1], U_initRotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[1].block(0,3,3,1), linkMats[1], rotCen);
        MultiplyVector(U_initRotN, linkMats[1], U_initRotN);
        MultiplyVector(U_initPinN, linkMats[1], U_initPinN);
        linkM2 = GetRotationMatrix(alpha, U_initPinN, rotCen) * GetRotationMatrix(beta, U_initRotN, rotCen)
                 *linkM1;
    }
    if(JointType[1] == 'c')
    {
        double theta = x[itr]; itr++;
        double len = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[1], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[1].block(0,3,3,1), linkMats[1], rotCen);
        MultiplyVector(R_rotN, linkMats[1], R_rotN);
        linkM2 = GetTranslateMatrix(R_rotN*len)* GetRotationMatrix(theta, R_rotN, rotCen)
                 *linkM1;
    }
    linkMats.push_back(linkM2);

    //J3
    Matrix4d linkM3;
    if(JointType[2] == 'r')
    {
        double theta = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[2], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[2].block(0,3,3,1), linkMats[2], rotCen);
        MultiplyVector(R_rotN, linkMats[2], R_rotN);
        linkM3 = GetRotationMatrix(theta, R_rotN, rotCen)*linkM2;
    }
    if(JointType[2] == 'p')
    {
        double len = x[itr];itr++;
        Vector3d T_line;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[2], T_line);
        MultiplyVector(T_line, linkMats[2], T_line);
        linkM3 = GetTranslateMatrix(T_line*len) * linkM2;
    }
    if(JointType[2] == 's')
    {
        double alpha, beta, gamma;
        alpha = x[itr]; itr++;
        beta = x[itr]; itr++;
        gamma = x[itr]; itr++;
        Vector3d S_RotX, S_RotY, S_RotZ;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[2], S_RotX);
        MultiplyVector(Vector3d(0,1,0), JointInitMat[2], S_RotY);
        MultiplyVector(Vector3d(0,0,1), JointInitMat[2], S_RotZ);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[2].block(0,3,3,1), linkMats[2], rotCen);
        linkM3 = GetRotationMatrix(alpha, S_RotX, rotCen)* GetRotationMatrix(beta, S_RotY, rotCen)*
                 GetRotationMatrix(gamma, S_RotZ, rotCen)*linkM2;
    }
    if(JointType[2] == 'u')
    {
        double alpha = x[itr]; itr++;
        double beta = x[itr]; itr++;
        Vector3d U_initPinN, U_initRotN;
        MultiplyVector(Vector3d(0,0,1), JointInitMat[2], U_initPinN);
        MultiplyVector(Vector3d(0,1,0), JointInitMat[2], U_initRotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[2].block(0,3,3,1), linkMats[2], rotCen);
        MultiplyVector(U_initRotN, linkMats[2], U_initRotN);
        MultiplyVector(U_initPinN, linkMats[2], U_initPinN);
        linkM3 = GetRotationMatrix(alpha, U_initPinN, rotCen) * GetRotationMatrix(beta, U_initRotN, rotCen)
                 *linkM2;
    }
    if(JointType[2] == 'c')
    {
        double theta = x[itr]; itr++;
        double len = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[2], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[2].block(0,3,3,1), linkMats[2], rotCen);
        MultiplyVector(R_rotN, linkMats[2], R_rotN);
        linkM3 = GetTranslateMatrix(R_rotN*len)* GetRotationMatrix(theta, R_rotN, rotCen)
                 *linkM2;
    }
    linkMats.push_back(linkM3);

    //J4
    Matrix4d linkM4;
    if(JointType[3] == 'r')
    {
        double theta = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[3], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[3].block(0,3,3,1), linkMats[3], rotCen);
        MultiplyVector(R_rotN, linkMats[3], R_rotN);
        linkM4 = GetRotationMatrix(theta, R_rotN, rotCen)*linkM3;
    }
    if(JointType[3] == 'p')
    {
        double len = x[itr];itr++;
        Vector3d T_line;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[3], T_line);
        MultiplyVector(T_line, linkMats[3], T_line);
        linkM4 = GetTranslateMatrix(T_line*len) * linkM3;
    }
    if(JointType[3] == 's')
    {
        double alpha, beta, gamma;
        alpha = x[itr]; itr++;
        beta = x[itr]; itr++;
        gamma = x[itr]; itr++;
        Vector3d S_RotX, S_RotY, S_RotZ;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[3], S_RotX);
        MultiplyVector(Vector3d(0,1,0), JointInitMat[3], S_RotY);
        MultiplyVector(Vector3d(0,0,1), JointInitMat[3], S_RotZ);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[3].block(0,3,3,1), linkMats[3], rotCen);
        linkM4 = GetRotationMatrix(alpha, S_RotX, rotCen)* GetRotationMatrix(beta, S_RotY, rotCen)*
                 GetRotationMatrix(gamma, S_RotZ, rotCen)*linkM3;
    }
    if(JointType[3] == 'u')
    {
        double alpha = x[itr]; itr++;
        double beta = x[itr]; itr++;
        Vector3d U_initPinN, U_initRotN;
        MultiplyVector(Vector3d(0,0,1), JointInitMat[3], U_initPinN);
        MultiplyVector(Vector3d(0,1,0), JointInitMat[3], U_initRotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[3].block(0,3,3,1), linkMats[3], rotCen);
        MultiplyVector(U_initRotN, linkMats[3], U_initRotN);
        MultiplyVector(U_initPinN, linkMats[3], U_initPinN);
        linkM4 = GetRotationMatrix(alpha, U_initPinN, rotCen) * GetRotationMatrix(beta, U_initRotN, rotCen)
                *linkM3;
    }
    if(JointType[3] == 'c')
    {
        double theta = x[itr]; itr++;
        double len = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[3], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[3].block(0,3,3,1), linkMats[3], rotCen);
        MultiplyVector(R_rotN, linkMats[3], R_rotN);
        linkM4 = GetTranslateMatrix(R_rotN*len)* GetRotationMatrix(theta, R_rotN, rotCen)
                *linkM3;
    }
    linkMats.push_back(linkM4);

    //J5
    Matrix4d linkM5;
    if(JointType[4] == 'r')
    {
        double theta = x[itr]; itr++;
        Vector3d R_rotN;
        MultiplyVector(Vector3d(0,1,0), JointInitMat[4], R_rotN);
        Vector3d rotCen;
        MultiplyPoint(JointInitMat[4].block(0,3,3,1), linkMats[4], rotCen);
        MultiplyVector(R_rotN, linkMats[4], R_rotN);
        linkM5 = GetRotationMatrix(theta, R_rotN, rotCen)
                *linkM4;
    }
    if(JointType[4] == 'p')
    {
        double len = x[itr];itr++;
        Vector3d T_line;
        MultiplyVector(Vector3d(1,0,0), JointInitMat[4], T_line);
        MultiplyVector(T_line, linkMats[4], T_line);
        linkM5 = GetTranslateMatrix(T_line*len) * linkM4;
    }
    linkMats.push_back(linkM5);

    return linkM5;
}
