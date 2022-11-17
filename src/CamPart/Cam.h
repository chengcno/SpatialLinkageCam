//////////////////////////////////////////////////
//Cam.h
//
// Create model of Cam
//
// Created by Yingjie Cheng on 2020/11/15.
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////

#ifndef _CAM_H
#define _CAM_H

#include "Groove.h"

class Mesh;

class Cam {
public:
    Cam();
    Cam(double radius, int Nsize);
    Cam(double radius, const vector<Vector3d>& _pitchCurve);
    Cam(double radius, const vector<Vector3d>& _pitchCurve, double Ualpha, double Dbeta);
    Cam(double radius, vector<Vector3d> _pitchCurve, vector<double> _alphas, vector<double> _betas);
    Cam(double radius, vector<Vector3d> _pitchCurve, vector<pair<double,Vector3d>> _FollOrient);
    ~Cam();

public:
    Mesh*       camMesh;

    int         numN;
    double      rCamAxis;
    double      rCFball;
    double      angleTor;
    Groove      *_groove;
    double      GrooveRadius;      //radius of groove

    int         CurveSize;         //Size of polyline(pitchCurve)
    vector<Vector3d>                Pitchcurve;//pitchcurve polyline
    vector<pair<double,Vector3d>>   FollOrient;
    Mesh* PitchTube;            // Tube for PitchCurve

    vector<double> alphas;      // Up open angle
    vector<double> betas;       // Down open angle
    double Ualpha,Dbeta;          // up,down open angle

    double zmax_,zmin_;                // Cam Thickness

public:
    void CreatPartMesh() ;
    Groove* GetGroove() ;
    Mesh* GetMinusGroove() ;
    void GetCamThickness(double &zmax, double &zmin) ;
    Mesh* GetPitchTube();


private:
    void FromGrooveToCam();
    Mesh* CreatePitchTube();

    ///open angle
    void ComputeCrossAngle();
    Vector3d GetPolyT(int i);
    Vector3d GetPolyN(int i);
    double getDegree(double cosx, double sinx);
};


#endif //CAMMECHCREATOR_CAM_H
