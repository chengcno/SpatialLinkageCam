//
// Created by cheng on 19/12/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_GROOVE_H
#define SPATIALLINKAGES_GROOVE_H

#include "Mesh/Mesh.h"
#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

class Groove {
public:
    Groove();
    Groove(double _radius, int _Nsize, Vector3d _orient = Vector3d(0,0,1), Vector3d _center = Vector3d(0,0,0));
    ~Groove();

public:
    double           Gap_tor_L;
    int              numN, numM;
    double           thickEdge;
    double           radius_;                       //radius of groove
    vector<Vector3d> pitchCurve;                    //Center curve of groove
    vector<double>   curvePara;                     //Para of curve in [0, 2Pi]
    int              NSize;                         //Size of polyline (pitch curve)
    vector<pair<double, double>> openAngles;        //open angle of groove [-Pi, Pi] -> [alpha, beta]
    double Ualpha, Dbeta;                           // UP,down open angle

    Vector3d vecOrient;                         // Orientation, i.e. rotation axis
    Vector3d verCenter;                         // Center Point

    Mesh*   GrooveMesh_;                        // Mesh of groove
    Mesh*   MinusMesh;                          // Minus Mesh of groove

    double           foll_radius;                   //radius of follower cylinder

    int additionalSize;                               //For storing edge mesh size

private:
    vector<Vector3d> tangentPitchCurve;             // tangent of pitch Curve
    vector<Vector3d> normalPitchCurve;              // normals of pitch curve

public:
    ///generate pitch curve
    void GeneratePitchCurve();
    void GenaratePitchCurve(vector<Vector3d> _pitchcurve);
    void GenaratePitchCurve(vector<Vector3d> _pitchcurve, vector<double> _paras);

    ///create groove mesh
    void CreateGrooveMesh();
    void CreateGrooveMesh(double alpha, double beta);
    void CreateGrooveMesh(vector<double> alphas, vector<double> betas);
private:
    int CreateverList(double alpha,double beta);
    int CreateverList(vector<double> alphas, vector<double> betas);
    void CreatetriList(int addSize) ;

public:
    void RefineGrooveOpen(vector<pair<double,double>> _openAngles);

    /// Get {C(s),T(S),N(S)} for any s
    Vector3d GetPitchCurve(double s);
    Vector3d GetCurveTangent(double s);
    Vector3d GetCurveNormal(double s);

private:
    ///utility
    double ParaIn0to2Pi(double s);
    int FindInParas(double s);
    void GetParaId(double s, int &id, int &id2, double &wL, double &wR);

public:
    Mesh* CreatClosedMesh();
};


#endif //SPATIALLINKAGES_GROOVE_H
