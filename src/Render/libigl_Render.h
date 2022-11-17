///////////////////////////////////////////////////////////////
//
// libigl_Render.h
//
//   Rendering various 3D contents with libigl
//
// by Peng Song
//
// 01/Dec/2020
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////////////////


#ifndef _LIBIGL_RENDER_H
#define _LIBIGL_RENDER_H


#include "Utility/HelpTypedef.h"
#include "Utility/HelpDefine.h"


using namespace std;
using namespace Eigen;



class libigl_Render
{
public:
    int mechMeshNum;
    int groundMeshNum;
    int axesMeshNum;


public:
    libigl_Render() = default;
    ~libigl_Render() {};

    /// Render Objects
    void RenderScene(iglViewer &viewer, vector<iglViewerData> myViewerData);
    void DrawMechanism(iglViewer &viewer, const vector<iglViewerData>& myViewerData);
    void DrawGround(iglViewer &viewer, Vector3d origin, double size, int gridNum, int sampNum);
    void DrawWorldAxes(iglViewer &viewer, Vector3d origin, double size, int sampNum);
    void DrawMeshForDebug(iglViewer &viewer);

    /// Show/hide Objects
    void ShowMyModels(iglViewer &viewer, bool isVisible);
    void ShowInCurve(iglViewer &viewer, bool isVisible);
    void ShowGround(iglViewer &viewer,bool isVisible) const;
    void ShowAxes(iglViewer &viewer, bool isVisible) const;

    /// Set Camera
    void SetCamera(iglViewer &viewer, float zoom, Vector3f eyePos);
    void ResetCamera(iglViewer &viewer);

    /// Utility
    void AppendDataToViewer(iglViewer &viewer, vector<iglViewerData> viewerDataList);
};


#endif //_LIBIGL_RENDER_H
