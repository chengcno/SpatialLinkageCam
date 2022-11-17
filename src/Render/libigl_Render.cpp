///////////////////////////////////////////////////////////////
//
// libigl_Render.cpp
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


#include <utility>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include "libigl_Render.h"
#include "Mesh/MeshObject.h"


///=========================================================================================///
///                                       Render Objects
///=========================================================================================///

void libigl_Render::RenderScene(iglViewer &viewer, vector<iglViewerData> myViewerData)
{
    viewer.data_list.clear();

    DrawMechanism(viewer, myViewerData); // We have to draw mechanism first (for animating it properly)
    DrawGround(viewer, Vector3d(0,0,0), 300.0, 6, 20);
    DrawWorldAxes(viewer, Vector3d(0,0,0), 50.0, 40);

    //DrawMeshForDebug(viewer);
}

void libigl_Render::DrawMechanism(iglViewer &viewer, const vector<iglViewerData>& myViewerData)
{
    viewer.data_list = myViewerData;

    mechMeshNum = myViewerData.size();
}

void libigl_Render::DrawGround(iglViewer &viewer, Vector3d origin, double size, int gridNum, int sampNum)
{
    MeshObject meshObject;
    vector<igl::opengl::ViewerData> viewerDataList =  meshObject.CreateGround(std::move(origin), size, gridNum, sampNum);

    AppendDataToViewer( viewer, viewerDataList );

    groundMeshNum = viewerDataList.size();
}

void libigl_Render::DrawWorldAxes(iglViewer &viewer, Vector3d origin, double size, int sampNum)
{
    MeshObject meshObject;
    vector<igl::opengl::ViewerData> viewerDataList =  meshObject.CreateAxes(std::move(origin), size, sampNum);

    AppendDataToViewer( viewer, viewerDataList );

    axesMeshNum = viewerDataList.size();
}

void libigl_Render::DrawMeshForDebug(iglViewer &viewer)
{
    MeshObject meshObject;
    vector<igl::opengl::ViewerData> viewerDataList =  meshObject.Function_Test();

    AppendDataToViewer( viewer, viewerDataList );
}




///=========================================================================================///
///                                     Show/hide Objects
///=========================================================================================///
void libigl_Render::ShowInCurve(iglViewer &viewer, bool isVisible)
{
    viewer.data_list[viewer.data_list.size() - 1].is_visible = isVisible;
}

void libigl_Render::ShowMyModels(iglViewer &viewer, bool isVisible)
{
    for(int i = 0; i < mechMeshNum; i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
    for(int i = mechMeshNum + groundMeshNum + axesMeshNum; i < viewer.data_list.size() - 1; i++)
        viewer.data_list[i].is_visible = isVisible;
}

void libigl_Render::ShowGround(iglViewer &viewer, bool isVisible) const
{
    for(int i = mechMeshNum; i < (mechMeshNum + groundMeshNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}

void libigl_Render::ShowAxes(iglViewer &viewer, bool isVisible) const
{
    for(int i = mechMeshNum + groundMeshNum; i < (mechMeshNum + groundMeshNum + axesMeshNum); i++)
    {
        viewer.data_list[i].is_visible = isVisible;
    }
}


///=========================================================================================///
///                                        Set Camera
///=========================================================================================///

void libigl_Render::SetCamera(iglViewer &viewer, float zoom, Vector3f eyePos)
{
    // change the camera parameters here
    // the rest of the parameters can be found at line 127(ViewerCore.h)

    viewer.core().camera_zoom  =  zoom;
    viewer.core().camera_eye   =  eyePos;

    viewer.core().camera_dfar = 10000;
    viewer.core().camera_dnear = 0.01;
    Eigen::Quaternionf initAngle;
    initAngle.x() = -0.00145438;
    initAngle.y() =  -0.991801;
    initAngle.z() = 0.0103245;
    initAngle.w() = 0.127365;

    viewer.core().light_position = Vector3f(0.0, 0.0, 0.3);
    viewer.core().lighting_factor = 0.6;
}

void libigl_Render::ResetCamera(iglViewer &viewer)
{
    viewer.core().camera_zoom  =  0.2;
    viewer.core().camera_eye   =  Vector3f(-7.5, 3, 3);
    viewer.core().camera_eye   =  Vector3f(-5, 2, 2);
    viewer.core().camera_dfar = 10000;
    viewer.core().camera_dnear = 0.01;
    viewer.core().camera_translation = Vector3f (-8, -2, -2 );

    //ro: -0.0599667 , -0.925036 , -0.0344004 , 0.373536
    Eigen::Quaternionf initAngle;
    initAngle.x() = -0.0408822;
    initAngle.y() =  -0.912295;
    initAngle.z() = -0.0300458;
    initAngle.w() = 0.406379;
    viewer.core().trackball_angle = initAngle;
}


///=========================================================================================///
///                                        Ground
///=========================================================================================///

void libigl_Render::AppendDataToViewer(iglViewer &viewer, vector<igl::opengl::ViewerData> viewerDataList)
{
    for (const auto & i : viewerDataList)
    {
        viewer.data_list.push_back( i );
    }
}