///////////////////////////////////////////////////////////////
//
// MeshObject.cpp
//
//   An object represented as a group of meshes, mainly for rendering
//
// by Peng Song
//
// 29/Nov/2020
//
// Modified by CHENG 2022/Aug/08
//
///////////////////////////////////////////////////////////////


#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include "Utility/HelpFunc.h"
#include "Mesh.h"
#include "MeshCreator.h"
#include "MeshObject.h"
#include "MeshBoolean.h"


///=========================================================================================///
///                                        Ground
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject::CreateGround(Vector3d origin, double size, int gridNum, int sampNum)
{
    double halfSize = 0.5f * size;
    double cyliRadius = 0.002 * size;


    /////////////////////////////////////////////////////////////////////////
    /// 1. Create 7 meshes for the axes (3 cylinders, 3 cones, and 1 sphere)

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    vector<Mesh*> meshList;

    for (int i = 0; i <= gridNum; i++)
    {
        double z = -halfSize + (i/(double)gridNum) * size;

        Vector3d point1 = origin + Vector3d( -halfSize, -halfSize/gridNum, z);
        Vector3d point2 = origin + Vector3d(  halfSize, -halfSize/gridNum ,z);

        Mesh *cylinder = meshCreator.CreateCylinder(point1, point2, cyliRadius, sampNum);

        meshList.push_back( cylinder );

    }

    for (int i = 0; i <= gridNum; i++)
    {
        double x = -halfSize + (i/(double)gridNum) * size;

        Vector3d point1 = origin + Vector3d( x, -halfSize/gridNum,  -halfSize);
        Vector3d point2 = origin + Vector3d( x, -halfSize/gridNum,   halfSize);

        Mesh *cylinder = meshCreator.CreateCylinder(point1, point2, cyliRadius, sampNum);

        meshList.push_back( cylinder );
    }

    /////////////////////////////////////////////////////////////////////////
    /// 1&1/2. Merge all cylinders as a ground
    /*
    Mesh* MGround = meshList[0];
    for(auto mcy : meshList)
        MGround = meshBoolean.MeshUnion(mcy, MGround);
    meshList.clear();
    meshList.push_back(MGround);
    */
    /////////////////////////////////////////////////////////////////////////
    /// 2. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;

    for (auto mesh : meshList)
    {
        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(mesh->verM, mesh->triM);

        viewerData.set_colors(Eigen::RowVector3d(0.8, 0.8, 0.8));

        viewerData.show_lines = unsigned (0);
        viewerData.face_based = true;

        viewerDataList.push_back( viewerData );
    }


    /////////////////////////////////////////////////////////////////////////
    /// 3. Release memory for the meshList

    for (auto & i : meshList)
    {
        delete i;
    }
    meshList.clear();

    return viewerDataList;
}




///=========================================================================================///
///                                         Axes
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject::CreateAxes(Vector3d origin, double size, int sampNum)
{
    /////////////////////////////////////////////////////////////////////////
    /// 1. Compute and specify parameters for the axes

    Vector3d xPoint1 = origin + Vector3d( 0.9*size, 0,         0);
    Vector3d xPoint2 = origin + Vector3d( 1.0*size, 0,         0);
    Vector3d yPoint1 = origin + Vector3d( 0,         0.9*size, 0);
    Vector3d yPoint2 = origin + Vector3d( 0,         1.0*size, 0);
    Vector3d zPoint1 = origin + Vector3d( 0,         0,         0.9*size);
    Vector3d zPoint2 = origin + Vector3d( 0,         0,         1.0*size);

    double cyliRadius = 0.02 * size;
    double coneRadius = 0.04 * size;
    double spheRadius = 0.04 * size;


    /////////////////////////////////////////////////////////////////////////
    /// 2. Create 7 meshes for the axes (3 cylinders, 3 cones, and 1 sphere)

    MeshCreator meshCreator;

    Mesh *cylinderX = meshCreator.CreateCylinder(origin, xPoint1, cyliRadius, sampNum);
    Mesh *cylinderY = meshCreator.CreateCylinder(origin, yPoint1, cyliRadius, sampNum);
    Mesh *cylinderZ = meshCreator.CreateCylinder(origin, zPoint1, cyliRadius, sampNum);

    Mesh *coneX = meshCreator.CreateCone(xPoint1, xPoint2, coneRadius, sampNum);
    Mesh *coneY = meshCreator.CreateCone(yPoint1, yPoint2, coneRadius, sampNum);
    Mesh *coneZ = meshCreator.CreateCone(zPoint1, zPoint2, coneRadius, sampNum);

    Mesh *sphere = meshCreator.CreateSphere(spheRadius, origin, sampNum, 2*sampNum);

    vector<Mesh*> meshList;

    meshList.push_back( cylinderX );
    meshList.push_back( coneX );

    meshList.push_back( cylinderY );
    meshList.push_back( coneY );

    meshList.push_back( cylinderZ );
    meshList.push_back( coneZ );

    meshList.push_back( sphere );


    /////////////////////////////////////////////////////////////////////////
    /// 3. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;

    for (int i = 0; i < meshList.size(); i++)
    {
        Mesh *mesh = meshList[i];

        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(mesh->verM, mesh->triM);

        if     ( i == 0 || i == 1 )    viewerData.set_colors(Eigen::RowVector3d(0.9, 0.4, 0.4));
        else if( i == 2 || i == 3 )    viewerData.set_colors(Eigen::RowVector3d(0.4, 0.9, 0.4));
        else if( i == 4 || i == 5 )    viewerData.set_colors(Eigen::RowVector3d(0.4, 0.4, 0.9));
        else                           viewerData.set_colors(Eigen::RowVector3d(0.6, 0.6, 0.6));

        viewerData.show_lines = unsigned (0);
        viewerData.face_based = true;

        viewerDataList.push_back( viewerData );
    }


    /////////////////////////////////////////////////////////////////////////
    /// 4. Release memory for the meshList

    for (auto & i : meshList)
    {
        delete i;
    }
    meshList.clear();

    return viewerDataList;
}




///=========================================================================================///
///                               Fuction for testing MeshCreator
///=========================================================================================///

vector<igl::opengl::ViewerData> MeshObject:: Function_Test()
{
    /////////////////////////////////////////////////////////////////////////
    /// 1. Create meshes for debug

    MeshCreator meshCreator;
    //Mesh *cuboid = meshCreator.CreateCuboid();
    //Mesh *sphere = meshCreator.CreateSphere(1.0, 10, 20);
    Mesh *sphere = meshCreator.CreateSphere(1.0, Vector3d(0, 2, 0), 100, 200);

    //Mesh *cylinder = meshCreator.CreateCylinder(2.0, 0.2, 100);
    Mesh *cylinder = meshCreator.CreateCylinder(Vector3d(0,0,0), Vector3d(1,1,1), 0.1, 100);

    Mesh *cone = meshCreator.CreateCone(Vector3d(1,1,1), Vector3d(1.2,1.2,1.2), 0.2, 100);
    //Mesh *cone = meshCreator.CreateCone(2.0, 2.0, 100);

    Mesh *cuboid = meshCreator.CreateCuboid(Vector3d(-1, 0.1, -1), Vector3d(1, 0.5, 1));
    // Mesh *cuboid = meshCreator.CreateCuboid(Vector3d(2, 0.2, 4));

    Mesh *fan = meshCreator.CreateFan(2.0, 0.6, 0.5, 0.4);

    vector<Mesh*> meshList;

    meshList.push_back( sphere );
    meshList.push_back( cylinder );

    meshList.push_back( cone );
    meshList.push_back( cuboid );

    meshList.push_back( fan );


    /////////////////////////////////////////////////////////////////////////
    /// 2. Create viewerData for each mesh to render it

    vector<igl::opengl::ViewerData> viewerDataList;
    for (auto mesh : meshList)
    {
        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(mesh->verM, mesh->triM);

        viewerData.show_lines = unsigned (0);
        viewerData.face_based = true;

        viewerDataList.push_back( viewerData );
    }


    /////////////////////////////////////////////////////////////////////////
    /// 3. Release memory for the meshList

    for (auto & i : meshList)
    {
        delete i;
    }
    meshList.clear();

    return viewerDataList;
}