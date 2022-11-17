//
// Created by cheng on 6/7/22.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_IOMECHS_H
#define SPATIALLINKAGES_IOMECHS_H

#include <vector>
#include <Eigen/Eigen>
#include <string>
#include <iostream>
#include <fstream>
#include "Utility/HelpTypedef.h"
#include "Mesh/Mesh.h"


class IOMechs {
public:
    IOMechs();
    ~IOMechs();

public:
    int sizeMesh;
    vector<Mesh*> meshList;
    vector<RowVector3d> colorList;

    int motionFrames;
    vector<vector<Matrix4d>> motionMats;

    string inputCurveName;
    string inputMechsName;
    vector<string> inModelNameList;
    vector<string> inMotionNameList;

    float motionVel;

public:
    void ClearViewerList(iglViewer &viewer);
    void ReadInputCurve(string curveName);
    void AppendReadMechs(iglViewer &viewer);

    void ReadInputMechs(string mechsName);

    void UpdateMotion(iglViewer &viewer, double frame);

private:
    void ReadModels();
    void ReadMotion();

    bool isCurve;

    RowVector3d colorTable[13] =
            {
            RowVector3d (0.8, 0.9, 0.9),
            RowVector3d (0.9, 0.8, 0.9),
            RowVector3d (0.9, 0.9, 0.8),

            RowVector3d (0.75, 0.85, 0.95),
            RowVector3d (0.75, 0.95, 0.85),
            RowVector3d (0.85, 0.75, 0.95),
            RowVector3d (0.85, 0.95, 0.75),
            RowVector3d (0.95, 0.85, 0.75),
            RowVector3d (0.95, 0.75, 0.85),

            RowVector3d (0.8, 0.8, 0.9),
            RowVector3d (0.9, 0.8, 0.8),
            RowVector3d (0.8, 0.9, 0.8),

            RowVector3d (0.8, 0.8, 0.8)
            };
};


#endif //SPATIALLINKAGES_IOMECHS_H
