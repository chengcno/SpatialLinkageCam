//
// Created by cheng on 26/11/21.
// Modified by CHENG 2022/Aug/08
//

#ifndef SPATIALLINKAGES_LMEIGEN_H
#define SPATIALLINKAGES_LMEIGEN_H

#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

#ifndef Uniscale
#define Uniscale 60
#endif//Uniscale

struct LMFunctor
{

    vector<Matrix4d> JointInitMat;
    vector<char> JointType;
    vector<double> InputBarX;

    // f = f(x) = ( ,fi(x), )
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

    // Jco = dfi/dxj
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const;

    // Number of data points, i.e. values.
    int m;

    // Returns 'm', the number of values.
    [[nodiscard]] int values() const { return m; }

    // The number of parameters, i.e. inputs.
    int n;

    // Returns 'n', the number of inputs.
    [[nodiscard]] int inputs() const { return n; }

    Matrix4d LoopMatrix(const Eigen::VectorXd &x, vector<Matrix4d> &linkMats) const;
};


#endif //SPATIALLINKAGES_LMEIGEN_H
