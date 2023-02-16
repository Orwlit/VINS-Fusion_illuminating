/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x); //原t
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3); //原q

    Eigen::Map<const Eigen::Vector3d> dp(delta); //t的增量，3自由度

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3)); //q的增量，3自由度

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta); //新t
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3); //新q

    p = _p + dp; //t相加
    q = (_q * dq).normalized(); //四元数右乘，4自由度的_q和3自由度的dq相乘后，得到4自由度的q；为了q可以表示旋转矩阵，需要归一化操作，自由度变为3
    //至此得到4自由度的q（已归一化，可表示旋转矩阵）和3自由度的p（表示空间位置）

    return true;
}

bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

int PoseLocalParameterization::GlobalSize() const {
    return 7; //x的维度
}
int PoseLocalParameterization::LocalSize() const {
    return 6; //扰动delta的维度
}
