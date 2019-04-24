#pragma once

#include <Eigen/Eigen>
#include <cmath>

#include "se3_ops.h"
#include "se3quat.h"

using std::cos;
using std::sin;
using std::pow;

namespace g2o {

Eigen::Matrix3d JacobianLeftSO3(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    Eigen::Matrix3d A = skew(phi);
    Matrix3d J_l = Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta * theta) * A + (theta - sin(theta)) / (theta * theta * theta) * A * A;
    if(J_l.hasNaN()) {
        return Matrix3d::Zero();
    }
    return J_l;
}

Eigen::Matrix3d JacobianLeftInvSO3(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    return Eigen::Matrix3d::Identity() - (1 / 2) * skew(phi) + (1 / (theta * theta) - (1 + cos(theta) / (2 * theta * sin(theta)))) * skew(phi) * skew(phi);
}

Matrix6d JacobianLeftSE3(const Vector6d& twist) {
    Eigen::Vector3d phi = twist.topRows<3>();
    Eigen::Vector3d ro = twist.bottomRows<3>();
    double theta = phi.norm();

    double a = (theta - sin(theta)) / pow(theta, 3);
    double b = (1 - theta * theta / 2 - cos(theta)) / pow(theta, 4);
    double c = (theta - sin(theta) - pow(theta, 3) / 6) / pow(theta, 5);
    Eigen::Matrix3d Q = (1 / 2) * skew(ro)
                                            + a * (skew(phi) * skew(ro) + skew(ro) * skew(phi) + skew(phi) * skew(ro) * skew(phi))
                                            - b * (skew(phi) * skew(phi) * skew(ro) + skew(ro) * skew(phi) * skew(phi) - 3 * skew(phi) * skew(ro) * skew(phi))
                                            - (1 / 2) * (b - 3 * c) * (skew(phi) * skew(ro) * skew(phi) * skew(phi) + skew(phi) * skew(phi) * skew(ro) * skew(phi));

    Matrix6d J_l = Matrix6d::Zero();
    J_l.block<3,3>(0, 0) = JacobianLeftSO3(phi);
    J_l.block<3, 3>(3, 0) = Q;
    J_l.block<3, 3>(3, 3) = JacobianLeftSO3(phi);

    if(J_l.hasNaN()) {
        return Matrix6d::Zero();
    }

    return J_l;
}

Matrix6d JacobianLeftInvSE3(const Vector6d& twist) {
    Matrix6d J_l_inv = JacobianLeftSE3(twist).inverse();
    if(J_l_inv.hasNaN()) {
        return Matrix6d::Zero();
    }
    return J_l_inv;
}

Matrix6d JacobianRightSE3(const Vector6d& twist) {
    Matrix6d J_r = (SE3Quat::exp(-twist)).adj() * JacobianLeftSE3(twist);
    if(J_r.hasNaN()) {
        return Matrix6d::Zero();
    }

    return J_r;
}

Matrix6d JacobianRightInvSE3(const Vector6d& twist) {
    Matrix6d J_r_inv = JacobianRightSE3(twist).inverse();
    if(J_r_inv.hasNaN()) {
        return Matrix6d::Zero();
    }

    return J_r_inv;
}

}

