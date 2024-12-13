#include "header/utility.hpp"
#include <cmath>


Eigen::Matrix3d v2t(const pose2d& pose){
    
    Eigen::Matrix3d T;
    T.setZero();

    T(0,0) = cos(pose[2]);
    T(0,1) = -sin(pose[2]);
    T(1,0) = sin(pose[2]);
    T(1,1) = cos(pose[2]);

    T(0,2) = pose[0];
    T(1,2) = pose[1];
    T(2,2) = 1;
    return T;
}

pose2d t2v(const Eigen::Matrix3d& T){
    pose2d pose;
    pose[0] = T(0,2);
    pose[1] = T(1,2);
    pose[2] = std::atan2(T(1,0), T(0,0));
    return pose;
}

Eigen::Vector2d Jac_atan2(const Eigen::Vector2d& x){
    Eigen::Vector2d jac;
    jac[0] = -x[1]/(x[0]*x[0] + x[1]*x[1]);
    jac[1] = x[0]/(x[0]*x[0] + x[1]*x[1]);
    return jac;
}