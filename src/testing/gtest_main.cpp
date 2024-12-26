#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>   
#include <utility.hpp>
#include <iostream>

TEST(BoxPlusMinus, v2t_inverse_times_v2t_Is_Identity) {
    
    double tolerance = 1e-6;

    pose2d pose(1., 2., 3.);
    Eigen::Matrix3d out = (v2t(pose).inverse()*v2t(pose));
    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(out.isApprox(eye, tolerance));

}

TEST(BoxPlusMinus, vt2_of_t2v_inverse_times_t2v_is_zero) {
    
    double tolerance = 1e-6;

    pose2d pose(1., 2., 2324.);
    Eigen::Matrix3d first = v2t(pose);
    Eigen::Matrix3d second = v2t(pose).inverse();

    Eigen::Matrix3d out = second*first;
    pose2d out_expected(0., 0., 0.);

    EXPECT_TRUE((t2v(out) - out_expected).norm() < tolerance);

}