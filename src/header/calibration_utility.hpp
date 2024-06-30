#pragma once 
#include <vector>
#include <ostream>
#include <eigen3/Eigen/Core>
#include <cstdint>  // define UINT32_MAX

// this file contains only utils funcition and alias to use around all the project
namespace cal_lib{

    constexpr int MAX_STEERING_TICK = 8192;
    constexpr int MAX_TRACTION_TICK = 5000;

    using pose2d =  Eigen::Vector3d;
    typedef std::vector<pose2d> trajectory;
    
    using tick = Eigen::Vector<u_int32_t, 2>;
    typedef std::vector<tick> tick_logs;

    using state_vector = Eigen::Vector4d;

}