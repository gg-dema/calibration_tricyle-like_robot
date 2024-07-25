#pragma once 
#include <vector>

#include <eigen3/Eigen/Core>
#include <cstdint>  // define UINT32_MAX

// this file contains only utils funcition and alias to use around all the project
namespace cal_lib{

    constexpr int STEERING = 0;
    constexpr int TRACTION = 1;

    using Pose2d =  Eigen::Vector3d;
    typedef std::vector<Pose2d> trajectory;
    
    // tick 0 = steering
    // tick 1 = traction
    using Tick = Eigen::Vector<u_int64_t, 2>;
    typedef std::vector<Tick> tick_logs;

    using StateVector = Eigen::Vector4d;


    struct model_parameters{
        long double K_steer; 
        long double K_traction;
        double axis_lenght;
        double steer_offset; 
    };

    struct sensor_parameters{
        cal_lib::Pose2d position;
        // other params 
    };

}