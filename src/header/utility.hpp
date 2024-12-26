/**
 * @file utility.hpp
 * @brief a set of utility stuff for the project
 * @details type definition, math transformation, file saving utility
 * 
 * @author Gabriele G. Di Marzo [github : gg-dema]
 */

#pragma once
#include <vector>
#include <math.h> 
#include <eigen3/Eigen/Core>


// Type definition and constants
// ----------------------------
constexpr int STEERING = 0;
constexpr int DRIVING = 1;

using pose2d = Eigen::Vector3d;
using stateVector = Eigen::Vector4d;
typedef std::vector<pose2d> poseTrajectory;

using tick64 = Eigen::Vector<int64_t, 2>;
using velocityInput = Eigen::Vector2d; // for reconstructed velocity


// -----------------------------------------------------------------
// parameters structure
// -----------------------------------------------------------------

struct RobotParameters{
    double axis_lenght;
    double steer_offset;

    RobotParameters(double axis_lenght, double steer_offset)
        : axis_lenght(axis_lenght), steer_offset(steer_offset){}

    std::string info(){
        return "Axis_lenght: " + std::to_string(axis_lenght) + " " +
               "Steer_offset: " + std::to_string(steer_offset) + "\n";
    }

};

struct EncoderParameters{
    double K_steering;
    double K_driving;

    EncoderParameters(double K_steering, double K_driving)
        : K_steering(K_steering), K_driving(K_driving){}
    
    std::string info(){
        return "K_steering_factor: " + std::to_string(K_steering) + " " +
               "K_driving_factor: " + std::to_string(K_driving) + "\n";
    }
};

struct SensorParameters{
    double x;
    double y;
    double theta;

    SensorParameters(double x, double y, double theta)
        : x(x), y(y), theta(theta){}
    
    std::string info(){
        return "robot2sensor_x: " + std::to_string(x) + " " +
               "robot2sensor_y: " + std::to_string(y) + " " +
               "robot2sensor_theta: " + std::to_string(theta) + "\n";
    }
    pose2d as_pose(){
        return pose2d(x, y, theta);
    }
};



// ----------------------------
// Math utility
// ----------------------------

// boxPlus and boxMinus
// --------------------
Eigen::Matrix3d v2t(const pose2d& pose);
pose2d t2v(const Eigen::Matrix3d& T);   


// sin-cos expansion with taylor:
// ------------------------------
inline double sin_taylor_expansion(const double x){
    return x/2 - pow(x, 3)/24 + pow(x, 5)/720;
}
inline double cos_taylor_expansion(const double x){
    return 1 - pow(x, 2)/6 + pow(x, 4)/120;
}
    
    

// -------------------
// file saving utility
// -------------------

void saveTrajToCSV(
    const std::vector<pose2d>& traj, 
    const std::string& file_path, 
    const std::string& header);