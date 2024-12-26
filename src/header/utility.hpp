// utility.hpp

#pragma once
#include <vector>
#include <math.h> 
#include <cstdint> // define max uint32
#include <memory>  // include shared pointers
#include <eigen3/Eigen/Core>


constexpr int STEERING = 0;
constexpr int DRIVING = 1;

using pose2d = Eigen::Vector3d;
using stateVector = Eigen::Vector4d;
typedef std::vector<pose2d> poseTrajectory;

using tick64 = Eigen::Vector<int64_t, 2>;
using velocityInput = Eigen::Vector2d; // for reconstructed velocity


// -----------------------------------------------------------------
// parameters structure and perturbation utility: 
// -----------------------------------------------------------------
struct RobotParameters{
    double axis_lenght;
    double steer_offset;

    RobotParameters(double axis_lenght, double steer_offset)
        : axis_lenght(axis_lenght), steer_offset(steer_offset){}

    std::string info(){
        return "Axis lenght: " + std::to_string(axis_lenght) + " " +
               "Steer offset: " + std::to_string(steer_offset) + "\n";
    }

};

struct EncoderParameters{
    double K_steering;
    double K_driving;

    EncoderParameters(double K_steering, double K_driving)
        : K_steering(K_steering), K_driving(K_driving){}
    
    std::string info(){
        return "K_steering factor: " + std::to_string(K_steering) + " " +
               "K_driving Factor: " + std::to_string(K_driving) + "\n";
    }
};

struct SensorParameters{
    double x;
    double y;
    double theta;

    SensorParameters(double x, double y, double theta)
        : x(x), y(y), theta(theta){}
    
    std::string info(){
        return "robot 2 sensor x: " + std::to_string(x) + " " +
               "robot 2 sensor y: " + std::to_string(y) + " " +
               "robot 2 sensor theta: " + std::to_string(theta) + "\n";
    }
    pose2d as_pose(){
        return pose2d(x, y, theta);
    }
};


Eigen::Matrix3d v2t(const pose2d& pose);
pose2d t2v(const Eigen::Matrix3d& T);   


// ------
// file saving utility

void saveTrajToCSV(
    const std::vector<pose2d>& traj, 
    const std::string& file_path, 
    const std::string& header);