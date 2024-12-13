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

//using u_tick32 = Eigen::Vector<u_int32_t, 2>;
//using tick32 = Eigen::Vector<int32_t, 2>;

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
};

/* TODO:  move this function to a class "perturber", 
          with internal variables the shared pointer, 
          and a function for perturb the parameters
*/

/*
void perturb_parameters(
    std::shared_ptr<RobotParameters> robotParameters,
    std::shared_ptr<EncoderParameters> encoderParameters, 
    std::vector<double> perturbation
    ){

    
    // Perturb robot parameters
    robotParameters->axis_lenght += perturbation[0];
    robotParameters->steer_offset += perturbation[1];

    // Perturb encoder parameters
    encoderParameters->K_steering += perturbation[2];
    encoderParameters->K_driving += perturbation[3];
}
*/

Eigen::Matrix3d v2t(const pose2d& pose);
pose2d t2v(const Eigen::Matrix3d& T);   
Eigen::Vector2d Jac_atan2(const Eigen::Vector2d& x);