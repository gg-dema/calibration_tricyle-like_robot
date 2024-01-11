#pragma once
#include <array>
#include "calibration_utility.hpp"


namespace RobotModel{

struct model_parameters{
    double K_steer; 
    double K_traction;
    double axis_lenght;
    double steer_offset; 
};




class TractionDriveRobotModel{
    
public: 

    using state_vect = std::array<double, 4>;   // q = [x, y, theta, phi]
    model_parameters _parameters;
    state_vect _q_state_vect; 


    TractionDriveRobotModel();
    TractionDriveRobotModel(state_vect q);
    //double reconstruct_steering_input(int tick_angular_encoder);
    //double reconstruct_driving_input(int tick_linear_encoder);
    //void perturb_model_parameter(model_parameters pertubation);
    pose2d dead_reckoning(double steering_v, double driving_v, double delta_t);
    state_vect forward_kinematic_model(double steering_v, double driving_v);


protected:

    const int _MAX_ENCODER_VAL_STEERING = 8192;
    const int _MAX_ENCODER_VAL_DRIVING = 5000;
    
};

TractionDriveRobotModel::state_vect operator+(const TractionDriveRobotModel::state_vect& q1, const TractionDriveRobotModel::state_vect& q2);
TractionDriveRobotModel::state_vect operator*(const TractionDriveRobotModel::state_vect& q, const double& scalar);



}; //end RobotModel namespace