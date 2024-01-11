#pragma once
#include <array>


namespace RobotModel{

struct model_parameters{
    double K_steer; 
    double K_traction;
    double axis_lenght;
    double steer_offset; 
};

using state_vect = std::array<double, 4>;   // q = [x, y, theta, phi]
using pose_2d = std::array<double, 3>;      // p = [x. y, theta]



class TractionDriveRobotModel{

    model_parameters _parameters;
    state_vect _q_state_vect; 
    
public: 

    TractionDriveRobotModel();
    TractionDriveRobotModel(state_vect q);
    //double reconstruct_steering_input(int tick_angular_encoder);
    //double reconstruct_driving_input(int tick_linear_encoder);
    //void perturb_model_parameter(model_parameters pertubation);
    pose_2d dead_reckoning(double steering_v, double driving_v, double delta_t);
    state_vect forward_kinematic_model(double steering_v, double driving_v);

protected:

    const int _MAX_ENCODER_VAL_STEERING = 8192;
    const int _MAX_ENCODER_VAL_DRIVING = 5000;
    
};




}; //end RobotModel namespace