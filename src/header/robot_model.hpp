#pragma once

#include <array>
#include <memory>
#include <ostream>
#include "calibration_utility.hpp"
#include "robot_dynamics.hpp"



namespace RobotModel{




class TractionDriveRobotModel{
    
public: 

    //the pose should be equals to a selection of the state vector
    cal_lib::StateVector q_state_vect_; 
    cal_lib::Pose2d pose_;

    TractionDriveRobotModel(StateVector q, model_parameters &params, Dynamics::DynamicModel &model);
    
    long double reconstruct_steering_input(int64_t tick_abs_encoder);
    long double reconstruct_traction_input(int64_t tick_incremental_encoder);
    

    //double reconstruct_steering_input(int tick_abs_encoder);
    //double reconstruct_traction_input(int tick_incremental_encoder);
    
    cal_lib::Pose2d forward_step(std::vector<long double> velocity, double delta_t);
    void error_and_jacobian(TractionDriveRobotModel& robot, const cal_lib::Pose2d& measurement); //---> still to define how return error, jac
    void perturb_model_parameter(model_parameters pertubation);


protected:

    Dynamics::DynamicModel* dyn_;
    model_parameters parameters_;
    sensor_parameters sensor_;
    static const u_int64_t kMaxEncoderValSteering = 8192;
    static const u_int64_t kMaxEncoderValDriving = 5000;
    
};


}; //end RobotModel namespace