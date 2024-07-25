#include "header/robot_model.hpp"
#include "header/robot_dynamics.hpp"

#include <math.h>
#include <memory>
#include <eigen3/Eigen/Core>

#include <iostream>


namespace RobotModel{ 

long double TractionDriveRobotModel::reconstruct_steering_input(int64_t tick_absolute_encoder){
    
    long double displacement = static_cast<long double>(tick_absolute_encoder) / static_cast<long double>(this->kMaxEncoderValSteering);
    long double steering_input = this->parameters_.K_steer * 2.0 * 3.14 * (displacement); 

    return steering_input;
}

long double TractionDriveRobotModel::reconstruct_traction_input(int64_t tick_incremental_encoder){
    
    long double displacement = static_cast<long double>(tick_incremental_encoder) / static_cast<long double>(this->kMaxEncoderValDriving);
    return (this->parameters_.K_traction * displacement); 
}

//void TractionDriveRobotModel::perturb_model_parameter(model_parameters pertubation){}

//constructor
TractionDriveRobotModel::TractionDriveRobotModel(StateVector q, model_parameters& params, Dynamics::DynamicModel& model){
    
    q_state_vect_ = q;
    parameters_ = params;
    dyn_ = &model;
    this->sensor_.position = {1.5, 0.0, 0.0};
}


cal_lib::Pose2d TractionDriveRobotModel::forward_step(std::vector<long double> velocity, double delta_t){   

    cal_lib::Pose2d pose_;
    pose_ = (*dyn_)(q_state_vect_, velocity, delta_t);
    return pose_;
}


void error_and_jacobian(TractionDriveRobotModel& robot, const cal_lib::Pose2d& measurement){

    // error : h(q, X) - measurement 
    // measurement: position of the sensor in world frame
    // h(q, X) with q = state vect, X = robot params
    // h(q, X) = [x, y, theta] = [ x + cos(theta)*sensor_params.x, ...] 
}

void perturb_model_parameter(model_parameters pertubation){}

};