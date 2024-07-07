#include "header/robot_model.hpp"
#include "header/robot_dynamics.hpp"

#include <math.h>
#include <memory>
#include <eigen3/Eigen/Core>

namespace RobotModel{ 

// ABSOLUTE VALUE AS TICK
double TractionDriveRobotModel::reconstruct_steering_input(int tick_angular_encoder){
    if (tick_angular_encoder > (double) this-> kMaxEncoderValSteering/2)
        tick_angular_encoder-=this-> kMaxEncoderValSteering;
    return this->parameters_.K_steer * 2 * M_PI * ((double) tick_angular_encoder/this->kMaxEncoderValSteering); 
}
// DELTA VALUE AS TICK
double TractionDriveRobotModel::reconstruct_driving_input(int tick_linear_encoder){
    return this->parameters_.K_traction * (tick_linear_encoder/this->kMaxEncoderValDriving); 
}

//void TractionDriveRobotModel::perturb_model_parameter(model_parameters pertubation){}

//constructor
TractionDriveRobotModel::TractionDriveRobotModel(StateVector q, model_parameters& params, Dynamics::DynamicModel& model){
    
    q_state_vect_ = q;
    parameters_ = params;
    dyn_ = &model;
    this->sensor_.position = {1.5, 0.0, 0.0};
}


cal_lib::Pose2d TractionDriveRobotModel::forward_step(cal_lib::Tick input, double delta_t){    
    cal_lib::Pose2d pose;
    return pose;
}


void error_and_jacobian(TractionDriveRobotModel& robot, const cal_lib::Pose2d& measurement){

    //error : h(q, X) - measurement 
    // measurement: position of the sensor in world frame
    // h(q, X) with q = state vect, X = robot params
    // h(q, X) = [x, y, theta] = [ x + cos(theta)*sensor_params.x, ...] 
}

void perturb_model_parameter(model_parameters pertubation){}

};