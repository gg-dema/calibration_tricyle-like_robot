#include "header/RobotModel.hpp"
#include "RobotDynamics.cpp"

#include <math.h>
//#include <ostream>
//#include <iostream>
#include <eigen3/Eigen/Core>

namespace RobotModel{ 

double TractionDriveRobotModel::reconstruct_steering_input(int tick_angular_encoder){
    return this->_parameters.K_steer * 2 * M_PI * ((double) tick_angular_encoder/this->_MAX_ENCODER_VAL_STEERING); 
}
double TractionDriveRobotModel::reconstruct_driving_input(int tick_linear_encoder){
    return this->_parameters.K_traction * (tick_linear_encoder/this->_MAX_ENCODER_VAL_DRIVING); 
}
//void TractionDriveRobotModel::perturb_model_parameter(model_parameters pertubation){}


/*
TractionDriveRobotModel::state_vect operator*(const TractionDriveRobotModel::state_vect& q, const double& scalar){

    TractionDriveRobotModel::state_vect out;
    out[0] = q[0]*scalar;
    out[1] = q[1]*scalar;
    out[2] = q[2]*scalar;
    out[3] = q[3]*scalar;
    return out;
}
TractionDriveRobotModel::state_vect operator+(const TractionDriveRobotModel::state_vect& q1, const TractionDriveRobotModel::state_vect& q2){
    TractionDriveRobotModel::state_vect out;
    out[0] = q1[0] + q2[0];
    out[1] = q1[1] + q2[1];
    out[2] = q1[2] + q2[2];
    out[3] = fmod(q1[3] + q2[3], (2*M_PI));
    return out;
}
std::ostream& operator<<(std::ostream& os, const TractionDriveRobotModel::state_vect& q){
    os << "state vect--> (x: " << q[0] << " y: " << q[1] << " theta: " << q[2] << " phi: " << q[3]  << ")";
    return os;
}; 
*/

//constructor
TractionDriveRobotModel::TractionDriveRobotModel(){
    this->_parameters = {0.1, 0.0106141, 1.4, 0};
    this->_sensor.position = {1.5, 0.0, 0.0};
    //
    this->_q_state_vect[0] = 0;
    this->_q_state_vect[1] = 0;
    this->_q_state_vect[2] = 0;
    this->_q_state_vect[3] = 0;

}
TractionDriveRobotModel::TractionDriveRobotModel(cal_lib::state_vector q){ 
    this->_parameters = {0.1, 0.0106141, 1.4, 0};
    this->_sensor.position = {1.5, 0.0, 0.0};
    //
    this->_q_state_vect[0] = q[0];
    this->_q_state_vect[1] = q[1];
    this->_q_state_vect[2] = q[2];
    this->_q_state_vect[3] = q[3];

}


cal_lib::pose2d TractionDriveRobotModel::dead_reckoning(double steering_v, double driving_v, double delta_t){
    
    cal_lib::tick input = cal_lib::tick(steering_v, driving_v);
    cal_lib::state_vector velocity = forward_kinematic_model<Dynamics::oriolo_model>(
        this->_q_state_vect,
        input,
        this->_parameters
        ); 
    this->_q_state_vect = this->_q_state_vect + (velocity*delta_t); 
    
    cal_lib::pose2d pose;
    pose[0] = this->_q_state_vect[0];
    pose[1] = this->_q_state_vect[1]; 
    pose[2] = fmod(this->_q_state_vect[2], 2*M_PI);
    
    return pose;
}

template<cal_lib::state_vector (*KinematicModel) (cal_lib::state_vector, cal_lib::tick , model_parameters)>
cal_lib::state_vector TractionDriveRobotModel::forward_kinematic_model(cal_lib::state_vector q, cal_lib::tick input, model_parameters params){
    /*
     kin model used : same of frontDrive bicycle
     v, w = driving, steering velocity ---> reconstruct from encoder
     x_dot = v cos(theta) cos(phi)
     y_dot = v sin(theta) cos(phi)
     theta_dot = v sin(phi) / l
     phi_dot = w
    */
   return KinematicModel(q, input, params);
}


void error_and_jacobian(TractionDriveRobotModel& robot, const cal_lib::pose2d& measurement){

    //error : h(q, X) - measurement 
    // measurement: position of the sensor in world frame
    // h(q, X) with q = state vect, X = robot params
    // h(q, X) = [x, y, theta] = [ x + cos(theta)*sensor_params.x, ...] 
}

};