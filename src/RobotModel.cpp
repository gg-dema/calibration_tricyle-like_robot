#include "header/RobotModel.hpp"
#include <math.h>
namespace RobotModel{ 



//double TractionDriveRobotModel::reconstruct_steering_input(int tick_angular_encoder){}
//double TractionDriveRobotModel::reconstruct_driving_input(int tick_linear_encoder){}
//void TractionDriveRobotModel::perturb_model_parameter(model_parameters pertubation){}



TractionDriveRobotModel::state_vect operator*(const TractionDriveRobotModel::state_vect& q, const double& scalar){
    TractionDriveRobotModel::state_vect out;
    out[0] = q[0]*scalar;
    out[1] = q[1]*scalar;
    out[2] = q[2]*scalar;
    out[3] = q[3]*scalar;
    return q;
}
TractionDriveRobotModel::state_vect operator+(const TractionDriveRobotModel::state_vect& q1, const TractionDriveRobotModel::state_vect& q2){
    TractionDriveRobotModel::state_vect out;
    out[0] = q1[0] + q2[0];
    out[1] = q1[1] + q2[1];
    out[2] = q1[2] + q2[2];
    out[3] = q1[3] + q2[3];
    return q1;
}

//constructor
TractionDriveRobotModel::TractionDriveRobotModel(){
    this->_q_state_vect[0] = 0;
    this->_q_state_vect[1] = 0;
    this->_q_state_vect[2] = 0;
    this->_q_state_vect[3] = 0;

}
TractionDriveRobotModel::TractionDriveRobotModel(state_vect q){
    this->_q_state_vect[0] = q[0];
    this->_q_state_vect[1] = q[1];
    this->_q_state_vect[2] = q[2];
    this->_q_state_vect[3] = q[3];

}


pose2d TractionDriveRobotModel::dead_reckoning(double steering_v, double driving_v, double delta_t){
    pose2d pose;
    TractionDriveRobotModel::state_vect velocity = forward_kinematic_model(steering_v, driving_v); 
    this->_q_state_vect = _q_state_vect + (velocity*delta_t);  // to def
    pose[0] = this->_q_state_vect[0];
    pose[1] = this->_q_state_vect[1]; 
    pose[2] = this->_q_state_vect[2];
    return pose;
}
TractionDriveRobotModel::state_vect TractionDriveRobotModel::forward_kinematic_model(double steering_v, double driving_v){
    /*
     kin model used : same of frontDrive bicycle
     v, w = driving, steering velocity ---> reconstruct from encoder
     x_dot = v cos(theta) cos(phi)
     y_dot = v sin(theta) cos(phi)
     theta_dot = v sin(phi) / l
     phi_dot = w
    */
   state_vect velocity_state_vect; 
   velocity_state_vect[0] = driving_v * std::cos(this->_q_state_vect[3]) * std::cos(this->_q_state_vect[4]);
   velocity_state_vect[1] = driving_v * std::sin(this->_q_state_vect[3]) * std::cos(this->_q_state_vect[4]);
   velocity_state_vect[2] = (driving_v * std::sin(this->_q_state_vect[4])) / _parameters.axis_lenght; 
   velocity_state_vect[3] = steering_v;
   return velocity_state_vect;
}

};