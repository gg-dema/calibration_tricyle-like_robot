#pragma once
#include <array>
#include <ostream>
#include "calibration_utility.hpp"


namespace RobotModel{

struct model_parameters{
    double K_steer; 
    double K_traction;
    double axis_lenght;
    double steer_offset; 
};

struct sensor_parameters{
    cal_lib::pose2d position;
    // other params 
};


class TractionDriveRobotModel{
    
public: 

    cal_lib::state_vector _q_state_vect; 

    TractionDriveRobotModel();
    TractionDriveRobotModel(cal_lib::state_vector q);

    double reconstruct_steering_input(int tick_angular_encoder);
    double reconstruct_driving_input(int tick_linear_encoder);

    cal_lib::pose2d dead_reckoning(double steering_v, double driving_v, double delta_t);
    
    template<cal_lib::state_vector (*T) (cal_lib::state_vector, cal_lib::tick , model_parameters)>
    cal_lib::state_vector forward_kinematic_model(cal_lib::state_vector q, cal_lib::tick i , model_parameters p);

    friend void error_and_jacobian(TractionDriveRobotModel& robot, const cal_lib::pose2d& measurement); //---> still to define how return error, jac
    void perturb_model_parameter(model_parameters pertubation);


protected:
    model_parameters _parameters;
    sensor_parameters _sensor;
    const int _MAX_ENCODER_VAL_STEERING = 8192;
    const int _MAX_ENCODER_VAL_DRIVING = 5000;
    
};

// In theory now this function are manage by the Eigen library
//TractionDriveRobotModel::state_vector operator+(const TractionDriveRobotModel::state_vect& q1, const TractionDriveRobotModel::state_vect& q2);
//TractionDriveRobotModel::state_vector operator*(const TractionDriveRobotModel::state_vect& q, const double& scalar);
//std::ostream& operator<<(std::ostream& os, const TractionDriveRobotModel::state_vect& p);


}; //end RobotModel namespace