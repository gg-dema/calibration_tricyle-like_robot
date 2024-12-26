#pragma once 

#include "utility.hpp"
#include <memory>

#include <iostream>
#include <optional>


class AbsoluteEncoder{
    static int MAX_STEERING_ENCODER_TICK;   
    public:
        double tick_to_input(int64_t delta_tick, double K_factor);
    private:
        int32_t last_reading; 

};

class IncrementalEncoder{
    
    static int MAX_DRIVING_ENCODER_TICK;

    public:
        double tick_to_input(int64_t delta_tick, double K_factor);
    private:
        u_int32_t last_reading; 
};


class Robot{


friend class CalibrationEngine;
protected:

    std::shared_ptr<RobotParameters> kinematicParams_;
    std::shared_ptr<EncoderParameters> encoderParams_;
    std::shared_ptr<SensorParameters> sensorParams_;

    AbsoluteEncoder steering_encoder_;
    IncrementalEncoder driving_encoder_;
    pose2d pose_;
    pose2d sensor_pose_;

public: 

    Robot(std::shared_ptr<RobotParameters> robotParameters,
          std::shared_ptr<EncoderParameters> encoderParameters,
          std::shared_ptr<SensorParameters> sensorParameters,
          pose2d pose={0,0,0}) 
          : kinematicParams_(robotParameters), 
            encoderParams_(encoderParameters),
            sensorParams_(sensorParameters),
            steering_encoder_(),
            driving_encoder_(),
            pose_(pose), 
            sensor_pose_(sensorParameters->x, sensorParameters->y, sensorParameters->theta)
            {
                std::cout << "Robot created with parameters: \n" << std::endl;
                std::cout << kinematicParams_->info();
                std::cout << encoderParams_->info();
                std::cout << sensorParams_->info();
                std::cout << std::endl;
                // sensor_pose_[0] += kinematicParams_->steer_offset;
            }

    inline double sin_taylor_expansion(const double x){
        return x/2 - pow(x, 3)/24 + pow(x, 5)/720;
    }
    inline double cos_taylor_expansion(const double x){
        return 1 - pow(x, 2)/6 + pow(x, 4)/120;
    }
    
    
    // integration funciton from last state A to new state (A + delta): return new position 
    pose2d integrate_displacement(const pose2d& displacement, const pose2d& last_state);
    
    // integrate the displacement from the current pose of the robot : update internal state
    void integrate_displacement(const pose2d& displacement);
    
    // predict the displacement from the encoder measure and a set of parameters 
    pose2d predict_displacement(const tick64& tick, const Eigen::Matrix<double, 4, 1>& params);
    
    // predict the displacement from the encoder measure with the actual set 
    // of parameters registerd to the robot 
    pose2d predict_displacement(const tick64& tick);  

    // rollout the trajectory from the encoder measures [entire trajectory]
    poseTrajectory rollout_trajectory(const std::vector<tick64>& encoder_measures);
    
    poseTrajectory rollout_trajectory_sensor(const std::vector<tick64>& encoder_measures);
    // return the pose of the robot respect the world frame at a certain time
    pose2d get_pose(){return pose_;}

    // return the pose of the sensor respect the robot frame at a certain time
    pose2d get_sensor_pose(){return sensor_pose_;}

    // return a copy of the parameters of the robot at a certain time
    Eigen::Vector4d get_kin_params(){
        Eigen::Vector4d params;
        params << 
            encoderParams_->K_steering,
            encoderParams_->K_driving,
            kinematicParams_->axis_lenght, 
            kinematicParams_->steer_offset;
        return params;
    }
};  