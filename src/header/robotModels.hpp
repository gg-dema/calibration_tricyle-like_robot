/**
 * @file robotModels.hpp
 * @brief all the classes that model the robot and his (active) components
 * @details robot class, encoder classes, kinematic model
 * 
 * @author Gabriele G. Di Marzo [github : gg-dema]
 */

#pragma once 

#include "utility.hpp"
#include <memory>

#include <iostream>
#include <optional>

/**
 * @brief Absolute encoder class. Take the absolute tick and return the input for the kinematic model
 */
class AbsoluteEncoder{
    static int MAX_STEERING_ENCODER_TICK;   
    public:
        //@param tick: the absolute tick of the encoder 
        //@param K_factor: the K factor of the encoder --> params to calibrate 
        double tick_to_input(int64_t tick, double K_factor);
};


/**
 * @brief Incremental encoder class. Take the incremental (delta) tick and return the input for the kinematic model
 */
class IncrementalEncoder{
    
    static int MAX_DRIVING_ENCODER_TICK;

    public:
        //@param tick: the delta tick of the encoder 
        //@param K_factor: the K factor of the encoder --> params to calibrate 
        double tick_to_input(int64_t delta_tick, double K_factor);

};

/**
 * @brief Robot class. The class that represent the robot model
 This class model the robot subject to our optimization. It's "assemble" with the relative encoders
 and shared structure with the parameters to calibrate. 

 how to use it:
    // For move the robot and update the position 
        - create the robot with the parameters
        - predict the displacement from the encoder measures
        - integrate the displacement to get the new pose
    
    // other option:
        - rollout the trajectory from the encoder measures and obtain the resulting traj. 
        - use the predict function for obtain the a displacement respect a generic set of params using the kin model
        - use the integrate function for update a generic pose
        - get the pose (robot/sensors), get the parameters as vector 
        (more details on the function's comments)
*/
class Robot{

// make the Calibration Engine friend for allow to access pose, params and modify it
friend class CalibrationEngine;

protected:

    // parameters to calibrate 
    std::shared_ptr<RobotParameters> kinematicParams_;
    std::shared_ptr<EncoderParameters> encoderParams_;
    std::shared_ptr<SensorParameters> sensorParams_;

    // wheel encoders
    AbsoluteEncoder steering_encoder_;
    IncrementalEncoder driving_encoder_;

    // internal state of the robot
    pose2d pose_;           // pose respect the world frame
    pose2d sensor_pose_;    // pose respect the robot frame

public: 

    // constructor
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
                std::cout << "\nRobot created with parameters: \n" << std::endl;
                std::cout << kinematicParams_->info();
                std::cout << encoderParams_->info();
                std::cout << sensorParams_->info();
                std::cout << std::endl;
            }


    /** @brief integration funciton from last state 'A' to new state 'A + delta'
    *   @details This function integrate a displacement into a "last state", 
    *            by apply the t2v and v2t transformations. No prior info about what the "last state" is. 
    *            It could be both the sensor pose or the robot pose.
    *   @return the new position as pose2d (x, y, theta) 
    */
    pose2d integrate_displacement(const pose2d& displacement, const pose2d& last_state);
    
    /** @brief integrate the displacement from the current pose of the robot : update internal state
    *   @details This is just a wrapper respect the other 'integrate_displacement' function
    *            Use the actual pose of the robot (internal state) as the last state and call the upper define 
    *            function.
    *   @param displacement: the displacement to integrate (displacement respect x, respect y, respect theta)
    */
    void integrate_displacement(const pose2d& displacement);
    

    /** @brief predict the displacement from the encoder measure and a set of parameters 
     *  @details This function predict the displacement of the robot from the encoder measure and a set of parameters
     *           The parameters are the one used in the kinematic model, in general. This could be used also for 
     *           predict the displacement respect a set of perturbed parameters. 
     *  @param tick: the encoder measure (single instant (steering tick, driving tick))
     *  @param params: the set of parameters to use for the prediction (k_steering, k_driving, axis_lenght, steer_offset)
     *  @return the displacement as pose2d (x, y, theta)
    */ 
    pose2d predict_displacement(const tick64& tick, const Eigen::Vector4d& params);
    
    /** @brief predict the displacement from the encoder measure with the actual set of parameters
     *  @details This function predict the displacement of the robot from the encoder measure and the actual set of parameters
     *           This is a wrapping around the previous "predict_displacment" function, using the parameters registered in the class
     *  @param tick: the encoder measure (single instant (steering tick, driving tick))
     *  @return the displacement as pose2d (x, y, theta)
    */ 
    pose2d predict_displacement(const tick64& tick);  

    
    /** @brief rollout a trajectory from a vector of encoder measures [world to robot transformations] 
     *  @return a trajectory expressed as pose2d (x, y, theta) [refered to the robot base]
     */
    poseTrajectory rollout_trajectory(const std::vector<tick64>& encoder_measures);
    
    /** @brief rollout a sensor trajectory from a vector of encoder measures [world to sensor transformations]
     *  @return a trajectory expressed as pose2d (x, y, theta) [world to sensor transformation]
     */
    poseTrajectory rollout_trajectory_sensor(const std::vector<tick64>& encoder_measures);
    
    
    
    // return the pose of the robot respect the world frame at a certain time
    pose2d get_pose(){return pose_;}

    // return the pose of the sensor respect the robot frame at a certain time
    pose2d get_sensor_pose(){return sensor_pose_;}

    /** @brief return a copy of the parameters of the robot at a certain time
    */
    Eigen::Vector4d get_kin_params(){
        Eigen::Vector4d params;
        params << 
            encoderParams_->K_steering,
            encoderParams_->K_driving,
            kinematicParams_->axis_lenght, 
            kinematicParams_->steer_offset;
        return params;
    }

    /** @brief give a good print of the parameters
    */
    std::string get_params_info(){
        return encoderParams_->info() + kinematicParams_->info() + sensorParams_->info(); 
    }

};  

