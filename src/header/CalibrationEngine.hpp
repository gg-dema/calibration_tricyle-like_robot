#pragma once

#include "utility.hpp"
#include "robotModels.hpp"
#include "DataModule.hpp"

#include <memory>
#include <eigen3/Eigen/Dense>

/** @brief data struct for the statistics of the calibration process*/
struct Statistics{
    int skipped_data = 0;
    std::vector<double> log_chi;

    void save_to_file(std::string file_path){
        std::ofstream file(file_path);
        file << "Skipped data: " << skipped_data << std::endl;
        file << "Chi: \n";
        for(auto& chi : log_chi){
            file << chi << std::endl;

        }

        file.close();
    }
};


/**
 * @brief CalibrationEngine is the class that will perform the calibration of the robot
 * @details The class has pointers to the robot to calibrate, 
 * can compute the solution in the least square sense and apply the perturbation dx
 * directly to the robot parameters. The robot class consider this as a friend class
   
   To use this:
    - create a robot
    - create a calibration engine with a shared robot pointers
    - iterate N times, calling OneIteration and passing the entire dataset object.

   To eval the process:
    - access directly to the Statistics struct for log_chi.
    - roll out a new trajectory with the robot, the parameters are automatically updated.
 */

class CalibrationEngine{

    std::shared_ptr<Robot> robot_;
    
    // state dim : 2 params encoder, 2 params kinematics, 3 params (pose) sensors
    // tot state dim : 7
    Eigen::Matrix<double, 7, 7> _H;   
    Eigen::Matrix<double, 7, 1> _b;
    Eigen::Matrix<double, 7, 1> _perturbation;
    
    // the error is basically a 3d vector, let's use the pose2d
    pose2d _error;                      // euclidian parametrization of the error
    Eigen::Matrix<double, 3, 7> _jac;   // jacobian

    double _delta_time_threshold = 0.0;   // threshold for the time between two measures
                                          // not effective, chi stay constant and parameters still on the same value
    public:

    Statistics _statistics;
    bool _first_iter = true;  // just for count how many sample are skipped

    // constructor 
    CalibrationEngine(std::shared_ptr<Robot> robot) : robot_(robot){}

    // @brief given a dataset, iterate over all the data and update the parameters of the robot
    // @params data : the dataset to use for the calibration
    // @details the function will iterate over all the data and update the parameters of the robot
    //          This function encapsulate all the step of a single least square iteration
    void OneIteration(const DataObject& data);


    private: 
   
    // @brief iven a single measurement (and a single input), update the jacobian for that values
    // @params endcoder_measure : 2d-vector with the encoder measure (both steering, driving)
    // @params z_observation : sensor pose (observation) at time t
    // @return void
    void OneRound(const tick64& encoder_measure, const pose2d& z_observation);


    // @brief update all the component of the jacobian matrix
    // @params Z_obs : the observation of the sensor at time t
    // @params robot_X_sensor : the pose of the sensor respect the robot frame
    // @params encoder_measure : the encoder measure at time t
    void update_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    );

    // @brief update the first part of jacobian (respect the kinematic parameters)
    void update_robot_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    );

    // @brief update the second part of jacobian (respect the encoder parameters)
    void update_sensor_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    );

    // @brief apply the perturbation to the robot parameters
    void perturb_parameters(Eigen::Matrix<double, 7, 1> perturbation);

    // @brief compute the solution of the least square problem (the perturbation)
    Eigen::Matrix<double, 7, 1> solver();
    
};