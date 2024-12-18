#include "utility.hpp"
#include "robotModels.hpp"
#include "DataModule.hpp"

#include <memory>
#include <eigen3/Eigen/Dense>

//@ TODO : to remove
#include <iostream>
#include <fstream>




struct Statistics{
    int skipped_data = 0;
};

class CalibrationEngine{

    std::shared_ptr<Robot> robot_;
    
    // state dim : 2 params encoder, 2 params kinematics, 3 params (pose) sensors
    // tot state dim : 7
    Eigen::Matrix<double, 7, 7> _H;   
    Eigen::Matrix<double, 7, 1> _b;
    Eigen::Matrix<double, 7, 1> _perturbation;
    
    // the error is basically a 3d vector, let's use the pose2d
    pose2d _error; // euclidian parametrization of the error
    Eigen::Matrix<double, 3, 7> _jac; // jacobian


    Statistics _statistics;

    public:
    CalibrationEngine(std::shared_ptr<Robot> robot) : robot_(robot){}


    // compute one step of the internal loop of the calibration:
    // given a single measurement (and a single input), update the jacobian for that values
    void OneRound(const tick64& encoder_measure, const pose2d& z_observation){
        
        // ! this one is to fix: the state of the robot must be:
        // ! 1) resetted
        // ! 2) not modify
        pose2d displacement = robot_->predict_displacement(encoder_measure);
        robot_->integrate_displacement(displacement);
        pose2d world_X_robot = robot_->get_pose();

        // prediction: world_X_robot * robot_X_sensor = world_X_sensor
        Eigen::Matrix3d robot_X_sensor = v2t(robot_->get_sensor_pose());
        Eigen::Matrix3d world_X_sensor_zHat = v2t(world_X_robot)*robot_X_sensor;

        // homogeneous matrix obs:
        Eigen::Matrix3d world_X_sensor_obs = v2t(z_observation);
        
        // basically the errorAndJacobian funct here:

            // update error:
        _error = t2v(world_X_sensor_obs.inverse() * world_X_sensor_zHat);

        std::ofstream file("error.csv", std::ios::app); // Open in append mode
            if (file.is_open()) {
                file << _error.transpose() << "\n";
                file.close(); // Close the file after writing
            } else {
                std::cerr << "Unable to open file for writing." << std::endl;
            }

            // update jacobian
        update_jacobian(world_X_sensor_obs, robot_X_sensor, encoder_measure); 
        

    
    }

    void update_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    ){
        // update the jacobian respect the kinematic parameters + encoder
        // both params can be updated togheter because they have the same contribs on the error
        
        update_robot_params_jacobian(Z_obs, robot_X_sensor, encoder_measure);
        // update the jacobian respect the sensor pose
        update_sensor_params_jacobian(Z_obs, robot_X_sensor, encoder_measure); 

        // scale up the jacobian --> avoid numerical instability
        // TODO: set this value as a parameters of the calibEngine
        _jac *= 0.5/1e-9;
    }

    void update_robot_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    ){

        
        for(int i=0; i<4; i++){
            
            // delta x perturbation
            Eigen::Vector4d small_perturbation(0, 0, 0, 0);  
            small_perturbation[i] = 1e-9;

            // positive perturbation
            pose2d pose_pred = robot_->predict_displacement(
                encoder_measure,
                robot_->get_kin_params() + small_perturbation);
            
            pose2d positive_side = t2v(
                Z_obs.inverse()*robot_X_sensor.inverse()*v2t(pose_pred)*robot_X_sensor
                );

            // negative perturbation
            pose_pred = robot_->predict_displacement(
                encoder_measure,
                robot_->get_kin_params() - small_perturbation);

            pose2d negative_side = t2v(
                Z_obs.inverse()*robot_X_sensor.inverse()*v2t(pose_pred)*robot_X_sensor
                );

            _jac.col(i) = (positive_side - negative_side);

            
            // left perturbation:
            //Eigen::Matrix3d left_perturb = v2t(robot_->predict_displacement(
            //                    encoder_measure,
            //                    robot_->perturb_single_parmas(i, perturb)
            //                    )
            //);
        }

    }

    void update_sensor_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    ){
        pose2d prediction = robot_->predict_displacement(encoder_measure);
        for(int i=0; i<3; i++){
            
            // delta x perturbation
            Eigen::Vector3d small_perturbation(0, 0, 0);  
            small_perturbation[i] = 1e-9;

            // positive perturb 
            Eigen::Matrix3d robot_X_sensor_perturb = robot_X_sensor * v2t(small_perturbation);
            pose2d positive_side = t2v(
                Z_obs.inverse()*robot_X_sensor_perturb.inverse()*v2t(prediction)*robot_X_sensor_perturb
            );

            // negative perturb
            robot_X_sensor_perturb = robot_X_sensor * v2t(-small_perturbation);
            pose2d negative_side = t2v(
                Z_obs.inverse()*robot_X_sensor_perturb.inverse()*v2t(prediction)*robot_X_sensor_perturb);


            _jac.col(i+4) = (positive_side - negative_side);
        }
        // pass
    }

    void OneIteration(const DataObject& data){

        _H.setZero();
        _b.setZero();

        robot_->pose_.setZero();
        for(int i=0; i<data.length-1; i++) // iterate over the dataset
        { 
            //if (data.delta_time[i] > 0.04)
            //{
            //if (i == 3){
            //    exit(0);
            //}
                //std::cout << "pose" << robot_->pose_ << std::endl;
                OneRound(data.process_ticks[i], data.ground_truth[i]);
                _H += _jac.transpose()*_jac;
                _b += _jac.transpose()*_error;
                
                _jac.setZero();
                _error.setZero();
            //}
            //else{
            //    _statistics.skipped_data++;
            //}

        }
        std::cout << _b << std::endl;
        std::cout << std::endl;
        Eigen::Matrix<double, 7, 1> dx = solver();
        std::cout << "dx" << dx << std::endl;
    }


    private: 


    void perturb_parameters(){
        
        // this should modify the parameters registerd in the robot class

        // apply kin params 
        robot_->kinematicParams_->axis_lenght += _perturbation[0];
        robot_->kinematicParams_->steer_offset += _perturbation[1];
        
        // perturn encoder k factor
        robot_->encoderParams_->K_steering += _perturbation[2];
        robot_->encoderParams_->K_driving += _perturbation[3];

        // perturb sensor pose
        robot_->sensorParams_->x += _perturbation[4];
        robot_->sensorParams_->y += _perturbation[5];
        robot_->sensorParams_->theta += _perturbation[6];

    }

    Eigen::Matrix<double, 7, 1> solver(){
        // solve the linear system
        // ldlt --> cholenski
        Eigen::Matrix<double, 7, 1> dx = -_H.transpose() * (_H * _H.transpose()).inverse() * _b;
        // Eigen::Matrix<double, 7, 1> dx; //  =  _H.ldlt().solve(-_b);
        // std::cout << "dx: \n" << dx << std::endl;
        return dx;

    }
};