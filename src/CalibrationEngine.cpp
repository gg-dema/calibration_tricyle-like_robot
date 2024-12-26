
/**
 * @file CalibrationEngine.hpp
 * @brief implementation of the calibration engine [see the header for more details]
 *
 * @author Gabriele G. Di Marzo [github : gg-dema]
 */


#include "header/CalibrationEngine.hpp"


void CalibrationEngine::OneRound(
    const tick64& encoder_measure,
    const pose2d& z_observation){
        
        Eigen::Matrix3d robot_X_sensor = v2t(robot_->get_sensor_pose());
        Eigen::Matrix3d displacement = v2t(robot_->predict_displacement(encoder_measure));
        Eigen::Matrix3d prediction = robot_X_sensor.inverse() * displacement * robot_X_sensor;


        // homogeneous matrix obs:
        Eigen::Matrix3d world_X_sensor_obs = v2t(z_observation);
        
        // basically the errorAndJacobian funct here:

        // update error:
        _error = t2v(world_X_sensor_obs.inverse() * prediction);

        // update jacobian
        update_jacobian(world_X_sensor_obs, robot_X_sensor, encoder_measure); 


}


void CalibrationEngine::update_jacobian(
    const Eigen::Matrix3d& Z_obs,
    const Eigen::Matrix3d& robot_X_sensor,
    const tick64& encoder_measure){

    // update the jacobian respect the kinematic parameters + encoder        
    update_robot_params_jacobian(Z_obs, robot_X_sensor, encoder_measure);
    
    // update the jacobian respect the sensor pose
    update_sensor_params_jacobian(Z_obs, robot_X_sensor, encoder_measure);
    
    _jac *= 0.5/1e-9;
    }


void CalibrationEngine::update_robot_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    ){

        
        for(int i=0; i<4; i++){
            
            // delta x perturbation
            Eigen::Vector4d small_perturbation(0.0, 0.0, 0.0, 0.0);  
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

        }

    }


void CalibrationEngine::update_sensor_params_jacobian(
        const Eigen::Matrix3d& Z_obs,
        const Eigen::Matrix3d& robot_X_sensor,
        const tick64& encoder_measure
    ){
        pose2d prediction = robot_->predict_displacement(encoder_measure);
        for(int i=0; i<3; i++){
            
            // delta x perturbation
            Eigen::Vector3d small_perturbation(0, 0, 0);  
            small_perturbation[i] = 1e-10;

            // positive perturb 
            Eigen::Matrix3d robot_X_sensor_perturb = robot_X_sensor * v2t(small_perturbation);
            pose2d positive_side = t2v(
                Z_obs.inverse()*robot_X_sensor_perturb.inverse()*v2t(prediction)*robot_X_sensor_perturb
            );

            // negative perturb
            robot_X_sensor_perturb = robot_X_sensor * v2t(-small_perturbation);
            pose2d negative_side = t2v(
                Z_obs.inverse()*robot_X_sensor_perturb.inverse()*v2t(prediction)*robot_X_sensor_perturb
                );


            _jac.col(i+4) = (positive_side - negative_side);
        }
    }


void CalibrationEngine::OneIteration(const DataObject& data){

    _H.setZero();
    _b.setZero();
    double chi_sum = 0.0; 
    robot_->pose_.setZero();
        
    // iterate over the dataset
    for(int i=0; i<data.length-1; i++)
    { 
        if(data.delta_time[i] < _delta_time_threshold && _first_iter){
            _statistics.skipped_data++;
            continue;
        }
        // update the jacobian and the error 
        OneRound(data.process_ticks[i], data.ground_truth_delta[i]);

        // check if error is superior tot the threshold
        double chi = _error.transpose()*_error;
        if (chi > 1.0){
            _error *= sqrt(1.0/chi);
        }

        //  update the H and the b vector
        _H += _jac.transpose() * _jac;
        _b += _jac.transpose() * _error;
        
        // update the chi statistics
        chi_sum += chi;

        _jac.setZero();
        _error.setZero();

    }

    // damp the H matrix
    _H += Eigen::Matrix<double, 7, 7>::Identity();

    // solve the linear system 
    Eigen::Matrix<double, 7, 1> dx = solver();

    // apply the boxplus operator
    perturb_parameters(dx);

    _statistics.log_chi.push_back(chi_sum);
    _first_iter = false;
}


void CalibrationEngine::perturb_parameters(Eigen::Matrix<double, 7, 1> perturbation){
// this should modify the parameters registerd in the robot class
    // equivalent to our boxPlus operator 

    // perturn encoder k factor
    robot_->encoderParams_->K_steering += perturbation[0];
    robot_->encoderParams_->K_driving += perturbation[1];

    // apply kin params 
    robot_->kinematicParams_->axis_lenght += perturbation[2];
    robot_->kinematicParams_->steer_offset += perturbation[3];

    // apply perturb to sensor pose by t2v(v2t())         
    pose2d params_sensor = t2v(v2t(perturbation.tail(3))*v2t(robot_->sensorParams_->as_pose()));
    robot_->sensorParams_->x = params_sensor[0];
    robot_->sensorParams_->y = params_sensor[1];
    robot_->sensorParams_->theta = params_sensor[2];
}


Eigen::Matrix<double, 7, 1> CalibrationEngine::solver(){
    // solve the linear system
    // ldlt --> cholenski
        
    // old version: just invert all  
    //Eigen::Matrix<double, 7, 1> dx = -_H.transpose() * (_H * _H.transpose()).inverse() * _b;
        
    Eigen::Matrix<double, 7, 1> dx  =  _H.ldlt().solve(-_b);
    return dx;

}