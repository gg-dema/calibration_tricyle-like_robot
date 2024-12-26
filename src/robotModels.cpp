#include "header/robotModels.hpp"
#include "header/utility.hpp"

// CONST OF ENCODER
//--------------------------
int AbsoluteEncoder::MAX_STEERING_ENCODER_TICK = 8192;
int IncrementalEncoder::MAX_DRIVING_ENCODER_TICK = 5000;
//-------------------------


// DECODING OF THE ENCODER TICK
//-----------------------------
// ABSOLUTE ENCODER
double AbsoluteEncoder::tick_to_input(int64_t tick, double K_factor){
    
    double_t normalized_tick;

    // center the data around 0
    // ?info?:  the static cast is due the fact thet otherwise we get integer in output
    if (tick >  MAX_STEERING_ENCODER_TICK/2){
        normalized_tick = tick - MAX_STEERING_ENCODER_TICK;
        normalized_tick /= static_cast<double>(MAX_STEERING_ENCODER_TICK);
    }
    else {
        normalized_tick = static_cast<double>(tick) / MAX_STEERING_ENCODER_TICK;
    }
    // ?info? the steering offset is a kinematic parameters, added into the robot class directly
    return (2*M_PI*K_factor*normalized_tick); 

}

// INCREMENTAL ENCODER
double IncrementalEncoder::tick_to_input(int64_t delta_tick, double K_factor){

    return K_factor*(static_cast<double>(delta_tick)/MAX_DRIVING_ENCODER_TICK); 
}


// ----------------------------
// ROBOT MODEL 
// ----------------------------

pose2d Robot::predict_displacement(
    const tick64& tick,
    const Eigen::Vector4d& kin_params
    ){

    // extract the kinematic parameters
    double k_factor_steering = kin_params[0];    
    double k_factor_driving = kin_params[1];

    double axis_length = kin_params[2];
    double steer_offset = kin_params[3];
    
    // convert the measurment of the encoder in input to the model
    double steer_input = steering_encoder_.tick_to_input(tick[STEERING], k_factor_steering);
    double driving_input = driving_encoder_.tick_to_input(tick[DRIVING], k_factor_driving);

    // add the offset in the steering encoder:
    steer_input += steer_offset;
    
    // kin models
    double rho = driving_input * cos(steer_input);
    double delta_theta = driving_input * (sin(steer_input)/axis_length);

    // sin/cos expansion 
    double sin_expansion = sin_taylor_expansion(delta_theta);
    double cos_expansion = cos_taylor_expansion(delta_theta);


    // calc x/y displacement
    double delta_x = rho*cos_expansion;
    double delta_y = rho*sin_expansion;

    pose2d displacement(delta_x, delta_y, delta_theta); 

    return displacement;       

}

pose2d Robot::predict_displacement(const tick64& tick){      
    Eigen::Vector4d local_parmas = get_kin_params();
    return predict_displacement(tick, local_parmas);

}


pose2d Robot::integrate_displacement(
    const pose2d& displacement,
    const pose2d& last_state){
    // ?info? workflow: 
    // convert to homo_matrix the displacement
    // convert to homo_matrix the actual state
    // multiply them X_state * X_disp
    // return t2v [transformation matrix to vector] of the results

    return t2v(
        v2t(last_state)*v2t(displacement)
    );

}


void Robot::integrate_displacement(const pose2d& displacement){
    pose_ = integrate_displacement(displacement, pose_);
}


poseTrajectory Robot::rollout_trajectory(
    const std::vector<tick64>& encoders_measures
    ){

    pose2d last_pose(0,0,0);
    poseTrajectory trajectory; 

    trajectory.push_back(last_pose); 
    for(size_t i=0; i < encoders_measures.size(); i++){

        pose2d displacement = predict_displacement(encoders_measures[i]);    
        pose2d predicted_pose = integrate_displacement(displacement, last_pose);
        
        trajectory.push_back(predicted_pose);

        last_pose = predicted_pose;
    
    }
    return trajectory;
}

poseTrajectory Robot::rollout_trajectory_sensor(
    const std::vector<tick64>&encoders_measures
    ){
    poseTrajectory sensor_trajectory;
    Eigen::Matrix3d robot_X_sensor = v2t(sensor_pose_);
    pose2d current_sensor_pose = pose2d(0., 0., 0.);

    pose2d delta_robot;
    Eigen::Matrix3d T_delta_sensor;
    
    sensor_trajectory.push_back(current_sensor_pose);

    for(size_t i=0; i < encoders_measures.size(); i++){
        
        // displacement of the robot 
        delta_robot = predict_displacement(encoders_measures[i]);
        
        // displacement of the sensor (for convenience, stored as matrix)
        T_delta_sensor = robot_X_sensor.inverse() * v2t(delta_robot) * robot_X_sensor;
        
        // integrate the delta_sensor with the last sensor pose
        current_sensor_pose = integrate_displacement(t2v(T_delta_sensor), current_sensor_pose);
        
        // store it 
        sensor_trajectory.push_back(current_sensor_pose);
        
    }
    
    return sensor_trajectory;

}

