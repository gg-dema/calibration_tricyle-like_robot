#include "header/robotModels.hpp"

#include <fstream> // For file operations
#include <iomanip> // For setting precision

// CONST OF ENCODER
//--------------------------
int AbsoluteEncoder::MAX_STEERING_ENCODER_TICK = 8192;
int IncrementalEncoder::MAX_DRIVING_ENCODER_TICK = 5000;
//-------------------------


// DECODING OF THE ENCODER TICK
//-----------------------------
// ABSOLUTE ENCODER
double AbsoluteEncoder::tick_to_input(int64_t delta_tick, double K_factor){
    
    // static int AbsoluteEncoder::MAX_STEERING_ENCODER_TICK;s


    double_t normalized_tick;

    // center the data around 0
    // ?info?:  the static cast is due the fact thet otherwise we get integer in output

    if (delta_tick > MAX_STEERING_ENCODER_TICK){

        normalized_tick = delta_tick - MAX_STEERING_ENCODER_TICK;
        normalized_tick /= static_cast<double>(MAX_STEERING_ENCODER_TICK);
    }
    else {
        normalized_tick = static_cast<double>(delta_tick) / MAX_STEERING_ENCODER_TICK;
    }
    // ?info? the steering offset is a kinematic parameters, added into the robot directly
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
    const Eigen::Matrix<double, 4, 1>& kin_params
    ){

    // extract the kinematic parameters

    double axis_length = kin_params[0];
    double steer_offset = kin_params[1];

    double k_factor_steering = kin_params[2];    
    double k_factor_driving = kin_params[3];

    // convert the measurment of the encoder in input to the model
    double steer_input = steering_encoder_.tick_to_input(tick[STEERING], k_factor_steering);
    double driving_input = driving_encoder_.tick_to_input(tick[DRIVING], k_factor_driving);

    // add the offset in the steering encoder:
    steer_input += steer_offset;
    
    // kin model
    double tmp1 = sin(steer_input);
    double tmp2 = cos(steer_input);

    double rho = driving_input * tmp2;
    double delta_theta = driving_input * (tmp1/axis_length);

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
    
    Eigen::Matrix<double, 4, 1> local_parmas = get_kin_params();
    return predict_displacement(tick, local_parmas);

    /*
    // convert the measurment of the encoder in input to the model
    double steer_input = steering_encoder_.tick_to_input(tick[STEERING], encoderParams_->K_steering);
    double driving_input = driving_encoder_.tick_to_input(tick[DRIVING], encoderParams_->K_driving);

    // add the offset in the steering encoder:
    steer_input += kinematicParams_->steer_offset;


    // kin model
    double tmp1 = sin(steer_input);
    double tmp2 = cos(steer_input);

    double rho = driving_input * tmp2;
    double delta_theta = driving_input * (tmp1/kinematicParams_->axis_lenght);

    // sin/cos expansion 
    double sin_expansion = sin_taylor_expansion(delta_theta);
    double cos_expansion = cos_taylor_expansion(delta_theta);


    // calc x/y displacement
    double delta_x = rho*cos_expansion;
    double delta_y = rho*sin_expansion;

    pose2d displacement(delta_x, delta_y, delta_theta); 
    */
}




pose2d Robot::integrate_displacement(const pose2d& displacement, const pose2d& last_state){
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
    const std::vector<tick64>& encoders_measure){
    
    //std::optional<bool> return_traj = std::nullopt){

    pose2d last_pose(0,0,0);

    poseTrajectory trajectory; 

    trajectory.push_back(last_pose); 
    for(int i=0; i < encoders_measure.size(); i++){

        pose2d displacement = predict_displacement(encoders_measure[i]);    
        pose2d predicted_pose = integrate_displacement(displacement, last_pose);
        
        trajectory.push_back(predicted_pose);

        std::ofstream file("actual-displacement-predict-dema.csv", std::ios::app); // Open in append mode
        if (file.is_open()) {
            file << last_pose[0] << ',' << last_pose[1] << ',' << last_pose[2] << "\t;" << 
            displacement[0] << ',' << displacement[1] << ',' << displacement[2] << "\t;" 
            << predicted_pose[0] <<',' << predicted_pose[1] << ',' << predicted_pose[2] << "\n";
            file.close(); // Close the file after writing
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }

        last_pose = predicted_pose;
    
    }
    return trajectory;
}