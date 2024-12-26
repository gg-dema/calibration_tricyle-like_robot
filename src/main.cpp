
/**
 * @file main.hpp
 * @brief main file for run the calibration over the tricycle robot
 * @author Gabriele G. Di Marzo [github : gg-dema]
 */



#include <iostream>     // input/output as cout
#include <string>       // string

#include <typeinfo>     // check the type of the variable
#include <fstream>      // file stream 

#include <memory>       // shared pointers
#include <unistd.h>     // sleep

#include "header/DataModule.hpp"
#include "header/utility.hpp"
#include "header/robotModels.hpp"
#include "header/CalibrationEngine.hpp"



int main() {
    
    std::cout << "# ------------------------------------------------------------ #" << std::endl;
    std::cout << "# LS-engine for calibration of tricycle robot : DEMA, prob-rob #" << std::endl;
    std::cout << "# ------------------------------------------------------------ #" << std::endl;

    std::cout << "for see the generated trajectory, and the log on the outliers, run script view_log.py\n" << std::endl;
    sleep(5);

    // Load the data from the file
    std::string file_path = "../data/dataset.txt";
    DataObject data = DataLoader::load(file_path);

    // pre-process ticks for overflow
    data.delta_tick_extraction();
    
    // pre-process ground truth
    data.concat_ground_truth();

    // extract the delta-t for each step
    data.delta_time_extraction();    


    // original parameters 

    // encoders params
    double K_steering = 0.1; 
    double K_driving = 0.0106141;
    
    // kin params robot
    double axis_length = 1.4;
    double steer_offset = 0; 
    
    // sensor pose:
    double x_sensor = 1.5;
    double y_sensor = 0;
    double theta_sensor = 0;

    // create parameters structs:
    std::shared_ptr<RobotParameters> robotParameters = std::make_shared<RobotParameters>(axis_length, steer_offset);
    std::shared_ptr<EncoderParameters> encoderParameters = std::make_shared<EncoderParameters>(K_steering, K_driving);
    std::shared_ptr<SensorParameters> sensorParameters = std::make_shared<SensorParameters>(x_sensor, y_sensor, theta_sensor);

    
    // create robot
    Robot robot(
        robotParameters,
        encoderParameters,
        sensorParameters
    );    
 
    // generate trajectory from the data with initial parameters
    poseTrajectory traj_robot = robot.rollout_trajectory(data.process_ticks);
    poseTrajectory traj_sensor = robot.rollout_trajectory_sensor(data.process_ticks);
    
    saveTrajToCSV(traj_robot, "../data/log/trajectory_robot.csv", "x;y;theta");
    saveTrajToCSV(traj_sensor, "../data/log/trajectory_sensor.csv", "x;y;theta");
    
    

    // calibrate
    CalibrationEngine engine(std::make_shared<Robot>(robot));
    int num_iterations = 80;
    
    for(int iteration=0; iteration<num_iterations; iteration++){
        engine.OneIteration(data);

        std::cout << "\nIteration: " << iteration << '\n' << std::endl;
        std::cout << "Params: \n" << robot.get_params_info() << '\n' << std::endl;
    }

    engine._statistics.save_to_file("../data/log/stats.csv");

    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "\nCalibration done! Check the log (run script/view_log) file for the chi statistics\n" << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    std::cout << "Result params: \n" << robot.get_params_info() << std::endl;
    traj_robot = robot.rollout_trajectory(data.process_ticks);
    traj_sensor = robot.rollout_trajectory_sensor(data.process_ticks);

    saveTrajToCSV(traj_robot, "../data/log/calib_traj_robot.csv", "x;y;theta");    
    saveTrajToCSV(traj_sensor, "../data/log/calib_traj_sensor.csv", "x;y;theta");

    // append the obtained params to the stats file
    std::ofstream stats_file("../data/log/stats.csv", std::ios::app);
    stats_file << robot.get_params_info() << std::endl;
    stats_file.close();
  

    
    return 0;
}

