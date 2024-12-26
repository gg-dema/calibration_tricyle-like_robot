#include <iostream>     // input/output as cout
#include <string>       // string

#include <typeinfo>     // check the type of the variable
#include <fstream>      // file stream 

#include <memory>       // shared pointers

#include "header/DataModule.hpp"
#include "header/utility.hpp"
#include "header/robotModels.hpp"
#include "header/CalibrationEngine.hpp"


int main() {
    
    std::cout << "# ------------------------------------------------------------ #" << std::endl;
    std::cout << "# LS-engine for calibration of tricycle robot : DEMA, prob-rob #" << std::endl;
    std::cout << "# ------------------------------------------------------------ #" << std::endl;


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
    poseTrajectory traj = robot.rollout_trajectory(data.process_ticks);
    
    saveTrajToCSV(traj, "../data/trajectory.csv", "x;y;theta");

    

    // calibrate
    CalibrationEngine engine(std::make_shared<Robot>(robot));
    int num_iterations = 10;
    
    for(int iteration=0; iteration<num_iterations; iteration++){
        engine.OneIteration(data);
    }
    
    traj = robot.rollout_trajectory_sensor(data.process_ticks);
    
    saveTrajToCSV(traj, "../data/calib_traj.csv", "x;y;theta");


    
    return 0;
}

