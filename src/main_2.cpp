#include <iostream>     // input/output as cout
#include <string>       // string

#include <typeinfo>     // check the type of the variable
#include <fstream>      // file stream 

#include <memory>       // shared pointers

#include "header/DataModule.hpp"
#include "header/utility.hpp"
#include "header/robotModels.hpp"
#include "header/CalibrationEngine.hpp"

//@ TODO to move to utility
template <typename VectorType>
void saveVectorToCSV(const std::vector<VectorType>& vec, const std::string& file_path, const std::string& header) {
    /*
    static_assert(VectorType::RowsAtCompileTime == 2 && VectorType::ColsAtCompileTime == 1,
                  "VectorType must be an Eigen vector with 2 rows and 1 column.");
    */
    std::ofstream file(file_path, std::ios::out);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << file_path << " for writing." << std::endl;
        return;
    }
    
    std::cout << "vec size: " << vec.size() << std::endl;
    file << header << "\n";

    for (size_t i = 0; i < vec.size(); ++i) {
        file << vec[i][0] << ";" << vec[i][1] ;
        if (i != vec.size() - 1) {
            file << "\n"; // Newline after each element except the last
        }
    }

    file.close();
    std::cout << "Vector saved to " << file_path << std::endl;
}

int main() {
    
    std::cout << "# ------------------------------------------------------------ #" << std::endl;
    std::cout << "# LS-engine for calibration of tricycle robot : DEMA, prob-rob #" << std::endl;
    std::cout << "# ------------------------------------------------------------ #" << std::endl;


    // Load the data from the file
    std::string file_path = "../data/dataset.txt";
    DataObject data = DataLoader::load(file_path);

    // pre-process ticks for overflow
    data.delta_tick_extraction();
    
    //saveVectorToCSV(data.ground_truth, "../data/gt.csv", "x;y;t");
    data.concat_ground_truth();
    


    // log
    saveVectorToCSV(data.process_ticks, "../data/tmp.csv", "steering;driving");

    // original parameters 

    // encoders
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
    //! degub section: encoder test
    /* AbsoluteEncoder steerEncoder(data.process_ticks[0][STEERING]);
    IncrementalEncoder driveEncoder(data.process_ticks[0][DRIVING]);


    std::vector<velocityInput> reconstructed_input;
    reconstructed_input.resize(data.length);
    for (auto i = 0; i < data.process_ticks.size(); i++){

        reconstructed_input[i][0] = steerEncoder.tick_to_input(data.process_ticks[i][STEERING], encoderParameters->K_steering);
        reconstructed_input[i][1] = driveEncoder.tick_to_input(data.process_ticks[i][DRIVING], encoderParameters->K_driving);
    }   

    std::cout << "im gonna save reconstr" << std::endl;
    saveVectorToCSV(reconstructed_input, "../data/reconstructed_input.csv", "steering;driving");
    */
   
    // create robot
    Robot robot(
        robotParameters,
        encoderParameters,
        sensorParameters
    );    
 
    // generate trajectory from the data with initial parameters
    poseTrajectory traj = robot.rollout_trajectory(data.process_ticks);
    
    //saveVectorToCSV(traj, "../data/trajectory.csv", "x;y");



    // calibrate
    CalibrationEngine engine(std::make_shared<Robot>(robot));
    int num_iterations = 100;
    
    for(int iteration=0; iteration<num_iterations; iteration++){
        engine.OneIteration(data);


    }
    
    traj = robot.rollout_trajectory(data.process_ticks);
    
    saveVectorToCSV(traj, "../data/calib_traj.csv", "x;y");



    return 0;
}

