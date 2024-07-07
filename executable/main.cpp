#include "dataset.hpp"
#include "robot_model.hpp"
#include "calibration_utility.hpp"
#include <iostream>
#include <memory>

using namespace std;
using namespace RobotModel;


int main(){
 
    const string source_path = "../data/dataset.txt";
    const string destination_path = "../data/odometry_reconstructed.csv";
    data_management::DataSet dataSet = data_management::DataSet(source_path);

    model_parameters params = {0.1, 0.1, 0.1, 0.1};
    Dynamics::OrioloModel model = Dynamics::OrioloModel(make_unique<model_parameters>(params));
    TractionDriveRobotModel robot = TractionDriveRobotModel({0,0,0,0}, params, model); 

    std::vector<std::array<double, 2>> reconstructed_velocity;
    double driving_v;
    double steering_v;
    double delta_t;
    cerr << __FILE__ << " " << __LINE__ << '\n';
    dataSet.convert_ticks_to_incremental_ticks();
    cal_lib::Pose2d p;
    cal_lib::trajectory odometry_trajectory;
    for(int i=1; i<dataSet.data_.len; i++){

        delta_t = dataSet.data_.time[i] - dataSet.data_.time[i-1];
        //driving_v = robot.reconstruct_driving_input(dataSet.data_.ticks[i][1]);
        steering_v = robot.reconstruct_steering_input(dataSet.data_.ticks[i][0]);
        driving_v = 1;
        //steering_v = 5;
        reconstructed_velocity.push_back({steering_v, driving_v});
        //p = robot.dead_reckoning(steering_v, driving_v, delta_t);
        cout << p <<'\n';
        odometry_trajectory.push_back(p);
    }
    data_management::write_trajectory(destination_path, odometry_trajectory, reconstructed_velocity);

}