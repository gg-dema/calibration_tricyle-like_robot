#include "DataSet.hpp"
#include "RobotModel.hpp"
#include "calibration_utility.hpp"
#include <iostream>

using namespace std;
using namespace RobotModel;


int main(){
 
    const string source_path = "../data/dataset.txt";
    const string destination_path = "../data/odometry_reconstructed.csv";
    data_management::DataSet dataSet = data_management::DataSet(source_path);
    TractionDriveRobotModel robot = TractionDriveRobotModel(); 

    std::vector<std::array<double, 2>> reconstructed_velocity;
    double driving_v;
    double steering_v;
    double delta_t;
    cal_lib::pose2d p;
    cal_lib::trajectory odometry_trajectory;
    for(int i=1; i<dataSet.data.len; i++){

        delta_t = dataSet.data.time[i] - dataSet.data.time[i-1];
        driving_v = robot.reconstruct_driving_input(dataSet.data.ticks[i][1]);
        steering_v = robot.reconstruct_steering_input(dataSet.data.ticks[i][0]);
        reconstructed_velocity.push_back({steering_v, driving_v});
        p = robot.dead_reckoning(steering_v, driving_v, delta_t);
        cout << p <<'\n';
        odometry_trajectory.push_back(p);
    }
    data_management::write_trajectory(destination_path, odometry_trajectory, reconstructed_velocity);

}