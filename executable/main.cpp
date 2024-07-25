#include "dataset.hpp"
#include "robot_model.hpp"
#include "calibration_utility.hpp"
#include <iostream>
#include <memory>
#include<unistd.h>     // for sleep

using namespace std;
using namespace RobotModel;


int main(){
 
    const string source_path = "../data/dataset.txt";
    const string destination_path = "../data/odometry_reconstructed.csv";
    data_management::DataSet dataSet = data_management::DataSet(source_path);

    model_parameters params = {0.1, 0.0106141, 1.4, 0 };
    Dynamics::GrisettiModel model = Dynamics::GrisettiModel(make_shared<model_parameters>(params));
    //Dynamics::OrioloModel model = Dynamics::OrioloModel(make_shared<model_parameters>(params));
    TractionDriveRobotModel robot = TractionDriveRobotModel({0.1,0.2,0.1,0.1}, params, model); 

    std::vector<std::array<long double, 2>> reconstructed_velocity;
    long double driving_v;
    long double steering_v;
    double delta_t;
    //dataSet.write_data("../data/dataset_w.csv");

    cal_lib::Pose2d p;
    cal_lib::trajectory odometry_trajectory;

    for(int i=1; i<dataSet.data_.len; i++){

        delta_t = dataSet.data_.time[i] - dataSet.data_.time[i-1];

        if (delta_t < 0.009)
        {
            std::cout << "skipping... \n";
            continue;
        }

        driving_v = robot.reconstruct_traction_input(
            static_cast<int64_t>(dataSet.data_.ticks[i][cal_lib::TRACTION]) - static_cast<int64_t>(dataSet.data_.ticks[i-1][cal_lib::TRACTION])
        );
        steering_v = robot.reconstruct_steering_input(
            //static_cast<int64_t>(dataSet.data_.ticks[i][cal_lib::STEERING]) - static_cast<int64_t>(dataSet.data_.ticks[i-1][cal_lib::STEERING])
            static_cast<int64_t>(dataSet.data_.ticks[i][cal_lib::STEERING])
        );

        reconstructed_velocity.push_back({steering_v, driving_v});
        std::vector<long double> velocity= {steering_v, driving_v};
        p = robot.forward_step(velocity, delta_t);
        
        odometry_trajectory.push_back(p);
    }
    data_management::write_trajectory(destination_path, odometry_trajectory, reconstructed_velocity);

}