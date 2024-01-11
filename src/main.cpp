#include "DataSet.hpp"
#include "RobotModel.hpp"
#include "calibration_utility.hpp"
#include <iostream>
#include <iomanip>

using namespace std;
using namespace RobotModel;

int main(){
 
    const string source_path = "../data/dataset.txt";
    const string destination_path = "../data/dataset_generated.txt";
    data_management::DataSet dataSet = data_management::DataSet(source_path);
    pose2d p;
    TractionDriveRobotModel robot = TractionDriveRobotModel(); 

    TractionDriveRobotModel::state_vect q = robot.forward_kinematic_model(0.5, 0.5);
    p = robot.dead_reckoning(0.5, 0.0, 1);
    cout << q[0] << " " << q[1] << " " << q[2]  << " " <<  q[3] << "\n";
    cout << "pose after 1 sec  " << p[0] << " "  << p[1]  << " " << p[2] << "\n";

    p = robot.dead_reckoning(2, 0, 1);
    cout << "pose after 1 sec  " << p[0] << " "  << p[1]  << " " << p[2] << "\n";

    /*int rand_numb = 2433;
    cout << "looks at line " << rand_numb + 9 << "\n";
    
    cout << "x.model " << dataSet.data.model_pose[rand_numb][0] << "\n";
    cout << "y.model " << dataSet.data.model_pose[rand_numb][1] << "\n";
    cout << "z.model " << dataSet.data.model_pose[rand_numb][2] << "\n";

    cout << "x.tracker " << dataSet.data.tracker_pose[rand_numb][0] << "\n";
    cout << "y.tracker" << dataSet.data.tracker_pose[rand_numb][1] << "\n";
    cout << "z.tracker" << dataSet.data.tracker_pose[rand_numb][2] << "\n";
    
    cout << "ticks 1 " << dataSet.data.ticks[rand_numb][0]<< "\n";
    cout << "ticks 2 " << dataSet.data.ticks[rand_numb][1]<< "\n";
    cout << "t " << dataSet.data.time[rand_numb]<< "\n";


    cout << "len " << dataSet.data.len << endl;
    long double delta_t = dataSet.data.time[rand_numb-1] - dataSet.data.time[rand_numb-2];
    cout <<  delta_t << "\n";
    */
}