#include "DataSet.hpp"
#include "RobotModel.hpp"
#include <iostream>
#include <iomanip>

 using namespace std;
 using namespace RobotModel;
int main(){
 
    const string source_path = "../data/dataset.txt";
    const string destination_path = "../data/dataset_generated.txt";
    data_management::DataSet dataSet = data_management::DataSet(source_path);
    state_vect q;
    q[0]=0; q[1]=0; q[2]=0; q[3]=0;
    TractionDriveRobotModel robot = TractionDriveRobotModel(q); 
    state_vect q_2 = robot.forward_kinematic_model(0.5, 0.5);

    cout << q_2[0] << " " << q_2[1] << " " << q_2[2]  << " " <<  q_2[3] << "\n";

    //q_2 = robot.forward_kinematic_model(0.5, 0.5);

    //cout << q_2[0] << " " << q_2[1] << " " << q_2[2]  << " " <<  q_2[3] << endl;

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