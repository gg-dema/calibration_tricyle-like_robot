#include "DataStruct.hpp"
#include "DataSet.hpp"
#include <iostream>


int main(){

    const std::string source_path = "../data/dataset.txt";
    const std::string destination_path = "../data/dataset_generated.txt";
    data_management::DataSet dataSet = data_management::DataSet(source_path);
    int rand_numb = 2433;

    std::cout << "looks at line " << rand_numb + 9 << "\n";
    std::cout << "x.model " << dataSet.data.model_pose[rand_numb][0] << "\n";
    std::cout << "y.model " << dataSet.data.model_pose[rand_numb][1] << "\n";
    std::cout << "z.model " << dataSet.data.model_pose[rand_numb][2] << "\n";
    std::cout << "x.model " << dataSet.data.tracker_pose[rand_numb][0] << "\n";
    std::cout << "y.model " << dataSet.data.tracker_pose[rand_numb][1] << "\n";
    std::cout << "z.model " << dataSet.data.tracker_pose[rand_numb][2] << "\n";
    std::cout << "ticks 1 " << dataSet.data.ticks[rand_numb][0]<< "\n";
    std::cout << "ticks 2 " << dataSet.data.ticks[rand_numb][1]<< "\n";

    std::cout << "len " << dataSet.data.len << std::endl;

}