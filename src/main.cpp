#include "DataStruct.hpp"
#include "DataManager.hpp"
#include <iostream>

int main(){

    data_management::DataSet_struct* trajectory;
    data_management::DataSet2_struct* trajectory2;
    data_management::DataManager FileManager;

    const std::string source_path = "../data/dataset.txt";
    const std::string destination_path = "../data/dataset_generated.txt";
    trajectory = FileManager.read_data(source_path);
    trajectory2 = FileManager.read_data2(source_path);

    delete trajectory;
    delete trajectory2;
    //trajectory = FileManager.write_data(destination_path);

}