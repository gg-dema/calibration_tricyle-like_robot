#include "DataStruct.hpp"
#include "DataManager.hpp"
#include <iostream>

int main(){

    data_management::DataSet_struct* trajectory;
    data_management::DataManager FileManager;

    const std::string source_path = "../data/dataset.txt";
    const std::string destination_path = "../data/dataset_generated.txt";
    trajectory = FileManager.read_data(source_path);
    std::cout << trajectory->txt.size() << std::endl;
    std::cout << trajectory->x.size() << std::endl;
    std::cout << trajectory->len << std::endl;


    delete trajectory;
    
    //trajectory = FileManager.write_data(destination_path);

}