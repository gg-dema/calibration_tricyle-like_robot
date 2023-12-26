#include "DataStruct.hpp"
#include "DataManager.hpp"

int main(){

    data_management::DataSet2_struct* trajectory;
    data_management::DataManager FileManager;

    const std::string source_path = "data/dataset.txt";
    const std::string destination_path = "data/dataset_generated.txt";
    trajectory = FileManager.read_data2(source_path);
    
    delete trajectory;
    //trajectory = FileManager.write_data(destination_path);

}