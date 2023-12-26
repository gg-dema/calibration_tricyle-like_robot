#include <iostream>
#include <vector>
#include "header/DataStruct.hpp"
#include "header/DataManager.hpp"


namespace data_management{
    //constructor
    DataManager::DataManager(){
        std::cout << "init data manager" << std::endl;
    }

    //funct
    DataSet_struct* DataManager::read_data(const std::string& source_path){
        std::cout << "read data manager" << source_path << std::endl;
        DataSet_struct* data = new DataSet_struct;
        data->x = 5;
        data->y = 10;
        data->len = 100;
        return data;
    }
    
    void DataManager::write_data(const std::string& destination_path){ 
        std::cout << "write data manager to: " << destination_path << std::endl;
        
    }

    DataSet2_struct* DataManager::read_data2(const std::string& source_path){   
        std::cout << "read data 2 manager" << source_path << std::endl;
        DataSet2_struct* data = new DataSet2_struct;
        std::vector<double> x;
        std::vector<double> y;
        data->len = 0;
        data->x = x;
        data->y = y;
        return data;


    }
}; //end data_management namespace