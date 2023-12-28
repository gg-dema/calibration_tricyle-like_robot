#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "header/DataStruct.hpp"
#include "header/DataManager.hpp"


namespace data_management{
    //constructor
    DataManager::DataManager(){
        std::cout << "init data manager" << std::endl;
    }

    //funct  
    void DataManager::write_data(const std::string& destination_path){ 
        std::cout << "write data manager to: " << destination_path << std::endl;
        
    }

    DataSet_struct* DataManager::read_data(const std::string& source_path){   
        
        DataSet_struct* data = new DataSet_struct;
        std::ifstream file(source_path);
        std::string line; 
        std::vector<std::string> data_lines;
        std::vector<double> tmp_x;
        // pre-allocate size of txt file

        if (file.is_open()){
            while(std::getline(file, line)){
                tmp_x.push_back(unpack_row(line));
                data_lines.push_back(line);
            }
        file.close();
        std::cout << "read data 2 manager" << source_path << std::endl;
        
        data->txt = data_lines;
        data->len = data_lines.size();
        data->x = tmp_x;
        return data;
        }
        else{
            delete data;
            return nullptr;
        }
    }

    double DataManager::unpack_row(const std::string& row){
        return 5.0;
    }
    std::vector<double> DataManager::unpack_row_full(const std::string& row){}
    std::string* DataManager::pack_sample(const std::vector<double>& sample){}
}; //end data_management namespace