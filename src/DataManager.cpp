#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
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
        std::vector<double> x;
        double temp_x;
        // pre-allocate size of txt file

        if (file.is_open()){
            while(std::getline(file, line)){
                temp_x = unpack_row(line);
                if (temp_x!=-1){x.push_back(temp_x);}
                data_lines.push_back(line);
            }
        file.close();
        std::cout << "read data 2 manager" << source_path << std::endl;
        
        data->txt = data_lines;
        data->len = x.size();
        data->x = x;
        return data;
        }
        else{
            delete data;
            return nullptr;
        }
    }

    double DataManager::unpack_row(const std::string& row){
        if(row[0]=='#'){return -1;}
        int pos_x;
        std::regex regex_pattern = std::regex("[\\[0-9]]");
        std::smatch flag;
        std::regex_search(row, flag, regex_pattern);
        // regex patter [\\[0-9]] for only numbers
        //if (flag)
        //    return pos_x;
        return 5.0;
    }

    std::string* DataManager::pack_sample(const std::vector<double>& sample){}
}; //end data_management namespace