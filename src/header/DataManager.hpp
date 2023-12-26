#pragma once
#include <iostream>
#include "DataStruct.hpp"


namespace data_management{
class DataManager{
public:
    DataManager();
    DataSet_struct* read_data(const std::string& source_path);
    DataSet2_struct* read_data2(const std::string& source_path);
    void write_data(const std::string& destination_path);

};
};//end data_management namespace
