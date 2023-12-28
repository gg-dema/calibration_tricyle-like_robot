#pragma once
#include <vector>
#include <string>
#include "DataStruct.hpp"


namespace data_management{
class DataManager{
public:
    DataManager();
    DataSet_struct* read_data(const std::string& source_path);
    void write_data(const std::string& destination_path);

private:
    double unpack_row(const std::string& row);
    std::string* pack_sample(const std::vector<double>& sample);

};
};//end data_management namespace
