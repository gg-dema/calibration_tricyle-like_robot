#pragma once
#include <vector>
#include <array>
#include <string>
#include "DataStruct.hpp"


namespace data_management{

class DataSet{

public:
    //params
    DataStruct data;
    // method 
    DataSet();
    DataSet(const std::string& source_path);
    void read_data(const std::string& source_path);
    void write_data(const std::string& destination_path);

private:
    void process_line(const std::string& line);
    std::array<u_int32_t, 2> extract_ticks(const std::string& line);
    std::array<double, 3> extract_model_pose(const std::string& line);
    std::array<double, 3> extract_tracker_pose(const std::string& line);
    //std::string pack_sample(const std::vector<double>& sample);

};//end DataSet class 
};//end data_management namespace
