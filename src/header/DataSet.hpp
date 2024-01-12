#pragma once
#include <vector>
#include <array>
#include <string>

#include "calibration_utility.hpp"

namespace data_management{

struct DataStruct{
    std::vector<long double> time; 
    std::vector<std::array<u_int32_t, 2>> ticks; // steering(absolute) driving(incremental)
    trajectory model_poses; //   vector < array(x y theta) >
    trajectory tracker_poses; // vector < array(x_real, y_real, theta_real) >
    int len=0;
};


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
    pose2d extract_model_pose(const std::string& line);
    pose2d extract_tracker_pose(const std::string& line);
    long double extract_time(const std::string& line);
    //std::string pack_sample(const std::vector<double>& sample);

};//end DataSet class 

void write_trajectory(const std::string& destination_path, const trajectory& t, const std::vector<std::array<double, 2>> reconstructed_vel);

};//end data_management namespace
