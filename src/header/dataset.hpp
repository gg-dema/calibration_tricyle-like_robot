#pragma once
#include <vector>
#include <array>
#include <string>

#include "calibration_utility.hpp"

namespace data_management{

struct DataStruct{
    std::vector<long double> time; 
    cal_lib::tick_logs ticks;              // steering(absolute) driving(incremental)
    cal_lib::tick_logs delta_ticks;        // steering, driving
    cal_lib::trajectory model_poses;       // vector < (eigenVector3d)array(x y theta) >
    cal_lib::trajectory tracker_poses;           
    int len=0;
};


class DataSet{

public:
    //params
    DataStruct data_;
    
    // method 
    DataSet();
    DataSet(const std::string& source_path);

    void read_data(const std::string& source_path);
    void write_data(const std::string& destination_path);
    void convert_ticks_to_incremental_ticks();


private:

    void process_line(const std::string& line);
    cal_lib::Tick extract_ticks(const std::string& line);   // should became capable of handle the overflow by himself
    cal_lib::Pose2d extract_model_pose(const std::string& line);
    cal_lib::Pose2d extract_tracker_pose(const std::string& line);
    long double extract_time(const std::string& line);

    //std::string pack_sample(const std::vector<double>& sample);

};//end DataSet class 

void write_trajectory(const std::string& destination_path, const cal_lib::trajectory& t, const std::vector<std::array<double, 2>> reconstructed_vel);

};//end data_management namespace
