// DataModule.hpp
#pragma once

#include "utility.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>


class DataObject{

friend class DataLoader; // for access private constructor
public:
    // generic data
    long int length = 0;
  
    // for dataset
    std::vector<long double> time;
    std::vector<tick64> ticks; 
    std::vector<tick64> process_ticks;
    poseTrajectory recorder_trajectory;
    poseTrajectory ground_truth;
    
    // from calculation (calibration, decondin etc)
    std::vector<double> delta_time;
    poseTrajectory calculated_trajectory;
    velocityInput velocity;

    void delta_tick_extraction();
    void delta_time_extraction();
    
private:
    DataObject()=default;
};

class DataLoader{

// number of char to skip before data (each data is preceed by the name of the data)
static u_int8_t CHAR_TIME;
static u_int8_t CHAR_TICK;
static u_int8_t CHAR_MODEL_POSE;
static u_int8_t CHAR_TRACKER_POSE;

public:
    static DataObject load(const std::string& file_path);

private:
  static void process_line(const std::string& line, DataObject& dataset);  
  static double extract_time(const std::string& line);
  static tick64 extract_tick(const std::string& line);
  static pose2d extract_recorder_pose(const std::string& line);
  static pose2d extract_ground_truth_pose(const std::string& line);
};