/**
 * @file DataModule.hpp
 * @brief Utility and classes for deal with the dataset of the project 
 * @details Here you can find two object: 
 *              1) DataObject class [store the data];
 *              2) DataLoader class [load the dataset and deal with file]
 *          Any preprocessing is done by the DataObject itself
 * 
 * @author Gabriele G. Di Marzo [github : gg-dema]
 */

#pragma once

#include "utility.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>


/**
 * @brief DataObject class. Store the dataset and the result of the calculation 
 * @details This class store the dataset and contains the methods for process the data.
 * usage: 
 *  - load the dataset with the DataLoader class
 *  - apply all the desired preprocessing by call the 3 method of the class
 */
class DataObject{

friend class DataLoader; // for access private constructor
public:
    // generic data
    long int length = 0;
  
    // for dataset
    std::vector<long double> time;              // time stamp of the measure
    std::vector<tick64> ticks;                  // ticks of the encoders
    std::vector<tick64> process_ticks;          // ticks after the overflow correction/processing
    poseTrajectory recorder_trajectory;         // odometry of the models
    poseTrajectory ground_truth;                // ground truth of the robot [for the sensors]
    poseTrajectory ground_truth_delta;          // incremental pose extracted from the ground truth 
    
    // from calculation (calibration, decondin etc)
    std::vector<double> delta_time;
    poseTrajectory calculated_trajectory;
    velocityInput velocity;

    //! disclaimer: all of the next public function operate over the internal data struct of the class
    /** @brief extract the relative displacement of the encoder (delta ticks) AND account for measure overflow */
    void delta_tick_extraction();
    /** @brief extract the time difference between the measures */
    void delta_time_extraction();
    /** @brief convert the ground truth by absolute pose to a set of consecutive transformations */
    void concat_ground_truth(); 
    
private:
    DataObject()=default;
};


/**
 * @brief DataLoader class. Load the dataset from a file and return a DataObject [just deal with the file]
 */

class DataLoader{

// number of char to skip before data (each data is preceed by the name of the data)
static u_int8_t CHAR_TIME;
static u_int8_t CHAR_TICK;
static u_int8_t CHAR_MODEL_POSE;
static u_int8_t CHAR_TRACKER_POSE;

public:
    /** @brief load the dataset at the given path. This just load the data, not apply any preprocessing here */
    static DataObject load(const std::string& file_path);

private:
    /** @brief process a single line of the dataset */
    static void process_line(const std::string& line, DataObject& dataset);  

    /** @brief extract the time from the line */
    static double extract_time(const std::string& line);

    /** @brief extract the ticks from the line */
    static tick64 extract_ticks(const std::string& line);

    /** @brief extract the recorder pose from the line [AKA the odometry of the robot]*/
    static pose2d extract_recorder_pose(const std::string& line);

    /** @brief extract the ground truth pose from the line [AKA the real pose of the robot (thanks sensor)]*/
    static pose2d extract_ground_truth_pose(const std::string& line);
};