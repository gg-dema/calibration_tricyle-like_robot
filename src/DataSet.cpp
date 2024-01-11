#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>

#include "header/DataSet.hpp"
#include "header/calibration_utility.hpp"



namespace data_management{
    //constructor
    DataSet::DataSet(){
        std::cout << "init empty data manager" << std::endl;
    }
    DataSet::DataSet(const std::string& source_path){
        read_data(source_path);
    };

    //funct  
    void DataSet::write_data(const std::string& destination_path){ 
        std::cout << "write data manager to: " << destination_path << std::endl;   
    }

    void DataSet::read_data(const std::string& source_path){   
        
        std::ifstream file(source_path);
        std::string line; 
        if (file.is_open()){
            while(std::getline(file, line)){
                if(line[0]!='#'){process_line(line);}
            }
        file.close();
        }

    }

    void DataSet::process_line(const std::string& line){
        
        data.time.push_back(extract_time(line));
        data.ticks.push_back(extract_ticks(line));
        data.model_poses.push_back(extract_model_pose(line));
        data.tracker_poses.push_back(extract_tracker_pose(line));
        data.len++;
        return;
    }
    
    std::array<u_int32_t, 2> DataSet::extract_ticks(const std::string& line){
        std::array<u_int32_t, 2> ticks;
        u_int8_t CHAR_MODEL_POSE_COUNT = 6; //number of char for "ticks:"

        size_t ticks_position_in_line = line.rfind("ticks:");
        std::string ticks_substring = line.substr(ticks_position_in_line + CHAR_MODEL_POSE_COUNT);
        std::istringstream iss(ticks_substring);
        iss >> ticks[0] >> ticks[1];
        return ticks;
    }
    
    pose2d DataSet::extract_model_pose(const std::string& line){
        u_int8_t CHAR_MODEL_POSE_COUNT = 11; //number of char for "tracker_pose:"
        pose2d model_pose;

        size_t model_pos_position_in_line = line.rfind("model_pose:");
        std::string model_pose_substring = line.substr(model_pos_position_in_line + CHAR_MODEL_POSE_COUNT);
        std::istringstream iss(model_pose_substring);
        iss >> model_pose[0] >> model_pose[1] >> model_pose[2];
        return model_pose;
    }

    pose2d  DataSet::extract_tracker_pose(const std::string& line){
        u_int8_t CHAR_TRACKER_POSE_COUNT = 13;
        pose2d tracker_pose;
        size_t tracker_pos_position_in_line = line.rfind("tracker_pose:");
        std::string tracker_pose_substring = line.substr(tracker_pos_position_in_line + CHAR_TRACKER_POSE_COUNT);
        std::istringstream iss(tracker_pose_substring);
        iss >> tracker_pose[0] >> tracker_pose[1] >> tracker_pose[2]; 
        return tracker_pose;
    }
   
    long double DataSet::extract_time(const std::string& line){
       
        u_int8_t CHAR_TIME = 5; //time: AKA 5 char
        long double time_step;
        std::string sub_string_time = line.substr(CHAR_TIME);
        std::istringstream iss(sub_string_time);
        iss >> time_step;
        return time_step; 
        }

    //std::string* DataSet::pack_sample(const std::vector<double>& sample){}
}; //end data_management namespace