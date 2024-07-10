#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>

#include "header/dataset.hpp"
#include "header/calibration_utility.hpp"



namespace data_management{
    //constructor
    DataSet::DataSet(){
        std::cout << "init empty data manager" << std::endl;
    }
    DataSet::DataSet(const std::string& source_path){
        read_data(source_path);
    };

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
        data_.time.push_back(extract_time(line));
        data_.ticks.push_back(extract_ticks(line));
        data_.model_poses.push_back(extract_model_pose(line));
        data_.tracker_poses.push_back(extract_tracker_pose(line));
        data_.len++;
        return;
    }
    
    cal_lib::Tick DataSet::extract_ticks(const std::string& line){
        cal_lib::Tick t;

        u_int8_t CHAR_MODEL_POSE_COUNT = 6; //number of char for "ticks:"

        size_t ticks_position_in_line = line.rfind("ticks:");
        std::string ticks_substring = line.substr(ticks_position_in_line + CHAR_MODEL_POSE_COUNT);
        std::istringstream iss(ticks_substring);
        iss >> t[0] >> t[1];
        return t;
    }
    
    cal_lib::Pose2d DataSet::extract_model_pose(const std::string& line){
        u_int8_t CHAR_MODEL_POSE_COUNT = 11; //number of char for "tracker_pose:"
        cal_lib::Pose2d model_pose;

        size_t model_pos_position_in_line = line.rfind("model_pose:");
        std::string model_pose_substring = line.substr(model_pos_position_in_line + CHAR_MODEL_POSE_COUNT);
        std::istringstream iss(model_pose_substring);
        iss >> model_pose[0] >> model_pose[1] >> model_pose[2];
        return model_pose;
    }

    cal_lib::Pose2d  DataSet::extract_tracker_pose(const std::string& line){
        
        u_int8_t CHAR_TRACKER_POSE_COUNT = 13;
        cal_lib::Pose2d tracker_pose;

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

    void DataSet::convert_ticks_to_incremental_ticks(){
        
        cal_lib::tick_logs delta_ticks;

        // inizialize with first val of ticks
        u_int64_t old_traction_abs = data_.ticks[0][0];
        u_int64_t old_steering_incremental = data_.ticks[0][1];;

        u_int64_t new_traction_abs;
        u_int64_t new_steering_incremental;

        int64_t delta_steering_ticks;
        int64_t delta_traction_ticks;

        for(int i=0; i<data_.len; i++){
            new_traction_abs = data_.ticks[i][0]; // steeroing is absolute, not traction
            new_steering_incremental = data_.ticks[i][1]; // same for traction
            
            //normalize the absolute val
            // problem: traction is incremental, not absolute
            delta_traction_ticks = (int64_t) (new_traction_abs - old_traction_abs);
            //if (delta_traction_ticks < -100000){
            //   delta_traction_ticks = (UINT32_MAX - old_traction_abs) + new_traction_abs;
            //}


            //normalize the steering val
            //maybe it's not needed

            data_.delta_ticks.push_back({delta_steering_ticks, delta_traction_ticks});

            //updat old_val
            old_traction_abs = new_traction_abs;
            old_steering_incremental = new_steering_incremental;
        }

    }
    //std::string* DataSet::pack_sample(const std::vector<double>& sample){}

    // GENERIC FUNCT
    void write_trajectory(
        const std::string& destination_path,
        const cal_lib::trajectory& t, 
        const std::vector<std::array<double, 2>> reconstructed_vel){

        std::ofstream outputFile(destination_path);
        if (outputFile.is_open()){
            outputFile << "x;y;theta;steering_v;driving_v\n";
            for (std::size_t i=0; i<t.size(); i++ ){
                outputFile << t[i][0] << "; " << t[i][1]  << "; " << t[i][2] << "; " << reconstructed_vel[i][0] << "; " << reconstructed_vel[i][1] << '\n';
            }
        }
        outputFile.close();
    }
  
      
    void DataSet::write_data(const std::string& destination_path){ 
        std::ofstream file;
        file.open(destination_path);
        file << "steering, driving, delta_steering, delta_driving\n";
        for(int i=0; i<data_.len; i++)
        {
            file << data_.ticks[i][0] <<";" <<data_.ticks[i][1] << ";"
                << data_.delta_ticks[i][0] <<";" <<data_.delta_ticks[i][1] << '\n';
        }
        file.close();
    }



}; //end data_management namespace
