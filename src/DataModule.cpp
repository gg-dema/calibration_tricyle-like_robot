// DataModule.cpp
#include "header/DataModule.hpp"

#include <unistd.h>  // just for sleep

u_int8_t DataLoader::CHAR_TIME = 5;
u_int8_t DataLoader::CHAR_TICK = 6;
u_int8_t DataLoader::CHAR_MODEL_POSE = 11;
u_int8_t DataLoader::CHAR_TRACKER_POSE = 13;

int64_t MAX_INT_32 = 4294967295;
// -------------------------------------------------------------------
// DataLoader methods
// -------------------------------------------------------------------

DataObject DataLoader::load(const std::string& file_path){
    DataObject dataset;
    std::ifstream file(file_path);
    std::string line;
    if (file.is_open())
    { 
        while(std::getline(file, line)){
            if(line[0] != '#'){
                process_line(line, dataset);

            }
        }
    }
    else{
        std::cerr << "Error: could not open file" << std::endl;
    }
    file.close();
    std::cout << "File loaded successfully" << std::endl;
    return dataset;
}

void DataLoader::process_line(const std::string& line, DataObject& dataset){
    dataset.time.push_back(extract_time(line));
    dataset.ticks.push_back(extract_tick(line));
    dataset.recorder_trajectory.push_back(extract_recorder_pose(line));
    dataset.ground_truth.push_back(extract_ground_truth_pose(line));
    dataset.length++;
}

double DataLoader::extract_time(const std::string& line){
    // std::cout << line << std::endl;
    long double time;
    std::string sub_string_time = line.substr(DataLoader::CHAR_TIME);
    std::istringstream iss(sub_string_time);
    iss >> time;
    
    //std::cout << " time string: " <<  sub_string_time << std::endl;
    //std::cout << "time: " << time << std::endl;
    
    return time;
}

tick64 DataLoader::extract_tick(const std::string& line){
    tick64 t;
    size_t ticks_position_in_line = line.rfind("ticks:");
    std::string ticks_substring = line.substr(ticks_position_in_line + CHAR_TICK);
    std::istringstream iss(ticks_substring);
    iss >> t[STEERING] >> t[DRIVING];

    // std::cout << std::endl;
    // std::cout << "extract line tick: \n" << line << std::endl;
    // std::cout << "tick substring: " << ticks_substring << '\n' <<
    //             "extracted " << t[STEERING]  << "  " << t[DRIVING] << std::endl;

    return t;
}
pose2d DataLoader::extract_ground_truth_pose(const std::string& line){
    pose2d p;
    size_t tracker_pos_position_in_line = line.rfind("tracker_pose:");
    std::string tracker_pose_substring = line.substr(tracker_pos_position_in_line + DataLoader::CHAR_TRACKER_POSE);
    std::istringstream iss(tracker_pose_substring);
    iss >> p[0] >> p[1] >> p[2];
    
    // std::cout << tracker_pos_position_in_line << std::endl;
    // std::cout << "gt pose line: " << line << '\n' << 
    // "gt pose substring:  " << tracker_pose_substring << 
    // '\n' << "gt saved: " << p << std::endl;
    
    return p;
}

pose2d DataLoader::extract_recorder_pose(const std::string& line){

    pose2d p;
    size_t model_pos_position_in_line = line.rfind("model_pose:");
    std::string pose_substring = line.substr(model_pos_position_in_line + DataLoader::CHAR_MODEL_POSE);
    std::istringstream iss(pose_substring);
    iss >> p[0] >> p[1] >> p[2];
    
    // std::cout << "model pose line: " << line << '\n' << 
    // "model pose substring:  " << pose_substring << std::endl;
    // std::cout << "model pose: " << p << '\n' << std::endl;

    return p;
}

// -------------------------------------------------------------------
// DataObject methods
// -------------------------------------------------------------------

void DataObject::delta_tick_extraction(){

    // we have 2 tipe of overflow --> due the stored variable and due the max resolution of the encoder
    // here we consider only the uint32_t overflow --> the max resolution problem is manage in the encoder classes

    int64_t last_driving_readed_tick = ticks[0][DRIVING];
    int64_t last_steering_readed_tick = ticks[0][STEERING]; 
    
    // pre-allocate the memory for the process ticks
    process_ticks.resize(this->length);

    // iterate over measurements
    for(int i=0; i < this->length; i++){

        // process_ticks[i][STEERING] = static_cast<int64_t>(ticks[i][STEERING]);
        process_ticks[i][STEERING] = ticks[i][STEERING];
        
        // int64_t delta_driving = static_cast<int64_t>(ticks[i][DRIVING]) - (last_driving_readed_tick);
        int64_t delta_driving = ticks[i][DRIVING] - (last_driving_readed_tick);

        // overfloaw correction
        if (delta_driving < -100000){
            delta_driving = (MAX_INT_32 - last_driving_readed_tick) + static_cast<int64_t>(ticks[i][DRIVING]);   
        }    

        // storing 
        process_ticks[i][DRIVING] = delta_driving;

        // update last measurement 
        last_driving_readed_tick = ticks[i][DRIVING];

    }
    
    /* for(int i=0; i < this->length-1; i++){

        // HEY, NON HAI BISOGNO DEI DELTA TICK PER LA TRACTION ---> USA I NORMALI
        // the steering part should not have overflow (at least, in my data)
        delta = static_cast<int64_t>(ticks[i+1][STEERING] - ticks[i][STEERING]);
        delta_tick[STEERING] = delta;

        // driving part
        int64_t current = static_cast<int64_t>(ticks[i][DRIVING]);
        int64_t next = static_cast<int64_t>(ticks[i + 1][DRIVING]);

        // Calculate the delta considering overflow
        if (next < current && (current - next) > (UINT32_MAX / 2)) {
            // Overflow occurred
            delta = static_cast<int64_t>(next + (UINT32_MAX - current) + 1);
        } else if (next > current && (next - current) > (UINT32_MAX / 2)) {
            // Underflow occurred
            delta = static_cast<int64_t>(next - (UINT32_MAX + current) - 1);
        } else {
            // Normal case
            delta = next - current;
        }
        this->ticks[i][DRIVING] = delta;
        //delta_tick[DRIVING] = delta;

        // store the delta
        //delta_ticks.push_back(delta_tick);
    }

    */
}


void DataObject::delta_time_extraction(){
    for(int i=1; i<length-1; i++){
        delta_time.push_back(time[i] - time[i-1]);
    }
}