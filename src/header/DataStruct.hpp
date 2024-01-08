#pragma once 
#include <vector>
#include <array>
#include <string>

namespace data_management
{
    struct DataStruct{
        std::vector<double> time; 
        std::vector<std::array<u_int32_t, 2>> ticks;
        std::vector< std::array<double, 3>> model_pose;
        std::vector<std::array<double, 3>> tracker_pose;
        int len;
    };

    
} // namespace data_management


