#pragma once 
#include <vector>
#include <string>

namespace data_management
{
    struct DataSet_struct{
        std::vector<std::string> txt;
        std::vector<double> x;
        std::vector<double> y;
        int len;
    };

    
} // namespace data_management


