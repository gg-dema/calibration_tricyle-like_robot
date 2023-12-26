#pragma once 
#include <vector>

namespace data_management
{
    struct DataSet_struct{ 
        int x;
        int y;
        int len;
    };

    struct DataSet2_struct{
        std::vector<double> x;
        std::vector<double> y;
        int len;
    };

    
} // namespace data_management


