#pragma once
#include <cstdint>
#include <string>

namespace data_manager{

    struct {
        int len;
        int64_t* x;
        int64_t* y;
    } dataset; 


    class FileManager{

        public: 
            struct dataset read_file_trajectory(const std::string &path);
            void wrote_new_trajectory(struct dataset);

    };

}
