//main.cpp

#include "header/utility.hpp"
#include "header/DataModule.hpp"
#include "header/robotModels.hpp"
//#include "header/CalibrationEngine.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>  // just for sleep in debug

#include <eigen3/Eigen/Dense>



// to move to utility
template <typename VectorType>
void saveVectorToCSV(const std::vector<VectorType>& vec, const std::string& file_path, const std::string& header) {
    /*
    static_assert(VectorType::RowsAtCompileTime == 2 && VectorType::ColsAtCompileTime == 1,
                  "VectorType must be an Eigen vector with 2 rows and 1 column.");
    */
    std::ofstream file(file_path, std::ios::out);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << file_path << " for writing." << std::endl;
        return;
    }
    
    std::cout << "vec size: " << vec.size() << std::endl;
    file << header << "\n";

    for (size_t i = 0; i < vec.size(); ++i) {
        file << vec[i][0] << ";" << vec[i][1] << ";"<<  vec[i][2] << ";";
        if (i != vec.size() - 1) {
            file << "\n"; // Newline after each element except the last
        }
    }

    file.close();
    std::cout << "Vector saved to " << file_path << std::endl;
}


int main(){


 // Load the data from the file
    std::string file_path = "../data/dataset.txt";
    DataObject data = DataLoader::load(file_path);
    data.delta_tick_extraction();

    // check the delta ground truth
    poseTrajectory delta_gt;
    pose2d last_pose(0.0, 0.0, 0.0);

    data.concat_ground_truth();

    //for(int i=0; i < data.length-1; i++){
    //    delta_gt.push_back(
    //        t2v( v2t(last_pose).inverse() * v2t(data.ground_truth[i]))
    //        );
    //        last_pose = data.ground_truth[i];
    //}

    // save the delta ground truth
    saveVectorToCSV(data.ground_truth, "../data/delta_gt.csv", "x;y;theta");


}

