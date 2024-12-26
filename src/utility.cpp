#include "header/utility.hpp"
#include <cmath>

#include <fstream>      // file stream 
#include <iostream>     


Eigen::Matrix3d v2t(const pose2d& pose){
    
    Eigen::Matrix3d T;
    T.setZero();

    T(0,0) = cos(pose[2]);
    T(0,1) = -sin(pose[2]);
    T(1,0) = sin(pose[2]);
    T(1,1) = cos(pose[2]);

    T(0,2) = pose[0];
    T(1,2) = pose[1];
    T(2,2) = 1;
    return T;
}

pose2d t2v(const Eigen::Matrix3d& T){
    pose2d pose;
    pose[0] = T(0,2);
    pose[1] = T(1,2);
    pose[2] = std::atan2(T(1,0), T(0,0));
    return pose;
}


void saveTrajToCSV(
    const std::vector<pose2d>& traj, 
    const std::string& file_path, 
    const std::string& header){

        std::ofstream file(file_path, std::ios::out);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << file_path << " for writing." << std::endl;
            return;
        }

        file << header << "\n";
        for (size_t i=0; i < traj.size(); i++){
            file << traj[i][0] << ";" << traj[i][1] << ";" << traj[i][2];
            
            if (i != traj.size() - 1){
                // new line for each element except the last
                file << "\n"; 
            }
        }

        file.close();
        std::cout << "trajectory saved to " << file_path << std::endl;  
    }