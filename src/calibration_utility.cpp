#include "calibration_utility.hpp"
#include <ostream>


std::ostream& operator<<(std::ostream& os, const pose2d& p){
    os << "pose-->  x: " << p[0] << " y: " << p[1] << " orientation: " << p[2];
    return os;
}
std::ostream& operator<<(std::ostream& os, const trajectory& t){}
