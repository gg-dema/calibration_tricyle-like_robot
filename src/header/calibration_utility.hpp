#pragma once 
#include <array>
#include <vector>
#include <ostream>

// this file contains only utils funcition and alias to use around all the project

using pose2d = std::array<double, 3>; //x y theta
using trajectory = std::vector<pose2d>;

std::ostream& operator<<(std::ostream& os, const pose2d& p);
std::ostream& operator<<(std::ostream& os, const trajectory& t);

