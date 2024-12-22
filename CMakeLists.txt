
cmake_minimum_required(VERSION 3.12)
project(calibration VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 14)


set(DEVEL 0)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/src/header)
link_directories(${PROJECT_SOURCE_DIR}/lib)

include(CTest)

add_subdirectory(src)
add_subdirectory(external) # contains google test
