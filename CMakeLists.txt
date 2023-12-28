
cmake_minimum_required(VERSION 3.12)
project(calibration VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 14)
set(DEVEL 0)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/src/header)
link_directories(${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(src)
