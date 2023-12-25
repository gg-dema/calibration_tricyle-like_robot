
cmake_minimum_required(VERSION 3.12)
project(project)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
add_executable(project
        src/DataStruct.cpp 
        src/DataManager.cpp
        src/main.cpp
)   