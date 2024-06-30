# create data management lib
add_library(calib_lib 
                ../src/DataSet.cpp 
                ../src/RobotModel.cpp
                )

# create new exec
add_executable(main main.cpp)

# link static (or dynamic, lib.so) lib
target_link_libraries(main calib_lib)   
