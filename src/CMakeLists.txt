add_library(calib_lib
            DataModule.cpp
            robotModels.cpp
            utility.cpp
            # .. 
)

add_executable(main main.cpp)
target_link_libraries(main calib_lib)

add_executable(dema main_2.cpp)
target_link_libraries(dema calib_lib)
