# create data management lib
add_library(data_managers DataManager.cpp)

# create new exec
add_executable(main main.cpp)

# link static (or dynamic, lib.so) lib
target_link_libraries(main data_managers)   