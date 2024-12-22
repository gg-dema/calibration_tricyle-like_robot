add_library(calib_lib
            DataModule.cpp
            robotModels.cpp
            utility.cpp
            # .. 
)

if(BUILD_TESTING)
    add_executable(dema_test  testing/gtest_main.cpp)
    target_link_libraries(dema_test PRIVATE GTest::gtest_main calib_lib)

    include(GoogleTest)
    gtest_discover_tests(dema_test) 
endif()


add_executable(main main.cpp)
target_link_libraries(main calib_lib)

add_executable(dema main_2.cpp)
target_link_libraries(dema calib_lib)
