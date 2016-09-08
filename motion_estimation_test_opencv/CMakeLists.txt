cmake_minimum_required(VERSION 2.8.3)
project(motion_estimation_test_opencv)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++11)

find_package(OpenCV 3.1.0 REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
       	src/motion_estimator.cpp
        )

############
# BINARIES #
############
cs_add_executable(motion_estimation_test_opencv_node
        src/motion_estimation_test_opencv_node.cpp
        )

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(motion_estimation_test_opencv_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
