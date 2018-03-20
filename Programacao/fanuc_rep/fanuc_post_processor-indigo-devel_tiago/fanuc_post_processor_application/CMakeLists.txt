cmake_minimum_required(VERSION 2.8.3)
project(fanuc_post_processor_application)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
             fanuc_post_processor_library
             moveit_ros_planning_interface
             roscpp
             tf tf_conversions
             visualization_msgs)

find_package(CURL)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS fanuc_post_processor_library)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable (fanuc_post_processor_application src/fanuc_post_processor_application.cpp)
set_property(TARGET fanuc_post_processor_application APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
target_link_libraries (fanuc_post_processor_application ${catkin_LIBRARIES} fanuc_post_processor_library curl curlite)
