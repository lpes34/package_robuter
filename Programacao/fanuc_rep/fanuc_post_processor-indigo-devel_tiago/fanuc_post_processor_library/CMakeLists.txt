cmake_minimum_required(VERSION 2.8.3)
project(fanuc_post_processor_library)

find_package(catkin REQUIRED COMPONENTS
             eigen_conversions
             roscpp
             tf tf_conversions)

find_package(CURL)

catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})

add_library (curlite_obj OBJECT src/curlite.cpp)
set_property(TARGET curlite_obj APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11 -fPIC")
add_library (curlite $<TARGET_OBJECTS:curlite_obj>)

add_library (fanuc_post_processor_library_obj OBJECT src/fanuc_post_processor_library.cpp)
set_property(TARGET fanuc_post_processor_library_obj APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11 -fPIC -Wall")
add_library (${PROJECT_NAME} $<TARGET_OBJECTS:fanuc_post_processor_library_obj>)

install(
    TARGETS ${PROJECT_NAME} curlite
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_custom_target (fanuc_post_processor_library_doc 
    rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/fanuc_post_processor_library ${CATKIN_DEVEL_PREFIX}/../src/fanuc_post_processor/fanuc_post_processor_library)

