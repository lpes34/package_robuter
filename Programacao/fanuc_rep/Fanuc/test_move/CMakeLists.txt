cmake_minimum_required(VERSION 2.8.3)
project(test_move)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  industrial_trajectory_filters
  pluginlib
  roscpp
  rqt_gui
  rqt_gui_cpp
  rviz
  visualization_msgs
  moveit_ros_move_group
  tf
  tf_conversions
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_cartesian_plan_plugin
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
${industrial_trajectory_filters_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


### Declare a cpp library

 

## Declare a cpp executable
#add_executable(move_1 src/move_1.cpp)
#add_executable(move_2 src/move_2.cpp)
add_executable(move_3 src/move_3.cpp)
#add_executable(move_4 src/move_4.cpp)
add_executable(move_5 src/move_5.cpp)
#add_executable(move_5 src/move_5.cpp)
#add_executable(move_6 src/move_6.cpp)
#add_executable(move_7 src/move_7.cpp)
#add_executable(move_8 src/move_8.cpp)
add_executable(moveee src/moveee.cpp)
#add_executable(pick_place_simoes src/pick_place_simoes.cpp)
add_executable(moveit_all src/moveit_all.cpp)
add_executable(move_teste src/move_teste.cpp)
add_executable(calibration src/calibration.cpp)
add_executable(calibration2 src/calibration2.cpp)
add_executable(posicao_calibracao src/posicao_calibracao.cpp)
add_executable(tf_converter src/tf_converter.cpp)




#add_executable(kinematic_model src/kinematic_model.cpp)
#add_executable(ros_api_tutorial src/ros_api_tutorial.cpp)
#add_executable(planning_scene src/planning_scene.cpp)
#add_executable(planning_scene_ros_api src/planning_scene_ros_api.cpp)
#add_executable(motion_planning_api src/motion_planning_api.cpp)


#add_executable(state_publisher src/state_publisher.cpp)
#add_executable(pcalvin_pick_n_place src/pcalvin_pick_n_place.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lesson_move_group_node lesson_move_group_generate_messages_cpp)









## Specify libraries to link a library or executable target against
# target_link_libraries(move_1 ${catkin_LIBRARIES})
# target_link_libraries(move_2 ${catkin_LIBRARIES})
 target_link_libraries(move_3 ${catkin_LIBRARIES})
# target_link_libraries(move_4 ${catkin_LIBRARIES})
 target_link_libraries(move_5 ${catkin_LIBRARIES})
# target_link_libraries(move_6 ${catkin_LIBRARIES})
# target_link_libraries(move_7 ${catkin_LIBRARIES})
# target_link_libraries(move_8 ${catkin_LIBRARIES})
 target_link_libraries(moveee ${catkin_LIBRARIES})
#  target_link_libraries(pick_place_simoes ${catkin_LIBRARIES})
   target_link_libraries(moveit_all ${catkin_LIBRARIES})
 target_link_libraries(move_teste ${catkin_LIBRARIES})
  target_link_libraries(calibration ${catkin_LIBRARIES})
    target_link_libraries(calibration2 ${catkin_LIBRARIES})
    target_link_libraries(posicao_calibracao ${catkin_LIBRARIES})
        target_link_libraries(tf_converter ${catkin_LIBRARIES})
    
# target_link_libraries(kinematic_model ${catkin_LIBRARIES})
# target_link_libraries(ros_api_tutorial ${catkin_LIBRARIES}) #//NAO ESTA A DAR
# target_link_libraries(planning_scene ${catkin_LIBRARIES}) 
# target_link_libraries(planning_scene_ros_api ${catkin_LIBRARIES}) 
# target_link_libraries(motion_planning_api ${catkin_LIBRARIES})
#   target_link_libraries(state_publisher ${catkin_LIBRARIES})
#  target_link_libraries(pcalvin_pick_n_place ${catkin_LIBRARIES})
 
#set(moveit_cartesian_plan_plugin_HDRS
#  include/moveit_cartesian_plan_plugin/add_way_point.h
#  include/moveit_cartesian_plan_plugin/on_mouse_click.h
#  include/moveit_cartesian_plan_plugin/point_tree_item.h
#  include/moveit_cartesian_plan_plugin/point_tree_model.h
#  include/moveit_cartesian_plan_plugin/widgets/path_planning_widget.h
#  include/moveit_cartesian_plan_plugin/generate_cartesian_path.h
#)
#rosbuild_add_executable(sphereo_node src/move_teste.cpp include/moveit_cartesian_plan_plugin/generate_cartesian_path.h)
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lesson_move_group lesson_move_group_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS move_teste
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/moveit_cartesian_plan_plugin/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lesson_move_group.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
