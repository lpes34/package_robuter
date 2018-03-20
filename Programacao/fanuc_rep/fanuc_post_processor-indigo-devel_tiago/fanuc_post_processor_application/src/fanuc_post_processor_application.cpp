#include "fanuc_post_processor_library/fanuc_post_processor_library.hpp"

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/StdVector>

/**
 * @file fanuc_post_processor_application.hpp
 * @brief This is an application using fanuc_post_processor_library.
 * @author Victor Lamoine - Institut Maupertuis (France)
 * @date Project started in March 2016
 */
/**@mainpage This application is a test of fanuc_post_processor_library.
 * Look at the [documentation](https://github.com/InstitutMaupertuis/fanuc_post_processor/blob/indigo-devel/README.md) in order to have more information.
 */

/** @brief The main function
 * @param[in] argc
 * @param[in] argv
 * @return Exit status */
int main(int argc, char **argv)
{
  std::string package = "fanuc_post_processor_application";
  ros::init(argc, argv, package);
  ros::NodeHandle node;

  FanucPostProcessor fanuc_pp;
  fanuc_pp.setProgramName("ros_tp_example_program");
  fanuc_pp.setProgramComment("ROS generated");
  fanuc_pp.useLineNumbers(false);
  fanuc_pp.appendComment("This is a ROS generated TP program");
  fanuc_pp.appendSetRegister(7, false);
  fanuc_pp.appendWait(0.5);
  fanuc_pp.appendSetRegister(7, true);
  fanuc_pp.appendWait((unsigned) 9);
  fanuc_pp.appendEmptyLine();
  fanuc_pp.appendPoseCNT(FanucPostProcessor::JOINT, Eigen::Isometry3d::Identity(), 2, 20, FanucPostProcessor::PERCENTAGE, 100);
  fanuc_pp.appendRun("MY_OTHER_TP_PROGRAM");
  
  std::string program;
  fanuc_pp.generateProgram(program);
  ROS_WARN_STREAM("This is the generated program:\n\n" << program);

  if (!fanuc_pp.uploadToFtp("192.168.0.231"))
    return -1;
  return 0;
}



