#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
//###############Includes para o grupo do inverse kinematic
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/joint_model_group.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
//###############Includes para o grupo do inverse kinematic
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit/joint_model_group.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <moveit_cartesian_plan_plugin/generate_cartesian_path.h>
#include <geometry_msgs/PoseArray.h>

////******************************************************************************************************
  // Todos os includes
////******************************************************************************************************
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
//#include <moveit/joint_model_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <moveit_msgs/GetPositionIK.h>




#include <industrial_trajectory_filters/filter_base.h>
#include <industrial_trajectory_filters/uniform_sample_filter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>








int main(int argc, char **argv)
{

	ros::init(argc, argv, "six_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);

	spinner.start();

	moveit::planning_interface::MoveGroup group("manipulator");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	group.setEndEffectorLink("tool0");
	
	geometry_msgs::PoseStamped robot_pose;
	robot_pose = group.getCurrentPose();

	geometry_msgs::Pose current_position;
	current_position = robot_pose.pose;
	
	 // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  
  

	/*Retrive position and orientation */
	geometry_msgs::Point exact_pose = current_position.position;
	geometry_msgs::Quaternion exact_orientation = current_position.orientation;

	std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
	std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;

	



	ROS_INFO("Reference frame : %s",group.getPlanningFrame().c_str());

	ROS_INFO("Reference frame : %s",group.getEndEffectorLink().c_str());
	
	// Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
	sleep(4.0);

	// Get the robot at a writing position
  std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
	group_variable_values[0] = 0.0;
  group_variable_values[1] = 0.0;
  group_variable_values[2] = 0.0;
  group_variable_values[3] = 0.0;
  group_variable_values[4] = 0.0;
  group_variable_values[5] = 0.0;
	group.setJointValueTarget(group_variable_values);
	group.plan(my_plan);
	group.execute(my_plan);



////******************************************************************************************************
////Planning to a Pose goal
////******************************************************************************************************
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
  geometry_msgs::Pose target_pose3;
    geometry_msgs::Pose target_pose4;
       geometry_msgs::Pose target_pose5;
  
//  Eigen::Affine3d target_pose1;
//  Eigen::Affine3d target_pose2;
//  Eigen::Affine3d target_pose3;

	 target_pose1.position.x = 0.415008;
	 target_pose1.position.y = 0.0;
	 target_pose1.position.z = 0.505;
//	 
	 // Orientation
    double angle =M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();

//	Eigen::Affine3d target_pose1 = Eigen::Translation3d(0.415008, 0.0, 0.505)
//         				  			* Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
//      				       			 //                     W       X       Y          Z
    
    
    angle =M_PI;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    
//    	Eigen::Affine3d target_pose2 = Eigen::Translation3d(0.415008, 0.0, 0.505)
//         				  			* Eigen::Quaterniond(quat1.w(), quat1.x(), quat1.y(), quat1.z());
//      				       			 //                     W       X       Y          Z
    
     target_pose2.position.x=target_pose1.position.x;
	 target_pose2.position.y = target_pose1.position.y;
	 target_pose2.position.z = target_pose1.position.z;
	 
	 target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 


	target_pose3.position.x = 0.575008;
	 target_pose3.position.y = -0.25;
	 target_pose3.position.z = 0.312;

 target_pose3.orientation.x = quat1.x();
    target_pose3.orientation.y = quat1.y();
    target_pose3.orientation.z = quat1.z();
    target_pose3.orientation.w = quat1.w();
    
//    Eigen::Affine3d target_pose3 = Eigen::Translation3d(0.575008, -0.25, 0.312)
//         				  			* Eigen::Quaterniond(quat1.w(), quat1.x(), quat1.y(), quat1.z());
//      				       			 //                     W       X       Y          Z
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(target_pose3);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
//  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success1 = group.plan(my_plan);
 

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success1?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);



////******************************************************************************************************
////Visualizing plans
////******************************************************************************************************
//   Now that we have a plan we can visualize it in Rviz.  This is not
//   necessary because the group.plan() call we made above did this
//   automatically.  But explicitly publishing plans is useful in cases that we
//   want to visualize a previously created plan.
 
 if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }
  
  
  // Moving to a pose goal
////******************************************************************************************************
   group.move();
   
   
   
   
    group.setEndEffectorLink("tool0");
  	 	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
   
   
// Now to make a drawing
	geometry_msgs::Pose start_pose = group.getCurrentPose().pose;
	
//	ROS_INFO("frame frame : %s",start_pose);
	ROS_WARN_STREAM("start_pose : =="<<start_pose);
	
  geometry_msgs::Pose target_pose = start_pose;
  std::vector<geometry_msgs::Pose> waypoints;
  
  waypoints.push_back(start_pose);

//	target_pose.position.y += 0.02;
//  waypoints.push_back(target_pose);
  target_pose.position.z -= 0.01;
  waypoints.push_back(target_pose);
  ROS_WARN_STREAM("target_pose : =="<<target_pose);
//  target_pose.position.y -= 0.004;
//  waypoints.push_back(target_pose);
  
//  target_pose.position.x -= 0.02;
//  waypoints.push_back(target_pose);
  
//  target_pose.position.y += 0.002;
//  waypoints.push_back(target_pose);
//  
//  target_pose.position.y -= 0.002;
//  waypoints.push_back(target_pose);
//  
//  target_pose.position.x += 0.02;
//  waypoints.push_back(target_pose);
//  
//  target_pose.position.y += 0.004;
//  waypoints.push_back(target_pose);
//  
//  target_pose.position.x -= 0.02;
//  waypoints.push_back(target_pose);
	
//	waypoints.push_back(start_pose);
	
	
	
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);  
   
   
   // Adding time parametrization to the "trajectory" group("manipulator");
     // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
  
     // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
  
     // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  
     // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  
     // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);
  
  
  //
  
//    // Finally plan and execute the trajectory
//  my_plan.trajectory_ = trajectory;
//  ROS_INFO("Visualizing plan 994 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
//  sleep(5.0);
//  group.execute(my_plan);
  
  //
  // Filtering the "trajectory" with Uniform Sampler

  industrial_trajectory_filters::MessageAdapter t_in;
  t_in.request.trajectory = trajectory.joint_trajectory;
  industrial_trajectory_filters::MessageAdapter t_out;
  industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
  std::cout << "AQUI1 \n";
  adapter.update(t_in, t_out);
  
  
  
  // Adding the "trajectory" to the plan.
	trajectory.joint_trajectory = t_out.request.trajectory;
	  std::cout << "AQUI2 \n";
  my_plan.trajectory_ = trajectory;
	if (fraction == 1.0) 
		group.execute(my_plan);
	else
		ROS_WARN("Could not compute the cartesian path :( ");
// END_TUTORIAL

	// Arm goes back to work position
	group_variable_values[0] = 0.0;
  group_variable_values[1] = 0.0;
  group_variable_values[2] = 0.0;
  group_variable_values[3] = 0.0;
  group_variable_values[4] = 0.0;
  group_variable_values[5] = 0.0;
	group.setJointValueTarget(group_variable_values);
	group.plan(my_plan);
	group.execute(my_plan);

  ros::shutdown();  
  return 0;
   
   
   //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Cartesian Paths 
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints 
  // for the end-effector to go through. Note that we are starting 
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.
  
////////    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
////////  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//////// 
////////  
//////////  end_eff_joint_groups = kinematic_model->getEndEffectors();
//////////  const std::string& parent_group_name = end_eff_joint_groups.at(i)->getName();
//////////   group_names.push_back(parent_group_name);
////////  
////////  std::vector<geometry_msgs::Pose> waypoints;

////////	target_pose4=target_pose3;
////////    
////////    target_pose4.position.z =target_pose4.position.z + 0.09;
////////    
////////  waypoints.push_back(target_pose4);   //up and out


////////	target_pose4.position.y =target_pose4.position.y + 0.09;
////////  waypoints.push_back(target_pose4);  // left
////////  
////////  
////////  
////////target_pose4.position.x =target_pose4.position.x - 0.09;
////////  waypoints.push_back(target_pose4);   //up and out
////////  
////////  
////////  
////////  
////////  
////////  
////////   group.setPlanningTime(0);
////////  // Now, we call the planner to compute the plan
////////  // and visualize it.
////////  // Note that we are just planning, not asking move_group 
////////  // to actually move the robot.
////////  moveit::planning_interface::MoveGroup::Plan my_plan1;
////////  
////////  // We want the cartesian path to be interpolated at a resolution of 1 cm
////////  // which is why we will specify 0.01 as the max step in cartesian
////////  // translation.  We will specify the jump threshold as 0.0, effectively
////////  // disabling it.
////////  moveit_msgs::RobotTrajectory trajectory;
////////  double fraction = group.computeCartesianPath(waypoints,
////////                                               0.8,  // eef_step
////////                                               0.0,   // jump_threshold
////////                                               trajectory);
//////////  robot_trajectory::RobotTrajectory rt(kinematic_model, group_names[selected_plan_group]);
////////  
////////      // Thrid create a IterativeParabolicTimeParameterization object
////////    trajectory_processing::IterativeParabolicTimeParameterization iptp;
//////////    bool success = iptp.computeTimeStamps(rt);
////////    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
////////    
////////    

////////  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
////////        fraction * 100.0);    
////////  /* Sleep to give Rviz time to visualize the plan. */
//////////  group.move();
////////  sleep(15.0);
   
   

//////////



//////******************************************************************************************************
////  // Cartesian Paths 222222222222222222222222222222222
//////******************************************************************************************************
////  // You can plan a cartesian path directly by specifying a list of waypoints 
////  // for the end-effector to go through. Note that we are starting 
////  // from the new start state above.  The initial pose (start state) does not
////  // need to be added to the waypoint list.
////  std::vector<geometry_msgs::Pose> waypoints;

////  geometry_msgs::Pose target_pose3 = target_pose1;
////  target_pose3.position.x += 0.1;
////  target_pose3.position.z += 0.05;
////  waypoints.push_back(target_pose3);  // up and out

////  target_pose3.position.z -= 0.05;
////  waypoints.push_back(target_pose3);  // left

////  target_pose3.position.z -= 0.1;
//////  target_pose3.position.y += 0.1;
////  target_pose3.position.x -= 0.05;
////  waypoints.push_back(target_pose3);  // down and right (back to start)

////	

////  // We want the cartesian path to be interpolated at a resolution of 1 cm
////  // which is why we will specify 0.01 as the max step in cartesian
////  // translation.  We will specify the jump threshold as 0.0, effectively
////  // disabling it.
////  moveit_msgs::RobotTrajectory trajectory;
////  double fraction = group.computeCartesianPath(waypoints,
////                                               0.01,  // eef_step
////                                               0.0,   // jump_threshold
////                                               trajectory);

////  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
////        fraction * 100.0);    

////moveit::planning_interface::MoveGroup::Plan plan;
////plan.trajectory_ = trajectory;
////group.execute(plan);
////  
////  ROS_INFO("aqui aqui aqui aqui aqui aqui aqui aqui");  
////      







//	group.setPlannerId("SBLkConfigDefault");
//	group.setPlanningTime(45);
//  // Adding/Removing Objects and Attaching/Detaching Objects
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  // First, we will define the collision object message.
//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = group.getPlanningFrame();
//	
//	double max_velocity_scaling_factor = 1;
//	group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);

//  /* The id of the object is used to identify it. */
//  collision_object.id = "box1";

//  /* Define a box to add to the world. */
//  shape_msgs::SolidPrimitive primitive;
//  primitive.type = primitive.BOX;
//  primitive.dimensions.resize(3);
//  primitive.dimensions[0] = 0.2;
//  primitive.dimensions[1] = 0.1;
//  primitive.dimensions[2] = 0.7;

//  /* A pose for the box (specified relative to frame_id) */
//  geometry_msgs::Pose box_pose;
//  box_pose.orientation.w = 0.0;
//  box_pose.position.x =  0.35;
//  box_pose.position.y = -0.3;
//  box_pose.position.z =  0.40;

//  collision_object.primitives.push_back(primitive);
//  collision_object.primitive_poses.push_back(box_pose);
//  collision_object.operation = collision_object.ADD;

//  std::vector<moveit_msgs::CollisionObject> collision_objects;  
//  collision_objects.push_back(collision_object);  

//  // Now, let's add the collision object into the world
//  ROS_INFO("Add an object into the world");  
//  planning_scene_interface.addCollisionObjects(collision_objects);
//  
//  /* Sleep so we have time to see the object in RViz */
//  sleep(2.0);

//  // Planning with collision detection can be slow.  Lets set the planning time
//  // to be sure the planner has enough time to plan around the box.  10 seconds
//  // should be plenty.
//  group.setPlanningTime(10.0);


//  // Now when we plan a trajectory it will avoid the obstacle
//  group.setStartState(*group.getCurrentState());
//  group.setPoseTarget(target_pose1);
//  success = group.plan(my_plan);

////	group.execute(my_plan);

//  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
//    success?"":"FAILED");
//  /* Sleep to give Rviz time to visualize the plan. */
//  sleep(10.0);
  
  
  
  
  

  // Now, let's attach the collision object to the robot.
//  ROS_INFO("Attach the object to the robot");  
//  group.attachObject(collision_object.id);  
//  /* Sleep to give Rviz time to show the object attached (different color). */
//  sleep(4.0);


  // Now, let's detach the collision object from the robot.
//  ROS_INFO("Detach the object from the robot");  
//  group.detachObject(collision_object.id);  
//  /* Sleep to give Rviz time to show the object detached. */
//  sleep(4.0);


//  // Now, let's remove the collision object from the world.
//  ROS_INFO("Remove the object from the world");  
//  std::vector<std::string> object_ids;
//  object_ids.push_back(collision_object.id);  
//  planning_scene_interface.removeCollisionObjects(object_ids);
//  /* Sleep to give Rviz time to show the object is no longer there. */
//  sleep(4.0);



//  ros::shutdown();  
  return 0;
}
