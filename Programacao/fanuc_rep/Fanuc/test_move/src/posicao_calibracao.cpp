
////******************************************************************************************************
  // Todos os includes
////******************************************************************************************************

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>


//----------------------------------------
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
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
//###############Includes para o grupo do inverse kinematic
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit/joint_model_group.h>
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


////******************************************************************************************************
  // Home position
////******************************************************************************************************
void home_position(moveit::planning_interface::MoveGroup &group)
{
static const std::string ROBOT_DESCRIPTION="robot_description";



	double posit=0.00;
	double max_velocity_scaling_factor=0.11;
	double max_acceleration_scaling_factor=1;
	double valores;
	
//for(int n=0;n<2;n++){
		//	group.setMaxVelocityScalingFactor (max_velocity_scaling_factor);
//			group.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

		  std::map<std::string, double> joints1;
		  std::map<std::string, double> joints2;
		  
		  std::cout<<"PASSEI DEPOIS\n";
		
		  joints2["joint_1"] = 0.00;
		  joints2["joint_2"] =  0.00;
		  joints2["joint_3"] =  0.00;
		  joints2["joint_4"] =  0.00;
		  joints2["joint_5"] =  0.00;
		  joints2["joint_6"] = 0.00;

		  group.setJointValueTarget(joints2);
		  std::cout<<"PASSEI\n";
		  group.move();

}






////******************************************************************************************************
  // approach_pick, que faz o ponto de aproximação para o pick
////******************************************************************************************************
void posicao_calibracao(moveit::planning_interface::MoveGroup &group,double x,double y, double z)
{
//group.asyncMove ();
  	 group.setEndEffectorLink("tool0");
  	 	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
  	 	
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
	
	double tolerance=15;
	group.setGoalJointTolerance (tolerance);
	
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
//  p.pose.position.x = x;
//  p.pose.position.y = y;
//  p.pose.position.z = z;
//  p.pose.orientation.x = 0;
//  p.pose.orientation.y = 0;
//  p.pose.orientation.z = 0;
//  p.pose.orientation.w = 1;
  
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
//  geometry_msgs::Pose target_pose3;

	 // Orientation
    double angle =0;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
	 target_pose1.position.x = x;
	 target_pose1.position.y = y;
	 target_pose1.position.z = z;
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();
    
    
    
    angle =0;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    target_pose2.position.x=target_pose1.position.x;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 

	p.pose.position.x = x;
	p.pose.position.y = y;
	p.pose.position.z = z;
 	p.pose.orientation.x = quat1.x();
    p.pose.orientation.y = quat1.y();
    p.pose.orientation.z = quat1.z();
    p.pose.orientation.w = quat1.w();
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(p);
  
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan1;
  bool success = group.plan(my_plan1);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  
    /* Sleep to give Rviz time to visualize the plan. */
//  sleep(1.0);
  
//  	group.asyncExecute(my_plan);
    group.move();
//    group.execute(my_plan1);
    
}




////******************************************************************************************************
  // approach_pick, que faz o ponto de aproximação para o pick
////******************************************************************************************************
void posicao_tcp(moveit::planning_interface::MoveGroup &group,double x,double y, double z)
{
//group.asyncMove ();
  	 group.setEndEffectorLink("tool0");
  	 	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
  	 	
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
	
	double tolerance=15;
	group.setGoalJointTolerance (tolerance);
	
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
//  p.pose.position.x = x;
//  p.pose.position.y = y;
//  p.pose.position.z = z;
//  p.pose.orientation.x = 0;
//  p.pose.orientation.y = 0;
//  p.pose.orientation.z = 0;
//  p.pose.orientation.w = 1;
  
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
//  geometry_msgs::Pose target_pose3;

	 // Orientation
    double angle =M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
	 target_pose1.position.x = x;
	 target_pose1.position.y = y;
	 target_pose1.position.z = z;
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();
    
    
    
    angle =M_PI;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    target_pose2.position.x=target_pose1.position.x;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 

	p.pose.position.x = x;
	p.pose.position.y = y;
	p.pose.position.z = z;
 	p.pose.orientation.x = quat1.x();
    p.pose.orientation.y = quat1.y();
    p.pose.orientation.z = quat1.z();
    p.pose.orientation.w = quat1.w();
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(p);
  
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan1;
  bool success = group.plan(my_plan1);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  
    /* Sleep to give Rviz time to visualize the plan. */
//  sleep(1.0);
  
//  	group.asyncExecute(my_plan);
    group.move();
//    group.execute(my_plan1);
    
}












int main(int argc, char **argv)
{
  ros::init (argc, argv, "posicao_de_calibracao");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco1 = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

//  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("manipulator");
  
  	 group.setEndEffectorLink("tool0");
  
//  group.setPlanningTime(5.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_link";

  
   // remove pole
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);
//ROS_INFO("ERRO AQUI!! 1111111111111111111.\n");
////////////  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.900;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.250;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.035;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.355;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = -0.0176;
  co.primitive_poses[0].orientation.w = 0;
  pub_co.publish(co);



	co.id = "barra_dir";
	  co.operation = moveit_msgs::CollisionObject::REMOVE;
	  pub_co.publish(co);
	 
	  moveit_msgs::AttachedCollisionObject aco1;
	  aco1.object = co;
	  pub_aco1.publish(aco1);
	  co.operation = moveit_msgs::CollisionObject::ADD;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.120;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.350;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.100;
	  co.primitive_poses[0].position.x = 0.500;
	  co.primitive_poses[0].position.y = 0.000;
	  co.primitive_poses[0].position.z = 0.800;
	  pub_co.publish(co);
	  
	  	co.id = "barra_dir";
	  co.operation = moveit_msgs::CollisionObject::REMOVE;
	  pub_co.publish(co);
	  moveit_msgs::AttachedCollisionObject aco2;
	  aco1.object = co;
	  pub_aco1.publish(aco2);
	  co.operation = moveit_msgs::CollisionObject::ADD;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.0600;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.0300;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.350;
	  co.primitive_poses[0].position.x = 0.775;
	  co.primitive_poses[0].position.y = -0.640;
	  co.primitive_poses[0].position.z = 0.6400;
	  pub_co.publish(co);
	  
  	  co.id = "barra_esq";
	  co.operation = moveit_msgs::CollisionObject::REMOVE;
	  pub_co.publish(co);
	  moveit_msgs::AttachedCollisionObject aco3;
	  aco1.object = co;
	  pub_aco1.publish(aco3);
	  co.operation = moveit_msgs::CollisionObject::ADD;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.0600;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.0300;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.350;
	  co.primitive_poses[0].position.x = 0.775;
	  co.primitive_poses[0].position.y = 0.640;
	  co.primitive_poses[0].position.z = 0.6400;
	  pub_co.publish(co);
	  
	  co.id = "barra_cima";
	  co.operation = moveit_msgs::CollisionObject::REMOVE;
	  pub_co.publish(co);
	  moveit_msgs::AttachedCollisionObject aco4;
	  aco1.object = co;
	  pub_aco1.publish(aco4);
	  co.operation = moveit_msgs::CollisionObject::ADD;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.0600;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.280;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.030;
	  co.primitive_poses[0].position.x = 0.775;
	  co.primitive_poses[0].position.y = 0.000;
	  co.primitive_poses[0].position.z = 0.900;
	  pub_co.publish(co);
	  
	  co.id = "barra_kin";
	  co.operation = moveit_msgs::CollisionObject::REMOVE;
	  pub_co.publish(co);
	  moveit_msgs::AttachedCollisionObject aco5;
	  aco1.object = co;
	  pub_aco1.publish(aco5);
	  co.operation = moveit_msgs::CollisionObject::ADD;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.400;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.030;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.030;
	  co.primitive_poses[0].position.x = 0.675;
	  co.primitive_poses[0].position.y = 0.000;
	  co.primitive_poses[0].position.z = 0.900+0.030;
	  pub_co.publish(co);
  
  ROS_INFO("Posicao_Calibracao\n");

//	home_position(group);

//    posicao_calibracao(group,0.480,0.000,0.680);//Posicao real de calibraçao
    
  // posicao_tcp(group,0.400,-0.300,0.000);//PARA AS DISTANCIAS

ROS_INFO("Fim Posicao Calibracao\n");

  sleep(4.0);
//  ros::waitForShutdown();
  return 0;
}

