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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "TIAGO_move_group");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ros::NodeHandle node_handle; 

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");

group.setPlanningTime(45);
 group.setEndEffectorLink("tool0");













//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//Informativo
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  
   // (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;


sleep(1.0);

  // We can plan a motion for this group to a desired pose for the end-effector.
  geometry_msgs::Pose pose;
 pose.position.x = -0.0628377;
 pose.position.y = -0.581449;
 pose.position.z = 0.753022;
pose.orientation.x =-0.9;
pose.orientation.y =-0.964875;
 pose.orientation.z =0.256041;
 pose.orientation.w =-0.0428377;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   // Visualizing plans
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan My_First_Plan;
  bool success = group.plan(My_First_Plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
	

  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");    
    display_trajectory.trajectory_start = My_First_Plan.start_state_;
    display_trajectory.trajectory.push_back(My_First_Plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }











//  group.setPoseTarget(pose);
//  group.move();









////  // Planning to a joint-space goal 
//////^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
////  
////  // First get the current set of joint values for the group.
////  std::vector<double> group_variable_values;
////  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
////   // Now, let's modify one of the joints, plan to the new joint
////  // space goal and visualize the plan.
////  group_variable_values[0] = -0.25;  
////  group.setJointValueTarget(group_variable_values);
////  success = group.plan(My_First_Plan);

////  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
////  /* Sleep to give Rviz time to visualize the plan. */
////  sleep(5.0);
////  
////  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
////  group.move();
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
//////^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
////  // Cartesian Paths 
//////^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
////  // You can plan a cartesian path directly by specifying a list of waypoints 
////  // for the end-effector to go through. Note that we are starting 
////  // from the new start state above.  The initial pose (start state) does not
////  // need to be added to the waypoint list.
////  std::vector<geometry_msgs::Pose> waypoints;

//////  geometry_msgs::Pose target_pose3 = start_pose2;
////geometry_msgs::Pose target_pose3 = pose;
//////  target_pose3.position.x += 0.1;
////  target_pose3.position.z += 0.3;
////  waypoints.push_back(target_pose3);  // up and out
//// group.move();
//////  sleep(5.0);
//////  target_pose3.position.x -= 0.1;
//////  waypoints.push_back(target_pose3);  // left
////////  target_pose3.position.z -= 0.1;
////////  target_pose3.position.y += 0.1;
//////  target_pose3.position.x += 0.1;
//////  waypoints.push_back(target_pose3);  // down and right (back to start)

//////  // We want the cartesian path to be interpolated at a resolution of 1 cm
//////  // which is why we will specify 0.01 as the max step in cartesian
//////  // translation.  We will specify the jump threshold as 0.0, effectively
//////  // disabling it.
//////  moveit_msgs::RobotTrajectory trajectory;
//////  double fraction = group.computeCartesianPath(waypoints,
//////                                               0.001,  // eef_step
//////                                               0.0,   // jump_threshold
//////                                               trajectory);

//////  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
//////        fraction * 100.0);    
//////  /* Sleep to give Rviz time to visualize the plan. */
//////  group.move();
//////  sleep(15.0);





















//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Adding/Removing Objects and Attaching/Detaching Objects
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";
  collision_object.id = "box2";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.5;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.1;
  box_pose.position.y = -0.6;
  box_pose.position.z =  0.4;
  
  geometry_msgs::Pose box_pose2;
  box_pose2.orientation.w = 1.0;
  box_pose2.position.x =  0.1;
  box_pose2.position.y = 1.6;
  box_pose2.position.z =  0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose2);
  collision_object.operation = collision_object.ADD;


  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  group.setPlanningTime(10.0);


  // Now when we plan a trajectory it will avoid the obstacle
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(pose);
  success = group.plan(My_First_Plan);

  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
    success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);
  

  // Now, let's attach the collision object to the robot.
  ROS_INFO("Attach the object to the robot");  
  group.attachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(4.0);


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");  
  group.detachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);
  
//  pose;
 group.move();


//  // Now, let's remove the collision object from the world.
//  ROS_INFO("Remove the object from the world");  
//    sleep(14.0);
//  std::vector<std::string> object_ids;
//  object_ids.push_back(collision_object.id);  
//  planning_scene_interface.removeCollisionObjects(object_ids);
//  /* Sleep to give Rviz time to show the object is no longer there. */
//  sleep(4.0);

spinner.stop();
ros::shutdown(); 
return 0;







}
