// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_arm");

  if ( argc < 7 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun marco_control plan_arm_cartesian  x y z  r p y");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of /arm_tool_link expressed in /base_footprint");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }



  geometry_msgs::Pose current_position;



  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  // goal_pose.pose.position.x = 0;
  // goal_pose.pose.position.y = 0;
  // goal_pose.pose.position.z = 0;
  // goal_pose.pose.orientation.x = 0;
  // goal_pose.pose.orientation.y = 0;
  // goal_pose.pose.orientation.z = 0;
  // goal_pose.pose.orientation.w = 0;

  
  goal_pose.pose.position.x = atof(argv[1]);
  goal_pose.pose.position.y = atof(argv[2]);
  goal_pose.pose.position.z = atof(argv[3]);
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints

  moveit::planning_interface::MoveGroupInterface group_arm("marco_arm");
  //choose your preferred planner
  
  geometry_msgs::PoseStamped robot_pose;
  robot_pose = group_arm.getCurrentPose();

  geometry_msgs::Point exact_pose = current_position.position;
  geometry_msgs::Quaternion exact_orientation = current_position.orientation;

  ROS_INFO("Reference frame : %s",group_arm.getPlanningFrame().c_str());
  ROS_INFO("End effector link : %s",group_arm.getEndEffectorLink().c_str());

  std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
  std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;

  group_arm.setPlannerId("SBLkConfigDefault");
  group_arm.setPoseReferenceFrame("base_footprint");
  
   
  
  //ROS_INFO_STREAM("Starting state: " <<
  //                *group_arm.getCurrentState()); 
  
  
  
  group_arm.setPoseTarget(goal_pose);
  


  

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm.getPlanningFrame());

  group_arm.setStartStateToCurrentState();
  group_arm.setMaxVelocityScalingFactor(1.0);

  

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm.setPlanningTime(5.0);

  ROS_INFO_STREAM("CHECK");

  
  auto success = group_arm.plan(my_plan);


  /*

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  */
  
  spinner.stop();

  return EXIT_SUCCESS;
}